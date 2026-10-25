"""SI loads, mm finite-link kinematics. Standalone, NumPy only (bundled with FreeCAD)."""
import json
import math
from pathlib import Path
import numpy as np

P = dict(radius=9300., chord=650., root=1600., joint=600., hub_z=2200.,
         swash_z=1100., swash_joint_z=100., horn_x=600., horn=250., collective_ref=12.,
         rpm=210., blade_mass=180., aircraft_mass=15600., rho=1.225,
         fold_pin=1500., link_length=float(np.hypot(250*(np.cos(np.deg2rad(12))-1),1000+250*np.sin(np.deg2rad(12)))))
OUT = Path(__file__).resolve().parents[1]

def rx(a):
    c,s=np.cos(a),np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def deg(a): return np.deg2rad(a)

def swash(collective=12., cyclic_c=0., cyclic_s=0., p=P):
    # Normal chosen so height at the horn's azimuth follows the defined harmonic.
    h=p['horn']*(np.sin(deg(collective))-np.sin(deg(p['collective_ref'])))
    phase=np.arctan2(p['horn'],p['horn_x'])
    scale=p['horn']*np.cos(deg(p['collective_ref']))/np.hypot(p['horn_x'],p['horn'])
    c,s=deg(cyclic_c)*scale,deg(cyclic_s)*scale
    gx=c*np.cos(phase)-s*np.sin(phase)
    gy=c*np.sin(phase)+s*np.cos(phase)
    n=np.array([-gx,-gy,1.]); n/=np.linalg.norm(n)
    z=np.array([0.,0.,1.]); v=np.cross(z,n)
    k=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    q=np.eye(3)+k+k@k/(1+n[2])
    return np.array([0.,0.,p['swash_z']+h]),q

def solve(psi, collective=12., cyclic_c=0., cyclic_s=0., flap=0., lag=0., p=P):
    center,q=swash(collective,cyclic_c,cyclic_s,p)
    az=rz(deg(psi)); articulated=az@rz(deg(lag))@ry(-deg(flap))
    pivot=az@np.array([p['joint'],0.,0.])+[0,0,p['hub_z']]
    bottom=center+q@az@np.array([p['horn_x'],p['horn'],p['swash_joint_z']])
    base=pivot+articulated@np.array([p['horn_x']-p['joint'],0.,0.])
    u=articulated@np.array([0.,p['horn'],0.])
    v=articulated@np.array([0.,0.,p['horn']])
    d=base-bottom
    a,b=2*d@u,2*d@v
    c=p['link_length']**2-d@d-p['horn']**2
    amp=np.hypot(a,b)
    if abs(c)>amp: raise ValueError('No finite-link closure')
    delta=np.arctan2(b,a); ac=np.arccos(np.clip(c/amp,-1,1))
    candidates=[delta-ac,delta+ac,delta-ac-2*np.pi,delta+ac-2*np.pi]
    candidates=[t for t in candidates if -deg(35)<t<deg(45)]
    if not candidates: raise ValueError('Pitch outside assembly branch')
    theta=min(candidates,key=lambda t:abs(t-deg(collective)))
    top=base+u*np.cos(theta)+v*np.sin(theta)
    sensitivity=abs((-a*np.sin(theta)+b*np.cos(theta))/(2*p['link_length']))
    return dict(theta=float(np.rad2deg(theta)),top=top,bottom=bottom,pivot=pivot,
                rotation=articulated@rx(theta),residual=float(abs(np.linalg.norm(top-bottom)-p['link_length'])),
                lever=float(sensitivity))

def loads(p=P):
    r=p['radius']/1000; r0=p['root']/1000; m=p['blade_mass']; om=p['rpm']*2*np.pi/60
    cg=(r+r0)/2; inertia=m*((r-r0)**2/12+(cg-p['joint']/1000)**2)
    thrust=1.05*p['aircraft_mass']*9.80665
    area=np.pi*r*r; vi=np.sqrt(thrust/(2*p['rho']*area))
    sigma=5*p['chord']/1000/(np.pi*r)
    profile=sigma*.012/8*p['rho']*area*(om*r)**3
    power=1.15*thrust*vi+profile
    # Uniform inflow + linear lift slope, no tip loss: screening, not performance validation.
    rr=np.linspace(r0,r,2001); vel=om*rr
    twist=deg(-8)*(rr-.75*r)/(r-r0)
    slope=.5*p['rho']*5*p['chord']/1000*5.7
    lift0=np.trapz(slope*vel**2*(twist-vi/vel),rr)
    liftgrad=np.trapz(slope*vel**2,rr)
    trim=(thrust-lift0)/liftgrad
    cf=m*om**2*cg
    # Linearized moment arm from the same bracket centres as build.py.
    da=np.array([-.180,-.400,0.]); db=rx(deg(12))@np.array([.450,-.350,0.])
    line=db-da;line/=np.linalg.norm(line)
    arm=abs(np.cross(db,line)[2])
    k_lag=80000.; damper=32000.; c_eff=damper*arm**2
    wn=np.sqrt(k_lag/inertia)
    # Mast equivalent section, resultant blade CF is self-balanced radially in the hub.
    D,d=.32,.23; j=np.pi*(D**4-d**4)/32; am=np.pi*(D**2-d**2)/4
    tau=1.5*(power/om)*D/(2*j); sig=1.5*thrust/am
    pin=.085; lock_force=1.5*cf
    pin_shear=lock_force/(2*np.pi*pin**2/4)
    return dict(omega_rad_s=om,tip_speed_m_s=om*r,tip_Mach=om*r/340.3,
                thrust_N=thrust,disk_loading_N_m2=thrust/area,induced_velocity_m_s=vi,
                profile_power_W=profile,power_W=power,torque_Nm=power/om,
                blade_cg_m=cg,blade_inertia_hinge_kgm2=inertia,centrifugal_N=cf,
                trim_pitch_75_deg=float(np.rad2deg(trim)),mast_VM_MPa=float(np.sqrt(sig**2+3*tau**2)/1e6),
                fold_pin_double_shear_MPa=pin_shear/1e6,
                damper_arm_m=arm,damper_linear_Ns_m=damper,lag_frequency_Hz=wn/(2*np.pi),lag_zeta=c_eff/(2*np.sqrt(k_lag*inertia)),
                lag_zeta_degraded=.5*c_eff/(2*np.sqrt(k_lag*inertia)),lag_zeta_no_damper=0.,
                lag_decay_5s=float(np.exp(-c_eff/(2*inertia)*5)),
                lag_decay_degraded_5s=float(np.exp(-.5*c_eff/(2*inertia)*5)))

def flight_allowed(rpm, pins, mast_locked, folding):
    return all(pins) and not mast_locked and not folding

def verify(p=P):
    max_error=max_res=0.; min_lever=1e9; tmin=999.; tmax=-999.; count=0
    worst=None
    for step in [12,6]:
        e=0.; res=0.; lever=1e9
        for psi in np.arange(0,360,step):
            for collective in (8,12,16):
                for ca in np.arange(0,360,45):
                    cc,cs=3*np.cos(deg(ca)),3*np.sin(deg(ca))
                    for flap,lag in [(0,0),(-3,-3),(-3,3),(6,-3),(6,3)]:
                        s=solve(psi,collective,cc,cs,flap,lag,p)
                        ideal=collective+cc*np.cos(deg(psi))+cs*np.sin(deg(psi))
                        err=abs(s['theta']-ideal)
                        if err>max_error: max_error=err; worst=[float(psi),collective,float(cc),float(cs),flap,lag,s['theta']]
                        if flap==lag==0: e=max(e,err)
                        res=max(res,s['residual']); lever=min(lever,s['lever'])
                        tmin=min(tmin,s['theta']); tmax=max(tmax,s['theta']); count+=1
        yield str(step)+'deg_grid',dict(samples=count,uncoupled_error_deg=e,residual_mm=res,min_lever_mm_rad=lever)
        max_res=max(max_res,res); min_lever=min(min_lever,lever)
    zero=np.array([solve(i*72,p=p)['theta'] for i in range(5)])
    mat=[]
    for inp in [(12.01,0,0),(12,.01,0),(12,0,.01)]:
        mat.append((np.array([solve(i*72,*inp,p=p)['theta'] for i in range(5)])-zero)/.01)
    matrix=np.array(mat).T
    assert np.linalg.matrix_rank(matrix)==3
    assert max_res<1e-6 and min_lever>100
    assert max_error<.5 and tmin>=-5 and tmax<=25 and np.linalg.cond(matrix)<3
    assert matrix[0,1]>0 and matrix[1,2]>0
    try: solve(0,p={**p,'link_length':20})
    except ValueError: rejected=True
    else: rejected=False
    assert rejected
    assert flight_allowed(210,[True]*5,False,False)
    assert not flight_allowed(210,[True,False,True,True,True],False,False)
    reverse=np.array([solve(i*72,12,-3,0,p=p)['theta'] for i in range(5)])
    expect=12+3*np.cos(deg(np.arange(5)*72))
    assert np.max(abs(reverse-expect))>5
    variants={}
    for name,pp in [('length_plus_2mm',{**p,'link_length':p['link_length']+2}),('horn_x_plus_5mm',{**p,'horn_x':p['horn_x']+5})]:
        state=solve(0,p=pp); assert state['residual']<1e-6
        variants[name]=dict(pitch_deg=state['theta'],top_mm=state['top'].tolist())
    yield 'summary',dict(max_coupled_error_deg=max_error,worst=worst,pitch_range_deg=[tmin,tmax],
                        jacobian=matrix.tolist(),rank=3,condition=float(np.linalg.cond(matrix)),
                        negative_tests=dict(impossible_link=rejected,reversed_phase=True,unlocked_flight=True),variants=variants)

if __name__=='__main__':
    result=dict(loads=loads(),kinematics=dict(verify()))
    (OUT/'verification'/'numerical.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

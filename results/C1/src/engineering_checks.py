"""Capacity demands, stroke/angle envelopes and uncertainty: no fabricated allowables."""
import json
import numpy as np
from mechanism import P,OUT,loads,solve,swash,rx,rz,deg

base=loads(); variants={}
for label,p in [('rpm_minus10',{**P,'rpm':189}),('rpm_plus10',{**P,'rpm':231}),
                ('mass_minus20',{**P,'blade_mass':144}),('mass_plus20',{**P,'blade_mass':216}),
                ('rpm10_mass20',{**P,'rpm':231,'blade_mass':216})]:variants[label]=loads(p)
F=1.5*base['centrifugal_N'];maxF=1.5*variants['rpm10_mass20']['centrifugal_N']
diam=.034;length=P['link_length']/1000
link_I=np.pi*diam**4/64
demands=dict(load_factor=1.5,blade_retention_base_N=F,blade_retention_uncertain_N=maxF,
             cuff_axial_MPa=maxF/(np.pi*.13**2/4)/1e6,
             fold_pin_double_shear_uncertain_MPa=maxF/(2*np.pi*.085**2/4)/1e6,
             fold_clevis_bearing_uncertain_MPa=maxF/(2*.04*.085)/1e6,
             elastomer_projected_compression_MPa=maxF/(np.pi*(.19**2-.095**2))/1e6,
             link_force_assumption_N=30000,link_factored_axial_stress_MPa=45000/(np.pi*diam**2/4)/1e6,
             link_Euler_pinned_N=np.pi**2*200e9*link_I/length**2,
             limitation='Nominal stresses only. No material allowables, fatigue, pin bending or bearing ratings; no strength pass.')
stroke=[];upper=[];lower=[];servo=[]
for psi in np.arange(0,360,12):
    az=rz(deg(psi))
    for collective in (8,12,16):
        for ca in range(0,360,45):
            cc,cs=3*np.cos(deg(ca)),3*np.sin(deg(ca));center,q=swash(collective,cc,cs)
            for flap,lag in [(0,0),(-3,-3),(-3,3),(6,-3),(6,3)]:
                s=solve(psi,collective,cc,cs,flap,lag)
                line=s['top']-s['bottom'];line/=np.linalg.norm(line)
                upper.append(float(np.rad2deg(np.arccos(np.clip(abs(line@(s['rotation']@np.array([0,0,1]))),-1,1)))))
                lower.append(float(np.rad2deg(np.arccos(np.clip(abs(line@q[:,2]),-1,1)))))
                da=az@np.array([420,-400,0])+[0,0,P['hub_z']]
                db=s['pivot']+s['rotation']@np.array([450,-350,0])
                stroke.append(float(np.linalg.norm(db-da)))
            for a in [30,150,270]:
                er=np.array([np.cos(deg(a)),np.sin(deg(a)),0]);bt=710*er+[0,0,150];tp=center+q@(710*er+[0,0,-100])
                servo.append(float(np.linalg.norm(tp-bt)))
z=base['lag_zeta']
res=dict(capacity_demands=demands,sensitivity=variants,
         articulation_demands=dict(upper_rod_end_deg=max(upper),lower_rod_end_deg=max(lower),
                                   damper_eye_distance_mm=[min(stroke),max(stroke)],
                                   servo_eye_distance_mm=[min(servo),max(servo)]),
         lag_sensitivity=dict(normal_zeta=z,degraded_zeta=.5*z,stiffness_plus50_zeta=z/np.sqrt(1.5),
                              stiffness_minus50_zeta=z/np.sqrt(.5),mass_plus20_zeta=z/np.sqrt(1.2),
                              degraded_mass20_stiff50_zeta=.5*z/np.sqrt(1.2*1.5)),
         warning='Damper coefficient and elastic stiffness are assumptions; off-diagonal stiffness and rotor-body stability unverified.')
assert demands['link_Euler_pinned_N']>45000
(OUT/'verification/engineering.json').write_text(json.dumps(res,indent=2))
print(json.dumps(res['articulation_demands'],indent=2))
print(json.dumps(res['lag_sensitivity'],indent=2))

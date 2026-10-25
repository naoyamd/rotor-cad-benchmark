"""Signed elastic control reactions, real servo strokes, and full-turn envelopes.

The 3x3 equilibrium solves vertical force and roll/pitch moment. Remaining
force and yaw moment are assigned to the spherical guide and scissors. This
does not supply the missing aerodynamic hinge-moment or bearing rating.
"""
import json
import numpy as np
from mechanism import P,OUT,swash_frames,solve,rz,deg

def evaluate(c,cc,cs,psi,flap=0,lag=0):
    f=swash_frames(c,cc,cs,psi);center=f['center'];qs=f['stator_rotation']
    F=np.zeros(3);M=np.zeros(3);pitch=[];mis=[];damper=[];forces=[]
    for i in range(5):
        s=solve(psi+72*i,c,cc,cs,flap,lag,mast_psi=psi)
        delta=s['top']-s['bottom'];u=delta/np.linalg.norm(delta)
        # Nm / m/rad: signed spring torque about the stress-free 12deg pitch.
        force=12000*deg(s['theta']-12)/(s['lever']/1000)
        ff=force*u;F+=ff;M+=np.cross((s['bottom']-center)/1000,ff)
        forces.append(float(force));pitch.append(s['theta'])
        for axis in [s['rotation'][:,0],f['rotor_rotation']@rz(deg(i*72))[:,0]]:
            mis.append(float(np.rad2deg(np.arcsin(np.clip(abs(axis@u),0,1)))))
        az=rz(deg(psi+i*72));a=az@np.array([420,-400,0])+[0,0,P['hub_z']]
        b=s['pivot']+s['rotation']@np.array([450,-350,0]);du=b-a;L=np.linalg.norm(du)
        damper.append(float(L))
        for axis in [az[:,2],s['rotation'][:,2]]:mis.append(float(np.rad2deg(np.arcsin(np.clip(abs(axis@du/L),0,1)))))
    cols=[];dirs=[];rs=[];lengths=[];servo_mis=[]
    for angle in [30,150,270]:
        az=rz(deg(angle));er=az[:,0];a=er*710+[0,0,150];b=center+qs@(er*710+[0,0,-190]);du=b-a;L=np.linalg.norm(du);u=du/L;r=(b-center)/1000
        cols.append([u[2],np.cross(r,u)[0],np.cross(r,u)[1]]);dirs.append(u);rs.append(r);lengths.append(float(L))
        for axis in [az[:,1],qs@az[:,1]]:servo_mis.append(float(np.rad2deg(np.arcsin(np.clip(abs(axis@u),0,1)))))
    mat=np.array(cols).T;reactions=np.linalg.solve(mat,-np.array([F[2],M[0],M[1]]))
    residual=mat@reactions+np.array([F[2],M[0],M[1]])
    leftover_F=F+sum((n*u for n,u in zip(reactions,dirs)),np.zeros(3))
    leftover_M=M+sum((np.cross(r,n*u) for r,n,u in zip(rs,reactions,dirs)),np.zeros(3))
    return dict(pitch=pitch,pitch_force_N=forces,servo_eye_lengths_mm=lengths,damper_eye_lengths_mm=damper,
                joint_misalignment_deg=max(mis+servo_mis),servo_forces_N=reactions.tolist(),
                elastic_swash_force_N=F.tolist(),elastic_swash_moment_Nm=M.tolist(),
                guide_reaction_N=(-leftover_F).tolist(),scissor_yaw_reaction_Nm=float(-leftover_M[2]),
                equilibrium_residual=float(max(abs(residual))),matrix_condition=float(np.linalg.cond(mat)))

def run():
    result=dict(samples=0,servo_eye_range_mm=[1e9,0],damper_eye_range_mm=[1e9,0],max_joint_misalignment_deg=0,max_abs_elastic_servo_force_N=0,
                max_equilibrium_residual=0,max_matrix_condition=0,case_samples={})
    cases=[(0,0),(-3,-3),(-3,3),(6,-3),(6,3)]
    for psi in range(0,360,15):
        for c in [8,12,16]:
            for phase in range(0,360,45):
                cc=3*np.cos(deg(phase));cs=3*np.sin(deg(phase))
                for flap,lag in cases:
                    r=evaluate(c,cc,cs,psi,flap,lag);result['samples']+=1
                    for key,source in [('servo_eye_range_mm','servo_eye_lengths_mm'),('damper_eye_range_mm','damper_eye_lengths_mm')]:
                        result[key][0]=min(result[key][0],min(r[source]));result[key][1]=max(result[key][1],max(r[source]))
                    result['max_joint_misalignment_deg']=max(result['max_joint_misalignment_deg'],r['joint_misalignment_deg'])
                    result['max_abs_elastic_servo_force_N']=max(result['max_abs_elastic_servo_force_N'],max(abs(np.array(r['servo_forces_N']))))
                    result['max_equilibrium_residual']=max(result['max_equilibrium_residual'],r['equilibrium_residual'])
                    result['max_matrix_condition']=max(result['max_matrix_condition'],r['matrix_condition'])
    for name,c,cc,cs in [('neutral',12,0,0),('max_combined',16,3/np.sqrt(2),3/np.sqrt(2)),('parked',float(np.rad2deg(np.arcsin((P['hub_z']-P['link_length']-P['swash_z']-P['swash_joint_z'])/P['horn']+np.sin(deg(12))))),0,0)]:
        result['case_samples'][name]=evaluate(c,cc,cs,0)
    s=result['servo_eye_range_mm'];dm=result['damper_eye_range_mm']
    # The piston must remain in its cylindrical chamber, including a 10mm end margin.
    result['servo_piston_envelope_from_lower_eye_mm']=[s[0]-580,s[1]-560]
    result['damper_piston_envelope_from_fixed_eye_mm']=[dm[0]-430,dm[1]-410]
    assert s[0]-580>125 and s[1]-560<490
    assert dm[0]-430>120 and dm[1]-410<340
    assert result['max_joint_misalignment_deg']<20
    assert result['max_equilibrium_residual']<1e-6
    result['limits']=['Discrete coupled sweep, not a continuous-domain proof.','Elastic reaction model only; aerodynamic hinge moments and transient loads need a qualified envelope.','20deg joint limit is a geometry screening limit; product rating remains open.']
    (OUT/'verification/control-checks.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':run()

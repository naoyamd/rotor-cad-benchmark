"""Fresh-process native/STEP, geometry-derived endpoints and folding checks."""
import csv,json
import FreeCAD as A
import Part
import numpy as np
from mechanism import P,OUT,solve,deg,rz
from build import components,build,save,moved,V

def snapshot(doc):
    objects=components(doc)
    shapes=[o.Shape for o in objects]
    assert all(s.isValid() and s.Volume>0 and len(s.Solids)>0 for s in shapes)
    comp=Part.makeCompound(shapes)
    bb=comp.BoundBox
    return dict(parts=len(objects),solids=len(comp.Solids),volume_mm3=comp.Volume,
                bounds_mm=[bb.XMin,bb.XMax,bb.YMin,bb.YMax,bb.ZMin,bb.ZMax])

results={}; clearance=[]
for stem in ['rotor-neutral','rotor-max-input','rotor-folded','rotor-parked']:
    d=A.openDocument(str(OUT/(stem+'.FCStd'))); d.recompute()
    state=snapshot(d)
    # Export strictly leaf components, avoiding implicit group compounds.
    Part.export(components(d),str(OUT/(stem+'.step')))
    step=Part.Shape();step.read(str(OUT/(stem+'.step')))
    bb=step.BoundBox
    delta=np.max(abs(np.array(state['bounds_mm'])-[bb.XMin,bb.XMax,bb.YMin,bb.YMax,bb.ZMin,bb.ZMax]))
    rel=abs(step.Volume-state['volume_mm3'])/state['volume_mm3']
    print(stem,'STEP',step.isValid(),rel,delta,'solids',len(step.Solids),state['solids'],flush=True)
    assert step.isValid() and rel<1e-6 and delta<1e-4
    state.update(step_solids=len(step.Solids),step_relative_volume_error=rel,step_max_bound_error_mm=float(delta))
    residual=[]; angle_errors=[]
    for i in range(1,6):
        a=d.getObject('Blade%dLowerBall'%i).Shape.CenterOfMass
        b=d.getObject('Blade%dUpperBall'%i).Shape.CenterOfMass
        residual.append(abs((b-a).Length-P['link_length']))
        if stem not in ['rotor-folded','rotor-parked']:
            c,cc,cs,psi=(12,0,0,0) if stem=='rotor-neutral' else (16,3,0,36)
            theoretical=solve(psi+(i-1)*72,c,cc,cs)
            angle=np.rad2deg(np.arcsin((b.z-P['hub_z'])/P['horn']))
            angle_errors.append(abs(angle-theoretical['theta']))
    assert max(residual)<1e-6
    state['measured_ball_closure_mm']=max(residual)
    state['measured_pitch_error_deg']=max(angle_errors) if angle_errors else None
    results[stem]=state
    # Specific non-contact critical pairs, no blanket assembly exclusion.
    pairs=[('Mast','FixedGuide'),('Mast','Stator'),('Mast','RotorRing')]
    for i in range(1,6):pairs.extend([('Blade%dPitchLink'%i,'Blade%dOuterRace'%i),('Blade%dPitchHorn'%i,'Blade%dOuterRace'%i),('Blade%dDamperBody'%i,'Blade%dOuterRace'%i)])
    for a,b in pairs:
        sa,sb=d.getObject(a).Shape,d.getObject(b).Shape
        clearance.append([stem,a,b,sa.distToShape(sb)[0],sa.common(sb).Volume])
    if stem=='rotor-neutral':
        # Paired negative interference test on a deliberately translated sleeve copy.
        original=d.FixedGuide.Shape; mast=d.Mast.Shape
        assert mast.common(original).Volume<1e-6
        broken=original.copy();broken.translate(V([100,0,0]))
        overlap=mast.common(broken).Volume; assert overlap>1
        results['negative_collision']={'pair':['Mast','FixedGuide'],'translation_mm':[100,0,0],'overlap_mm3':overlap,'detected':True}
    A.closeDocument(d.Name)

# Conservative full-span XY rectangles include the twisted section and thickness.
def polygon_gap(a,b):
    separated=False
    for poly in (a,b):
        for k in range(4):
            edge=poly[(k+1)%4]-poly[k]; normal=np.array([-edge[1],edge[0]])
            aa=a@normal;bb=b@normal
            if aa.max()<bb.min() or bb.max()<aa.min():separated=True
    if not separated:return 0.
    gap=1e30
    for aa,bb in [(a,b),(b,a)]:
        for point in aa:
            for k in range(4):
                origin=bb[k];edge=bb[(k+1)%4]-origin
                t=np.clip((point-origin)@edge/(edge@edge),0,1)
                gap=min(gap,float(np.linalg.norm(point-origin-t*edge)))
    return gap

angles=[0,-72,36,-36,72]
pivots=[np.array([P['fold_pin']*np.cos(deg(i*72)),P['fold_pin']*np.sin(deg(i*72)),P['hub_z']]) for i in range(5)]
local=np.array([[P['root']-P['fold_pin'],-.81*P['chord'],0],[P['radius']-P['fold_pin'],-.81*P['chord'],0],
                [P['radius']-P['fold_pin'],.31*P['chord'],0],[P['root']-P['fold_pin'],.31*P['chord'],0]])
polys=[(local@rz(deg(i*72)).T+pivots[i])[:,:2] for i in range(5)]
min_clear=1e9;worst=None;samples=0
for index in [1,4,2,3]:
    for fraction in np.linspace(0,1,int(abs(angles[index])*2)+1):
        current=(local@rz(deg(index*72+angles[index]*fraction)).T+pivots[index])[:,:2]
        for j in range(5):
            if j==index:continue
            gap=polygon_gap(current,polys[j])
            if gap<min_clear:min_clear=gap;worst=[index+1,j+1,float(fraction)]
        samples+=1
    polys[index]=current
motion_bound=float(np.max(np.linalg.norm(local[:,:2],axis=1))*deg(.5))
results['fold_airfoil_path']=dict(samples=samples,minimum_projected_envelope_gap_mm=min_clear,worst=worst,
                               between_samples_motion_bound_mm=motion_bound,continuous_lower_bound_mm=min_clear-motion_bound,
                               limitation='Full airfoil conservative XY envelopes only. Root lock/tang sweep not certified.')
assert min_clear-motion_bound>10
width=results['rotor-folded']['bounds_mm'][3]-results['rotor-folded']['bounds_mm'][2]
results['folded_width_mm']=width
assert width<9300

# Two separate, real native regeneration tests. Keep files in work, not deliverables.
variants={}
for label,p in [('link_plus_2',{**P,'link_length':P['link_length']+2}),('mount_plus_5',{**P,'horn_x':P['horn_x']+5})]:
    d,b=build('Variant_'+label,16,3,0,p=p)
    ob=d.Blade1UpperBall
    centre=ob.Shape.CenterOfMass
    expected=solve(0,16,3,0,p=p)
    assert (centre-V(expected['top'])).Length<1e-6
    variants[label]={'measured_upper_ball_mm':[centre.x,centre.y,centre.z],'shape_valid':ob.Shape.isValid(), 'parts':len(components(d))}
    A.closeDocument(d.Name)
results['parameter_regeneration']=variants
d,b=build('Articulated_Check',16,3,0,flap=[6,0,-3,0,0],lag=[-3,0,3,0,0])
art=[]
for i in range(5):
    a=d.getObject('Blade%dLowerBall'%(i+1)).Shape.CenterOfMass
    b=d.getObject('Blade%dUpperBall'%(i+1)).Shape.CenterOfMass
    art.append(abs((b-a).Length-P['link_length']))
assert max(art)<1e-6
results['articulated_CAD_check']={'flap_deg':[6,0,-3,0,0],'lag_deg':[-3,0,3,0,0],'max_closure_mm':max(art),'shape_snapshot':snapshot(d)}
A.closeDocument(d.Name)
(OUT/'verification/cad-checks.json').write_text(json.dumps(results,indent=2))
with (OUT/'verification/clearance.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['state','part_a','part_b','nominal_gap_mm','overlap_mm3']);w.writerows(clearance)
print(json.dumps(results,indent=2),flush=True)

import FreeCAD as A,Part,sys,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
results={}
for state in ['rotor-neutral','rotor-max-input','rotor-parked','rotor-folded']:
 d=A.openDocument(str(root/(state+'.FCStd')))
 pairs=[]
 for o in d.Objects:
  if o.Name.endswith('Ball'):
   pre=o.Name[:-4]
   pairs += [(o.Name,pre+'Eye'),(o.Name,pre+'Fork'),(pre+'Eye',pre+'Fork'),(pre+'Fork',pre+'PinHead'),(pre+'Fork',pre+'PinRetainer')]
 for n in ['Antirotation','RotatingDrive']:pairs += [(n+'ArmA',n+'ArmB'),(n+'ArmA',n+'BaseBearing')]
 pairs += [('SliderSphere','StatorSocket'),('LowerRace','BearingBalls'),('UpperRace','BearingBalls'),('UpperRace','BearingCage'),('LowerRace','BearingCage'),('MastIndexPin','FixedGuide'),('MastIndexPin','GuideBridge'),('UpperRaceBolts','StatorSocket')]
 for i in range(1,6):
  g='Blade%d'%i
  for a,b in [('Housing','CuffHorn'),('Housing','PitchUpperEye'),('OuterRace','PitchUpperFork'),('CuffHorn','StopCradle'),('PitchUpperEye','CuffHorn'),('FoldTang','FoldClevis'),('FoldTang','LockGuide')]:pairs.append((g+a,g+b))
  pairs.append(('UpperRaceBolts',g+'PitchLowerFork'))
 bad=[];gaps=[]
 for a,b in pairs:
  oa=d.getObject(a);ob=d.getObject(b)
  if oa and ob:
   v=oa.Shape.common(ob.Shape).Volume
   if v>1:bad.append([a,b,v])
   gaps.append([a,b,oa.Shape.distToShape(ob.Shape)[0]])
 s=Part.Shape();s.read(str(root/(state+'.step')))
 shapes=[o.Shape for o in d.Objects if o.TypeId=='PartDesign::Feature' and o.Name!='AirframeEnvelope']
 native=Part.makeCompound(shapes)
 results[state]={'parts':len(shapes),'step_valid':s.isValid(),'native_volume':native.Volume,'step_volume':s.Volume,'relative_volume_error':abs(s.Volume-native.Volume)/native.Volume,'bad_pairs':bad,'gaps':gaps}
 import math
 results[state]['rod_lengths_mm']={n: d.getObject(n).Shape.Volume/(math.pi*r*r) for n,r in [('Servo0Rod',25),('Blade1DamperRod',20)]}
 assert abs(results[state]['rod_lengths_mm']['Servo0Rod']-500)<1e-5
 assert abs(results[state]['rod_lengths_mm']['Blade1DamperRod']-350)<1e-5
 print(state,results[state]['parts'],results[state]['step_valid'],results[state]['relative_volume_error'],bad,flush=True)
 A.closeDocument(d.Name)
(root/'verification/critical-cad-checks.json').write_text(json.dumps(results,indent=2))
assert all(not row['bad_pairs'] and row['step_valid'] and row['relative_volume_error']<1e-6 for row in results.values())

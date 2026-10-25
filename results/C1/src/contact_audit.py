"""Neutral rigid geometry overlap audit, retaining contacts rather than hiding them."""
import csv
import FreeCAD as A
from mechanism import OUT
from build import components
d=A.openDocument(str(OUT/'rotor-neutral.FCStd'))
for i in range(1,6):
    prefix='Blade%d'%i
    for a,b in [(prefix+'HubArm',prefix+'Elastomer'),(prefix+'HubArm',prefix+'Shims'),
                (prefix+'LowerBall','UpperRace'),(prefix+'LowerBall','BearingRollingElements'),
                (prefix+'FoldHinge',prefix+'RootSparEnvelope'),(prefix+'DamperBody',prefix+'DamperRod')]:
        assert d.getObject(a).Shape.common(d.getObject(b).Shape).Volume<1e-5,(a,b)
rows=[]
for i,a in enumerate(components(d)):
    for b in components(d)[i+1:]:
        if not a.Shape.BoundBox.intersect(b.Shape.BoundBox):continue
        v=a.Shape.common(b.Shape).Volume
        if v>1:
            rows.append([a.Name,b.Name,v])
with (OUT/'verification/neutral-overlaps.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['part_a','part_b','overlap_mm3']);w.writerows(rows)
print(rows)

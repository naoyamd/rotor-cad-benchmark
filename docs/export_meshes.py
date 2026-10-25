"""Run with FreeCAD Python. Tessellate saved C1 states, never modify source CAD."""
import hashlib
import json
from pathlib import Path
import FreeCAD as A
import MeshPart

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs' / 'assets'
OUT.mkdir(exist_ok=True)
manifest = {}
for state in ['neutral', 'max-input', 'parked', 'folded']:
    source = ROOT / 'results' / 'C1' / ('rotor-' + state + '.FCStd')
    doc = A.openDocument(str(source))
    parts = []
    for ob in doc.Objects:
        if ob.TypeId != 'PartDesign::Feature' or ob.Shape.isNull() or ob.Name == 'AirframeEnvelope':
            continue
        mesh = MeshPart.meshFromShape(Shape=ob.Shape, LinearDeflection=3.0,
                                      AngularDeflection=0.35, Relative=False)
        vertices, faces = mesh.Topology
        group = next((g.Name for g in ob.InList if g.TypeId == 'App::DocumentObjectGroup'), '')
        parts.append(dict(name=ob.Name, group=group,
                          vertices=[round(c / 1000, 6) for v in vertices for c in (v.x, v.y, v.z)],
                          indices=[i for face in faces for i in face],
                          connection=getattr(ob, 'Connection', ''),
                          note=getattr(ob, 'EngineeringNote', '')))
    assert len(parts) == 145, len(parts)  # Airframe reference excluded.
    payload = dict(state=state, units='m', source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   tolerance_mm=3, parts=parts)
    target = OUT / (state + '.json')
    target.write_text(json.dumps(payload, separators=(',', ':')), encoding='utf-8')
    manifest[state] = dict(parts=len(parts), bytes=target.stat().st_size, sha256=payload['source_sha256'])
    A.closeDocument(doc.Name)
    print(state, manifest[state])
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')



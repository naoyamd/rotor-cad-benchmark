"""Export the saved C2 FreeCAD states to compact browser meshes.

Run with the FreeCAD Python that can import ``FreeCAD`` and ``MeshPart``.
The source FCStd files are read-only; the JSON meshes are derived assets for
the standalone C2 page.
"""
import hashlib
import json
from pathlib import Path

import FreeCAD as App
import MeshPart


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "results" / "C2"
OUT = ROOT / "docs" / "assets" / "c2"
STATES = ("neutral", "max-input", "parked", "folded")


def export_state(state):
    source = SOURCE / f"rotor-{state}.FCStd"
    if not source.is_file():
        raise FileNotFoundError(source)
    doc = App.openDocument(str(source))
    try:
        parts = []
        for obj in doc.Objects:
            if obj.TypeId != "PartDesign::Feature" or obj.Name == "AirframeEnvelope":
                continue
            if obj.Shape.isNull() or not obj.Shape.isValid():
                raise ValueError(f"invalid shape: {obj.Name}")
            mesh = MeshPart.meshFromShape(
                Shape=obj.Shape,
                LinearDeflection=3.0,
                AngularDeflection=0.35,
                Relative=False,
            )
            vertices, faces = mesh.Topology
            group = next(
                (item.Name for item in obj.InList if item.TypeId == "App::DocumentObjectGroup"),
                "",
            )
            parts.append(
                {
                    "name": obj.Name,
                    "group": group,
                    "vertices": [
                        round(coord / 1000, 6)
                        for vertex in vertices
                        for coord in (vertex.x, vertex.y, vertex.z)
                    ],
                    "indices": [index for face in faces for index in face],
                    "connection": getattr(obj, "Connection", ""),
                    "material": getattr(obj, "MaterialBasis", ""),
                    "note": getattr(obj, "EngineeringNote", ""),
                }
            )
        if len(parts) != 425:
            raise AssertionError(f"{state}: expected 425 parts, got {len(parts)}")
        payload = {
            "state": state,
            "units": "m",
            "parts": parts,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "tolerance_mm": 3,
        }
        target = OUT / f"{state}.json"
        target.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
        return {
            "parts": len(parts),
            "bytes": target.stat().st_size,
            "sha256": payload["source_sha256"],
        }
    finally:
        App.closeDocument(doc.Name)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    manifest = {state: export_state(state) for state in STATES}
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    for state, result in manifest.items():
        print(state, result)


if __name__ == "__main__":
    main()

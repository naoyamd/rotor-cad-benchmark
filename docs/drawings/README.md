# C1 reference drawing

`rotor_reference_drawing_A3.pdf` is a three-sheet, vector-only A3 reference drawing generated from the saved C1 FreeCAD assemblies. It is explicitly **not a manufacturing drawing**: no tolerances, material allowables, bearing fits, fastener details, load cases, or airworthiness claims are supplied.

## Source and method

- Neutral source: `../../results/C1/rotor-neutral.FCStd`
- Folded source: `../../results/C1/rotor-folded.FCStd`
- FreeCAD 1.1.4 (`4fd3bf320d9566a27e60069fc8387448aaa3a094` in the source file metadata)
- Geometry extraction: `TechDraw.projectToSVG()` in a headless `FreeCADCmd.exe` process, using the `Fixed`, `Swashplate`, `Hub`, and `Blade1`-`Blade5` assembly groups. Root detail uses the Blade 1 root/fold parts and excludes the long airfoil surface.
- PDF composition: ReportLab paths sampled from the TechDraw SVG commands; the PDF contains no raster model images.
- Projection basis: CAD coordinates are +X forward, +Y left, +Z up. TechDraw direction vectors are Top `(0,0,1)`, Front `(0,1,0)`, and Right `(1,0,0)`. View positions are arranged for readability and are labelled by direction; they are not asserted to be an ISO first-angle or third-angle layout.
- Units: millimetres. Displayed scales are reference scales; each view may reduce slightly to fit its panel.

## Measured values

Values below are derived from the loaded shapes and are deliberately separated into nominal design values and geometry envelopes.

| Quantity | Measured value | Meaning |
|---|---:|---|
| Nominal rotor diameter | 18,600.0 mm | Design radius value 9,300 mm, not a hardware envelope |
| Neutral maximum radial envelope | 18,624.8 mm DIA | Maximum vertex radius across the selected C1 assembly |
| Neutral XY bounding envelope | 17,106.4 x 17,887.7 mm | Axis-aligned CAD bounding box |
| Neutral Z envelope | 2,530.0 mm | -90.0 to +2,440.0 mm |
| Folded Y projection | 3,503.9 mm | Fixed-axis storage-width evidence from folded CAD |
| Folded XYZ bounding envelope | 18,313.5 x 3,503.9 x 2,765.0 mm | Axis-aligned folded CAD bounding box |
| Rotor ring envelope | DIA 1,480 mm | `RotorRing` bounding-box diameter |
| Hub cap envelope | DIA 800 mm | `HubCap` bounding-box diameter |
| Mast envelope | DIA 320 mm | `Mast` bounding-box diameter |
| Blade 1 root spar envelope | 270 x 177 x 131 mm | `Blade1RootSparEnvelope` bounding box in neutral state |

The full extraction record, including source SHA-256 hashes, is kept in `work/drawings/geometry.json`; raw TechDraw SVGs are in `work/drawings/svg/`. The public, repo-relative provenance summary is `provenance.json` beside this README.

## Regenerate

From the project repository root (`outputs/rotor-cad-benchmark`), with the bundled Python and FreeCADCmd available:

```text
python docs/drawings/generate_reference_drawing.py ^
  --cad-neutral results/C1/rotor-neutral.FCStd ^
  --cad-folded results/C1/rotor-folded.FCStd ^
  --output docs/drawings/rotor_reference_drawing_A3.pdf ^
  --work-dir ../../work/drawings ^
  --freecadcmd C:\path\to\FreeCADCmd.exe
```

The generator writes only the requested PDF and intermediate extraction files; it does not save or modify either source FCStd document.

## Verification record

- `pdfinfo`: 3 pages, A3 landscape (1190.55 x 841.89 pt), PDF 1.4.
- `pypdf`: all three pages have zero image XObjects; the model linework is vector PDF content.
- `pdfplumber`: all sheet titles, page numbers, projection labels, status notes, and measured values extract successfully.
- `pdftoppm -png -r 150`: all three pages rendered to `previews/page-1.png`, `previews/page-2.png`, and `previews/page-3.png` and were visually inspected. No clipping, missing panels, or unreadable geometry was found at the rendered overview.
- Rendering emitted Poppler font warnings for unused `Symbol`/`ArialUnicode` display fonts; the generated PDF itself uses only ReportLab Helvetica fonts and renders correctly.


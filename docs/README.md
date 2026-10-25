# ROTOR / C1 + C2 benchmark site

GitHub Pages: https://rotor.nightly.dedyn.io/

`index.html` is the side-by-side comparison landing. `c1.html` preserves the
original C1 explorer; `c2.html` presents the redesigned C2. Shared root URLs
with `#explore`, `#benchmark` or `#evidence` redirect to C1 to preserve old links.
C2 is a redesign informed by C1, not an independent same-condition trial.
C2 reported time (1h07m03s), models and Standard token-only estimate ($24.42)
are transcribed from the redesign chat; see `metrics/C2/README.md` for scope.
Unlike C1, no request-level public token breakdown is included.

Buildless Three.js 0.180.0 website. All runtime dependencies are vendored under
`vendor/` with the upstream MIT license; there is no CDN, analytics, or backend.
GitHub Pages serves `main:/docs`. `.nojekyll` preserves the static files directly.

## Local verification

```sh
node --test docs/tests/*.test.mjs
python -m http.server 18795 --bind 127.0.0.1 --directory docs
```

Open http://127.0.0.1:18795/ in a WebGL-enabled browser. Keyboard-accessible
controls offer camera presets, saved states, component isolation, and mechanism
inputs. Rotation starts only on request; reduced-motion preferences disable
smooth scrolling. Network/WebGL errors leave the evidence and source links usable.

The C1 initial view is the live mechanism with X-ray enabled. X-ray makes outer
surfaces translucent and draws their edges while retaining opaque pitch links.
The same toggle applies to the saved CAD states. Pitch presets combine collective
and cyclic amplitude: 8/0, 12/1.5, and 16/3 degrees. They stay within the original
solver envelope; no pitch exaggeration is applied. The small section indicator
shows blade 1's solved angle. Sliders remain available for individual adjustments.

Palette source: [training-report-blue theme tokens](https://github.com/naoyamd/codex-training-report-deck/blob/main/skills/create-training-report-deck/assets/theme-tokens.json).
Blue `#004EA2`, navy `#002060`, cyan `#38BEE2`, and the reference neutral colors
are used without corporate logos or proprietary footer assets. Japanese UI copy
was revised with the `japanese-humanizer` skill to use direct descriptive labels.

## Provenance and limits

- C2's opaque view combines BOM materials with illustrative finishes: blue-gray
  and olive painted structures, warm metallic fasteners, silver sliding parts,
  dark composite/elastomer, and restrained red lock-head accents. Coatings are
  visualization choices, not verified aircraft finishes. X-ray retains its
  diagnostic colors. [MRO surface-treatment reference](https://www.kongsberg.com/what-we-do/defence-and-security/aviation-maintenance-repair-overhaul/helicopter-dynamic-components/)
  documents painting and protective treatments; it does not specify C2's colors.

- C2's motion tab now uses continuous Three.js animation, not the 12 PNG
  snapshots. `c2-kinematics.mjs` ports C2's constrained swash yaw, planar RR-S
  and finite pitch-link solver. `c2-rig.mjs` maps all 425 neutral CAD meshes to
  the solved frames, without rewriting source vertices. All five blades use
  the same mast phase. The rotating hub/mast frame also carries the flange.
- C2 supports collective 8–16°, cyclic amplitude 0–3°, arbitrary phase and
  azimuth, and display speeds 3/12/30 rpm. Flap/lag are zero and folding is
  shown through the original saved-state tab, not a simulated transition.
- The two scissor RingBracket meshes retain their neutral rigid lengths.
  Their endpoint discrepancy reaches 1.2763 mm on the documented test grid
  (1.3 mm rounded bound for those samples). This is a display approximation,
  not a tolerance, clearance or engineering acceptance claim. No BREP rebuild,
  contact, elastic deformation, or structural calculation runs in the browser.
- C2 tests compare the JS solver to original Python fixtures, verify all mesh
  bindings, pin and link endpoints, rigid 500/350 mm rods, and the 360° wrap.
  The historical 12 CAD-generated PNGs remain in the published evidence but
  are not used for live animation. Original CAD/verification files are unchanged.

- The benchmark section embeds the recorded C1 specification, model/effort,
  timing and token-only cost estimates from `EXECUTION_REPORT.md` and
  `metrics/C1/summary.json`. These are historical estimates dated 2026-10-25,
  not current pricing or actual billing. Website and additional drawing work
  are outside the measured scope. A regression test checks displayed table
  values against the source JSON and model counts against request records.

- [2D reference drawing PDF](drawings/rotor_reference_drawing_A3.pdf): three
  A3 sheets projected from the actual FreeCAD assemblies (neutral assembly,
  rotor-head/root detail, and folded state). See [drawing notes](drawings/README.md)
  for regeneration and dimensions. This is not a toleranced manufacturing drawing.

- Exact-CAD mode shows tessellated geometry of four saved FCStd files, with 145
  functional parts per state. The airframe reference envelope is omitted.
- Run `export_meshes.py` using FreeCAD Python to regenerate `assets/*.json`.
  Linear deflection is 3 mm and angular deflection 0.35 rad. Coordinates are m.
  SHA-256 in each asset and `manifest.json` binds it to its source FCStd.
- Live mode uses `kinematics.mjs`, a direct finite-link port of
  `results/C1/src/mechanism.py`. Input/output lengths remain mm in the solver.
  Its simplified display geometry is not CAD verification geometry. The blade
  envelope is untwisted; flap and lag are fixed at zero in the UI.
- Live display speed presets are 3, 12, and 30 rpm, versus the candidate design value of 210 rpm.
  The mechanism does not compute aerodynamic response or aircraft attitude.
- Saved states do not establish safe intermediate folding motion. Original gate
  decisions remain G0 complete; G1 provisional/not passed; G2 and G3 unmet.

Tests check the finite-link port against source evidence, input boundaries,
all exported mesh indices, source hashes, and local HTML links.

Implementation references: [Three.js installation](https://threejs.org/manual/pages/installation.html),
[OrbitControls](https://threejs.org/docs/pages/OrbitControls.html),
[GitHub Pages source configuration](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site).

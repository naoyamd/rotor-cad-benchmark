# ROTOR / C1 public explorer

GitHub Pages: https://naoyamd.github.io/rotor-cad-benchmark/

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

## Provenance and limits

- Exact-CAD mode shows tessellated geometry of four saved FCStd files, with 145
  functional parts per state. The airframe reference envelope is omitted.
- Run `export_meshes.py` using FreeCAD Python to regenerate `assets/*.json`.
  Linear deflection is 3 mm and angular deflection 0.35 rad. Coordinates are m.
  SHA-256 in each asset and `manifest.json` binds it to its source FCStd.
- Live mode uses `kinematics.mjs`, a direct finite-link port of
  `results/C1/src/mechanism.py`. Input/output lengths remain mm in the solver.
  Its simplified display geometry is not CAD verification geometry. The blade
  envelope is untwisted; flap and lag are fixed at zero in the UI.
- Live display speed is 3 rpm, versus the candidate design value of 210 rpm.
  The mechanism does not compute aerodynamic response or aircraft attitude.
- Saved states do not establish safe intermediate folding motion. Original gate
  decisions remain G0 complete; G1 provisional/not passed; G2 and G3 unmet.

Tests check the finite-link port against source evidence, input boundaries,
all exported mesh indices, source hashes, and local HTML links.

Implementation references: [Three.js installation](https://threejs.org/manual/pages/installation.html),
[OrbitControls](https://threejs.org/docs/pages/OrbitControls.html),
[GitHub Pages source configuration](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site).

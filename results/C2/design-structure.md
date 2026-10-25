# Rotor design v2 — initial structural sizing screen

**Status: G1 structural screen only.** This document is a demand, geometry,
and sensitivity assessment for the v2 concept. It is not a material allowable,
bearing rating, fatigue substantiation, flight-safety approval, or certification
report. The accompanying src/structure.py is the source of the numerical
values and writes verification/structure.json and
verification/structure-load-cases.csv when run with the v2 mechanism.py.

## 1. Frozen design inputs

The rotor remains 18.6 m diameter, five blades, 180 kg equivalent blade mass,
and 210 rpm. The structural screen uses the following v2 candidate dimensions:

| Item | Candidate | Treatment |
|---|---:|---|
| Rotor radius / blades / speed | 9,300 mm / 5 / 210 rpm | Frozen design input |
| Fold hinge and lock pin | actual Ø109.8 mm (r=55−0.1), Ø111 mm bore | Current geometry screen; fatigue/contact still open |
| Fold clevis / tang | t40 mm / tang load span160 mm; widths300/220 mm | 35F bending screen; tear-out and local details unresolved |
| Cuff / stop collar | radius65 mm, solid Ø130 mm; collar radius85 mm | Net section screen; local concentration/fatigue unresolved |
| Pitch link | Ø40 mm, 1,052 mm centre length | Axial and Euler screens only |
| Pitch pin / clevis ears | actual Ø31.8 mm, bore Ø32 / 20 mm ears | Double-shear, bearing, and eccentric bending screens |
| Pitch horn | design arm250 mm; solid cross-section radius40 mm | Arm sets link force; section gives arithmetic screen |
| Mast | OD 320 mm / ID 230 mm | Nominal axial/torsion screen; boundary and fatigue unresolved |
| Swash race candidate | fixed inner OD651.9 / rotating outer ID652.1 mm; Ø30 mm balls (r15), pitch radius652 mm, 72 balls | Matches v2 CAD; contact capacity unresolved |
| Existing layered support | radii 145–174 mm; 5 × (4.5 mm rubber + 1.3 mm steel) | Red-flag geometry screen |

The CAD convention is millimetres; the script converts to metres at force,
inertia, and stiffness boundaries and contains assertions for the frozen values.
The mechanism.loads() result is used for nominal thrust, torque, blade CG, and
inertia when those values are available. The old C1 model is therefore a load
input, not a proof that the v2 hardware is adequate.

## 2. Material treatment

17-4 PH / Custom 630 H1150 is the candidate steel. The Carpenter datasheet
reference is [Custom 630 (17-4 PH)](https://www.carpentertechnology.com/hubfs/7407324/Material%20Saftey%20Data%20Sheets/Custom%20630%20(17-4%20PH).pdf).
The 869 MPa value used in the source is representative/typical and is **not**
treated as a minimum. For a transparent arithmetic screen only, the script
uses 400 MPa von-Mises allowable and 400/sqrt(3) = 230.9 MPa shear. This is
46% of the typical reference and is still not an approved design allowable:
product form, orientation, heat treatment, temperature, notch, fatigue,
corrosion, and manufacturing quality remain open.

## 3. Load cases and what is deliberately unknown

The script writes five rows:

1. **Stopped/parked:** rotor centrifugal load is zero; a 1.5× blade-weight
   screen and the 8.56 kN·m nominal gravity moment about the flap hinge are
   reported. Parking is zero absolute pitch, or −12° from the spring reference; wind, brake torque, and stop
   transient remain open.
2. **Startup ramp:** a linear 0-to-210 rpm ramp in 30 s with +20% blade mass
   reports inertial torque per blade and total rotor inertial torque. This is an
   explicit ramp assumption, not a measured drivetrain torque history.
3. **Hover nominal:** sea-level hover reference from mechanism.loads(), with
   12° stress-free pitch plus 3° cyclic. No forward-flight loads are included.
4. **Extreme controls/overspeed:** 231 rpm, 216 kg, 16° collective plus 3°
   cyclic, and 6° flap/3° lag support inputs. Aerodynamic control loads and
   phase remain unresolved.
5. **Ground gust:** stopped 10 m/s gust at the ground; dynamic pressure is
   reported, but drag area and parked restraint response are unresolved.

The explicit 1.5 factor is only a screening assumption. The maximum specified
uncertainty case gives a retained centrifugal demand of roughly 1.03 MN per
blade when the old C1 CG and inertia model are used. That figure is a demand,
not a permitted load or a rating. The CSV is the authoritative computable case
table after running the script. The operating envelope is sea-level hover/low
speed only, with a 0–40 °C temperature range for this initial screen.

## 4. Immediate structural findings

### Fold pin, clevis, tang, and cuff

For the 1.03 MN envelope the actual Ø109.8 mm pin (build radius 55−0.1 mm)
gives approximately 54 MPa double-shear and 117 MPa bearing on two 40 mm
clevis ears. Using the actual frozen clevis/tang distribution screen, the
effective bending arm is `35F` mm (`110/2 − 160/8`), giving about 277 MPa
bending and 293 MPa combined von Mises. This is a **screening pass only**
against the explicit 400 MPa value, with no credit for hinge/lock load sharing
and no fatigue, tear-out, local bearing, or manufacturing evidence. The Ø111 mm
bore, tang width220 mm/length460 mm, hinge x=0, and lock x=−200 mm are now the
CAD geometry to carry into the next review.

The solid Ø130 mm cuff is now included rather than treated as an unknown net
section. At 65 mm radius and the same demand, its axial stress is about 78 MPa,
bending stress about 311 MPa, and simple extreme-fibre sum about 389 MPa. This
is below the 400 MPa arithmetic screen but leaves little room for the horn
junction, fillets, stop-collar transition, bearing seats, stress concentration,
or fatigue. Increasing cuff radius to 70 mm reduces the same simple screen to
about 336 MPa; retain 65 mm only if the local transition analysis supports it.

The fixed stop cradle has lower/upper faces at z=−115/+146 mm, nominal −5°
drop, and a 30 mm parked shoe that is intended to hold zero. The stopped
gravity moment is 8.56 kN·m per blade about the flap
hinge; with the 350 mm stop-load arm this is 24.46 kN per blade, or 36.69 kN
with the 1.5 screening factor. Stop-face contact and cradle/tang load
distribution remain unresolved.

The mast is screened at OD 320 mm / ID 230 mm using the nominal thrust and
torque from mechanism.loads(), with the same explicit 1.5 arithmetic factor.
The resulting nominal axial/torsion von-Mises value is recorded in
mast_screen in structure.json, but it is not a mast strength result: the
airframe boundary moments, local discontinuities, bearing-seat effects,
stability, fatigue, and manufacturing details are not in the model.

### Pitch link, Ø31.8 pin, and radius40 horn

The link is Ø40 mm at a centre length of approximately 1,052 mm. The design
horn **arm** is 250 mm; the 40 mm value is the solid horn cross-section radius,
not the pitch moment arm. With Kpitch = 12 kN·m/rad, stress-free pitch is 12°:
the extreme control envelope uses about 7–8° relative pitch, with 6° flap and
3° lag support inputs, while parking uses a −12° spring deflection at zero absolute pitch.

The conservative combined-support screen gives about 7.7 kN·m and 31 kN at the
horn, while pitch-only force is about 8.8 kN. The Ø40 rod Euler load is about
224 kN, and the Ø32 pin with 48 mm ear/ball geometry gives an 18 mm bending arm;
the actual Ø31.8 pin combined screen is about 179.5 MPa. The solid Ø80 horn section screen is
about 203 MPa combined. Both are **screening passes under the equivalent
stiffness and 400 MPa assumptions**, not strength approvals. Actual
aerodynamic hinge moment and exact flap/lag load sharing remain open.

**Recommendation to the parent design:** retain the 250 mm arm / 40 mm solid
horn distinction in CAD metadata, keep the pin load line at the 48 mm ear/ball
geometry, and verify the 15–30 kN local reaction range with actual blade
moments before release. Do not size the Ø32 pin from a false 40 mm moment arm.

### Swash bearing candidate

The 72 CAD balls (15 mm radius, Ø30 mm) sit at a 652 mm pitch radius with
56.9 mm centre spacing. The fixed inner race outer radius is 651.9 mm and the
rotating outer race inner radius is 652.1 mm; the groove minor radius is 15.1
mm, with the race stack spanning z=−80…−10 mm and balls at z=−50 mm. This
control bearing is screened against the five pitch-link reactions, not rotor
thrust: the worst parked zero-pitch case (−12° spring deflection) sums about 75 kN of absolute link force. Signed forces and actual joint positions come from the finite-link model in control_checks.py; the magnitude sum is used only as a demand bound.
Using axial link reactions, `sum(abs(F)) × radius` is a conservative **horizontal
overturning-moment** bound of about 49 kN·m, not torsional torque about Z. The
five-point phasor overturning result and a separate `torsional_torque_z`=
**not inferred** status are reported; no Z torque is inferred from axial force. Equal-share
arithmetic is about 1.05 kN per ball and is not a rating. Race contact,
preload, internal clearance, load distribution, speed, friction, and L10 life
remain unverified, so the custom race remains **UNRESOLVED**.

### Layered elastomer support — geometry red flag

The radial window is only 30 mm. Five 4.5 mm rubber layers plus five 1.3 mm
steel layers occupy 29 mm, leaving roughly 1 mm radial reserve before any
manufacturing tolerance or deformation. At the mean radius of 160 mm, 6° of
angular motion creates roughly 16.8 mm tangential displacement. Because the
five bonded rubber layers act in series for this global rotation, the stack
geometric shear strain is about 0.745 (74.5%) against 22.5 mm total rubber,
before edge effects. This is above the transparent 50% screen; actual compound
allowable is unknown. The 19° absolute pitch is not substituted for the
stress-free 12° reference in this support calculation.

For a 50% geometric shear-strain screen, 6° requires about 33.5 mm of rubber
thickness; with the existing 6.5 mm steel stack the radial package would be
about 40 mm, not 30 mm. Conversely, one 4.5 mm layer permits only about 0.8°
at 50% shear strain at this radius. The dimensions should therefore be changed
promptly—reduce angular travel, increase active rubber thickness to roughly
33.5 mm and the full radial package to about 40 mm, change the support
architecture, or explicitly validate a different strain regime. Compression
strain cannot be obtained from force alone without modulus, bonded shape factor,
preload, and allowable data; the JSON reports geometry ratios and the roughly
approximately 25.5 MPa maximum projected compression-stress screen, using the smallest loaded spherical interface projected area pi*(145²−90²) mm², rather than the radial layer cross-section. No elastomer capacity is
promoted to pass.

## 5. Support stiffness and lag damping sensitivity

The old equivalent diagonal stiffness assumptions (Kpitch=12, Kflap=25,
Klag=80 kN·m/rad) are retained only as a screening model. The script reports
modal frequency for each support and varies Klag by 0.5×/1×/1.5×, blade mass
by 1×/1.2×, and damper effectiveness by 1×/0.5×. The reduced-lag-damping row
is intentionally visible: a 50% damper reduction is not silently treated as a
pass. Cross-coupling, support nonlinearities, blade flexibility, hub/mast
flexibility, and full rotor-body stability are outside this script.

## 6. R13–R17 trace

| Requirement | v2 screen status | Remaining gate |
|---|---|---|
| R13 load paths and reactions | Screened, not verified | Five-point pitch-link reactions are computed; balance six-component joint reactions, mast boundary moments, and local load transfer |
| R14 operating/load cases | Screened, not verified | Freeze acceleration, wind/gust restraint, stop transient, simultaneous aero/control loads, and envelope |
| R15 material/strength/capacity | Not pass | Establish product-form allowables, bearing/elastomer ratings, pin bending, fatigue, and lock capacity |
| R16 elastic support | Red flag + unresolved | Redesign or validate layer geometry, modulus, shape factor, temperature, strain, and life |
| R17 risk plan | Pass — risk plan only | Full aircraft dynamics, fatigue, aeroelastic, coupled modal analysis, and test evidence are future work, not claimed complete |

## 7. Computable evidence and gate status

Run from FreeCAD's Python or a normal Python environment with v2 mechanism.py:

~~~text
python outputs/rotor-design-v2/src/structure.py
~~~

The script creates:

- verification/structure.json — inputs, load cases, demand screens,
  sensitivities, statuses, and limitations;
- verification/structure-load-cases.csv — one row per stopped, startup,
  hover, extreme-control, and ground-gust case.

Arithmetic/unit assertions, the actual Ø109.8/Ø111 fold-pin screen, the solid
Ø130 cuff arithmetic (plus the Ø140 radius-70 sensitivity), the corrected
pitch-pin screen, and the solid horn section screen are **screening-only**
results under explicit assumptions. The custom swash bearing capacity,
elastomer capacity,
aerodynamic hinge moment, fatigue, startup drivetrain torque, and full dynamic
stability remain **UNRESOLVED**. No strength or flight claim should be made
until those gaps are closed with frozen load paths, product-form allowables,
bearing/elastomer data, and analysis/test.

Runtime assertions also check the actual pin diameter/radius relation, the
35F pin-bending arm, the 8.56 kN·m / 350 mm stop-load relationship, the
pitch-arm versus horn-section distinction, and positive race/ball geometry.


"""Initial structural sizing screen for rotor-design-v2.

This is a demand and geometry screen, not a certification stress report. It
uses the v2 mechanism module for the nominal rotor loads, then writes
machine-readable results under verification/ when run as a script.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Mapping

try:  # FreeCAD runs this file with src on sys.path.
    from mechanism import P, loads
except ImportError:  # Also allow package-style test imports.
    from .mechanism import P, loads


OUT = Path(__file__).resolve().parents[1]
G = 9.80665
E_STEEL_MPA = 200_000.0
LOAD_FACTOR = 1.5  # Explicit screening assumption; not an approved factor.

# Frozen v2 geometry. Values are kept in mm here because that is the CAD
# convention; calculations convert at the boundary and assert the units.
RADIUS_MM = 9_300.0
BLADE_COUNT = 5
BLADE_MASS_KG = 180.0
RPM = 210.0
CUFF_RADIUS_MM = 65.0
FOLD_PIN_RADIUS_MM = 55.0 - 0.1
FOLD_PIN_D_MM = 2.0 * FOLD_PIN_RADIUS_MM
FOLD_RECOMMENDED_PIN_D_MM = 110.0
FOLD_RECOMMENDED_BORE_MM = 111.0
FOLD_CLEVIS_T_MM = 40.0
FOLD_CLEVIS_WIDTH_MM = 300.0
FOLD_TANG_WIDTH_MM = 220.0
FOLD_TANG_LENGTH_MM = 460.0
FOLD_TANG_LOAD_SPAN_MM = 160.0
FOLD_EAR_CENTER_SPAN_MM = 110.0
FOLD_HINGE_X_MM = 0.0
FOLD_LOCK_X_MM = -200.0
STOP_COLLAR_RADIUS_MM = 85.0
STOP_COLLAR_X_MIN_MM = 335.0
STOP_COLLAR_X_MAX_MM = 365.0
STOP_LOWER_Z_MM = -115.0
STOP_UPPER_Z_MM = 146.0
PARK_SHOE_WIDTH_MM = 30.0
PARK_SHOE_HOLDS_ZERO = True
STOP_NOMINAL_DROP_DEG = -5.0
STOP_LOAD_ARM_MM = 350.0
PITCH_LINK_D_MM = 40.0
PITCH_LINK_LENGTH_MM = float(P['link_length'])
PITCH_PIN_D_MM = 31.8
PITCH_EAR_T_MM = 20.0
PITCH_PIN_EAR_CENTER_SPAN_MM = 48.0
PITCH_PIN_BALL_LOAD_WIDTH_MM = 48.0
PITCH_HORN_ARM_MM = 250.0
PITCH_HORN_CROSS_RADIUS_MM = 40.0
MAST_OD_MM = 320.0
MAST_ID_MM = 230.0

# Custom swash bearing candidate. The v2 CAD uses Part.makeSphere(15), so
# "ball15" is recorded as a 15 mm radius / 30 mm diameter. No capacity is
# inferred from these values.
SWASH_BALL_R_MM = 15.0
SWASH_BALL_D_MM = 2.0 * SWASH_BALL_R_MM
SWASH_PITCH_RADIUS_MM = 652.0
SWASH_BALL_COUNT = 72
SWASH_FIXED_INNER_OUTER_R_MM = 651.9
SWASH_ROTATING_OUTER_INNER_R_MM = 652.1
SWASH_GROOVE_MINOR_RADIUS_MM = 15.1
SWASH_AXIAL_Z_MIN_MM = -80.0
SWASH_AXIAL_Z_MAX_MM = -10.0
SWASH_BALL_Z_MM = -50.0

# Existing concentric layered support candidate.
BEARING_RI_MM = 145.0
BEARING_RO_MM = 174.0
RUBBER_LAYERS = 5
RUBBER_T_MM = 4.5
STEEL_LAYER_T_MM = 1.3

# Candidate material reference. 869 MPa is a representative value supplied
# from the datasheet, not a minimum. 400 MPa is an explicit screen only.
STEEL_SOURCE = (
    "https://www.carpentertechnology.com/hubfs/7407324/Material%20Saftey%20"
    "Data%20Sheets/Custom%20630%20(17-4%20PH).pdf"
)
STEEL_TYPICAL_YIELD_MPA = 869.0
STEEL_SCREEN_ALLOWABLE_MPA = 400.0
STEEL_SCREEN_SHEAR_MPA = STEEL_SCREEN_ALLOWABLE_MPA / math.sqrt(3.0)


def _pvalue(p: Mapping[str, Any], *names: str, default: float) -> float:
    """Get the first numeric mechanism parameter, retaining old key names."""

    for name in names:
        if name in p and p[name] is not None:
            return float(p[name])
    return float(default)


def _mechanism_loads(p: Mapping[str, Any]) -> Dict[str, Any]:
    """Read the v2 screening loads without hiding model errors."""

    return dict(loads(p))


def _mm(value: float) -> float:
    assert value > 0.0, f"dimension must be positive: {value} mm"
    return value / 1_000.0


def _circle_area_mm2(d_mm: float) -> float:
    return math.pi * d_mm * d_mm / 4.0


def _vm(sigma_mpa: float, tau_mpa: float) -> float:
    return math.sqrt(sigma_mpa * sigma_mpa + 3.0 * tau_mpa * tau_mpa)


def _json_number(value: Any) -> Any:
    """Convert common scalar types while refusing non-finite evidence."""

    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        number = float(value)
        if not math.isfinite(number):
            return None
        return number
    if isinstance(value, Mapping):
        return {str(k): _json_number(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_number(v) for v in value]
    # numpy scalar support without making numpy a direct dependency here.
    if hasattr(value, "item"):
        return _json_number(value.item())
    return value


def _assert_frozen_geometry(p: Mapping[str, Any]) -> None:
    """Catch mm/m mixups and accidental use of the old 34 mm link."""

    dimensions = {
        "radius": RADIUS_MM,
        "blade_mass": BLADE_MASS_KG,
        "rpm": RPM,
        "cuff_radius": CUFF_RADIUS_MM,
    }
    assert dimensions["radius"] / 1_000.0 == 9.3
    assert dimensions["blade_mass"] == 180.0
    assert dimensions["rpm"] == 210.0
    assert dimensions["cuff_radius"] / 1_000.0 == 0.065
    assert FOLD_PIN_RADIUS_MM == 54.9 and FOLD_PIN_D_MM == 2.0 * FOLD_PIN_RADIUS_MM
    assert FOLD_PIN_D_MM == 109.8
    assert FOLD_RECOMMENDED_PIN_D_MM == 110.0
    assert FOLD_RECOMMENDED_BORE_MM == 111.0
    assert FOLD_CLEVIS_WIDTH_MM == 300.0 and FOLD_TANG_WIDTH_MM == 220.0
    assert FOLD_TANG_LENGTH_MM == 460.0
    assert FOLD_TANG_LOAD_SPAN_MM == 160.0 and FOLD_EAR_CENTER_SPAN_MM == 110.0
    assert FOLD_HINGE_X_MM == 0.0 and FOLD_LOCK_X_MM == -200.0
    assert STOP_COLLAR_RADIUS_MM == 85.0
    assert STOP_COLLAR_X_MIN_MM == 335.0 and STOP_COLLAR_X_MAX_MM == 365.0
    assert STOP_LOWER_Z_MM == -115.0 and STOP_UPPER_Z_MM == 146.0
    assert PARK_SHOE_WIDTH_MM == 30.0 and PARK_SHOE_HOLDS_ZERO
    assert STOP_NOMINAL_DROP_DEG == -5.0 and STOP_LOAD_ARM_MM == 350.0
    assert PITCH_LINK_D_MM == 40.0 and abs(PITCH_LINK_LENGTH_MM-1052.0)<0.1
    assert PITCH_PIN_D_MM == 31.8 and PITCH_EAR_T_MM == 20.0
    assert PITCH_PIN_EAR_CENTER_SPAN_MM == 48.0
    assert PITCH_PIN_BALL_LOAD_WIDTH_MM == 48.0
    assert PITCH_HORN_ARM_MM == 250.0 and PITCH_HORN_CROSS_RADIUS_MM == 40.0
    assert MAST_OD_MM == 320.0 and MAST_ID_MM == 230.0
    assert SWASH_BALL_R_MM == 15.0 and SWASH_BALL_D_MM == 2.0 * SWASH_BALL_R_MM
    assert SWASH_BALL_D_MM == 30.0
    assert SWASH_PITCH_RADIUS_MM == 652.0
    assert SWASH_BALL_COUNT == 72
    assert SWASH_FIXED_INNER_OUTER_R_MM == 651.9
    assert SWASH_ROTATING_OUTER_INNER_R_MM == 652.1
    assert SWASH_GROOVE_MINOR_RADIUS_MM == 15.1
    assert (SWASH_AXIAL_Z_MIN_MM, SWASH_AXIAL_Z_MAX_MM, SWASH_BALL_Z_MM) == (-80.0, -10.0, -50.0)
    assert RUBBER_LAYERS == 5 and RUBBER_T_MM == 4.5 and STEEL_LAYER_T_MM == 1.3
    # If the mechanism exposes these fields, fail loudly on an accidental unit
    # or revision mismatch. Missing fields remain valid for early v2 imports.
    for key, expected in (
        ("radius", RADIUS_MM),
        ("blade_mass", BLADE_MASS_KG),
        ("rpm", RPM),
    ):
        if key in p:
            assert math.isclose(float(p[key]), expected, rel_tol=0.0, abs_tol=1e-9), (
                f"mechanism.{key}={p[key]!r} does not match v2 {expected}"
            )


def _base_inputs(p: Mapping[str, Any], mech: Mapping[str, Any]) -> Dict[str, float]:
    radius_mm = _pvalue(p, "radius", default=RADIUS_MM)
    root_mm = _pvalue(p, "root", default=1_600.0)
    joint_mm = _pvalue(p, "joint", default=600.0)
    blade_mass_kg = _pvalue(p, "blade_mass", default=BLADE_MASS_KG)
    rpm = _pvalue(p, "rpm", default=RPM)
    aircraft_mass_kg = _pvalue(p, "aircraft_mass", default=15_600.0)
    cg_m = float(mech.get("blade_cg_m", (radius_mm + root_mm) / 2_000.0))
    inertia = float(
        mech.get(
            "blade_inertia_hinge_kgm2",
            blade_mass_kg
            * (
                ((radius_mm - root_mm) / 1_000.0) ** 2 / 12.0
                + ((radius_mm + root_mm) / 2_000.0 - joint_mm / 1_000.0) ** 2
            ),
        )
    )
    omega = rpm * 2.0 * math.pi / 60.0
    thrust = mech.get("thrust_N")
    if thrust is None:
        thrust = 1.05 * aircraft_mass_kg * G
    power = mech.get("power_W")
    torque = mech.get("torque_Nm")
    if torque is None and power is not None and omega > 0.0:
        torque = float(power) / omega
    return {
        "radius_m": radius_mm / 1_000.0,
        "root_m": root_mm / 1_000.0,
        "flap_hinge_m": joint_mm / 1_000.0,
        "blade_mass_kg": blade_mass_kg,
        "rpm": rpm,
        "aircraft_mass_kg": aircraft_mass_kg,
        "blade_cg_m": cg_m,
        "blade_inertia_kgm2": inertia,
        "omega_rad_s": omega,
        "thrust_N": float(thrust),
        "torque_Nm": None if torque is None else float(torque),
    }


def _centrifugal(mass_kg: float, rpm: float, cg_m: float) -> float:
    omega = rpm * 2.0 * math.pi / 60.0
    return mass_kg * omega * omega * cg_m


def _mast_screen(base: Mapping[str, float]) -> Dict[str, Any]:
    """Report the old nominal mast section screen without treating it as a pass."""

    od_m = _mm(MAST_OD_MM)
    id_m = _mm(MAST_ID_MM)
    area_m2 = math.pi * (od_m**2 - id_m**2) / 4.0
    j_m4 = math.pi * (od_m**4 - id_m**4) / 32.0
    thrust = base["thrust_N"]
    torque = base["torque_Nm"]
    sigma_MPa = LOAD_FACTOR * thrust / area_m2 / 1_000_000.0
    tau_MPa = (
        None
        if torque is None
        else LOAD_FACTOR * torque * od_m / (2.0 * j_m4) / 1_000_000.0
    )
    vm_MPa = None if tau_MPa is None else _vm(sigma_MPa, tau_MPa)
    return {
        "outer_diameter_mm": MAST_OD_MM,
        "inner_diameter_mm": MAST_ID_MM,
        "area_mm2": area_m2 * 1_000_000.0,
        "polar_moment_mm4": j_m4 * 1_000_000_000_000.0,
        "nominal_thrust_axial_stress_MPa": sigma_MPa,
        "nominal_torque_shear_MPa": tau_MPa,
        "nominal_combined_vm_MPa": vm_MPa,
        "capacity_status": "UNRESOLVED_BOUNDARY_AND_FATIGUE",
        "basis": "Same nominal thrust/torque screen as mechanism.loads(), with the v2 OD/ID; mast boundary moments and fatigue are not supplied.",
    }


def _support_stiffness(p: Mapping[str, Any]) -> Dict[str, float]:
    """Read the existing equivalent support assumptions with SI normalization."""

    def kvalue(*names: str, default: float) -> float:
        value = _pvalue(p, *names, default=default)
        # v2 uses Nm/rad; accept a legacy kN m/rad field explicitly.
        if names == ("Kpitch", "k_pitch_Nm_rad") and "k_pitch_kNm_rad" in p:
            value = float(p["k_pitch_kNm_rad"]) * 1_000.0
        return value

    return {
        "pitch_Nm_rad": kvalue("Kpitch", "k_pitch_Nm_rad", default=12_000.0),
        "flap_Nm_rad": kvalue("Kflap", "k_flap_Nm_rad", default=25_000.0),
        "lag_Nm_rad": kvalue("Klag", "k_lag_Nm_rad", default=80_000.0),
    }


def _pitch_demand(
    relative_pitch_deg: float,
    flap_deg: float,
    lag_deg: float,
    stiffness: Mapping[str, float],
) -> Dict[str, float]:
    """Screen link, pin, and solid horn using the correct 250 mm pitch arm."""

    assert PITCH_HORN_ARM_MM > PITCH_HORN_CROSS_RADIUS_MM
    pitch_moment_Nm = stiffness["pitch_Nm_rad"] * math.radians(abs(relative_pitch_deg))
    flap_moment_Nm = stiffness["flap_Nm_rad"] * math.radians(abs(flap_deg))
    lag_moment_Nm = stiffness["lag_Nm_rad"] * math.radians(abs(lag_deg))
    combined_moment_Nm = math.sqrt(
        pitch_moment_Nm**2 + flap_moment_Nm**2 + lag_moment_Nm**2
    )
    design_pitch_moment_Nm = LOAD_FACTOR * pitch_moment_Nm
    design_combined_moment_Nm = LOAD_FACTOR * combined_moment_Nm
    horn_arm_m = _mm(PITCH_HORN_ARM_MM)
    pitch_force_N = design_pitch_moment_Nm / horn_arm_m if horn_arm_m else 0.0
    combined_force_N = design_combined_moment_Nm / horn_arm_m if horn_arm_m else 0.0
    force_N = max(pitch_force_N, combined_force_N)
    rod_area_mm2 = _circle_area_mm2(PITCH_LINK_D_MM)
    rod_stress_MPa = force_N / rod_area_mm2
    link_I_m4 = math.pi * _mm(PITCH_LINK_D_MM) ** 4 / 64.0
    link_length_m = _mm(PITCH_LINK_LENGTH_MM)
    euler_N = math.pi**2 * E_STEEL_MPA * 1_000_000.0 * link_I_m4 / link_length_m**2

    pin_area_double_mm2 = 2.0 * _circle_area_mm2(PITCH_PIN_D_MM)
    pin_shear_MPa = force_N / pin_area_double_mm2
    pin_bearing_MPa = force_N / (2.0 * PITCH_EAR_T_MM * PITCH_PIN_D_MM)
    pin_ecc_mm = PITCH_PIN_EAR_CENTER_SPAN_MM / 2.0 - PITCH_PIN_BALL_LOAD_WIDTH_MM / 8.0
    pin_bending_MPa = 32.0 * force_N * pin_ecc_mm / (math.pi * PITCH_PIN_D_MM**3)
    pin_vm_MPa = _vm(pin_bending_MPa, pin_shear_MPa)
    pin_d_for_screen_bending_mm = (
        32.0 * force_N * pin_ecc_mm / (math.pi * STEEL_SCREEN_ALLOWABLE_MPA)
    ) ** (1.0 / 3.0) if force_N else 0.0

    horn_d_mm = 2.0 * PITCH_HORN_CROSS_RADIUS_MM
    horn_Z_mm3 = math.pi * horn_d_mm**3 / 32.0
    horn_J_mm4 = math.pi * horn_d_mm**4 / 32.0
    horn_bending_MPa = design_combined_moment_Nm * 1_000.0 / horn_Z_mm3
    horn_torsion_MPa = (
        design_combined_moment_Nm
        * 1_000.0
        * PITCH_HORN_CROSS_RADIUS_MM
        / horn_J_mm4
    )
    horn_vm_MPa = _vm(horn_bending_MPa, horn_torsion_MPa)
    return {
        "relative_pitch_deg": float(relative_pitch_deg),
        "flap_deg": float(flap_deg),
        "lag_deg": float(lag_deg),
        "support_stiffness_pitch_Nm_rad": float(stiffness["pitch_Nm_rad"]),
        "support_stiffness_flap_Nm_rad": float(stiffness["flap_Nm_rad"]),
        "support_stiffness_lag_Nm_rad": float(stiffness["lag_Nm_rad"]),
        "pitch_support_moment_Nm": float(pitch_moment_Nm),
        "flap_support_moment_Nm": float(flap_moment_Nm),
        "lag_support_moment_Nm": float(lag_moment_Nm),
        "combined_support_moment_Nm": float(combined_moment_Nm),
        "design_pitch_moment_Nm": float(design_pitch_moment_Nm),
        "design_combined_moment_Nm": float(design_combined_moment_Nm),
        "pitch_horn_arm_mm": PITCH_HORN_ARM_MM,
        "horn_cross_section_radius_mm": PITCH_HORN_CROSS_RADIUS_MM,
        "pitch_link_force_pitch_only_N": float(pitch_force_N),
        "pitch_link_force_combined_upper_N": float(combined_force_N),
        "design_link_force_N": float(force_N),
        "link_area_mm2": float(rod_area_mm2),
        "link_axial_stress_MPa": float(rod_stress_MPa),
        "link_euler_pinned_N": float(euler_N),
        "link_euler_margin_ratio": float(euler_N / force_N) if force_N else None,
        "pitch_pin_double_shear_MPa": float(pin_shear_MPa),
        "pitch_pin_ear_bearing_MPa": float(pin_bearing_MPa),
        "pitch_pin_screen_eccentricity_mm": float(pin_ecc_mm),
        "pitch_pin_bending_MPa": float(pin_bending_MPa),
        "pitch_pin_combined_vm_MPa": float(pin_vm_MPa),
        "pitch_pin_diameter_for_bending_screen_mm": float(pin_d_for_screen_bending_mm),
        "pitch_horn_section_modulus_mm3": float(horn_Z_mm3),
        "pitch_horn_bending_MPa": float(horn_bending_MPa),
        "pitch_horn_torsion_MPa": float(horn_torsion_MPa),
        "pitch_horn_combined_vm_MPa": float(horn_vm_MPa),
        "pitch_horn_screen_status": "PASS_SCREEN_ONLY"
        if horn_vm_MPa <= STEEL_SCREEN_ALLOWABLE_MPA
        else "RED_FLAG",
        "capacity_status": "UNRESOLVED",
        "basis": "Pitch force uses the 250 mm design horn arm. Flap/lag moments form a conservative upper-bound shared-support reaction; aerodynamic hinge moment and load sharing remain unresolved.",
    }


def _fold_pin_screen(retention_N: float, pin_d_mm: float = FOLD_PIN_D_MM) -> Dict[str, Any]:
    pin_area_double_mm2 = 2.0 * _circle_area_mm2(pin_d_mm)
    bearing_area_mm2 = 2.0 * FOLD_CLEVIS_T_MM * pin_d_mm
    shear_MPa = retention_N / pin_area_double_mm2
    bearing_MPa = retention_N / bearing_area_mm2
    eccentricity_mm = FOLD_EAR_CENTER_SPAN_MM / 2.0 - FOLD_TANG_LOAD_SPAN_MM / 8.0
    bending_MPa = 32.0 * retention_N * eccentricity_mm / (math.pi * pin_d_mm**3)
    combined_vm_MPa = _vm(bending_MPa, shear_MPa)
    return {
        "pin_diameter_mm": pin_d_mm,
        "retention_force_N": retention_N,
        "pin_double_shear_area_mm2": pin_area_double_mm2,
        "pin_double_shear_MPa": shear_MPa,
        "clevis_bearing_area_mm2": bearing_area_mm2,
        "clevis_bearing_MPa": bearing_MPa,
        "effective_bending_eccentricity_mm": eccentricity_mm,
        "pin_bending_MPa": bending_MPa,
        "pin_combined_vm_MPa": combined_vm_MPa,
        "screen_allowable_MPa": STEEL_SCREEN_ALLOWABLE_MPA,
        "screen_status": "PASS_SCREEN_ONLY"
        if max(shear_MPa, bearing_MPa, combined_vm_MPa) <= STEEL_SCREEN_ALLOWABLE_MPA
        else "RED_FLAG",
        "clevis_thickness_mm": FOLD_CLEVIS_T_MM,
        "clevis_width_mm": FOLD_CLEVIS_WIDTH_MM,
        "tang_width_mm": FOLD_TANG_WIDTH_MM,
        "tang_length_mm": FOLD_TANG_LENGTH_MM,
        "tang_load_span_mm": FOLD_TANG_LOAD_SPAN_MM,
        "capacity_status": "UNRESOLVED",
        "basis": "Conservative full centrifugal force on one pin; symmetric hinge/lock sharing is not credited. Bending arm is 35F from the frozen clevis/tang distribution screen.",
    }


def _cuff_screen(retention_N: float, radius_mm: float = CUFF_RADIUS_MM) -> Dict[str, Any]:
    cuff_d_mm = 2.0 * radius_mm
    assert math.isclose(cuff_d_mm, 2.0 * radius_mm, rel_tol=0.0, abs_tol=1e-12)
    area_mm2 = _circle_area_mm2(cuff_d_mm)
    section_modulus_mm3 = math.pi * cuff_d_mm**3 / 32.0
    moment_Nm = retention_N * _mm(radius_mm)
    axial_MPa = retention_N / area_mm2
    bending_MPa = moment_Nm * 1_000.0 / section_modulus_mm3
    extreme_fiber_MPa = axial_MPa + bending_MPa
    return {
        "solid_diameter_mm": cuff_d_mm,
        "radius_mm": radius_mm,
        "net_area_mm2": area_mm2,
        "section_modulus_mm3": section_modulus_mm3,
        "retention_moment_Nm": moment_Nm,
        "axial_stress_MPa": axial_MPa,
        "bending_stress_MPa": bending_MPa,
        "extreme_fiber_stress_MPa": extreme_fiber_MPa,
        "screen_allowable_MPa": STEEL_SCREEN_ALLOWABLE_MPA,
        "screen_status": "PASS_SCREEN_ONLY"
        if extreme_fiber_MPa <= STEEL_SCREEN_ALLOWABLE_MPA
        else "RED_FLAG",
        "capacity_status": "UNRESOLVED_LOCAL_STRESS_AND_FATIGUE",
        "basis": "Solid 130 mm cuff section from the v2 CAD; local fillets, holes, horn junction, stress concentration, and fatigue are not included.",
    }


def _elastomer_screen(max_retention_N: float) -> Dict[str, Any]:
    radial_window_mm = BEARING_RO_MM - BEARING_RI_MM
    rubber_total_mm = RUBBER_LAYERS * RUBBER_T_MM
    steel_total_mm = RUBBER_LAYERS * STEEL_LAYER_T_MM
    stack_mm = rubber_total_mm + steel_total_mm
    mean_radius_mm = (BEARING_RO_MM + BEARING_RI_MM) / 2.0
    projected_area_mm2 = math.pi * (BEARING_RI_MM**2 - 90.0**2)
    compression_stress_MPa = max_retention_N / projected_area_mm2
    angle_rows = []
    for angle_deg in (1.0, 3.0, 6.0, 10.0, 19.0):
        tangential_mm = mean_radius_mm * math.radians(angle_deg)
        # The five bonded rubber layers act in series through the spherical
        # stack for this global rotation; use total rubber thickness.
        shear_strain = tangential_mm / rubber_total_mm
        required_rubber_at_50pct_mm = tangential_mm / 0.50
        angle_rows.append(
            {
                "angle_deg": angle_deg,
                "tangential_displacement_mm": tangential_mm,
                "stack_shear_strain": shear_strain,
                "stack_shear_percent": 100.0 * shear_strain,
                "rubber_thickness_for_50pct_strain_mm": required_rubber_at_50pct_mm,
                "full_stack_radial_width_if_50pct_mm": required_rubber_at_50pct_mm
                + steel_total_mm,
            }
        )
    compression_rows = [
        {
            "assumed_layer_deflection_mm": delta_mm,
            "geometric_compression_strain": delta_mm / rubber_total_mm,
        }
        for delta_mm in (0.5, 1.0, 2.0, 5.0)
    ]
    six_deg = next(row for row in angle_rows if row["angle_deg"] == 6.0)
    return {
        "inner_radius_mm": BEARING_RI_MM,
        "outer_radius_mm": BEARING_RO_MM,
        "radial_window_mm": radial_window_mm,
        "rubber_layers": RUBBER_LAYERS,
        "rubber_layer_thickness_mm": RUBBER_T_MM,
        "steel_layer_thickness_mm": STEEL_LAYER_T_MM,
        "rubber_total_thickness_mm": rubber_total_mm,
        "steel_total_thickness_mm": steel_total_mm,
        "radial_stack_mm": stack_mm,
        "radial_reserve_mm": radial_window_mm - stack_mm,
        "projected_annulus_area_mm2": projected_area_mm2,
        "screen_compression_stress_MPa": compression_stress_MPa,
        "compression_strain_sensitivity": compression_rows,
        "shear_strain_sensitivity": angle_rows,
        "six_degree_shear_strain": six_deg["stack_shear_strain"],
        "six_degree_screen": "RED_FLAG",
        "compression_capacity_status": "UNRESOLVED_NO_ELASTOMER_RATING",
        "recommendation": "Reduce angular travel or enlarge the rubber layer/window; verify bonded shape factor, modulus, temperature, fatigue, and edge strain before release.",
    }


def _pitch_link_reaction_envelope(p: Mapping[str, Any]) -> Dict[str, Any]:
    """Sum five actual azimuth link reactions for swash bearing demands."""

    stiffness = _support_stiffness(p)
    azimuths_deg = [float(i * 360.0 / BLADE_COUNT) for i in range(BLADE_COUNT)]
    arm_m = _mm(PITCH_HORN_ARM_MM)
    from control_checks import evaluate
    park_input=math.degrees(math.asin((p['hub_z']-p['link_length']-p['swash_z']-p['swash_joint_z'])/p['horn']+math.sin(math.radians(12))))
    cases = {
        "parked_pitch0": (park_input, 0.0, 0.0, 0.0),
        "hover_collective12_cyclic3": (12.0, 3.0, 0.0, 0.0),
        "extreme_collective16_cyclic3": (16.0, 3.0, 0.0, 0.0),
    }
    results = []
    for name, (collective_deg, cyclic_c_deg, cyclic_s_deg, parking_deg) in cases.items():
        actual=evaluate(collective_deg,cyclic_c_deg,cyclic_s_deg,0)
        abs_pitch=actual['pitch']
        relative=[pitch-12 for pitch in abs_pitch]
        forces=[LOAD_FACTOR*force for force in actual['pitch_force_N']]
        sum_abs_N = sum(abs(force) for force in forces)
        overturn_actual_Nm = LOAD_FACTOR*math.hypot(*actual['elastic_swash_moment_Nm'][:2])
        overturn_upper_Nm = sum_abs_N * math.sqrt(p['horn_x']**2+p['horn']**2+p['swash_joint_z']**2)/1000
        results.append(
            {
                "name": name,
                "azimuth_deg": azimuths_deg,
                "absolute_pitch_deg": abs_pitch,
                "relative_pitch_deg": relative,
                "link_force_N": forces,
                "sum_abs_link_force_N": sum_abs_N,
                "actual_five_point_overturn_Nm": overturn_actual_Nm,
                "conservative_overturn_upper_Nm": overturn_upper_Nm,
                "torsional_torque_z_Nm": None,
            }
        )
    worst = max(results, key=lambda row: row["sum_abs_link_force_N"])
    return {
        "pitch_arm_mm": PITCH_HORN_ARM_MM,
        "blade_count": BLADE_COUNT,
        "pitch_radius_mm": SWASH_PITCH_RADIUS_MM,
        "cases": results,
        "worst_case": worst["name"],
        "worst_sum_abs_link_force_N": worst["sum_abs_link_force_N"],
        "basis": "Five equally spaced pitch-link reactions; axial link forces create horizontal overturning moment. Sum(abs(F))*pitch radius is an overturning upper bound, not torsional torque about Z. Rotor thrust is not applied to this control bearing.",
    }


def _swash_bearing_screen(link_reactions: Mapping[str, Any]) -> Dict[str, Any]:
    circumference_mm = 2.0 * math.pi * SWASH_PITCH_RADIUS_MM
    pitch_spacing_mm = circumference_mm / SWASH_BALL_COUNT
    assert SWASH_ROTATING_OUTER_INNER_R_MM > SWASH_FIXED_INNER_OUTER_R_MM
    assert pitch_spacing_mm > SWASH_BALL_D_MM
    worst_sum_N = float(link_reactions["worst_sum_abs_link_force_N"])
    equal_share_N = worst_sum_N / SWASH_BALL_COUNT
    worst_case = next(
        row for row in link_reactions["cases"] if row["name"] == link_reactions["worst_case"]
    )
    return {
        "ball_radius_mm": SWASH_BALL_R_MM,
        "ball_diameter_mm": SWASH_BALL_D_MM,
        "pitch_radius_mm": SWASH_PITCH_RADIUS_MM,
        "ball_count": SWASH_BALL_COUNT,
        "fixed_inner_race_outer_radius_mm": SWASH_FIXED_INNER_OUTER_R_MM,
        "rotating_outer_race_inner_radius_mm": SWASH_ROTATING_OUTER_INNER_R_MM,
        "race_radial_split_mm": SWASH_ROTATING_OUTER_INNER_R_MM - SWASH_FIXED_INNER_OUTER_R_MM,
        "groove_minor_radius_mm": SWASH_GROOVE_MINOR_RADIUS_MM,
        "race_axial_z_span_mm": [SWASH_AXIAL_Z_MIN_MM, SWASH_AXIAL_Z_MAX_MM],
        "ball_z_mm": SWASH_BALL_Z_MM,
        "pitch_circumference_mm": circumference_mm,
        "ball_center_spacing_mm": pitch_spacing_mm,
        "center_spacing_minus_ball_diameter_mm": pitch_spacing_mm - SWASH_BALL_D_MM,
        "control_link_equal_share_radial_N": equal_share_N,
        "control_link_sum_abs_N": worst_sum_N,
        "torsional_torque_z_Nm": worst_case["torsional_torque_z_Nm"],
        "torsional_torque_z_status": "NOT_INFERRED_FROM_AXIAL_LINK_FORCES",
        "actual_five_point_overturn_Nm": worst_case["actual_five_point_overturn_Nm"],
        "conservative_overturn_upper_Nm": worst_case["conservative_overturn_upper_Nm"],
        "capacity_status": "UNRESOLVED",
        "basis": "Five-point axial pitch-link reactions; sum(abs(F))*radius is an overturning upper bound, not a torsional torque-Z calculation.",
        "limitation": "This custom bearing carries pitch-link control reactions in this screen, not rotor thrust. No race groove contact, preload, internal clearance, load distribution, speed rating, or L10 evidence exists; equal share is arithmetic only. Torsional torque about Z is not inferred from axial force.",
    }


def _support_sensitivity(
    p: Mapping[str, Any], base: Mapping[str, float], mech: Mapping[str, Any]
) -> Dict[str, Any]:
    stiffness = _support_stiffness(p)
    inertia = float(base["blade_inertia_kgm2"])
    arm_m = float(mech.get("damper_arm_m", 0.37943149894404365))
    c_linear = float(mech.get("damper_linear_Ns_m", 32_000.0))
    c_eff = c_linear * arm_m * arm_m

    modes = {}
    for name, k in stiffness.items():
        frequency_Hz = math.sqrt(k / inertia) / (2.0 * math.pi)
        modes[name] = {
            "K_Nm_rad": k,
            "frequency_Hz": frequency_Hz,
        }

    lag_k = stiffness["lag_Nm_rad"]
    sensitivity = []
    for k_factor in (0.5, 1.0, 1.5):
        for mass_factor in (1.0, 1.2):
            for damping_factor in (1.0, 0.5):
                k = lag_k * k_factor
                i = inertia * mass_factor
                c = c_eff * damping_factor
                sensitivity.append(
                    {
                        "Klag_factor": k_factor,
                        "blade_mass_factor": mass_factor,
                        "damping_factor": damping_factor,
                        "frequency_Hz": math.sqrt(k / i) / (2.0 * math.pi),
                        "zeta": c / (2.0 * math.sqrt(k * i)),
                    }
                )
    return {
        "stiffness": stiffness,
        "blade_inertia_kgm2": inertia,
        "damper_arm_m": arm_m,
        "damper_linear_Ns_m": c_linear,
        "damper_equivalent_Nms_rad": c_eff,
        "modal_screen": modes,
        "lag_sensitivity": sensitivity,
        "basis": "Diagonal equivalent support stiffness and a velocity-proportional damper; no cross-coupling or full rotor-body stability model.",
    }


def _load_cases(
    base: Mapping[str, float], p: Mapping[str, Any], mech: Mapping[str, Any]
) -> list[Dict[str, Any]]:
    stiffness = _support_stiffness(p)
    rpm_nominal = base["rpm"]
    mass_nominal = base["blade_mass_kg"]
    rho = _pvalue(p, "rho", default=1.225)
    flap_arm_m = base["blade_cg_m"] - base["flap_hinge_m"]
    # Startup is a nominal 210 rpm, 30 s linear ramp with +20% mass. The
    # 231 rpm/+20% case is kept for the extreme operating envelope instead.
    cases = (
        {
            "name": "stopped_parked",
            "rpm": 0.0,
            "blade_mass_kg": mass_nominal,
            "rpm_factor": 0.0,
            "mass_factor": 1.0,
            "absolute_pitch_deg": 0.0,
            "relative_pitch_deg": -12.0,
            "flap_deg": 0.0,
            "lag_deg": 0.0,
            "ground_gust_m_s": None,
            "thrust_N": 0.0,
            "torque_Nm": None,
            "note": "Gravity/park support; absolute pitch zero, spring deflection -12 deg. Wind/brake/stop transient remain open.",
        },
        {
            "name": "startup_ramp_30s",
            "rpm": rpm_nominal,
            "blade_mass_kg": 1.20 * mass_nominal,
            "rpm_factor": 1.0,
            "mass_factor": 1.20,
            "absolute_pitch_deg": 12.0,
            "relative_pitch_deg": 0.0,
            "flap_deg": 0.0,
            "lag_deg": 0.0,
            "ground_gust_m_s": None,
            "thrust_N": None,
            "torque_Nm": None,
            "startup_ramp_time_s": 30.0,
            "note": "Linear 0-to-210 rpm/30 s inertial torque screen with +20% blade mass; acceleration profile and drivetrain torque are not measured.",
        },
        {
            "name": "hover_nominal",
            "rpm": rpm_nominal,
            "blade_mass_kg": mass_nominal,
            "rpm_factor": 1.0,
            "mass_factor": 1.0,
            "absolute_pitch_deg": 15.0,
            "relative_pitch_deg": 3.0,
            "flap_deg": 0.0,
            "lag_deg": 0.0,
            "ground_gust_m_s": None,
            "thrust_N": base["thrust_N"],
            "torque_Nm": base["torque_Nm"],
            "note": "Sea-level hover reference; 12 deg stress-free pitch plus 3 deg cyclic. No forward-flight model.",
        },
        {
            "name": "extreme_controls_overspeed",
            "rpm": 1.10 * rpm_nominal,
            "blade_mass_kg": 1.20 * mass_nominal,
            "rpm_factor": 1.10,
            "mass_factor": 1.20,
            "absolute_pitch_deg": 19.0,
            "relative_pitch_deg": 7.0,
            "flap_deg": 6.0,
            "lag_deg": 3.0,
            "ground_gust_m_s": None,
            "thrust_N": None,
            "torque_Nm": None,
            "note": "231 rpm/+20% mass and 16 deg collective +3 deg cyclic; 6 deg flap/3 deg lag are conservative support inputs. Aero control loads remain unresolved.",
        },
        {
            "name": "ground_gust_10m_s",
            "rpm": 0.0,
            "blade_mass_kg": mass_nominal,
            "rpm_factor": 0.0,
            "mass_factor": 1.0,
            "absolute_pitch_deg": 0.0,
            "relative_pitch_deg": -12.0,
            "flap_deg": 0.0,
            "lag_deg": 0.0,
            "ground_gust_m_s": 10.0,
            "thrust_N": None,
            "torque_Nm": None,
            "note": "10 m/s ground-gust scenario; dynamic pressure is reported, projected area/drag and parked restraint are unresolved.",
        },
    )
    rows = []
    cg_m = base["blade_cg_m"]
    for case in cases:
        rpm = float(case["rpm"])
        mass = float(case["blade_mass_kg"])
        cf = _centrifugal(mass, rpm, cg_m)
        gravity = mass * G
        retention = LOAD_FACTOR * max(cf, gravity)
        pitch = _pitch_demand(
            case["relative_pitch_deg"],
            case["flap_deg"],
            case["lag_deg"],
            stiffness,
        )
        fold = _fold_pin_screen(retention)
        inertia = base["blade_inertia_kgm2"] * case["mass_factor"]
        ramp_time = case.get("startup_ramp_time_s")
        ramp_accel = (
            rpm_nominal * 2.0 * math.pi / 60.0 / ramp_time
            if ramp_time
            else None
        )
        inertial_torque = inertia * ramp_accel if ramp_accel is not None else None
        gust = case.get("ground_gust_m_s")
        rows.append(
            {
                **case,
                "rpm": rpm,
                "blade_mass_kg": mass,
                "centrifugal_N": cf,
                "gravity_N": gravity,
                "design_retention_N": retention,
                "flap_stop_moment_Nm": gravity * flap_arm_m,
                "design_flap_stop_moment_Nm": LOAD_FACTOR * gravity * flap_arm_m,
                "stop_cradle_load_per_blade_N": gravity * flap_arm_m / _mm(STOP_LOAD_ARM_MM),
                "design_stop_cradle_load_per_blade_N": LOAD_FACTOR
                * gravity
                * flap_arm_m
                / _mm(STOP_LOAD_ARM_MM),
                "stop_collar_radius_mm": STOP_COLLAR_RADIUS_MM,
                "stop_collar_x_span_mm": [STOP_COLLAR_X_MIN_MM, STOP_COLLAR_X_MAX_MM],
                "stop_faces_z_mm": [STOP_LOWER_Z_MM, STOP_UPPER_Z_MM],
                "park_shoe_width_mm": PARK_SHOE_WIDTH_MM,
                "park_shoe_holds_zero": PARK_SHOE_HOLDS_ZERO,
                "stop_nominal_drop_deg": STOP_NOMINAL_DROP_DEG,
                "startup_angular_accel_rad_s2": ramp_accel,
                "startup_inertial_torque_per_blade_Nm": inertial_torque,
                "startup_inertial_torque_rotor_Nm": None
                if inertial_torque is None
                else BLADE_COUNT * inertial_torque,
                "ground_gust_dynamic_pressure_Pa": None
                if gust is None
                else 0.5 * rho * gust**2,
                "fold_pin_double_shear_MPa": fold["pin_double_shear_MPa"],
                "fold_clevis_bearing_MPa": fold["clevis_bearing_MPa"],
                "fold_pin_combined_vm_MPa": fold["pin_combined_vm_MPa"],
                "pitch_design_moment_Nm": pitch["design_pitch_moment_Nm"],
                "pitch_combined_support_moment_Nm": pitch["design_combined_moment_Nm"],
                "pitch_link_force_N": pitch["design_link_force_N"],
                "pitch_link_force_pitch_only_N": pitch["pitch_link_force_pitch_only_N"],
                "pitch_link_axial_stress_MPa": pitch["link_axial_stress_MPa"],
                "pitch_link_euler_pinned_N": pitch["link_euler_pinned_N"],
                "pitch_pin_double_shear_MPa": pitch["pitch_pin_double_shear_MPa"],
                "pitch_pin_bearing_MPa": pitch["pitch_pin_ear_bearing_MPa"],
                "pitch_pin_combined_vm_MPa": pitch["pitch_pin_combined_vm_MPa"],
                "pitch_horn_combined_vm_MPa": pitch["pitch_horn_combined_vm_MPa"],
            }
        )
    return rows


def _status_summary(
    max_fold: Mapping[str, Any],
    revised_fold: Mapping[str, Any],
    pitch_extreme: Mapping[str, Any],
    cuff: Mapping[str, Any],
    elastomer: Mapping[str, Any],
    swash: Mapping[str, Any],
) -> Dict[str, Any]:
    red_flags = [
        "Elastomer six-degree shear geometry remains beyond the transparent 50% stack-strain screen"
    ]
    if max_fold["pin_combined_vm_MPa"] > STEEL_SCREEN_ALLOWABLE_MPA:
        red_flags.append("Current 109.8 mm fold-pin combined stress exceeds the 400 MPa screen")
    pitch_screen_pass = (
        pitch_extreme["link_axial_stress_MPa"] <= STEEL_SCREEN_ALLOWABLE_MPA
        and pitch_extreme["pitch_pin_combined_vm_MPa"] <= STEEL_SCREEN_ALLOWABLE_MPA
        and pitch_extreme["pitch_horn_combined_vm_MPa"] <= STEEL_SCREEN_ALLOWABLE_MPA
        and pitch_extreme["link_euler_pinned_N"] > pitch_extreme["design_link_force_N"]
    )
    return {
        "passes": [
            "Frozen dimensions and mm-to-m boundary assertions",
            "Finite positive ball spacing arithmetic",
            "Rod Euler and solid 130 mm cuff arithmetic screens are computable",
            "Actual 109.8 mm fold-pin geometry is below the 400 MPa arithmetic screen under the stated 35F bending arm",
        ],
        "screening_only": [
            "17-4 H1150 400 MPa value is an explicit non-certification screen",
            "Current 109.8 mm fold-pin and 32 mm pitch-pin shear/bearing/bending calculations",
            "Solid 80 mm pitch horn section screen using the 250 mm design arm",
            "Support stiffness/modal and reduced-damping sensitivity",
        ],
        "red_flags": red_flags,
        "unresolved": [
            "Cuff local concentration, horn junction, fillets, holes, fatigue, and lock load path",
            "Measured aerodynamic hinge moment and exact pitch/flap/lag load sharing",
            "Swash race/ball contact capacity, preload, life, and combined reaction distribution",
            "Elastomer compression/shear allowable, modulus, temperature, fatigue, and bonded shape factor",
            "Startup drivetrain torque, wind/park restraint, fatigue, modal coupling, and full rotor-body stability",
        ],
        "capacity_status": {
            "fold_pin": max_fold["screen_status"],
            "recommended_fold_pin_screen": revised_fold["screen_status"],
            "pitch_link_and_pin": "PASS_SCREEN_ONLY" if pitch_screen_pass else "RED_FLAG",
            "cuff": cuff["screen_status"],
            "elastomer": elastomer["compression_capacity_status"],
            "swash_bearing": swash["capacity_status"],
        },
        "qualification_status": {
            "fold_pin": max_fold["capacity_status"],
            "pitch_link_and_pin": pitch_extreme["capacity_status"],
            "cuff": cuff["capacity_status"],
            "swash_bearing": swash["capacity_status"],
        },
    }


def build_assessment(p: Mapping[str, Any] = P) -> Dict[str, Any]:
    """Return the complete structural screen without writing files."""

    _assert_frozen_geometry(p)
    mech = _mechanism_loads(p)
    base = _base_inputs(p, mech)
    cases = _load_cases(base, p, mech)
    stopped_case = next(row for row in cases if row["name"] == "stopped_parked")
    expected_stop_moment = base["blade_mass_kg"] * G * (
        base["blade_cg_m"] - base["flap_hinge_m"]
    )
    assert math.isclose(
        stopped_case["flap_stop_moment_Nm"], expected_stop_moment, rel_tol=0.0, abs_tol=1e-9
    )
    assert math.isclose(
        stopped_case["stop_cradle_load_per_blade_N"] * _mm(STOP_LOAD_ARM_MM),
        stopped_case["flap_stop_moment_Nm"],
        rel_tol=0.0,
        abs_tol=1e-9,
    )
    max_case = max(cases, key=lambda row: row["design_retention_N"])
    max_fold = _fold_pin_screen(max_case["design_retention_N"])
    revised_fold = _fold_pin_screen(
        max_case["design_retention_N"], FOLD_RECOMMENDED_PIN_D_MM
    )
    cuff = _cuff_screen(max_case["design_retention_N"])
    cuff_radius70 = _cuff_screen(max_case["design_retention_N"], 70.0)
    stiffness = _support_stiffness(p)
    pitch_extreme = max(
        (
            _pitch_demand(
                row["relative_pitch_deg"],
                row["flap_deg"],
                row["lag_deg"],
                stiffness,
            )
            for row in cases
        ),
        key=lambda row: row["design_link_force_N"],
    )
    elastomer = _elastomer_screen(max_case["design_retention_N"])
    link_reactions = _pitch_link_reaction_envelope(p)
    swash = _swash_bearing_screen(link_reactions)
    support = _support_sensitivity(p, base, mech)
    mast = _mast_screen(base)
    return {
        "assessment": "initial structural sizing screen; not a strength or certification approval",
        "units": {
            "geometry": "mm",
            "loads": "N",
            "stress": "MPa",
            "stiffness": "N m/rad",
        },
        "inputs": {
            **base,
            "blade_count": BLADE_COUNT,
            "load_factor": LOAD_FACTOR,
            "cuff_radius_mm": CUFF_RADIUS_MM,
            "cuff_stop_collar_radius_mm": STOP_COLLAR_RADIUS_MM,
            "fold_pin_d_mm": FOLD_PIN_D_MM,
            "fold_pin_radius_mm": FOLD_PIN_RADIUS_MM,
            "fold_pin_bore_mm": FOLD_RECOMMENDED_BORE_MM,
            "recommended_fold_pin_d_mm": FOLD_RECOMMENDED_PIN_D_MM,
            "recommended_fold_pin_bore_mm": FOLD_RECOMMENDED_BORE_MM,
            "fold_clevis_thickness_mm": FOLD_CLEVIS_T_MM,
            "fold_clevis_width_mm": FOLD_CLEVIS_WIDTH_MM,
            "fold_tang_width_mm": FOLD_TANG_WIDTH_MM,
            "fold_tang_length_mm": FOLD_TANG_LENGTH_MM,
            "fold_tang_load_span_mm": FOLD_TANG_LOAD_SPAN_MM,
            "fold_hinge_x_mm": FOLD_HINGE_X_MM,
            "fold_lock_x_mm": FOLD_LOCK_X_MM,
            "stop_collar_x_span_mm": [STOP_COLLAR_X_MIN_MM, STOP_COLLAR_X_MAX_MM],
            "stop_faces_z_mm": [STOP_LOWER_Z_MM, STOP_UPPER_Z_MM],
            "park_shoe_width_mm": PARK_SHOE_WIDTH_MM,
            "park_shoe_holds_zero": PARK_SHOE_HOLDS_ZERO,
            "stop_nominal_drop_deg": STOP_NOMINAL_DROP_DEG,
            "stop_load_arm_mm": STOP_LOAD_ARM_MM,
            "pitch_link_d_mm": PITCH_LINK_D_MM,
            "pitch_link_center_length_mm": PITCH_LINK_LENGTH_MM,
            "pitch_pin_d_mm": PITCH_PIN_D_MM,
            "pitch_clevis_ear_thickness_mm": PITCH_EAR_T_MM,
            "pitch_pin_ear_center_span_mm": PITCH_PIN_EAR_CENTER_SPAN_MM,
            "pitch_pin_ball_load_width_mm": PITCH_PIN_BALL_LOAD_WIDTH_MM,
            "pitch_horn_arm_mm": PITCH_HORN_ARM_MM,
            "pitch_horn_cross_radius_mm": PITCH_HORN_CROSS_RADIUS_MM,
            "mast_outer_diameter_mm": MAST_OD_MM,
            "mast_inner_diameter_mm": MAST_ID_MM,
            "swash_fixed_inner_outer_radius_mm": SWASH_FIXED_INNER_OUTER_R_MM,
            "swash_rotating_outer_inner_radius_mm": SWASH_ROTATING_OUTER_INNER_R_MM,
            "swash_groove_minor_radius_mm": SWASH_GROOVE_MINOR_RADIUS_MM,
            "swash_axial_z_span_mm": [SWASH_AXIAL_Z_MIN_MM, SWASH_AXIAL_Z_MAX_MM],
            "swash_ball_z_mm": SWASH_BALL_Z_MM,
        },
        "material": {
            "candidate": "17-4 PH / Custom 630 H1150",
            "source": STEEL_SOURCE,
            "typical_yield_reference_MPa": STEEL_TYPICAL_YIELD_MPA,
            "typical_not_minimum": True,
            "screen_allowable_MPa": STEEL_SCREEN_ALLOWABLE_MPA,
            "screen_shear_allowable_MPa": STEEL_SCREEN_SHEAR_MPA,
            "limitation": "No product-form, orientation, temperature, process, notch, fatigue, or certification allowable is established.",
        },
        "load_cases": cases,
        "fold_pin_and_cuff": {
            **max_fold,
            "maximum_case": max_case["name"],
            "recommended_revision": revised_fold,
            "recommended_pin_bore_mm": FOLD_RECOMMENDED_BORE_MM,
            "recommended_clevis_width_mm": FOLD_CLEVIS_WIDTH_MM,
            "recommended_tang_width_mm": FOLD_TANG_WIDTH_MM,
            "hinge_x_mm": FOLD_HINGE_X_MM,
            "lock_x_mm": FOLD_LOCK_X_MM,
            "cuff": cuff,
            "radius70_sensitivity": cuff_radius70,
        },
        "pitch_link_and_horn": {
            "rod_d_mm": PITCH_LINK_D_MM,
            "rod_center_length_mm": PITCH_LINK_LENGTH_MM,
            "pin_d_mm": PITCH_PIN_D_MM,
            "clevis_ear_thickness_mm": PITCH_EAR_T_MM,
            "pin_screen_eccentricity_mm": PITCH_PIN_EAR_CENTER_SPAN_MM / 2.0
            - PITCH_PIN_BALL_LOAD_WIDTH_MM / 8.0,
            "horn_arm_mm": PITCH_HORN_ARM_MM,
            "horn_cross_section_radius_mm": PITCH_HORN_CROSS_RADIUS_MM,
            "extreme_control_screen": pitch_extreme,
            "actual_horn_section_status": pitch_extreme["pitch_horn_screen_status"],
            "aerodynamic_hinge_moment_status": "UNRESOLVED",
        },
        "elastomer_support": elastomer,
        "swash_bearing": swash,
        "swash_link_reactions": link_reactions,
        "mast_screen": mast,
        "support_sensitivity": support,
        "operating_envelope": {
            "flight_mode": "sea-level hover/low-speed initial design only; no forward-flight loads",
            "temperature_C": [0.0, 40.0],
            "air_density_kg_m3": _pvalue(p, "rho", default=1.225),
            "temperature_density_variation_status": "NOT_MODELED",
            "ground_gust_m_s": 10.0,
            "ground_gust_status": "dynamic pressure only; drag area and restraint response unresolved",
        },
        "status": _status_summary(
            max_fold, revised_fold, pitch_extreme, cuff, elastomer, swash
        ),
        "requirements_R13_R17": {
            "R13": {
                "status": "SCREENED_NOT_VERIFIED",
                "evidence": "Load path, retained centrifugal demand, thrust/torque reference, pin/cuff/horn demands in this JSON.",
                "gap": "Six-component reactions, mast boundary moments, and joint load transfer are not balanced.",
            },
            "R14": {
                "status": "SCREENED_NOT_VERIFIED",
                "evidence": "Stopped, 30 s startup ramp, hover, extreme-control/overspeed, and 10 m/s ground-gust cases with mass +20% and rpm +10% envelope.",
                "gap": "Acceleration profile, drag area, stop transient, simultaneous control/aero loads, and operating envelope remain assumptions.",
            },
            "R15": {
                "status": "NOT_PASS",
                "evidence": "Candidate material and explicit 400 MPa arithmetic screen only.",
                "gap": "No product-form allowables, bearing rating, elastomer rating, pin bending, fatigue, or lock capacity.",
            },
            "R16": {
                "status": "RED_FLAG_AND_UNRESOLVED",
                "evidence": "5 x (4.5 mm rubber + 1.3 mm steel) geometry and shear/compression strain sensitivities.",
                "gap": "Six-degree shear geometry is incompatible with the transparent 50% screen; modulus, shape factor, temperature, and life are missing.",
            },
            "R17": {
                "status": "PASS_RISK_PLAN_ONLY",
                "evidence": "Risk/next-analysis plan and support stiffness, mass, and 50% lag-damping sensitivity rows.",
                "gap": "Full aircraft dynamics are intentionally deferred; fatigue, aeroelasticity, mode coupling, and test evidence are future work.",
            },
        },
        "verification_limits": [
            "No strength item is promoted to PASS from a screening allowable.",
            "Pin bending uses explicit eccentricity sensitivity because the load line is not frozen.",
            "Swash ball loads are derived from five pitch-link reactions, not rotor thrust, and remain arithmetic only.",
            "Compression and stack shear strain are reported against geometry; force-to-strain needs elastomer material data.",
        ],
    }


CSV_FIELDS = [
    "name",
    "rpm",
    "blade_mass_kg",
    "rpm_factor",
    "mass_factor",
    "centrifugal_N",
    "gravity_N",
    "design_retention_N",
    "thrust_N",
    "torque_Nm",
    "absolute_pitch_deg",
    "relative_pitch_deg",
    "flap_deg",
    "lag_deg",
    "flap_stop_moment_Nm",
    "design_flap_stop_moment_Nm",
    "stop_cradle_load_per_blade_N",
    "design_stop_cradle_load_per_blade_N",
    "stop_collar_radius_mm",
    "stop_faces_z_mm",
    "park_shoe_width_mm",
    "park_shoe_holds_zero",
    "stop_nominal_drop_deg",
    "startup_angular_accel_rad_s2",
    "startup_inertial_torque_per_blade_Nm",
    "startup_inertial_torque_rotor_Nm",
    "ground_gust_m_s",
    "ground_gust_dynamic_pressure_Pa",
    "pitch_design_moment_Nm",
    "pitch_combined_support_moment_Nm",
    "pitch_link_force_N",
    "pitch_link_force_pitch_only_N",
    "pitch_link_axial_stress_MPa",
    "pitch_link_euler_pinned_N",
    "pitch_pin_double_shear_MPa",
    "pitch_pin_bearing_MPa",
    "pitch_pin_combined_vm_MPa",
    "pitch_horn_combined_vm_MPa",
    "fold_pin_double_shear_MPa",
    "fold_clevis_bearing_MPa",
    "fold_pin_combined_vm_MPa",
    "note",
]


def write_outputs(assessment: Mapping[str, Any]) -> None:
    verification = OUT / "verification"
    verification.mkdir(parents=True, exist_ok=True)
    (verification / "structure.json").write_text(
        json.dumps(_json_number(assessment), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (verification / "structure-load-cases.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in assessment["load_cases"]:
            writer.writerow({field: _json_number(row.get(field)) for field in CSV_FIELDS})


def main() -> None:
    assessment = build_assessment()
    write_outputs(assessment)
    print(json.dumps(_json_number(assessment["status"]), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()


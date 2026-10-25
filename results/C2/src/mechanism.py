"""Rotor-head kinematics and screening loads.

The v1 model rotated a common scissor point with ``Rz(psi)``.  That is not a
physical anti-rotation constraint: a stationary scissor must stay in its XZ
plane and a rotating scissor must stay in the radial/Z plane of its mast.  v2
solves the two yaw angles first, then closes each linkage as a planar RR-S
mechanism.  Units are millimetres, degrees at the public API, and SI units in
``loads``.
"""

import json
from pathlib import Path

import numpy as np


# Keep the v1 public values.  The added entries describe the two real scissor
# mechanisms and are deliberately plain tuples so P remains easy to serialize.
P = dict(
    radius=9300.0,
    chord=650.0,
    root=1600.0,
    joint=600.0,
    hub_z=2200.0,
    swash_z=1100.0,
    swash_joint_z=100.0,
    horn_x=600.0,
    horn=250.0,
    collective_ref=12.0,
    rpm=210.0,
    blade_mass=180.0,
    aircraft_mass=15600.0,
    rho=1.225,
    fold_pin=1500.0,
    link_length=float(
        np.hypot(
            250.0 * (np.cos(np.deg2rad(12.0)) - 1.0),
            1000.0 + 250.0 * np.sin(np.deg2rad(12.0)),
        )
    ),
    stator_base=(900.0, 0.0, 350.0),
    rotor_base=(350.0, 0.0, 1900.0),
    # Outer bracket centres keep the 48 mm eye/clevis clear of the swash-ring
    # envelope; the short bracket can return inward to the ring.
    stator_end_offset=(820.0, 0.0, -150.0),
    rotor_end_offset=(820.0, 0.0, 70.0),
    scissor_arm_lengths=(550.0, 510.0),
    stator_elbow_sign=1,
    rotor_elbow_sign=-1,
    stator_joint_limits_deg=((-180.0, 180.0), (-180.0, 180.0)),
    rotor_joint_limits_deg=((-180.0, 180.0), (-180.0, 180.0)),
    scissor_yaw_limit_deg=35.0,
    pitch_limits_deg=(-35.0, 45.0),
    scissors_tolerance_mm=1e-7,
)

OUT = Path(__file__).resolve().parents[1]
_X = np.array([1.0, 0.0, 0.0])
_Y = np.array([0.0, 1.0, 0.0])
_Z = np.array([0.0, 0.0, 1.0])


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def deg(a):
    return np.deg2rad(a)


def _vector(value, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return value


def _unit(value, name):
    value = _vector(value, name)
    length = np.linalg.norm(value)
    if length <= 0.0:
        raise ValueError(f"{name} must be non-zero")
    return value / length


def _wrap_near(angle, reference):
    """Return the equivalent angle nearest an unwrapped reference angle."""
    return float(angle + 2.0 * np.pi * np.floor((reference - angle + np.pi) / (2.0 * np.pi)))


def _within(value, limits, tol=1e-9):
    return limits[0] - tol <= value <= limits[1] + tol


def swash(collective=12.0, cyclic_c=0.0, cyclic_s=0.0, p=P):
    """Return swash centre and the rotation taking local vectors to the plane."""
    h = p["horn"] * (
        np.sin(deg(collective)) - np.sin(deg(p["collective_ref"]))
    )
    phase = np.arctan2(p["horn"], p["horn_x"])
    scale = p["horn"] * np.cos(deg(p["collective_ref"])) / np.hypot(
        p["horn_x"], p["horn"]
    )
    c, s = deg(cyclic_c) * scale, deg(cyclic_s) * scale
    gx = c * np.cos(phase) - s * np.sin(phase)
    gy = c * np.sin(phase) + s * np.cos(phase)
    n = np.array([-gx, -gy, 1.0])
    n /= np.linalg.norm(n)
    v = np.cross(_Z, n)
    k = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
    )
    q = np.eye(3) + k + k @ k / (1.0 + n[2])
    return np.array([0.0, 0.0, p["swash_z"] + h]), q


def _solve_sinusoid(a, b, c, reference, limit, name):
    """Solve ``a*cos(x)+b*sin(x)+c == 0`` on the branch near reference."""
    radius = float(np.hypot(a, b))
    if radius <= 1e-14:
        if abs(c) <= 1e-10:
            angle = float(reference)
        else:
            raise ValueError(f"No finite {name} closure")
        return angle
    ratio = -float(c) / radius
    if abs(ratio) > 1.0 + 1e-12:
        raise ValueError(f"No finite {name} closure")
    phase = np.arctan2(b, a)
    offset = np.arccos(np.clip(ratio, -1.0, 1.0))
    candidates = []
    for candidate in (phase - offset, phase + offset):
        candidate = _wrap_near(candidate, reference)
        if _within(candidate - reference, (-limit, limit), tol=1e-10):
            candidates.append(candidate)
    if not candidates:
        raise ValueError(f"{name} branch outside yaw limit")
    return min(candidates, key=lambda value: abs(value - reference))


def _planar_rrs(base, end, radial, axis, arm_lengths, joint_limits, name, elbow_sign=1):
    """Close a planar 2R linkage and return physical points, axes and angles."""
    base = _vector(base, f"{name} base")
    end = _vector(end, f"{name} end")
    radial = _unit(radial, f"{name} radial basis")
    axis = _unit(axis, f"{name} revolute axis")
    z_axis = _unit(np.cross(axis, radial), f"{name} plane basis")
    # Ensure the plane basis points with global +Z.  This is important for the
    # rotor frame where radial x tangent = +Z, not -Z.
    if z_axis @ _Z < 0.0:
        z_axis = -z_axis
    lengths = tuple(float(v) for v in arm_lengths)
    if len(lengths) != 2 or any(v <= 0.0 or not np.isfinite(v) for v in lengths):
        raise ValueError(f"{name} arm lengths must be positive")
    limits = tuple(tuple(float(v) for v in lim) for lim in joint_limits)
    if len(limits) != 2 or any(len(lim) != 2 or lim[0] > lim[1] for lim in limits):
        raise ValueError(f"{name} joint limits are invalid")

    d = end - base
    tangential_residual = float(d @ axis)
    if abs(tangential_residual) > 1e-6:
        raise ValueError(f"{name} endpoint is outside its revolute plane")
    x, z = float(d @ radial), float(d @ z_axis)
    l1, l2 = lengths
    radius_sq = x * x + z * z
    cos_q2 = (radius_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if cos_q2 < -1.0 - 1e-10 or cos_q2 > 1.0 + 1e-10:
        raise ValueError(f"No finite {name} arm closure")
    q2 = float(elbow_sign) * np.arccos(np.clip(cos_q2, -1.0, 1.0))
    q1 = np.arctan2(z, x) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    angles = np.array([q1, q2])
    angles_deg = np.rad2deg(angles)
    if not all(_within(float(v), lim) for v, lim in zip(angles_deg, limits)):
        raise ValueError(f"{name} joint angle outside limit")
    elbow = base + radial * (l1 * np.cos(q1)) + z_axis * (l1 * np.sin(q1))
    first = elbow - base
    second = end - elbow
    closure = max(abs(np.linalg.norm(first) - l1), abs(np.linalg.norm(second) - l2))
    radial_projection = np.array([base @ radial, elbow @ radial, end @ radial])
    radii = np.hypot(
        np.array([base[0], elbow[0], end[0]]),
        np.array([base[1], elbow[1], end[1]]),
    )
    return {
        "base": base,
        "elbow": elbow,
        "end": end,
        "joint_1": elbow,
        "joint_2": end,
        "endpoint": end,
        "axes": np.vstack((axis, axis)),
        "base_axis": axis,
        "elbow_axis": axis,
        "radial_basis": radial,
        "z_basis": z_axis,
        "arm_lengths_mm": [l1, l2],
        "joint_angles_rad": angles,
        "joint_angles_deg": angles_deg,
        "joint_limits_deg": limits,
        "radial_projection_mm": radial_projection,
        "radii_mm": radii,
        "min_radius_mm": float(np.min(radii)),
        "min_radial_projection_mm": float(np.min(radial_projection)),
        "planar_residual_mm": abs(tangential_residual),
        "closure_residual_mm": float(closure),
        "valid": bool(closure <= 1e-7 and abs(tangential_residual) <= 1e-6),
    }


def swash_frames(c=12.0, cc=0.0, cs=0.0, psi=0.0, p=P):
    """Return physically constrained stationary and rotating swash frames.

    ``psi`` is the mast azimuth in degrees for this low-level function.  The
    returned ``stator_rotation`` is ``q @ Rz(alpha)`` and
    ``rotor_rotation`` is ``q @ Rz(beta)``.  The stator endpoint is solved in
    the global XZ plane; the rotor endpoint is solved in the radial/Z plane
    whose tangent axis is ``Rz(psi) @ Y``.  Both scissor linkages use fixed
    550/510 mm arms and two physical revolute axes.
    """
    psi_rad = float(deg(psi))
    if not np.isfinite(psi_rad):
        raise ValueError("psi must be finite")
    center, q = swash(c, cc, cs, p)
    end_stator = _vector(p["stator_end_offset"], "stator end offset")
    end_rotor = _vector(p["rotor_end_offset"], "rotor end offset")
    yaw_limit = float(deg(p.get("scissor_yaw_limit_deg", 35.0)))
    if yaw_limit <= 0.0 or not np.isfinite(yaw_limit):
        raise ValueError("scissor_yaw_limit_deg must be positive")

    # b_y = 0 for the stationary anti-rotation link.
    a_s = float(q[1, 0] * end_stator[0])
    b_s = float(q[1, 1] * end_stator[0])
    c_s = float(center[1] + q[1, 2] * end_stator[2])
    alpha = _solve_sinusoid(a_s, b_s, c_s, 0.0, yaw_limit, "stator yaw")
    stator_rotation = q @ rz(alpha)
    stator_end = center + stator_rotation @ end_stator
    stator_axis = _Y
    stator = _planar_rrs(
        p["stator_base"],
        stator_end,
        _X,
        stator_axis,
        p["scissor_arm_lengths"],
        p.get("stator_joint_limits_deg", ((-180.0, 180.0), (-180.0, 180.0))),
        "stator scissor",
        elbow_sign=int(p.get("stator_elbow_sign", 1)),
    )

    # b dot tangent = 0 for the rotating scissor's radial/Z plane.
    mast = rz(psi_rad)
    rotor_base = mast @ _vector(p["rotor_base"], "rotor base")
    rotor_radial = mast @ _X
    rotor_axis = mast @ _Y
    row_t = rotor_axis @ q
    a_r = float(row_t[0] * end_rotor[0])
    b_r = float(row_t[1] * end_rotor[0])
    c_r = float(rotor_axis @ center + row_t[2] * end_rotor[2] - rotor_axis @ rotor_base)
    beta = _solve_sinusoid(a_r, b_r, c_r, psi_rad, yaw_limit, "rotor yaw")
    rotor_rotation = q @ rz(beta)
    rotor_end = center + rotor_rotation @ end_rotor
    rotor = _planar_rrs(
        rotor_base,
        rotor_end,
        rotor_radial,
        rotor_axis,
        p["scissor_arm_lengths"],
        p.get("rotor_joint_limits_deg", ((-180.0, 180.0), (-180.0, 180.0))),
        "rotor scissor",
        elbow_sign=int(p.get("rotor_elbow_sign", -1)),
    )

    # Keep both verbose and short names: the verbose names are the stable API,
    # while the aliases make the result convenient for CAD scripts.
    return {
        "center": center,
        "q": q,
        "stator_rotation": stator_rotation,
        "rotor_rotation": rotor_rotation,
        "alpha_rad": float(alpha),
        "alpha_deg": float(np.rad2deg(alpha)),
        "beta_rad": float(beta),
        "beta_deg": float(np.rad2deg(beta)),
        "alpha": float(alpha),
        "beta": float(beta),
        "stator_end": stator_end,
        "rotor_end": rotor_end,
        "stator_endpoint": stator_end,
        "rotor_endpoint": rotor_end,
        "scissors_endpoints": {"stator": stator_end, "rotor": rotor_end},
        "stator_revolute_axes": stator["axes"],
        "rotor_revolute_axes": rotor["axes"],
        "stator_scissors": stator,
        "rotor_scissors": rotor,
        "stator": stator,
        "rotor": rotor,
        "stator_plane_residual_mm": float(abs(stator_end[1])),
        "rotor_plane_residual_mm": float(abs((rotor_end - rotor_base) @ rotor_axis)),
    }


def _pitch_candidates(base, bottom, articulated, p):
    horn = float(p["horn"])
    d = base - bottom
    u = articulated @ np.array([0.0, horn, 0.0])
    v = articulated @ np.array([0.0, 0.0, horn])
    a, b = 2.0 * d @ u, 2.0 * d @ v
    c = float(p["link_length"] ** 2 - d @ d - horn**2)
    amplitude = float(np.hypot(a, b))
    if abs(c) > amplitude + 1e-10:
        raise ValueError("No finite-link closure")
    if amplitude <= 1e-14:
        raise ValueError("Pitch linkage is singular")
    delta = np.arctan2(b, a)
    ac = np.arccos(np.clip(c / amplitude, -1.0, 1.0))
    lower, upper = (float(v) for v in p.get("pitch_limits_deg", (-35.0, 45.0)))
    candidates = []
    for value in (delta - ac, delta + ac):
        value = _wrap_near(value, deg(p.get("collective_ref", 12.0)))
        if lower <= np.rad2deg(value) <= upper:
            candidates.append(value)
    if not candidates:
        raise ValueError("Pitch outside assembly branch")
    return candidates, u, v, a, b


def solve(
    psi,
    collective=12.0,
    cyclic_c=0.0,
    cyclic_s=0.0,
    flap=0.0,
    lag=0.0,
    p=P,
    mast_psi=None,
):
    """Solve actuator pitch for one blade azimuth.

    ``psi`` is always the blade's global azimuth.  If ``mast_psi`` is omitted,
    the rotating swash frame is aligned with that blade (v1-compatible use).
    For a globally phased rotor pass the mast azimuth explicitly; the local
    blade index used below is then ``psi - mast_psi``.
    """
    psi = float(psi)
    if not np.isfinite(psi):
        raise ValueError("psi must be finite")
    frame_psi = psi if mast_psi is None else float(mast_psi)
    if not np.isfinite(frame_psi):
        raise ValueError("mast_psi must be finite")
    frames = swash_frames(collective, cyclic_c, cyclic_s, frame_psi, p)
    center = frames["center"]
    rotor_rotation = frames["rotor_rotation"]
    blade_offset = deg(psi - frame_psi)
    rotor_horn = np.array([p["horn_x"], p["horn"], p["swash_joint_z"]])
    bottom = center + rotor_rotation @ rz(blade_offset) @ rotor_horn

    az = rz(deg(psi))
    articulated = az @ rz(deg(lag)) @ ry(-deg(flap))
    pivot = az @ np.array([p["joint"], 0.0, 0.0]) + np.array([0.0, 0.0, p["hub_z"]])
    base = pivot + articulated @ np.array([p["horn_x"] - p["joint"], 0.0, 0.0])
    candidates, u, v, a, b = _pitch_candidates(base, bottom, articulated, p)
    theta = min(candidates, key=lambda value: abs(value - deg(collective)))
    top = base + u * np.cos(theta) + v * np.sin(theta)
    sensitivity = abs((-a * np.sin(theta) + b * np.cos(theta)) / (2.0 * p["link_length"]))
    actuator_length = float(np.linalg.norm(top - bottom))
    residual = float(abs(actuator_length - p["link_length"]))
    pitch_deg = float(np.rad2deg(theta))
    pitch_limits = tuple(float(v) for v in p.get("pitch_limits_deg", (-35.0, 45.0)))
    actuator = {
        "base": base,
        "swash_endpoint": bottom,
        "blade_endpoint": top,
        "servo_pin_axis": az @ _Y,
        "pitch_pin_axis": az @ _X,
        "length_mm": actuator_length,
        "target_length_mm": float(p["link_length"]),
        "length_error_mm": residual,
        "lever_mm_per_rad": float(sensitivity),
        "joint_angle_deg": pitch_deg,
        "joint_limits_deg": pitch_limits,
        "joint_in_limits": bool(_within(pitch_deg, pitch_limits)),
        "valid": bool(residual <= 1e-6 and _within(pitch_deg, pitch_limits)),
    }
    return {
        # v1 keys retained for build/render callers.
        "theta": pitch_deg,
        "top": top,
        "bottom": bottom,
        "pivot": pivot,
        "rotation": articulated @ rx(theta),
        "residual": residual,
        "lever": float(sensitivity),
        # v2 analysis.
        "pitch_deg": pitch_deg,
        "blade_azimuth_deg": psi,
        "mast_azimuth_deg": frame_psi,
        "blade_offset_deg": float(psi - frame_psi),
        "center": center,
        "stator_rotation": frames["stator_rotation"],
        "rotor_rotation": rotor_rotation,
        "stator_scissors": frames["stator_scissors"],
        "rotor_scissors": frames["rotor_scissors"],
        "swash_frames": frames,
        "actuator": actuator,
        "actuator_length_mm": actuator_length,
        "servo_pin_axis": actuator["servo_pin_axis"],
        "pitch_pin_axis": actuator["pitch_pin_axis"],
        "joint_angles_deg": {
            "pitch": pitch_deg,
            "stator_scissors": frames["stator_scissors"]["joint_angles_deg"],
            "rotor_scissors": frames["rotor_scissors"]["joint_angles_deg"],
        },
    }


def loads(p=P):
    """Return the v1 SI screening loads with unchanged meanings."""
    r = p["radius"] / 1000.0
    r0 = p["root"] / 1000.0
    mass = p["blade_mass"]
    omega = p["rpm"] * 2.0 * np.pi / 60.0
    cg = (r + r0) / 2.0
    inertia = mass * ((r - r0) ** 2 / 12.0 + (cg - p["joint"] / 1000.0) ** 2)
    thrust = 1.05 * p["aircraft_mass"] * 9.80665
    area = np.pi * r * r
    induced = np.sqrt(thrust / (2.0 * p["rho"] * area))
    sigma = 5.0 * p["chord"] / 1000.0 / (np.pi * r)
    profile = sigma * 0.012 / 8.0 * p["rho"] * area * (omega * r) ** 3
    power = 1.15 * thrust * induced + profile
    rr = np.linspace(r0, r, 2001)
    vel = omega * rr
    twist = deg(-8.0) * (rr - 0.75 * r) / (r - r0)
    slope = 0.5 * p["rho"] * 5.0 * p["chord"] / 1000.0 * 5.7
    lift0 = np.trapz(slope * vel**2 * (twist - induced / vel), rr)
    liftgrad = np.trapz(slope * vel**2, rr)
    trim = (thrust - lift0) / liftgrad
    centrifugal = mass * omega**2 * cg
    da = np.array([-0.180, -0.400, 0.0])
    db = rx(deg(12.0)) @ np.array([0.450, -0.350, 0.0])
    line = db - da
    line /= np.linalg.norm(line)
    arm = abs(np.cross(db, line)[2])
    k_lag, damper = 80000.0, 32000.0
    c_eff = damper * arm**2
    wn = np.sqrt(k_lag / inertia)
    diameter, bore = 0.32, 0.23
    j = np.pi * (diameter**4 - bore**4) / 32.0
    annulus = np.pi * (diameter**2 - bore**2) / 4.0
    tau = 1.5 * (power / omega) * diameter / (2.0 * j)
    sigma_mast = 1.5 * thrust / annulus
    pin = 0.085
    lock_force = 1.5 * centrifugal
    pin_shear = lock_force / (2.0 * np.pi * pin**2 / 4.0)
    return dict(
        omega_rad_s=omega,
        tip_speed_m_s=omega * r,
        tip_Mach=omega * r / 340.3,
        thrust_N=thrust,
        disk_loading_N_m2=thrust / area,
        induced_velocity_m_s=induced,
        profile_power_W=profile,
        power_W=power,
        torque_Nm=power / omega,
        blade_cg_m=cg,
        blade_inertia_hinge_kgm2=inertia,
        centrifugal_N=centrifugal,
        trim_pitch_75_deg=float(np.rad2deg(trim)),
        mast_VM_MPa=float(np.sqrt(sigma_mast**2 + 3.0 * tau**2) / 1e6),
        fold_pin_double_shear_MPa=pin_shear / 1e6,
        damper_arm_m=arm,
        damper_linear_Ns_m=damper,
        lag_frequency_Hz=wn / (2.0 * np.pi),
        lag_zeta=c_eff / (2.0 * np.sqrt(k_lag * inertia)),
        lag_zeta_degraded=0.5 * c_eff / (2.0 * np.sqrt(k_lag * inertia)),
        lag_zeta_no_damper=0.0,
        lag_decay_5s=float(np.exp(-c_eff / (2.0 * inertia) * 5.0)),
        lag_decay_degraded_5s=float(np.exp(-0.5 * c_eff / (2.0 * inertia) * 5.0)),
    )


FOLD_STATES = frozenset(("flight", "neutral", "folding", "folded", "parked"))


def _pins(pins):
    if isinstance(pins, (str, bytes)):
        raise TypeError("pins must be a sequence of five booleans")
    try:
        values = tuple(pins)
    except TypeError as exc:
        raise TypeError("pins must be a sequence of five booleans") from exc
    if len(values) != 5 or any(not isinstance(value, (bool, np.bool_)) for value in values):
        raise ValueError("pins must contain exactly five booleans")
    return tuple(bool(value) for value in values)


def flight_allowed(rpm, pins, mast_locked, folding, fold_state=None):
    """Validate the flight interlock, including the explicit fold state."""
    rpm = float(rpm)
    if not np.isfinite(rpm):
        raise ValueError("rpm must be finite")
    if not isinstance(mast_locked, (bool, np.bool_)) or not isinstance(
        folding, (bool, np.bool_)
    ):
        raise TypeError("mast_locked and folding must be booleans")
    pin_values = _pins(pins)
    state = ("folding" if folding else "flight") if fold_state is None else str(fold_state).lower()
    if state not in FOLD_STATES:
        raise ValueError(f"unknown fold state: {fold_state!r}")
    return bool(
        rpm > 0.0
        and all(pin_values)
        and not mast_locked
        and not folding
        and state in {"flight", "neutral"}
    )


def fold_state_checks(state, rpm=0.0, pins=(True,) * 5, mast_locked=False, folding=False):
    """Return whether a named stored/transition state is internally consistent."""
    state = str(state).lower()
    if state not in FOLD_STATES:
        raise ValueError(f"unknown fold state: {state!r}")
    rpm = float(rpm)
    if not np.isfinite(rpm):
        raise ValueError("rpm must be finite")
    pin_values = _pins(pins)
    if not isinstance(mast_locked, (bool, np.bool_)) or not isinstance(
        folding, (bool, np.bool_)
    ):
        raise TypeError("mast_locked and folding must be booleans")
    if state in {"flight", "neutral"}:
        return flight_allowed(rpm, pin_values, mast_locked, folding, state)
    if state == "folding":
        return bool(rpm == 0.0 and folding and mast_locked)
    # Parked/folded are stopped, mast-locked states.  A folded model may still
    # have all pins engaged; requiring that is the safe CAD-state invariant.
    return bool(rpm == 0.0 and mast_locked and not folding and all(pin_values))


def verify(p=P):
    """Yield compact numerical checks used to create verification/numerical.json."""
    max_error = 0.0
    max_frame = 0.0
    max_actuator = 0.0
    min_lever = np.inf
    tmin, tmax = np.inf, -np.inf
    count = 0
    worst = None
    for step in (12, 6):
        error = frame_residual = actuator_error = 0.0
        lever = np.inf
        for psi in np.arange(0.0, 360.0, step):
            for collective in (8.0, 12.0, 16.0):
                for ca in np.arange(0.0, 360.0, 45.0):
                    cc, cs = 3.0 * np.cos(deg(ca)), 3.0 * np.sin(deg(ca))
                    for flap, lag in ((0.0, 0.0), (-3.0, -3.0), (-3.0, 3.0), (6.0, -3.0), (6.0, 3.0)):
                        state = solve(psi, collective, cc, cs, flap, lag, p)
                        ideal = collective + cc * np.cos(deg(psi)) + cs * np.sin(deg(psi))
                        err = abs(state["theta"] - ideal)
                        error = max(error, err)
                        max_error = max(max_error, err)
                        if err >= max_error:
                            worst = [float(psi), collective, float(cc), float(cs), flap, lag, state["theta"]]
                        frame = max(
                            state["swash_frames"]["stator_plane_residual_mm"],
                            state["swash_frames"]["rotor_plane_residual_mm"],
                            state["swash_frames"]["stator_scissors"]["closure_residual_mm"],
                            state["swash_frames"]["rotor_scissors"]["closure_residual_mm"],
                        )
                        actuator_error = max(actuator_error, state["residual"])
                        frame_residual = max(frame_residual, frame)
                        lever = min(lever, state["lever"])
                        max_frame = max(max_frame, frame)
                        max_actuator = max(max_actuator, state["residual"])
                        min_lever = min(min_lever, state["lever"])
                        tmin = min(tmin, state["theta"])
                        tmax = max(tmax, state["theta"])
                        count += 1
        yield f"{step}deg_grid", dict(
            samples=count,
            uncoupled_error_deg=error,
            frame_residual_mm=frame_residual,
            actuator_residual_mm=actuator_error,
            min_lever_mm_rad=lever,
        )

    zero = np.array([solve(i * 72.0, p=p)["theta"] for i in range(5)])
    matrix = []
    for inp in ((12.01, 0.0, 0.0), (12.0, 0.01, 0.0), (12.0, 0.0, 0.01)):
        matrix.append(
            (np.array([solve(i * 72.0, *inp, p=p)["theta"] for i in range(5)]) - zero) / 0.01
        )
    matrix = np.asarray(matrix).T
    assert np.linalg.matrix_rank(matrix) == 3
    assert max_frame < 1e-5 and max_actuator < 1e-6 and min_lever > 100.0
    assert tmin >= -35.0 and tmax <= 45.0
    try:
        solve(0.0, p={**p, "link_length": 20.0})
    except ValueError:
        rejected = True
    else:
        rejected = False
    assert rejected
    assert flight_allowed(210.0, [True] * 5, False, False)
    assert not flight_allowed(210.0, [True, False, True, True, True], False, False)
    assert not flight_allowed(0.0, [True] * 5, False, False)
    assert fold_state_checks("folded", 0.0, [True] * 5, True, False)
    assert not fold_state_checks("folded", 210.0, [True] * 5, True, False)
    reverse = np.array([solve(i * 72.0, 12.0, -3.0, 0.0, p=p)["theta"] for i in range(5)])
    expect = 12.0 + 3.0 * np.cos(deg(np.arange(5) * 72.0))
    assert np.max(abs(reverse - expect)) > 5.0
    variants = {}
    for name, pp in (
        ("length_plus_2mm", {**p, "link_length": p["link_length"] + 2.0}),
        ("horn_x_plus_5mm", {**p, "horn_x": p["horn_x"] + 5.0}),
    ):
        state = solve(0.0, p=pp)
        assert state["residual"] < 1e-6
        variants[name] = dict(pitch_deg=state["theta"], top_mm=state["top"].tolist())
    yield "summary", dict(
        max_coupled_error_deg=max_error,
        worst=worst,
        pitch_range_deg=[tmin, tmax],
        frame_residual_mm=max_frame,
        actuator_residual_mm=max_actuator,
        jacobian=matrix.tolist(),
        rank=3,
        condition=float(np.linalg.cond(matrix)),
        negative_tests=dict(
            impossible_link=rejected,
            reversed_phase=True,
            unlocked_flight=True,
            zero_rpm=True,
            folded_state=True,
        ),
        variants=variants,
    )


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


if __name__ == "__main__":
    result = {"loads": loads(), "kinematics": dict(verify())}
    path = OUT / "verification" / "numerical.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(result), indent=2))
    print(json.dumps(_jsonable(result), indent=2))

"""Fresh-process CAD checks for the v2 rotor.

This is a diagnostic report, not a blanket ``assert no intersections`` test.
Every positive native bounding-box candidate is written to
``verification/neutral-overlaps.csv`` with its exact BREP common volume when
that operation succeeds.  Only the small, named contact pairs below are
classified as intentional; all other positive intersections remain
``unexpected_overlap`` for the parent build to fix.

The folding sweep rotates the saved neutral ``FoldTang`` BREP about its actual
hinge axis.  It therefore avoids rebuilding 145 full documents while still
measuring exact shape gaps at 1 and 0.5 degree samples.  It is a clearance
screen, not a flexible-material or tolerance certification.
"""

import csv
import json
import math
from pathlib import Path

import FreeCAD as A
import Part
import numpy as np

from mechanism import OUT, P, deg, solve, swash_frames, rz
from build import build, components
from geometry import V


TOL_VOLUME = 1e-6
TOL_DISTANCE = 1e-6
FOLD_ANGLES_DEG = (0.0, -72.0, 36.0, -36.0, 72.0)

# Exact pair names only.  No group, prefix, or assembly-wide exemption is
# allowed here: a new overlap must stay visible in the CSV.
INTENTIONAL_CONTACTS = {
    **{(f"Blade{i}PitchLink", f"Blade{i}PitchLowerEye"): "joined_rod_eye" for i in range(1, 6)},
    **{(f"Blade{i}PitchLink", f"Blade{i}PitchUpperEye"): "joined_rod_eye" for i in range(1, 6)},
    **{(f"Servo{i}Rod", f"Servo{i}UpperEye"): "joined_servo_rod_eye" for i in range(3)},
}


def _pair_key(a, b):
    return tuple(sorted((str(a), str(b))))


def _classification(a, b):
    key = _pair_key(a, b)
    if key in {_pair_key(*pair) for pair in INTENTIONAL_CONTACTS}:
        return INTENTIONAL_CONTACTS.get((a, b), INTENTIONAL_CONTACTS.get((b, a)))
    if key in {
        _pair_key("BearingBalls", "LowerRace"),
        _pair_key("BearingBalls", "UpperRace"),
        _pair_key("BearingBalls", "BearingCage"),
    }:
        return "rolling_contact"
    return "unexpected_overlap"


def _vector(value):
    if hasattr(value, "x"):
        return np.array([float(value.x), float(value.y), float(value.z)])
    value = np.asarray(value, dtype=float)
    if value.shape != (3,):
        raise ValueError(f"expected 3-vector, got {value!r}")
    return value


def _bounds(shape):
    bb = shape.BoundBox
    return [float(bb.XMin), float(bb.XMax), float(bb.YMin), float(bb.YMax), float(bb.ZMin), float(bb.ZMax)]


def _bbox_intersection(a, b):
    a = a.BoundBox if hasattr(a, "BoundBox") else a
    b = b.BoundBox if hasattr(b, "BoundBox") else b
    amin = np.array([a.XMin, a.YMin, a.ZMin], dtype=float)
    amax = np.array([a.XMax, a.YMax, a.ZMax], dtype=float)
    bmin = np.array([b.XMin, b.YMin, b.ZMin], dtype=float)
    bmax = np.array([b.XMax, b.YMax, b.ZMax], dtype=float)
    size = np.minimum(amax, bmax) - np.maximum(amin, bmin)
    return size, bool(np.all(size > 0.0))


def _bbox_gap(a, b):
    a = a.BoundBox if hasattr(a, "BoundBox") else a
    b = b.BoundBox if hasattr(b, "BoundBox") else b
    amin = np.array([a.XMin, a.YMin, a.ZMin], dtype=float)
    amax = np.array([a.XMax, a.YMax, a.ZMax], dtype=float)
    bmin = np.array([b.XMin, b.YMin, b.ZMin], dtype=float)
    bmax = np.array([b.XMax, b.YMax, b.ZMax], dtype=float)
    separation = np.maximum(np.maximum(amin - bmax, bmin - amax), 0.0)
    return float(np.linalg.norm(separation))


def _common_volume(a, b):
    try:
        return float(a.common(b).Volume)
    except Exception as exc:  # pragma: no cover - OCC version dependent
        return None, str(exc)


def _distance(a, b):
    try:
        return float(a.distToShape(b)[0])
    except Exception as exc:  # pragma: no cover - OCC version dependent
        return None, str(exc)


def _metric(a, b, exact=True):
    """Return bbox, exact distance and exact common volume for two shapes."""
    intersection, bbox_positive = _bbox_intersection(a, b)
    result = {
        "bbox_intersection_mm": intersection.tolist(),
        "bbox_positive_volume_candidate": bbox_positive,
        "bbox_gap_mm": _bbox_gap(a, b),
    }
    if exact:
        distance = _distance(a, b)
        common = _common_volume(a, b) if bbox_positive else 0.0
        if isinstance(distance, tuple):
            result["distance_error"] = distance[1]
            distance = None
        if isinstance(common, tuple):
            result["common_error"] = common[1]
            common = None
        result["distance_mm"] = distance
        result["common_volume_mm3"] = common
    return result


def _shape_ok(obj):
    shape = obj.Shape
    return bool(not shape.isNull() and shape.isValid() and shape.Volume > 0.0 and len(shape.Solids) > 0)


def _center_of_mass(shape):
    """FreeCAD compounds expose mass on their solids, not CenterOfMass."""
    if hasattr(shape, "CenterOfMass"):
        return _vector(shape.CenterOfMass)
    solids = list(shape.Solids)
    if not solids:
        raise ValueError("shape has no solids for center-of-mass measurement")
    volumes = np.array([float(solid.Volume) for solid in solids])
    centers = np.array([_vector(solid.CenterOfMass) for solid in solids])
    return np.average(centers, axis=0, weights=volumes)


def _snapshot(doc):
    objects = components(doc)
    invalid = [obj.Name for obj in objects if not _shape_ok(obj)]
    shapes = [obj.Shape for obj in objects if _shape_ok(obj)]
    if shapes:
        compound = Part.makeCompound(shapes)
        bounds = _bounds(compound)
        volume = float(compound.Volume)
        solids = int(len(compound.Solids))
    else:
        bounds = None
        volume = 0.0
        solids = 0
    return {
        "parts": len(objects),
        "solids": solids,
        "volume_mm3": volume,
        "bounds_mm": bounds,
        "all_parts_valid": not invalid,
        "invalid_parts": invalid,
    }


def _step_snapshot(path, native_objects=None):
    shape = Part.Shape()
    try:
        loaded = shape.read(str(path))
        loaded = loaded is not False
        solids = list(shape.Solids) if loaded and not shape.isNull() else []
        solid_names = []
        if native_objects is not None:
            for obj in native_objects:
                solid_names.extend([obj.Name] * len(obj.Shape.Solids))
        invalid_details = []
        for index, solid in enumerate(solids):
            if solid.isValid():
                continue
            check_result = ""
            try:
                result = solid.check()
                check_result = repr(result)
            except Exception as exc:  # OCC commonly raises "No error" on valid topology checks.
                check_result = str(exc)
            reasons = ["step_solid_isValid_false"]
            if solid.Volume <= 0.0:
                reasons.append("non_positive_imported_volume")
            if not solid.isClosed():
                reasons.append("open_imported_shell")
            invalid_details.append(
                {
                    "step_solid_index": index,
                    "native_component": solid_names[index] if index < len(solid_names) else None,
                    "volume_mm3": float(solid.Volume),
                    "faces": len(solid.Faces),
                    "closed": bool(solid.isClosed()),
                    "check": check_result,
                    "reason": reasons,
                }
            )
        all_solids_valid = bool(solids) and all(solid.isValid() and solid.Volume > 0.0 for solid in solids)
        valid = bool(loaded and not shape.isNull() and shape.isValid() and all_solids_valid and shape.Volume > 0.0)
        return {
            "read": bool(loaded),
            "valid": valid,
            "shape_valid": bool(shape.isValid()) if loaded and not shape.isNull() else False,
            "all_solids_valid": all_solids_valid,
            "valid_solids": int(sum(solid.isValid() for solid in solids)),
            "solids": int(len(solids)),
            "invalid_solid_details": invalid_details,
            "volume_mm3": float(shape.Volume) if loaded and not shape.isNull() else 0.0,
            "bounds_mm": _bounds(shape) if loaded and not shape.isNull() else None,
        }
    except Exception as exc:  # pragma: no cover - import implementation varies
        return {"read": False, "valid": False, "error": str(exc), "solids": 0, "volume_mm3": 0.0, "bounds_mm": None}


def _compare_snapshots(native, step):
    if not native.get("bounds_mm") or not step.get("bounds_mm"):
        return {"volume_relative_error": None, "max_bound_error_mm": None, "comparable": False}
    nb = np.asarray(native["bounds_mm"], dtype=float)
    sb = np.asarray(step["bounds_mm"], dtype=float)
    denominator = max(abs(float(native["volume_mm3"])), 1.0)
    return {
        "volume_relative_error": float(abs(step["volume_mm3"] - native["volume_mm3"]) / denominator),
        "max_bound_error_mm": float(np.max(abs(sb - nb))),
        "comparable": True,
    }


def _parameters(doc):
    p = doc.getObject("Parameters")
    if p is None:
        return {"collective": 12.0, "cc": 0.0, "cs": 0.0, "psi": 0.0, "flap": [0.0] * 5, "lag": [0.0] * 5}
    return {
        "collective": float(p.Collective),
        "cc": float(p.CyclicCos),
        "cs": float(p.CyclicSin),
        "psi": float(p.Azimuth),
        "fold": float(getattr(p, "FoldFraction", 0.0)),
        "parked": bool(getattr(p, "Parked", False)),
        "flap": [float(v) for v in getattr(p, "FlapAngles", [0.0] * 5)],
        "lag": [float(v) for v in getattr(p, "LagAngles", [0.0] * 5)],
    }


def _pitch_state(doc, stem, p=P):
    params = _parameters(doc)
    output = {"parameters": params, "blades": {}, "max_ball_closure_mm": None, "max_geometry_closure_mm": None, "max_sphere_com_error_mm": None, "max_pitch_error_deg": None}
    ball_errors = []
    geometry_errors = []
    sphere_errors = []
    angle_errors = []
    for index in range(1, 6):
        names = {key: f"Blade{index}{key}" for key in ("PitchLowerBall", "PitchUpperBall", "PitchLowerEye", "PitchUpperEye", "PitchLowerPin", "PitchUpperPin", "PitchLink")}
        missing = [name for name in names.values() if doc.getObject(name) is None]
        if missing:
            output["blades"][str(index)] = {"missing": missing}
            continue
        lower = doc.getObject(names["PitchLowerBall"])
        upper = doc.getObject(names["PitchUpperBall"])
        link = doc.getObject(names["PitchLink"])
        lower_com = _center_of_mass(lower.Shape)
        upper_com = _center_of_mass(upper.Shape)
        measured = float(np.linalg.norm(upper_com - lower_com))
        measured_error = float(abs(measured - p["link_length"]))
        bottom = _vector(getattr(link, "BottomCentre", lower_com))
        top = _vector(getattr(link, "TopCentre", upper_com))
        geometry_error = float(abs(np.linalg.norm(top - bottom) - p["link_length"]))
        lower_error = float(np.linalg.norm(bottom - lower_com))
        upper_error = float(np.linalg.norm(top - upper_com))
        sphere_error = max(lower_error, upper_error)
        ball_errors.append(measured_error)
        geometry_errors.append(geometry_error)
        sphere_errors.append(sphere_error)
        blade = {
            "lower_ball_com_mm": lower_com.tolist(),
            "upper_ball_com_mm": upper_com.tolist(),
            "measured_ball_distance_mm": measured,
            "measured_ball_closure_error_mm": measured_error,
            "geometry_bottom_centre_mm": bottom.tolist(),
            "geometry_top_centre_mm": top.tolist(),
            "geometry_closure_error_mm": geometry_error,
            "geometry_vs_lower_ball_com_mm": lower_error,
            "geometry_vs_upper_ball_com_mm": upper_error,
            "solved_pitch_deg": float(getattr(link, "SolvedPitch", float("nan"))),
        }
        if stem not in {"rotor-folded", "rotor-parked"}:
            angle = params["psi"] + (index - 1) * 72.0
            expected = solve(angle, params["collective"], params["cc"], params["cs"], params["flap"][index - 1], params["lag"][index - 1], p=p, mast_psi=params["psi"])
            blade["expected_pitch_deg"] = float(expected["theta"])
            blade["pitch_error_deg"] = abs(float(getattr(link, "SolvedPitch")) - float(expected["theta"]))
            angle_errors.append(blade["pitch_error_deg"])
        output["blades"][str(index)] = blade
    output["max_ball_closure_mm"] = max(ball_errors) if ball_errors else None
    output["max_geometry_closure_mm"] = max(geometry_errors) if geometry_errors else None
    output["max_sphere_com_error_mm"] = max(sphere_errors) if sphere_errors else None
    output["max_pitch_error_deg"] = max(angle_errors) if angle_errors else None
    return output


def _named_metric(doc, a_name, b_name, exact=True):
    a = doc.getObject(a_name)
    b = doc.getObject(b_name)
    if a is None or b is None:
        return {"part_a": a_name, "part_b": b_name, "missing": [name for name, obj in ((a_name, a), (b_name, b)) if obj is None]}
    result = {"part_a": a_name, "part_b": b_name}
    result.update(_metric(a.Shape, b.Shape, exact=exact))
    return result


def _key_gaps(doc):
    checks = []
    for index in range(1, 6):
        checks.extend(
            [
                (f"Blade{index}PitchLowerBall", f"Blade{index}PitchLowerEye", "spherical_inner_ball_eye"),
                (f"Blade{index}PitchLowerPin", f"Blade{index}PitchLowerBall", "pin_inner_ball"),
                (f"Blade{index}PitchUpperBall", f"Blade{index}PitchUpperEye", "spherical_inner_ball_eye"),
                (f"Blade{index}PitchUpperPin", f"Blade{index}PitchUpperBall", "pin_inner_ball"),
                (f"Blade{index}PitchLink", f"Blade{index}PitchLowerEye", "joined_rod_eye"),
                (f"Blade{index}PitchLink", f"Blade{index}PitchUpperEye", "joined_rod_eye"),
                (f"Blade{index}OuterRace", f"Blade{index}DamperBody", "damper_to_outer_race_clearance"),
                (f"Blade{index}PitchLowerFork", f"Blade{index}PitchLowerPin", "pitch_lower_fork_pin"),
                (f"Blade{index}PitchUpperFork", f"Blade{index}PitchUpperPin", "pitch_upper_fork_pin"),
            ]
        )
    for index in range(3):
        checks.append((f"Servo{index}Rod", f"Servo{index}UpperEye", "joined_servo_rod_eye"))
    checks.extend(
        [
            ("AntirotationArmA", "AntirotationArmB", "scissor_armA_armB_nonoverlap"),
            ("RotatingDriveArmA", "RotatingDriveArmB", "scissor_armA_armB_nonoverlap"),
            ("BearingBalls", "LowerRace", "bearing_ball_lower_race"),
            ("BearingBalls", "UpperRace", "bearing_ball_upper_race"),
            ("BearingBalls", "BearingCage", "bearing_ball_cage"),
            ("Mast", "FixedGuide", "mast_fixed_guide"),
            ("SliderSphere", "FixedGuide", "slider_to_fixed_guide"),
        ]
    )
    output = []
    for a_name, b_name, kind in checks:
        result = _named_metric(doc, a_name, b_name)
        result["check"] = kind
        if "common_volume_mm3" in result:
            result["pass_nonintersection"] = result["common_volume_mm3"] is not None and result["common_volume_mm3"] <= TOL_VOLUME
        if kind.startswith("joined_"):
            result["intentional_join"] = True
            result["joined_contact"] = result.get("distance_mm") is not None and result["distance_mm"] <= TOL_DISTANCE
        output.append(result)
    return output


def _neutral_overlap_rows(doc):
    objects = components(doc)
    rows = []
    for left_index, left in enumerate(objects):
        for right in objects[left_index + 1 :]:
            intersection, bbox_positive = _bbox_intersection(left.Shape, right.Shape)
            if not bbox_positive:
                continue
            volume = _common_volume(left.Shape, right.Shape)
            if isinstance(volume, tuple):
                common = None
                error = volume[1]
            else:
                common = volume
                error = ""
            rows.append(
                {
                    "part_a": left.Name,
                    "part_b": right.Name,
                    "classification": _classification(left.Name, right.Name) if common is None or common > TOL_VOLUME else "bbox_candidate_no_volume_overlap",
                    "bbox_intersection_x_mm": float(intersection[0]),
                    "bbox_intersection_y_mm": float(intersection[1]),
                    "bbox_intersection_z_mm": float(intersection[2]),
                    "common_volume_mm3": common,
                    "common_error": error,
                    "bbox_gap_mm": 0.0,
                }
            )
            if len(rows) % 100 == 0:
                print(json.dumps({"stage": "neutral_overlap_progress", "rows": len(rows)}), flush=True)
    return rows


def _negative_collision(doc):
    a = doc.getObject("Mast")
    b = doc.getObject("FixedGuide")
    if a is None or b is None:
        return {"available": False, "reason": "Mast/FixedGuide missing"}
    for shift in (50.0, 100.0, 150.0, 200.0, 300.0):
        moved = b.Shape.copy()
        moved.translate(V([shift, 0.0, 0.0]))
        common = float(a.Shape.common(moved).Volume)
        if common > 1.0:
            return {
                "available": True,
                "normal_pair": ["Mast", "FixedGuide"],
                "translation_mm": [shift, 0.0, 0.0],
                "overlap_mm3": common,
                "detected": True,
            }
    return {"available": True, "normal_pair": ["Mast", "FixedGuide"], "detected": False}


def _fold_scan(doc):
    params = _parameters(doc)
    output = {}
    csv_rows = []
    for step in (1.0, 0.5):
        minimum_gap = float("inf")
        worst = None
        max_overlap = 0.0
        worst_overlap = None
        samples = 0
        for index, fold_angle in enumerate(FOLD_ANGLES_DEG, start=1):
            moving_objs = {
                "tang": doc.getObject(f"Blade{index}FoldTang"),
                "root_spar": doc.getObject(f"Blade{index}RootSpar"),
                "airfoil": doc.getObject(f"Blade{index}Airfoil"),
            }
            target_objs = {
                "clevis": doc.getObject(f"Blade{index}FoldClevis"),
                "lock": doc.getObject(f"Blade{index}FlightLock"),
                "guide": doc.getObject(f"Blade{index}LockGuide"),
            }
            if any(obj is None for obj in moving_objs.values()) or any(obj is None for obj in target_objs.values()):
                continue
            angle = params["psi"] + (index - 1) * 72.0
            state = solve(angle, params["collective"], params["cc"], params["cs"], params["flap"][index - 1], params["lag"][index - 1], p=P, mast_psi=params["psi"])
            origin = state["pivot"] + state["rotation"] @ np.array([P["fold_pin"] - P["joint"], 0.0, 0.0])
            axis = state["rotation"][:, 2]
            count = int(math.ceil(abs(fold_angle) / step))
            for sample in range(count + 1):
                signed_angle = math.copysign(min(abs(fold_angle), sample * step), fold_angle)
                for moving_name, moving_obj in moving_objs.items():
                    moved = moving_obj.Shape.copy()
                    if abs(signed_angle) > 0.0:
                        moved.rotate(V(origin), V(axis), signed_angle)
                    for target_name, target_obj in target_objs.items():
                        intersection, bbox_positive = _bbox_intersection(moved, target_obj.Shape)
                        bbox_gap = _bbox_gap(moved, target_obj.Shape)
                        if bbox_positive:
                            # A positive AABB intersection is the only case
                            # that needs an exact boolean for overlap volume.
                            common = _common_volume(moved, target_obj.Shape)
                            overlap = common if not isinstance(common, tuple) else 0.0
                            gap = 0.0
                        elif np.isfinite(minimum_gap) and bbox_gap >= minimum_gap:
                            # AABB gap is a rigorous lower bound on BREP gap.
                            gap = bbox_gap
                            overlap = 0.0
                        else:
                            gap_value = _distance(moved, target_obj.Shape)
                            gap = gap_value if not isinstance(gap_value, tuple) else bbox_gap
                            overlap = 0.0
                        if gap is not None and gap < minimum_gap:
                            minimum_gap = gap
                            worst = [index, moving_name, target_name, float(signed_angle)]
                        if overlap > max_overlap:
                            max_overlap = overlap
                            worst_overlap = [index, moving_name, target_name, float(signed_angle)]
                        csv_rows.append([step, index, moving_name, target_name, signed_angle, gap, overlap])
                samples += 1
        output[str(step)] = {
            "step_deg": step,
            "samples": samples,
            "minimum_gap_mm": None if not np.isfinite(minimum_gap) else float(minimum_gap),
            "worst_gap": worst,
            "maximum_overlap_mm3": float(max_overlap),
            "worst_overlap": worst_overlap,
            "sequence": [2, 5, 3, 4],
            "method": "parked FoldTang/RootSpar/Airfoil BREPs rotated about solved hinge axis; FoldClevis/withdrawn FlightLock/LockGuide held at parked positions",
        }
        print(json.dumps({"stage": "fold", "step_deg": step, **output[str(step)]}), flush=True)
    return output, csv_rows


def _variant_checks():
    variants = {}
    for label, params in (
        ("link_plus_2mm", {**P, "link_length": P["link_length"] + 2.0}),
        ("horn_x_plus_5mm", {**P, "horn_x": P["horn_x"] + 5.0}),
    ):
        doc = None
        try:
            doc, _ = build("CADCheck_" + label, 16.0, 3.0, 0.0, p=params)
            snap = _snapshot(doc)
            pitch = _pitch_state(doc, "variant", params)
            link = doc.getObject("Blade1PitchLink")
            expected = solve(0.0, 16.0, 3.0, 0.0, p=params, mast_psi=0.0)
            lower = _vector(link.BottomCentre)
            upper = _vector(link.TopCentre)
            variants[label] = {
                "parts": snap["parts"],
                "all_parts_valid": snap["all_parts_valid"],
                "measured_link_length_mm": float(np.linalg.norm(upper - lower)),
                "target_link_length_mm": float(params["link_length"]),
                "link_length_error_mm": float(abs(np.linalg.norm(upper - lower) - params["link_length"])),
                "bottom_vs_solver_mm": float(np.linalg.norm(lower - expected["bottom"])),
                "top_vs_solver_mm": float(np.linalg.norm(upper - expected["top"])),
                "pitch": pitch,
            }
        except Exception as exc:
            variants[label] = {"error": str(exc)}
        finally:
            if doc is not None:
                A.closeDocument(doc.Name)
    return variants


def _servo_checks(doc):
    """Check the three fixed servo endpoints and pin-axis orientation."""
    output = []
    for index, angle in enumerate((30.0, 150.0, 270.0)):
        lower = doc.getObject(f"Servo{index}LowerBall")
        upper = doc.getObject(f"Servo{index}UpperBall")
        if lower is None or upper is None:
            output.append({"index": index, "missing": True})
            continue
        er = rz(deg(angle))[:, 0]
        tangent = rz(deg(angle))[:, 1]
        expected_lower = er * 710.0 + np.array([0.0, 0.0, 150.0])
        # The upper eye is attached to the solved stator frame, not the pitch link.
        frames = swash_frames(_parameters(doc)["collective"], _parameters(doc)["cc"], _parameters(doc)["cs"], _parameters(doc)["psi"], P)
        expected_upper = frames["center"] + frames["stator_rotation"] @ (er * 710.0 + np.array([0.0, 0.0, -190.0]))
        lower_com = _center_of_mass(lower.Shape)
        upper_com = _center_of_mass(upper.Shape)
        measured_stroke = float(np.linalg.norm(upper_com - lower_com))
        expected_stroke = float(np.linalg.norm(expected_upper - expected_lower))
        output.append(
            {
                "index": index,
                "lower_endpoint_error_mm": float(np.linalg.norm(lower_com - expected_lower)),
                "upper_endpoint_error_mm": float(np.linalg.norm(upper_com - expected_upper)),
                "measured_stroke_mm": measured_stroke,
                "expected_stroke_mm": expected_stroke,
                "stroke_error_mm": abs(measured_stroke - expected_stroke),
                "tangent_axis": tangent.tolist(),
                "lower_ball_com_mm": lower_com.tolist(),
                "upper_ball_com_mm": upper_com.tolist(),
            }
        )
    return output


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def main():
    results = {
        "method": "fresh native FCStd reopen and independent STEP import; exact OCC BREP checks where available",
        "tolerances": {"common_volume_mm3": TOL_VOLUME, "distance_mm": TOL_DISTANCE},
        "limitations": [
            "STEP comparison is aggregate volume/bounds, not a semantic part-name comparison.",
            "Fold sweep rigidly rotates parked FoldTang/RootSpar/Airfoil; it does not model flex, tolerance stack-up, or a rebuilt shape at every angle.",
            "A zero BREP gap can mean tangent contact; supplier fits, bolt preload, bearing load rating, and material strength remain outside this script.",
            "All positive native bounding-box candidates are reported; only exact named pairs are classified intentional.",
        ],
        "states": {},
    }
    neutral_doc = None
    overlap_rows = []
    fold_rows = []
    try:
        for stem in ("rotor-neutral", "rotor-max-input", "rotor-folded", "rotor-parked"):
            print(json.dumps({"stage": "state_start", "state": stem}), flush=True)
            native_path = OUT / f"{stem}.FCStd"
            step_path = OUT / f"{stem}.step"
            state = {"available": native_path.exists() and step_path.exists(), "native_path": str(native_path), "step_path": str(step_path)}
            if not state["available"]:
                state["missing"] = [str(path) for path in (native_path, step_path) if not path.exists()]
                results["states"][stem] = state
                continue
            doc = A.openDocument(str(native_path))
            try:
                doc.recompute()
                native = _snapshot(doc)
                native_objects = components(doc)
                step = _step_snapshot(step_path, native_objects)
                state.update({"native": native, "step": step, "native_step_compare": _compare_snapshots(native, step), "pitch": _pitch_state(doc, stem), "key_gaps": _key_gaps(doc), "servo_endpoints": _servo_checks(doc)})
                if stem == "rotor-neutral":
                    neutral_doc = doc
                    overlap_rows = _neutral_overlap_rows(doc)
                    print(json.dumps({"stage": "neutral_overlaps", "rows": len(overlap_rows)}), flush=True)
                    state["negative_collision"] = _negative_collision(doc)
                results["states"][stem] = state
                print(json.dumps({"stage": "state_done", "state": stem, "parts": native["parts"], "step_valid": step["valid"]}), flush=True)
            finally:
                if stem != "rotor-neutral":
                    A.closeDocument(doc.Name)
        parked_path = OUT / "rotor-parked.FCStd"
        if parked_path.exists():
            print(json.dumps({"stage": "fold_start", "source": str(parked_path)}), flush=True)
            parked_doc = A.openDocument(str(parked_path))
            try:
                parked_doc.recompute()
                fold_results, fold_rows = _fold_scan(parked_doc)
                results["folding_clearance"] = fold_results
            finally:
                A.closeDocument(parked_doc.Name)
        else:
            results["folding_clearance"] = {"available": False, "missing": str(parked_path)}
        results["parameter_variants"] = _variant_checks()
        print(json.dumps({"stage": "variants_done", "variants": list(results["parameter_variants"])}), flush=True)
    finally:
        if neutral_doc is not None:
            A.closeDocument(neutral_doc.Name)

    verification = OUT / "verification"
    verification.mkdir(parents=True, exist_ok=True)
    (verification / "cad-checks.json").write_text(json.dumps(_jsonable(results), indent=2), encoding="utf-8")
    with (verification / "neutral-overlaps.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["part_a", "part_b", "classification", "bbox_intersection_x_mm", "bbox_intersection_y_mm", "bbox_intersection_z_mm", "common_volume_mm3", "common_error", "bbox_gap_mm"])
        for row in overlap_rows:
            writer.writerow([row.get(key) for key in ("part_a", "part_b", "classification", "bbox_intersection_x_mm", "bbox_intersection_y_mm", "bbox_intersection_z_mm", "common_volume_mm3", "common_error", "bbox_gap_mm")])
    with (verification / "folding-clearance.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["step_deg", "blade", "moving_part", "target", "fold_angle_deg", "distance_mm", "overlap_mm3"])
        writer.writerows(fold_rows)
    print(json.dumps(_jsonable(results), indent=2), flush=True)
    return results


if __name__ == "__main__":
    main()

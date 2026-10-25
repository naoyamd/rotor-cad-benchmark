"""Generate compact JS-port fixtures from the original C2 mechanism module."""

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "results" / "C2" / "src"))
import mechanism  # noqa: E402


CASES = [
    {
        "name": "neutral",
        "collective": 12.0,
        "cc": 0.0,
        "cs": 0.0,
        "psi": 0.0,
        "flap": 0.0,
        "lag": 0.0,
        "mastPsi": 0.0,
    },
    {
        "name": "phase-wrap",
        "collective": 15.5,
        "cc": 1.75,
        "cs": -1.4,
        "psi": 359.999,
        "flap": 4.0,
        "lag": -2.25,
        "mastPsi": 17.5,
    },
]

BLADES = {
    "collective": 14.25,
    "cc": -1.6,
    "cs": 1.8,
    "flap": -2.0,
    "lag": 2.5,
    "mastPsi": 31.25,
}


def jsonable(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {key: jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(item) for item in value]
    return value.item() if hasattr(value, "item") else value


def frame_fixture(frames):
    return {
        "center": frames["center"],
        "q": [item for row in frames["q"] for item in row],
        "stator_rotation": [item for row in frames["stator_rotation"] for item in row],
        "rotor_rotation": [item for row in frames["rotor_rotation"] for item in row],
        "stator_scissors": {
            key: frames["stator_scissors"][key]
            for key in ("base", "elbow", "end", "base_axis", "elbow_axis", "radial_basis", "z_basis")
        },
        "rotor_scissors": {
            key: frames["rotor_scissors"][key]
            for key in ("base", "elbow", "end", "base_axis", "elbow_axis", "radial_basis", "z_basis")
        },
    }


def solve_fixture(state):
    result = {key: state[key] for key in ("theta", "top", "bottom", "pivot", "residual")}
    result["rotation"] = [item for row in state["rotation"] for item in row]
    return result


def make_case(case):
    frames = mechanism.swash_frames(case["collective"], case["cc"], case["cs"], case["mastPsi"])
    state = mechanism.solve(
        case["psi"],
        case["collective"],
        case["cc"],
        case["cs"],
        case["flap"],
        case["lag"],
        mast_psi=case["mastPsi"],
    )
    return {"input": case, "frames": frame_fixture(frames), "solve": solve_fixture(state)}


def main():
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("fixtures") / "c2-kinematics.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    output = {
        "source": "results/C2/src/mechanism.py",
        "cases": [make_case(case) for case in CASES],
        "five_blades": {
            "input": BLADES,
            "solves": [
                solve_fixture(
                    mechanism.solve(
                        BLADES["mastPsi"] + index * 72.0,
                        BLADES["collective"],
                        BLADES["cc"],
                        BLADES["cs"],
                        BLADES["flap"],
                        BLADES["lag"],
                        mast_psi=BLADES["mastPsi"],
                    )
                )
                for index in range(5)
            ],
        },
    }
    target.write_text(json.dumps(jsonable(output), indent=2) + "\n", encoding="utf-8")
    print(target)


if __name__ == "__main__":
    main()

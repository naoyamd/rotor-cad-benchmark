import sys
import unittest
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechanism as m  # noqa: E402


class MechanismTests(unittest.TestCase):
    def test_scissors_are_planar_and_use_tangent_axes(self):
        for psi in (0.0, 37.0, 180.0, 359.0):
            frames = m.swash_frames(12.0, 3.0, -2.0, psi)
            stator = frames["stator_scissors"]
            rotor = frames["rotor_scissors"]
            tangent = m.rz(m.deg(psi)) @ np.array([0.0, 1.0, 0.0])
            self.assertLess(frames["stator_plane_residual_mm"], 1e-6)
            self.assertLess(frames["rotor_plane_residual_mm"], 1e-6)
            self.assertLess(stator["closure_residual_mm"], 1e-6)
            self.assertLess(rotor["closure_residual_mm"], 1e-6)
            np.testing.assert_allclose(stator["axes"], np.tile(m._Y, (2, 1)))
            np.testing.assert_allclose(rotor["axes"], np.tile(tangent, (2, 1)))
            self.assertEqual(stator["arm_lengths_mm"], [550.0, 510.0])
            self.assertEqual(rotor["arm_lengths_mm"], [550.0, 510.0])
            self.assertGreater(rotor["min_radial_projection_mm"], 0.0)
            self.assertGreater(rotor["min_radius_mm"], 0.0)

    def test_global_blade_phase_uses_mast_offset(self):
        aligned = m.solve(42.0, 12.0, 0.0, 0.0, mast_psi=42.0)
        global_phase = m.solve(42.0, 12.0, 0.0, 0.0, mast_psi=0.0)
        self.assertEqual(aligned["blade_offset_deg"], 0.0)
        self.assertEqual(global_phase["blade_offset_deg"], 42.0)
        # With zero cyclic both phasing choices describe the same physical
        # blade point; only the frame factorization differs.
        np.testing.assert_allclose(aligned["bottom"], global_phase["bottom"], atol=1e-6)

    def test_actuator_closure_and_legacy_keys(self):
        state = m.solve(117.0, 16.0, -2.0, 1.5, flap=3.0, lag=-2.0)
        for key in ("theta", "top", "bottom", "pivot", "rotation", "residual", "lever"):
            self.assertIn(key, state)
        self.assertLess(state["residual"], 1e-6)
        self.assertLess(abs(state["actuator"]["length_error_mm"]), 1e-6)
        self.assertTrue(state["actuator"]["valid"])
        np.testing.assert_allclose(state["pitch_pin_axis"], m.rz(m.deg(117.0)) @ m._X)
        np.testing.assert_allclose(state["servo_pin_axis"], m.rz(m.deg(117.0)) @ m._Y)

    def test_interlocks_validate_fold_states(self):
        self.assertTrue(m.flight_allowed(210.0, [True] * 5, False, False))
        self.assertFalse(m.flight_allowed(0.0, [True] * 5, False, False))
        self.assertFalse(m.flight_allowed(210.0, [True, False, True, True, True], False, False))
        self.assertTrue(m.fold_state_checks("folded", 0.0, [True] * 5, True, False))
        self.assertFalse(m.fold_state_checks("folded", 1.0, [True] * 5, True, False))
        with self.assertRaises(ValueError):
            m.flight_allowed(210.0, [True] * 4, False, False)


if __name__ == "__main__":
    unittest.main()

"""Regression checks for the nonlinear C172 model."""
import ast
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "04_nonlinear_6dof"))
import solve as model


def trim_state(speed, branch=0):
    alphas, elevators, throttles = model.find_trim(speed, branch)
    for alpha, elevator, throttle in zip(alphas, elevators, throttles):
        if 0 <= throttle <= 1:
            state = [speed*model.KNOT*math.cos(alpha), 0, speed*model.KNOT*math.sin(alpha),
                     0, 0, 0, 0, alpha, 0, 0, 0, 0]
            return state, elevator, throttle
    raise RuntimeError("No feasible trim in test setup.")


def run_test_steps(state, elevator, throttle, seconds, dt, pulse=False):
    # The same explicit forward-Euler operation as the simulation.
    state = state[:]
    stalled = model.stall_state(math.atan2(state[2], state[0]), 0)
    history = [(0, state[:], stalled)]
    for step in range(round(seconds/dt)):
        time = step*dt
        active_elevator = elevator
        if pulse and 2 <= time < 3:
            active_elevator -= 1
        rates = model.calculate_rates(
            *state[:9], state[11], throttle, active_elevator, 0, 0, stalled)
        state = [value + dt*rate for value, rate in zip(state, rates)]
        stalled = model.stall_state(math.atan2(state[2], state[0]), stalled)
        history.append(((step+1)*dt, state[:], stalled))
    return history


class NonlinearModelChecks(unittest.TestCase):
    def test_lift_tables_match_independent_transcription(self):
        # Independent transcription of the reference document, page 7.
        values = {
            'ALPHA_RAD': [-0.09, 0.0, 0.09, 0.1, 0.12, 0.14, 0.16, 0.17, 0.19, 0.21, 0.24, 0.26, 0.28, 0.3, 0.32, 0.34, 0.36],
            'CL_ATTACHED': [-0.22, 0.25, 0.73, 0.83, 0.92, 1.02, 1.08, 1.13, 1.19, 1.25, 1.35, 1.44, 1.47, 1.43, 1.38, 1.3, 1.15],
            'CL_STALLED': [-0.22, 0.25, 0.73, 0.78, 0.79, 0.81, 0.82, 0.83, 0.85, 0.86, 0.88, 0.9, 0.92, 0.95, 0.99, 1.05, 1.15],
        }
        self.assertEqual(model.ALPHA, values['ALPHA_RAD'])
        self.assertEqual(model.CL_ATTACHED, values['CL_ATTACHED'])
        self.assertEqual(model.CL_STALLED, values['CL_STALLED'])

    def test_propeller_table_matches_original_exactly(self):
        source = ast.parse((ROOT/'01_simple_6dof/simple_c172_6dof.py').read_text())
        for node in source.body:
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                if node.targets[0].id == 'PROP_CT_TABLE':
                    reference = ast.literal_eval(node.value)
        self.assertEqual(list(zip(model.PROP_J, model.PROP_CT)), reference)

    def test_hysteresis_thresholds_and_memory(self):
        self.assertEqual(model.stall_state(.36, 0), 0)
        self.assertEqual(model.stall_state(.360001, 0), 1)
        self.assertEqual(model.stall_state(.09, 1), 1)
        self.assertEqual(model.stall_state(.089999, 1), 0)
        self.assertEqual(model.stall_state(.28, 0), 0)
        for alpha in (.32, .15, .25):
            self.assertEqual(model.stall_state(alpha, 1), 1)

    def test_prescribed_stall_cycle_switches_both_ways(self):
        stalled = 0
        changes = []
        for i in range(1645):
            fraction = i/822 if i <= 822 else (1644-i)/822
            alpha = -.05 + .411*fraction
            previous = stalled
            stalled = model.stall_state(alpha, stalled)
            if stalled != previous:
                changes.append((alpha, stalled))
        self.assertEqual([state for alpha, state in changes], [1, 0])
        self.assertGreater(changes[0][0], .36)
        self.assertLess(changes[1][0], .09)

    def test_post_peak_trim_is_retained(self):
        alphas, elevators, throttles = model.find_trim(55)
        self.assertEqual(len(alphas), 2)
        self.assertLess(alphas[0], .28)
        self.assertGreater(alphas[1], .28)
        self.assertTrue(all(0 <= throttle <= 1 for throttle in throttles))

    def test_all_feasible_trims_balance_the_6dof_equations(self):
        for speed in range(40, 126, 5):
            for branch in (0, 1):
                alphas, elevators, throttles = model.find_trim(speed, branch)
                for alpha, elevator, throttle in zip(alphas, elevators, throttles):
                    if not 0 <= throttle <= 1:
                        continue
                    error, _, _ = model.trim_balance(speed, alpha, branch)
                    rates = model.calculate_rates(
                        speed*model.KNOT*math.cos(alpha), 0, speed*model.KNOT*math.sin(alpha),
                        0, 0, 0, 0, alpha, 0, 0, throttle, elevator, 0, 0, branch)
                    self.assertLess(abs(error), 1e-6)
                    self.assertLess(max(abs(rate) for rate in rates[:9]), 1e-8)
                    self.assertLess(abs(rates[11]), 1e-10)

    def test_unavailable_and_overpowered_trims(self):
        self.assertEqual(model.find_trim(40), ([], [], []))
        alphas, elevators, throttles = model.find_trim(110)
        self.assertTrue(alphas)
        self.assertTrue(all(throttle > 1 for throttle in throttles))

    def test_unforced_trim_does_not_drift(self):
        state, elevator, throttle = trim_state(100)
        history = run_test_steps(state, elevator, throttle, 5, .001)
        self.assertLess(max(abs(a-b) for a, b in zip(state[:9], history[-1][1][:9])), 1e-8)
        self.assertLess(abs(history[-1][1][11]), 1e-8)

    def test_euler_response_converges_as_step_is_halved(self):
        state, elevator, throttle = trim_state(100)
        final = []
        for dt in (.002, .001, .0005):
            final.append(run_test_steps(state, elevator, throttle, 5, dt, True)[-1][1])
        coarse = max(abs(a-b) for a, b in zip(final[0], final[1]))
        fine = max(abs(a-b) for a, b in zip(final[1], final[2]))
        self.assertLess(fine, .6*coarse)
        self.assertLess(fine, .01)

    def test_stall_entry_and_recovery_in_6dof_time_steps(self):
        state, elevator, throttle = trim_state(100)
        speed = 60*model.KNOT
        state[0], state[2], state[7], state[4] = speed*math.cos(.34), speed*math.sin(.34), .34, .8
        history = run_test_steps(state, elevator, throttle, 3, .001)
        changes = []
        for i in range(1, len(history)):
            if history[i][2] != history[i-1][2]:
                changes.append(history[i])
        self.assertEqual([row[2] for row in changes], [1, 0])
        self.assertGreater(math.atan2(changes[0][1][2], changes[0][1][0]), .36)
        self.assertLess(math.atan2(changes[1][1][2], changes[1][1][0]), .09)

    def test_reference_cg_matches_saved_source_configuration(self):
        root = ET.parse(ROOT/'04_nonlinear_6dof/reference/c172p.xml').getroot()
        cg = root.find("mass_balance/location[@name='CG']")
        self.assertEqual(cg.attrib['unit'], 'IN')
        for actual, axis in zip((model.CG_X, model.CG_Y, model.CG_Z), ('x', 'y', 'z')):
            self.assertAlmostEqual(actual, float(cg.findtext(axis))*.0254)
        # The PDF coefficients belong to its own reference, not the upstream RP.
        self.assertEqual((model.AERO_X, model.AERO_Y, model.AERO_Z),
                         (43.8*.0254, 0, 58.7*.0254))

    def test_cg_transfer_matches_independent_cross_product_in_all_axes(self):
        # Moving the CG must leave aerodynamic forces unchanged and change
        # angular accelerations by (old_CG - new_CG) cross F, divided by inertia.
        state, elevator, throttle = trim_state(100)
        state[1] = 1.2
        state[3:6] = [.03, -.04, .02]
        state[6] = .05
        args = (*state[:9], state[11], throttle, elevator, 2, -3, 0)
        rates = model.calculate_rates(*args)
        u,v,w,p,q,r,roll,pitch,yaw = state[:9]
        speed = math.sqrt(u*u+v*v+w*w)
        # Recover aerodynamic force from Newton's equations, independently
        # of the aerodynamic force-to-moment implementation being checked.
        fx = model.MASS*(rates[0]-r*v+q*w+model.G*math.sin(pitch))
        fx -= model.thrust(throttle, speed, 1.225)
        fy = model.MASS*(rates[1]-p*w+r*u-model.G*math.sin(roll)*math.cos(pitch))
        fz = model.MASS*(rates[2]-q*u+p*v-model.G*math.cos(roll)*math.cos(pitch))
        # Structural CG displacement (x aft, y right, z up), in metres.
        dx,dy,dz = .12,-.08,.09
        with patch.multiple(model, CG_X=model.CG_X+dx, CG_Y=model.CG_Y+dy, CG_Z=model.CG_Z+dz):
            moved = model.calculate_rates(*args)
        # Arm change in body coordinates is (dx, -dy, dz).
        expected = ((-dy*fz-dz*fy)/model.IXX,
                    (dz*fx-dx*fz)/model.IYY,
                    (dx*fy+dy*fx)/model.IZZ)
        for i in range(3):
            self.assertAlmostEqual(moved[i], rates[i], delta=1e-12)
            self.assertAlmostEqual(moved[i+3]-rates[i+3], expected[i], delta=1e-12)

    def test_trim_changes_when_reference_and_cg_are_coincident(self):
        corrected = model.find_trim(100)[1][0]
        with patch.multiple(model, AERO_X=model.CG_X, AERO_Y=model.CG_Y, AERO_Z=model.CG_Z):
            coincident = model.find_trim(100)[1][0]
        self.assertGreater(abs(corrected-coincident), .1)


if __name__ == '__main__':
    unittest.main(verbosity=2)

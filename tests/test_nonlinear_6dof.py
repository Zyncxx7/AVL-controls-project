"""Numerical and state-memory regression checks; run directly or with unittest."""
import ast
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'04_nonlinear_6dof'))
import aerodynamics as aero
import model
from solve import hysteresis_cycle


class NonlinearModelChecks(unittest.TestCase):
    def test_matches_preexisting_pdf_transcription(self):
        # The user's earlier, independent table check is preserved unchanged.
        source=ast.parse((ROOT/'scripts/test_jsbsim_stall_table.py').read_text())
        values={node.targets[0].id:ast.literal_eval(node.value) for node in source.body
                if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name)
                and node.targets[0].id in ('ALPHA_RAD','CL_ATTACHED','CL_STALLED')}
        self.assertEqual(aero.ALPHA,values['ALPHA_RAD'])
        self.assertEqual(aero.CL_ATTACHED,values['CL_ATTACHED'])
        self.assertEqual(aero.CL_STALLED,values['CL_STALLED'])

    def test_branch_memory_and_strict_thresholds(self):
        self.assertEqual(aero.update_hysteresis(0.36,0),0)
        self.assertEqual(aero.update_hysteresis(0.360001,0),1)
        for alpha in (0.32,0.15,0.25,0.09):
            self.assertEqual(aero.update_hysteresis(alpha,1),1)
        self.assertEqual(aero.update_hysteresis(0.089999,1),0)
        self.assertEqual(aero.update_hysteresis(0.28,0),0)
        self.assertNotEqual(aero.wing_lift(0.2,0),aero.wing_lift(0.2,1))

    def test_complete_cycle_switches_once_each_way(self):
        rows=hysteresis_cycle()
        transitions=[r for i,r in enumerate(rows) if i and r['hysteresis']!=rows[i-1]['hysteresis']]
        self.assertEqual(len(transitions),2)
        self.assertGreater(transitions[0]['alpha_rad'],0.36)
        self.assertLess(transitions[1]['alpha_rad'],0.09)
        self.assertEqual(rows[-1]['hysteresis'],0)
        increasing=next(r for r in rows if r['direction']=='increasing' and abs(r['alpha_rad']-0.2)<0.0003)
        decreasing=next(r for r in rows if r['direction']=='decreasing' and abs(r['alpha_rad']-0.2)<0.0003)
        self.assertGreater(increasing['CL_wing'],decreasing['CL_wing'])

    def test_multiple_trim_roots_are_retained(self):
        roots=model.trim_solutions(55,0)
        self.assertEqual(len(roots),2)
        self.assertLess(roots[0]['alpha_rad'],0.28)
        self.assertGreater(roots[1]['alpha_rad'],0.28)
        self.assertTrue(all(r['feasible'] for r in roots))

    def test_all_feasible_trim_roots_balance_equations(self):
        for speed in range(40,126,5):
            for branch in (0,1):
                for root in model.trim_solutions(speed,branch):
                    if root['feasible']:
                        self.assertLess(abs(root['force_residual_N']),1e-6)
                        rates=model.derivatives(root['state'],root['controls'],branch)
                        self.assertLess(max(abs(x) for x in rates[:9]),1e-8)
                        self.assertLess(abs(rates[11]),1e-10)

    def test_infeasible_thrust_and_missing_trim_are_not_accepted(self):
        self.assertEqual(model.trim_solutions(40),[])
        roots=model.trim_solutions(110)
        self.assertTrue(roots)
        self.assertTrue(all(not r['feasible'] and r['throttle']>1 for r in roots))

    def test_unforced_trim_does_not_drift(self):
        root=next(r for r in model.trim_solutions(100) if r['feasible'])
        history=model.simulate(root['controls'],root['state'],seconds=10)
        self.assertLess(max(abs(a-b) for a,b in zip(history[-1][1][:9],root['state'][:9])),1e-8)
        self.assertLess(abs(history[-1][1][11]),1e-8)

    def test_implicit_alpha_rate_consistency(self):
        root=next(r for r in model.trim_solutions(80) if r['feasible'])
        state=root['state'][:]
        state[4]=0.08
        first=model.derivatives(state,root['controls'],0)
        u,_,w=state[:3]
        alpha_dot=(u*first[2]-w*first[0])/(u*u+w*w)
        load,_=model.forces(state,root['controls'],0,alpha_dot)
        independent=model.rigid_body_derivatives(state,load)
        self.assertLess(max(abs(a-b) for a,b in zip(first,independent)),1e-10)

    def test_pulse_response_converges_with_timestep(self):
        root=next(r for r in model.trim_solutions(100) if r['feasible'])
        states=[model.simulate(root['controls'],root['state'],seconds=5,dt=dt,
                               pulse=model.original.PULSE)[-1][1] for dt in (0.02,0.01,0.005)]
        coarse=max(abs(a-b) for a,b in zip(states[0],states[1]))
        fine=max(abs(a-b) for a,b in zip(states[1],states[2]))
        self.assertLess(fine,coarse/2)
        self.assertLess(fine,1e-4)

    def test_hysteresis_is_used_in_free_6dof_time_stepping(self):
        # An entry/recovery probe starting below the switching threshold.
        # This is a regression probe,
        # not a claimed trim, stall-entry manoeuvre or spin solution.
        root=next(r for r in model.trim_solutions(100) if r['feasible'])
        state=root['state'][:]
        speed=60*model.KNOT
        state[0],state[2],state[7]=speed*math.cos(0.34),speed*math.sin(0.34),0.34
        state[4]=0.6
        history=model.simulate(root['controls'],state,seconds=3,dt=0.005)
        self.assertEqual(history[0][2],0)
        transitions=[(s,h) for i,(_,s,h) in enumerate(history) if i and h!=history[i-1][2]]
        self.assertEqual([h for _,h in transitions],[1,0])
        self.assertGreater(math.atan2(transitions[0][0][2],transitions[0][0][0]),0.36)
        self.assertLess(math.atan2(transitions[1][0][2],transitions[1][0][0]),0.09)

    def test_original_100_kt_trim_is_unchanged(self):
        controls,state=model.original.trim_level_flight('DATCOM',100)
        self.assertAlmostEqual(state[7]/aero.DEG,2.49,delta=0.01)
        self.assertAlmostEqual(controls['elevator'],-1.38,delta=0.01)


if __name__=='__main__':
    unittest.main(verbosity=2)

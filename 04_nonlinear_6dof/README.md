# Nonlinear C172P trim and stall hysteresis

This implements the two immediate requests in the 31 October email thread:
compare **trim elevator deflection versus speed** for the original and supplied
models, and incorporate the stall loop for **increasing and decreasing alpha**.
The spin solution was explicitly left for discussion after these two steps.

## Where to start reading

Start at `main()` at the bottom of `solve.py`. It has four numbered steps:
trim comparison, stall loop, time response, and output. Settings such as
timestep, starting speed and elevator pulse are at the top of that file.

Then follow these functions in order:

1. `aerodynamics.coefficients()` calculates each of the six coefficient axes.
   Each force/moment contribution has its own descriptive variable.
2. `model.forces()` converts the coefficients into forces and moments.
3. `model.rigid_body_derivatives()` writes the 12 equations one at a time.
4. `model.simulate()` applies the controls and advances the state in time.
5. `model.trim_at_alpha()` and `model.trim_solutions()` find level-flight trim.

`plots.py` contains the drawing code; it is separate from the aircraft model.
The code uses ordinary functions, lists, dictionaries and loops. RK4 has its
own short function, and the coupled trim/alpha-rate equations are explained
next to their calculations. All force and moment derivatives are retained.

## Run and review

From the project root:

```powershell
python -m pip install -r 04_nonlinear_6dof/requirements.txt
python 04_nonlinear_6dof/solve.py
python tests/test_nonlinear_6dof.py
```

The numerical model uses Python's standard library. ReportLab produces the
four-page plot report. Poppler, when available, renders the PNG previews.
`--no-plots` runs the numerical work without either plotting dependency.

| File | Purpose |
| --- | --- |
| `aerodynamics.py` | Clean-wing tables and all six aerodynamic coefficient axes from the supplied PDF |
| `model.py` | Coupled trim equations, 12-state rigid-body dynamics and RK4 time stepping |
| `solve.py` | Original/new trim comparison, prescribed stall cycle and free-flight response |
| `plots.py` | Report and plot generation |
| `reference/aerodynamics.pdf` | Supplied 14-page parameter document |
| `results/research_results.pdf` | Four-page comparison plots |
| `results/trim_comparison.csv` | All trim roots, parameters, residuals and unavailable/infeasible cases |
| `results/stall_cycle.csv` | Increasing/decreasing alpha, persistent stall state, CL and endpoint-holding flags |
| `results/time_response.csv` | All 12 states versus time for the three model cases |
| `results/summary.json` | Settings, results, source hash and modelling assumptions |

## Trim comparison

The original DATCOM and AVL-table models are imported unchanged from
`01_simple_6dof/simple_c172_6dof.py`. The new model uses the PDF's geometry,
clean CL/CD tables, pitch terms and elevator effectiveness. At each speed,
the solver balances longitudinal force, vertical force and pitching moment
with zero body rates and level flight. Pitch equals alpha; airspeed and forward
body-axis speed are exported separately.

The new elevator/thrust balance includes elevator drag and the tail propwash
pressure. All bracketed roots within the lift-table range are retained. This
includes post-peak equilibria and both admissible stall-memory states. The
CSV distinguishes these cases; a mathematical equilibrium does not establish
stability. Throttle outside 0-1 is reported as infeasible, and unavailable roots
are recorded rather than replaced with zeros. No unspecified actuator limits
are imposed. Gaps in plotted curves correspond to unavailable/infeasible cases.

The speed sweep is a set of equilibrium calculations, not an aircraft
deceleration manoeuvre. Both memory states are evaluated explicitly. Selecting
the stalled branch solely because speed is increasing would be incorrect.

## Stateful stall loop and time integration

The lift table on PDF page 7 has two branches. The limits on page 3 are
interpreted using the actual JSBSim `FGAerodynamics` state update:

- alpha > 0.36 rad: set stalled state to 1;
- alpha < 0.09 rad: reset state to 0;
- otherwise: retain the previous state, including at the exact thresholds.

The CL maximum at 0.28 rad is a separate characteristic of the attached
table. It is not substituted for the 0.36-rad memory-switch threshold.
Reversing alpha inside the band does not immediately reset the stall state.

`stall_cycle.csv` prescribes alpha from -0.05 to 0.361 rad and back at 0.02 s
sampling. The 0.001-rad overshoot is needed to exercise the strict upper
switch. Linear interpolation with endpoint holding is used as in JSBSim.
Samples beyond each table's endpoints are flagged; no unsupported curve is
extrapolated. This is an imposed-alpha diagnostic, not a free-flight response.

The **same memory update is used inside the 6DOF integrator**. Its history
belongs to accepted states; trial RK stages do not mutate global memory.
Alpha-dot lift and pitch terms are included, with the implicit alpha-dot
feedback solved from the translational equations rather than using a stale
finite difference. A separate free-flight comparison starts each model at
its own 100 kt trim, then applies -1 deg additional elevator from 2 to 3 s.
Duration is 15 s with dt = 0.01 s. The original Euler integrator is preserved;
the new model uses RK4. These are actual integrated time histories.

## Common conditions and explicit approximations

The PDF is an aerodynamic specification, not a complete loading/engine model.
For a controlled comparison, mass (1043 kg), inertias (1285, 1825, 2667 kg m2),
atmosphere and the original fixed-RPM/throttle-scaled propeller model are
retained. New reference geometry is 174 ft2, 35.8 ft and 4.9 ft, converted to SI.
The comparison uses zero flaps and free-air aerodynamics. Sea-level density
does not mean the aircraft is being modelled on the runway. Ground-effect and
flap tables are outside this clean-flight comparison; nonzero flap input is
rejected instead of silently using clean coefficients.

Tail pressure uses the PDF formula, with induced velocity estimated by ideal
actuator-disk momentum theory from the common thrust model. It is not a full
JSBSim propeller simulation. The aerodynamic reference and common CG are
co-located because the PDF does not provide a loading-dependent CG. Therefore
this is a PDF-parameter comparison within the existing 6DOF framework, not an
exact reproduction of a fully loaded JSBSim aircraft. An exact JSBSim replay
would also require its loading, CG, engine, propeller and control configuration.

The attachment's wind/body force formulas on page 13 are internally
inconsistent at nonzero beta. The implementation uses the orthogonal standard
wind-to-body rotation. This has no effect on the beta = 0 trim comparisons.
The original body alpha definition is retained; the PDF's wing incidence is
not added a second time to tables already indexed by body alpha.

The PDF describes a post-stall roll-damping sign reversal but supplies no
numerical replacement for Clp = -0.484. The code does not invent one and does
not claim a spin solution. Euler-angle singularity and forward-flight guards
remain active. A spin analysis is a separate follow-up.

## Verification and sources

Regression checks cover independent agreement with the user's prior table
transcription, threshold equality and memory, a complete up/down loop,
multiple trim roots, force/moment balance, unavailable thrust, unforced trim
drift, the implicit alpha-rate equation, timestep convergence, unchanged
original trim, and stall recovery in the actual 6DOF time integrator.

The supplied PDF is the coefficient source; its SHA256 is recorded in the
summary. Its parameter values are retained even where current upstream
`c172p.xml` differs. Only the generic switching and interpolation semantics
are checked against upstream JSBSim:

- [FGAerodynamics.cpp](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGAerodynamics.cpp), `Run`, stall-hysteresis update.
- [FGTable.cpp](https://github.com/JSBSim-Team/jsbsim/blob/master/src/math/FGTable.cpp), endpoint interpolation.
- [c172p.xml](https://github.com/JSBSim-Team/jsbsim/blob/master/aircraft/c172p/c172p.xml), upstream reference, not a substitute for the attachment.

Thread: [Update on MIT AVL integration and results](https://mail.google.com/mail/u/?authuser=ae23b037%40smail.iitm.ac.in#all/1a1932982886ebb8).

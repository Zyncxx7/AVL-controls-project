# C172 nonlinear model: one script

The new aircraft model is in **`solve.py`**. It uses ordinary numbers, table
lists and an explicit forward-Euler time loop. There are no model classes,
state dictionaries, nested result dictionaries or custom plotting framework.
The original file is imported only for the reference comparison curves.

## Run

From the project root:

```powershell
python -m pip install -r 04_nonlinear_6dof/requirements.txt
python 04_nonlinear_6dof/solve.py
```

Add `--no-plots` to write the numerical CSVs without loading Matplotlib.

## Where to start reading

1. Settings and PDF tables are at the top of `solve.py`.
2. `interpolate()` reads between table rows; `thrust()` is the original propeller calculation.
3. `stall_state()` is the three-line branch-memory rule.
4. `trim_balance()` calculates elevator/throttle at a trial alpha; `find_trim()` halves intervals to find force balance.
5. `calculate_rates()` contains all six aerodynamic coefficients and all 12 aircraft equations, in one place.
6. `main()` finds the starting trim, sets the aircraft variables, runs the time loop,
   compares trim across speeds, and plots the results, in that order.

In the time loop, `u`, `v`, `w`, `p`, `q`, `r`, `roll`, `pitch`, `yaw`,
`north`, `east` and `down` are separate numerical variables. Each update is
written explicitly, for example `u = u + DT*u_dot`. Lists accumulate values
for plotting. The default `DT = 0.001` s is smaller than the earlier RK4
version's 0.01 s because forward Euler is less accurate at the same timestep.
Tests check convergence when this timestep is halved.

## Outputs

- `results/trim_comparison.csv`: original DATCOM/AVL and PDF-model equilibria, including infeasible cases.
- `results/time_response.csv`: all 12 states of the new model versus time.
- `results/research_results.pdf`: three pages: trim elevator, trim alpha/throttle, and time response.
- The same three figures are also saved as PNGs.

The time-response figure also runs the original models for comparison.
It starts each model at its own 100 kt trim and applies an additional -1 deg
elevator from 2 to 3 seconds. All three now use the same forward-Euler timestep.

## Physical assumptions

The source is `reference/aerodynamics.pdf`. Its zero-flap lift/drag tables
and force/moment derivatives are used directly. The alpha-rate lift and
pitch terms, lateral derivatives and propwash effects are retained.
No JSBSim or live AVL process is called.

Mass, inertias, atmosphere and propeller are retained from the original model.
The PDF does not provide a complete loading/engine configuration. Free-air
ground-effect factors are one; the aerodynamic reference and CG are assumed
coincident. Propwash is estimated by an actuator-disk formula. These choices
mean this is a PDF-parameter comparison, not an exact loaded JSBSim replay.

Stall memory sets above 0.36 rad, resets below 0.09 rad, and otherwise holds.
The 0.28-rad lift maximum is not substituted for the switching threshold.
This rule follows [FGAerodynamics.cpp](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGAerodynamics.cpp).
Tables hold their endpoint values, as in JSBSim's table lookup.

There is no separate imposed-alpha experiment in `main()`. Stall memory is
still updated from the actual aircraft alpha during the time loop. The
time-response plot comes from the 6DOF integration. Trim roots are equilibria, not proof of
stability; roots requiring more than full throttle are not plotted as feasible.
The PDF gives no numerical post-stall roll-damping replacement, so none is
invented. Spin remains deferred as Ranjith requested.

## Checks

```powershell
python tests/test_nonlinear_6dof.py
```

The tests are separate from the script. They check tables, all-axis equations,
trim balance, multiple roots, stall entry/recovery, trim drift and timestep
convergence. The original model and the two standalone AVL folders are unchanged.

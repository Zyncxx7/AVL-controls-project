# Nonlinear C172 6DOF model

`solve.py` computes level-flight trim and integrates the aircraft response to an
elevator pulse. The baseline DATCOM and AVL-table models from stage 1 provide
comparison curves. No JSBSim or AVL process is called during this analysis.

## Run

Commands run from the project root:

```powershell
python -m pip install -r 04_nonlinear_6dof/requirements.txt
python 04_nonlinear_6dof/solve.py
```

The `--no-plots` option produces numerical results without loading Matplotlib.

## Model and numerical method

Aircraft settings, coefficient tables and derivatives are defined at the top
of `solve.py`. `trim_balance()` balances pitch moment and forward force at a
trial alpha. `find_trim()` searches for vertical-force balance using a scan of
901 points augmented by table breakpoints, followed by 50 bisection iterations.
All bracketed roots within the source table range are retained. A root requires
throttle between zero and one to be classified as feasible; this classification
does not establish stability or check elevator travel limits.

`calculate_rates()` evaluates aerodynamic forces, moments and the 12 state
rates. `main()` initializes trim, advances the state using forward Euler,
records the time history and computes the trim-speed comparison. The state is
`[u, v, w, p, q, r, roll, pitch, yaw, north, east, down]`. Body axes are forward,
right and down. Angles are radians internally; control inputs are degrees.

| Parameter | Value |
| --- | --- |
| Mass | 1043 kg |
| Principal inertias Ixx, Iyy, Izz | 1285, 1825, 2667 kg m2 |
| Wing area, span, chord | 174 ft2, 35.8 ft, 4.9 ft |
| Trim sweep | 40-125 kt, 1 kt spacing, sea-level density |
| Trim condition | Straight, level, zero bank/sideslip/angular rates; neutral aileron and rudder |
| Initial response | First feasible attached-state trim at 100 kt |
| Elevator input | Additional -1 deg from 2 to 3 s; trim throttle held fixed |
| Duration and timestep | 15 s; 0.001 s |
| Initial position and heading | Zero in local North-East-Down coordinates |

Each comparison model starts at its own trim and uses the same pulse and timestep.

## Aerodynamic data and CG

The coefficient tables, geometry and force/moment derivatives are transcribed
from `reference/aerodynamics.pdf`. Source provenance is recorded in
[reference/SOURCES.md](reference/SOURCES.md).

The fixed reference CG is (41, 0, 36.5) inches, taken from the saved JSBSim
`reference/c172p.xml` mass-balance block. This is the empty-aircraft reference
CG. The retained comparison mass and inertias do not represent either the XML
empty-aircraft or default loaded configuration. Pilot and fuel loading are not
used to compute a loaded CG.

The aerodynamic reference is (43.8, 0, 58.7) inches, as specified on page 2 of
the aerodynamic reference document. Structural axes are x aft, y right, z up.
The CG-to-reference arm in body axes is (-0.07112, 0, -0.56388) m. Trim and
all-axis dynamics use `moment_CG = moment_reference + arm cross force`.

The saved XML has a different aerodynamic reference, (43.2, 0, 59.4) inches.
The document's reference remains paired with its aerodynamic coefficients.
Combining the external reference CG with those coefficients defines a fixed-CG
comparison configuration, not a reconstruction of a verified aircraft loading.
AVL-stage coordinates use their own datum and are separate from this model.

## Assumptions and limitations

- Mass, diagonal inertias and CG are constant. Products of inertia, fuel burn,
  moving loads and structural flexibility are omitted.
- The baseline 75-inch propeller uses a fixed 2400 rpm and linear throttle
  scaling. Thrust acts along body x through the CG. Engine dynamics, torque,
  gyroscopic effects and P-factor are omitted. Thrust is zero outside the
  advance-ratio table; negative tabulated thrust coefficients remain possible.
- Propwash and wake corrections are disabled. The listed force/moment
  derivatives, including alpha-rate terms, are retained.
- Clean, zero-flap, free-air aerodynamics are used. Ground-effect multipliers
  equal one. No additional Mach or Reynolds-number corrections are applied.
- Density follows `1.225*exp(-max(altitude, 0)/8500)` kg/m3. Gravity is
  9.81 m/s2; the Earth is flat and nonrotating. Wind, gusts and ground contact
  are omitted. Initial zero altitude is a coordinate/density datum.
- Tables use linear interpolation and endpoint holding. Different tables
  cover different alpha ranges; a lift-table state can already use held drag
  or lateral-rate coefficients. Table bounds are not flight-envelope limits.
- The lift table uses body alpha without a separate 1.5 deg incidence offset.
  Side force is treated in wind axes with an orthogonal wind-to-body rotation;
  the reference document's force-frame descriptions require clarification.
- Alpha-rate lift feedback is solved algebraically using same-step
  accelerations. No filtered or previous-step alpha-rate estimate is used.
- Stall memory sets above 0.36 rad, resets below 0.09 rad and holds between
  these values, following [JSBSim stall-state logic](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGAerodynamics.cpp).
  The state initially is attached. Branch switches are instantaneous; no
  separation time constant is modelled. Trim branches are independently seeded
  equilibria and do not establish a reachable acceleration/deceleration path.
- The attached-branch search extends beyond its 0.28 rad lift peak to the table
  endpoint. No numerical post-stall roll-damping replacement is supplied, so
  the attached-flow damping value remains. Spin behaviour is not established.
- The trim scan identifies bracketed roots; tangent or closely spaced roots
  can be missed. Root tolerances are 1e-8 N for endpoint balance and 1e-7 rad
  for deduplication. No stability classification is performed.
- Euler-angle integration stops below 1 m/s, at nonpositive forward velocity,
  or when `abs(cos(pitch)) < 0.1`. Fixed timesteps must divide the pulse edges
  and duration. Timestep convergence checks numerical accuracy, not aerodynamic
  validity.
- The baseline comparison includes its own 0.029 AVL parasite-drag correction,
  substituted rate/control derivatives and `Cm0 + Cm_alpha(alpha)*alpha`
  construction. Its interpolation/extrapolation differs from the nonlinear
  tables. Agreement with the baseline is not experimental validation.

## Outputs and checks

- `results/trim_comparison.csv`: all trim roots, feasibility and unavailable cases.
- `results/time_response.csv`: the 12 integrated states and stall memory.
- `results/research_results.pdf`: trim elevator, trim alpha/throttle and time response.
- PNG copies of the three report figures.

```powershell
python tests/test_nonlinear_6dof.py
```

Checks cover source-table transcription, the retained propeller table, all-axis
CG transfer, feasible trim balance, multiple roots, unforced trim drift, stall
entry/recovery and timestep convergence. Stall memory evolves from the aircraft
state in the time loop; no separate imposed-alpha plotting experiment is run.

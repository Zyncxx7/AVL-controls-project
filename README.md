# AVL controls project

C172 flight-dynamics modelling and standalone aerodynamic calculations with
Athena Vortex Lattice (AVL).

| Stage | Folder | Analysis |
| --- | --- | --- |
| 1 | [01_simple_6dof](01_simple_6dof/) | Baseline C172 6DOF model using DATCOM and AVL coefficient tables |
| 2 | [02_c172_live_avl](02_c172_live_avl/) | Standalone C172 AVL solutions and interpolation comparison |
| 3 | [03_naca0012_live_avl](03_naca0012_live_avl/) | NACA 0012 finite-wing solutions and experimental lift comparison |
| 4 | [04_nonlinear_6dof](04_nonlinear_6dof/) | Nonlinear C172 trim, stall memory and time-domain response |

## Running the analyses

Python and Matplotlib are required. The bundled AVL executables run on Windows.
Commands below run from the project root.

```powershell
python -m pip install -r 01_simple_6dof/requirements.txt
python 01_simple_6dof/simple_c172_6dof.py
python 02_c172_live_avl/solve.py
python 03_naca0012_live_avl/solve.py
python -m pip install -r 04_nonlinear_6dof/requirements.txt
python 04_nonlinear_6dof/solve.py
```

Each stage contains its code, requirements and documentation. Stages 2 and 3
call AVL for each prescribed operating point; they do not integrate the aircraft
motion. Stages 1 and 4 integrate the 6DOF equations using tabulated aerodynamics.

## Geometry and mass inputs

The standalone AVL stages use separate `.avl` and `.mass` files under `inputs/`.
The programs load these files directly. NACA 0012 aspect ratios have separate
subfolders with a fixed mesh of 8 chordwise panels and 48 panels per half-span.

Single-point calculations:

```powershell
python 02_c172_live_avl/run_avl.py --alpha 5
python 03_naca0012_live_avl/run_avl.py --alpha 4
```

Both runners accept `--geometry`, `--mass`, `--alpha` and `--output`.
Geometry and mass pairs share a folder. Each calculation records its AVL
commands, console output, forces and stability derivatives.

## Results

- [Baseline C172 report](output/reports/C172_plot_summary.pdf): trim and control-pulse response.
- [Nonlinear C172 report](04_nonlinear_6dof/results/research_results.pdf): trim elevator, trim alpha/throttle and elevator-pulse response.
- Stages 2 and 3: coefficient CSVs, comparison figures and numerical summaries under `results/`.
- Stage 4: trim solutions and time histories under `results/`.

The nonlinear model uses clean-configuration C172P aerodynamic tables and
force/moment derivatives. Propwash and wake corrections are disabled.
Aerodynamic moments are transferred to a fixed reference CG. Loading,
reference-point and aerodynamic limitations are documented in the
[model description](04_nonlinear_6dof/README.md). Spin analysis is outside the
current scope.

## Verification

```powershell
python tests/verify_avl.py
python tests/test_nonlinear_6dof.py
```

The AVL check compares both wrappers with independent console sessions at
11 operating points, including raw force/derivative values and input hashes.
The [verification summary](output/verification/summary.json) is retained;
per-case logs are generated during the check. The nonlinear tests cover source
data, CG moment transfer, trim balance, stall switching and timestep convergence.

These checks establish implementation consistency, not flight-test validation.
AVL's inviscid model does not reproduce viscous drag or stall. Source data,
software attribution and stage-specific limitations are documented in each
stage README. AVL is third-party GPL software.

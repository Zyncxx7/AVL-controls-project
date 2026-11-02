# C172 standalone AVL program

This stage performs standalone aerodynamic calculations. The workflow is:

1. `solve.py` defines the angle sweep and comparisons.
2. `inputs/cessna.avl` and `inputs/cessna.mass` define geometry and mass properties.
3. `run_avl.py` sends commands to the external AVL executable.

## Run

Commands run from this folder:

```powershell
python -m pip install -r requirements.txt
python solve.py
```

For one point instead of the whole loop:

```powershell
python run_avl.py --alpha 5
```

The supplied `avl.exe` is Windows AVL 3.52. Other operating systems
require a compatible binary configured through `AVL_EXE` in `run_avl.py`.

## Configuration

`START_ANGLE_DEG`, `END_ANGLE_DEG` and `ANGLE_STEP_DEG` define the sweep.
The default sweep is -2.5 to 10 degrees in 0.25-degree increments. The four
original angles are always included so reference reproduction can be checked.

Geometry, mass, CG and inertia settings are in the separate files under `inputs/`.
Neither `solve.py` nor `run_avl.py` builds or overwrites those inputs.
The mass file is loaded explicitly. Its values are illustrative rather than
measured mass properties. The original geometry has no elevator
CONTROL definitions. This program therefore does not claim elevator derivatives.

## Outputs and comparison

- `inputs/cessna.avl` and `inputs/cessna.mass`: supplied editable AVL input files.
- `results/comparison.png`: six direct-solve versus interpolation plots.
- `results/interpolation_error.png`: CL, CD and Cm interpolation errors.
- `results/coefficients.csv`: all numerical results.
- `results/summary.json`: error measures and original-point check.
- `results/cases/`: the commands, console log and raw output of every solve.

The `REFERENCE_RESULTS` table in `solve.py` was transcribed from the four C172 AVL stability
runs at Mach 0.16. It tests reproducibility, not accuracy against flight tests.
No external reference CSV is needed. Runtime inputs are `avl.exe`, `inputs/cessna.avl` and `inputs/cessna.mass`.
Alternative geometry and mass pairs can be selected with `--geometry` and
`--mass`; both files must share a folder.
CD is the raw AVL drag, without the baseline model's 0.029 parasite correction.
Cm is the total moment at each angle; Cm_alpha is a derivative, not another
moment contribution to add to that total.

The loop prescribes angles, not flight states over time. No trim or time
integration is performed in this version. Plots save to PNG without popup windows.

AVL is third-party GPL software. The unmodified executable comes from
https://web.mit.edu/drela/Public/web/avl/avl352.exe, with corresponding source at
https://web.mit.edu/drela/Public/web/avl/avl3.52.tgz. The User Guide and licence
information are available at https://web.mit.edu/drela/Public/web/avl/.

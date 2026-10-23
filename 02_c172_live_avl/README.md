# C172 standalone AVL program

This folder is independent of the older flight model. Its input files and two Python scripts
are intended to be read in this order:

1. `solve.py`: start at `main()` and follow its numbered steps.
2. `inputs/cessna.avl` and `inputs/cessna.mass`: edit geometry and mass directly.
3. `run_avl.py`: see the exact commands sent to the external AVL executable.

## Run

In a terminal inside this folder, using your Python installation:

```powershell
python -m pip install -r requirements.txt
python solve.py
```

For one point instead of the whole loop:

```powershell
python run_avl.py --alpha 5
```

The supplied `avl.exe` is Windows AVL 3.52. On another OS, use the appropriate
AVL binary and change `AVL_EXE` at the top of `run_avl.py`.

## What to change

Edit `START_ANGLE_DEG`, `END_ANGLE_DEG` and `ANGLE_STEP_DEG` at the top of `solve.py`.
The default sweep is -2.5 to 10 degrees in 0.25-degree increments. The four
original angles are always included so reference reproduction can be checked.

Geometry, mass, CG and inertia settings are in the separate files under `inputs/`.
Neither `solve.py` nor `run_avl.py` builds or overwrites those inputs.
The mass file is actually loaded, but its values are teaching-model assumptions,
not a measured mass distribution. The original geometry has no elevator
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
To use another existing pair, run `python run_avl.py --geometry path/to/model.avl
--mass path/to/model.mass --alpha 5` on one line. Keep the pair in one folder.
CD is the raw AVL drag, without the older model's 0.029 parasite correction.
Cm is the total moment at each angle; Cm_alpha is a derivative, not another
moment contribution to add to that total.

The loop prescribes angles, not flight states over time. No trim or time
integration is performed in this version. Plots save to PNG without popup windows.

AVL is third-party GPL software. The unmodified executable comes from
https://web.mit.edu/drela/Public/web/avl/avl352.exe, with corresponding source at
https://web.mit.edu/drela/Public/web/avl/avl3.52.tgz. The User Guide and licence
information are available at https://web.mit.edu/drela/Public/web/avl/.

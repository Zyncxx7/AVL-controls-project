# NACA 0012 standalone AVL program

This folder is independent of the older flight model. Read `solve.py`, starting
at its `main()` function. It reads the separate geometry and mass files in
`inputs/` and calls `run_avl.py` for each operating point. No inputs are built
or overwritten during a run.

## Run

In a terminal inside this folder, using your Python installation:

```powershell
python -m pip install -r requirements.txt
python solve.py
```

For just one point, run `python run_avl.py --alpha 4`. Use `--geometry` and
`--mass` to select another existing pair in the same folder.
The supplied `avl.exe` is the Windows executable. On another OS, replace it
with a compatible binary and update `AVL_EXE` in `run_avl.py`.

## What the loop does

AVL solves a finite wing, not an isolated 2D airfoil. The supplied input files describe NACA
0012 rectangular wings at aspect ratios 10, 100 and 1000. The highest aspect
ratio approximates a 2D section. Each wing is solved over the requested angles
and the eight exact experimental angles. The numbers are compared with the
80 grit Ladson data at Mach 0.15 and Reynolds number 6 million.

Settings are visible at the top of `solve.py`: `ASPECT_RATIOS`, `ANGLES`,
and `INPUT_FOLDER`. Edit panel counts directly in each `.avl` file.
Keep AR 1000 for the near-2D comparison. The supplied mass file contains an artificial 1 kg mass at the quarter chord,
with illustrative positive inertias. This is not measured airfoil mass data.
It preserves the moment reference while showing how a mass file is loaded.

All wings use one fixed mesh: 8 chordwise panels and 48 panels per half-span.
No mesh comparison is run. Zero lift at zero alpha, near-zero symmetric-section
moment and the thin-airfoil lift-slope limit provide separate sanity checks.

## Outputs

- `inputs/`: editable .avl and .mass files for the aspect ratios.
- `results/comparison.png`: lift, lift error, moment and drag-component plots.
- `results/coefficients.csv`: all operating points.
- `results/experiment_comparison.csv`: exact-angle experimental comparisons.
- `results/summary.json`: lift errors, fitted slopes and theory checks.
- `results/cases/`: exact AVL commands, console logs and raw outputs.

These are uncalibrated inviscid calculations. AVL does not reproduce the
experimental viscous/profile drag, transition or stall here. Reynolds number
belongs to the reference experiment. NACA 0012 supplies the mean camber line;
AVL does not resolve its physical thickness. The plotted induced and measured
total drag are deliberately identified as different components.

This loop sweeps angles; it does not integrate motion through time.

## Reference and executable sources

The `EXPERIMENTAL_RESULTS` table inside `solve.py` contains eight rows from the 80 grit dataset linked by the NASA
Turbulence Modeling Resource:
https://tmbwg.github.io/turbmodels/NACA0012_validation/CLCD_Ladson_expdata.dat

No external reference-data file is needed. Matplotlib is the only third-party
Python package, and `avl.exe` is the required external solver.

Experimental case description:
https://tmbwg.github.io/turbmodels/naca0012_val.html

AVL is third-party GPL software. The included unmodified Windows executable
is from https://web.mit.edu/drela/Public/web/avl/avl352.exe. Its corresponding
source is https://web.mit.edu/drela/Public/web/avl/avl3.52.tgz; see the licence
and User Guide at https://web.mit.edu/drela/Public/web/avl/.

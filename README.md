# AVL controls project

The project is organized into three stages. Each stage can run independently.

| Stage | Folder | Purpose |
| --- | --- | --- |
| 1 | [01_simple_6dof](01_simple_6dof/) | C172 six degree of freedom model using tabulated aerodynamic coefficients |
| 2 | [02_c172_live_avl](02_c172_live_avl/) | Fresh C172 AVL solutions at prescribed angles of attack |
| 3 | [03_naca0012_live_avl](03_naca0012_live_avl/) | NACA 0012 wing calculations and experimental lift comparison |

The standalone solvers call the bundled AVL executable for every operating
point. They sweep angles of attack; they do not yet update the 6DOF simulation
with live aerodynamic coefficients.

## Running the stages

Python and Matplotlib are required. The bundled AVL executables are for Windows.
From this directory:

```powershell
python -m pip install -r 01_simple_6dof/requirements.txt
python 01_simple_6dof/simple_c172_6dof.py
python 02_c172_live_avl/solve.py
python 03_naca0012_live_avl/solve.py
```

Each stage has its own README, requirements and results. Stage 1 opens plot
windows; stages 2 and 3 save their plots to their respective `results/` folders.

## Separate AVL inputs

Geometry and mass files are supplied under each solver's `inputs/` directory.
Edit these files directly. No input builder is needed or called. Airfoil aspect
ratios have separate subfolders, all using 8 chordwise panels and 48 panels per
half-span. No mesh comparison is run.

For a single operating point:

```powershell
python 02_c172_live_avl/run_avl.py --alpha 5
python 03_naca0012_live_avl/run_avl.py --alpha 4
```

Both runners accept `--geometry`, `--mass`, `--alpha` and `--output`. Keep each
geometry/mass pair in the same folder. New runs save their commands, console
output, force report and stability derivatives for inspection.

## Verification and report

[C172 plot summary](output/reports/C172_plot_summary.pdf) contains the plots
from the earlier flight-model work, with simulation setup and parameter captions.
The associated 6DOF plots are in `01_simple_6dof/output/plots/`.

```powershell
python tests/verify_avl.py
```

This compares both Python wrappers against separate AVL console sessions using
explicit `LOAD` and `MASS` commands. It checks 11 operating points, all eight
returned fields, every number in the force and derivative reports, and input
file hashes. The summary and paired raw outputs are in
[output/verification](output/verification/).

This establishes wrapper consistency with the same AVL executable, rather than
experimental accuracy. C172 comparison data are earlier AVL results. The
airfoil study compares lift with published measurements; AVL's inviscid model
does not reproduce viscous drag or stall. The supplied mass properties are
illustrative, rather than measured aircraft data.

AVL is third-party GPL software. Solver READMEs identify its source and the
comparison data sources.

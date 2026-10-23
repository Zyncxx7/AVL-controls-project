# Stage 1: C172 six degree of freedom model

`simple_c172_6dof.py` is a self-contained model with tabulated DATCOM and AVL
aerodynamics. It computes level-flight trim, integrates a control-pulse
response and plots trim elevator against forward speed. It does not call AVL.

```powershell
python -m pip install -r requirements.txt
python simple_c172_6dof.py
```

Run these commands from this folder. Change `DATASET` at the bottom of the
script to select DATCOM or AVL. Aircraft properties, coefficient tables and
control settings are in the same file. Units are SI; angular conventions and
assumptions are documented alongside the code.

The state is `[u, v, w, p, q, r, roll, pitch, yaw, north, east, down]`.
Body axes are forward, right and down. Positive elevator is trailing edge down.

Plots are saved under `output/plots/`. The earlier PDF report is retained at
`../output/reports/C172_plot_summary.pdf`. Numerical checks establish model
consistency, rather than validation against aircraft flight measurements.

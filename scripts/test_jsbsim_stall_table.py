"""Plot and verify the C172P lift table transcribed from aerodynamics.pdf.

This is deliberately a coefficient-table test, not a full spin simulation.
The PDF supplies attached and stalled CL branches, but it does not supply a
numerical post-stall Clp after stating that roll damping reverses sign.
"""
from pathlib import Path
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_FOLDER = Path(__file__).resolve().parents[1]
OUTPUT_FOLDER = PROJECT_FOLDER / "output" / "verification" / "jsbsim_stall"

# Page 7, "Wing-Body Lift vs. Alpha & Stall Hysteresis (CLwbh)".
ALPHA_RAD = [-0.09, 0.00, 0.09, 0.10, 0.12, 0.14, 0.16, 0.17,
             0.19, 0.21, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36]
CL_ATTACHED = [-0.22, 0.25, 0.73, 0.83, 0.92, 1.02, 1.08, 1.13,
               1.19, 1.25, 1.35, 1.44, 1.47, 1.43, 1.38, 1.30, 1.15]
CL_STALLED = [-0.22, 0.25, 0.73, 0.78, 0.79, 0.81, 0.82, 0.83,
              0.85, 0.86, 0.88, 0.90, 0.92, 0.95, 0.99, 1.05, 1.15]
RAD_TO_DEG = 180.0 / 3.141592653589793


def interpolate(x, x_values, y_values):
    """Linear interpolation inside the supplied table; never extrapolate."""
    if x < x_values[0] or x > x_values[-1]:
        raise ValueError("Angle is outside the PDF table range.")
    for index in range(len(x_values) - 1):
        left = x_values[index]
        right = x_values[index + 1]
        if left <= x <= right:
            fraction = (x - left) / (right - left)
            return y_values[index] + fraction * (y_values[index + 1] - y_values[index])
    return y_values[-1]


def main():
    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    peak_index = max(range(len(CL_ATTACHED)), key=lambda i: CL_ATTACHED[i])
    peak_alpha_rad = ALPHA_RAD[peak_index]
    peak_cl = CL_ATTACHED[peak_index]
    post_stall_drop = peak_cl - CL_ATTACHED[-1]

    # Evaluate a fine grid only within the documented table bounds.
    sample_count = 451
    alpha_samples = [
        ALPHA_RAD[0] + index * (ALPHA_RAD[-1] - ALPHA_RAD[0]) / (sample_count - 1)
        for index in range(sample_count)
    ]
    alpha_deg = [alpha * RAD_TO_DEG for alpha in alpha_samples]
    attached_samples = [interpolate(alpha, ALPHA_RAD, CL_ATTACHED) for alpha in alpha_samples]
    stalled_samples = [interpolate(alpha, ALPHA_RAD, CL_STALLED) for alpha in alpha_samples]

    figure, axis = plt.subplots(figsize=(9, 5.5))
    axis.plot(alpha_deg, attached_samples, label="Attached-flow branch")
    axis.plot(alpha_deg, stalled_samples, label="Stalled-flow branch")
    axis.scatter([alpha * RAD_TO_DEG for alpha in ALPHA_RAD], CL_ATTACHED, s=18)
    axis.scatter([alpha * RAD_TO_DEG for alpha in ALPHA_RAD], CL_STALLED, s=18)
    axis.axvline(peak_alpha_rad * RAD_TO_DEG, color="black", linestyle="--",
                 label=f"CLmax at {peak_alpha_rad * RAD_TO_DEG:.2f} deg")
    axis.set_xlabel("Angle of attack (deg)")
    axis.set_ylabel("Wing-body lift coefficient CLwbh")
    axis.set_title("C172P JSBSim lift table from aerodynamics.pdf")
    axis.grid(alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(OUTPUT_FOLDER / "stall_table.png", dpi=180)
    plt.close(figure)

    report = {
        "verified_from_pdf": {
            "table_points": len(ALPHA_RAD),
            "alpha_range_rad": [ALPHA_RAD[0], ALPHA_RAD[-1]],
            "alpha_range_deg": [ALPHA_RAD[0] * RAD_TO_DEG, ALPHA_RAD[-1] * RAD_TO_DEG],
            "clean_CLmax": peak_cl,
            "CLmax_alpha_rad": peak_alpha_rad,
            "CLmax_alpha_deg": peak_alpha_rad * RAD_TO_DEG,
            "attached_CL_at_20_63_deg": CL_ATTACHED[-1],
            "post_stall_CL_drop": post_stall_drop,
            "nonlinear_lift_table": True,
            "post_stall_decrease_present": post_stall_drop > 0,
        },
        "assumptions": [
            "Straight-line interpolation is used only between the PDF's tabulated points.",
            "Both documented hysteresis branches are plotted; no switching law is invented.",
        ],
        "not_yet_claimed": [
            "No full spin trajectory is claimed from this coefficient-only test.",
            "The PDF says post-stall Clp becomes positive but gives no numerical post-stall value.",
            "Mass, CG and inertia values needed by the 6-DOF equations are not listed in this PDF.",
        ],
        "specific_input_needed": (
            "Use the original JSBSim c172p.xml (and its mass/propulsion files) to obtain the exact "
            "hysteresis function, mass properties and any omitted post-stall lateral derivatives."
        ),
    }
    (OUTPUT_FOLDER / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Saved {OUTPUT_FOLDER / 'stall_table.png'}")


if __name__ == "__main__":
    main()

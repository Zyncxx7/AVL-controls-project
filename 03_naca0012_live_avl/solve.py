"""Solve NACA 0012 test wings and compare with measured airfoil lift.

Start reading at main(), then follow the function calls. No old project
modules are imported. This is an angle sweep, not a time simulation.
"""
from math import pi, sqrt
from pathlib import Path
import csv
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from run_avl import run_avl


PROGRAM_FOLDER = Path(__file__).resolve().parent
ASPECT_RATIOS = [10, 100, 1000]  # AR 1000 is the approximate 2D case.
ANGLES = list(range(-4, 11))     # -4, -3, ..., 10 degrees
INPUT_FOLDER = PROGRAM_FOLDER / "inputs"  # Edit the .avl/.mass files directly.
MACH = 0.15  # Theory comparison only; keep consistent with the input files.
DEBUG = True  # Print AVL commands, full output and parsed coefficients for each case.

# Eight published Ladson measurements: Mach 0.15, Re 6 million, 80 grit trip.
# These are measured values, not fitted to AVL. No external CSV is needed.
EXPERIMENTAL_RESULTS = [
    {
        "alpha_deg": -4.04,
        "CL": -0.4417,
        "CD": 0.00871,
    },
    {
        "alpha_deg": -2.14,
        "CL": -0.2385,
        "CD": 0.00800,
    },
    {
        "alpha_deg": -0.05,
        "CL": -0.0126,
        "CD": 0.00809,
    },
    {
        "alpha_deg": 2.05,
        "CL": 0.2125,
        "CD": 0.00816,
    },
    {
        "alpha_deg": 4.04,
        "CL": 0.4316,
        "CD": 0.00823,
    },
    {
        "alpha_deg": 6.09,
        "CL": 0.6546,
        "CD": 0.00885,
    },
    {
        "alpha_deg": 8.30,
        "CL": 0.8873,
        "CD": 0.01050,
    },
    {
        "alpha_deg": 10.12,
        "CL": 1.0707,
        "CD": 0.01201,
    },
]


def fit_lift_slope(angles_deg, lift_coefficients):
    """Slope of the best-fit straight line, including a fitted intercept."""
    mean_angle = sum(angles_deg) / len(angles_deg)
    mean_lift = sum(lift_coefficients) / len(lift_coefficients)
    numerator = 0.0
    denominator = 0.0
    for index in range(len(angles_deg)):
        angle_deg = angles_deg[index]
        lift_coefficient = lift_coefficients[index]
        angle_difference = angle_deg - mean_angle
        numerator += angle_difference * (lift_coefficient - mean_lift)
        denominator += angle_difference ** 2
    return numerator / denominator


def make_plots(results_by_aspect_ratio, comparison_results, reference, results_folder):
    """Separate lift agreement from the drag physics missing in AVL."""
    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    for aspect_ratio, rows in results_by_aspect_ratio.items():
        angles = []
        lift_values = []
        moment_values = []
        for row in rows:
            angles.append(row["alpha_deg"])
            lift_values.append(row["CL"])
            moment_values.append(row["Cm"])
        axes[0, 0].plot(angles, lift_values, label=f"AVL AR {aspect_ratio}")
        axes[1, 0].plot(angles, moment_values, label=f"AVL AR {aspect_ratio}")

    reference_angles = []
    reference_lift = []
    reference_drag = []
    for row in reference:
        reference_angles.append(row["alpha_deg"])
        reference_lift.append(row["CL"])
        reference_drag.append(row["CD"])
    axes[0, 0].scatter(reference_angles, reference_lift,
                       color="black", marker="o", label="Ladson experiment")
    axes[0, 0].set_title("Lift versus published NACA 0012 measurements")
    axes[0, 0].set_ylabel("Lift coefficient CL")
    axes[0, 0].legend(fontsize=8)

    comparison_angles = []
    lift_errors = []
    for row in comparison_results:
        comparison_angles.append(row["alpha_deg"])
        lift_errors.append(row["CL_error"])
    axes[0, 1].plot(comparison_angles, lift_errors, "o-")
    axes[0, 1].axhline(0, color="black", linewidth=0.8)
    axes[0, 1].set_title("AR 1000 lift error at the measured angles")
    axes[0, 1].set_ylabel("AVL CL minus experimental CL")
    axes[1, 0].set_title("Quarter-chord moment approaches zero in 2D")
    axes[1, 0].set_ylabel("Pitching moment coefficient Cm")
    axes[1, 0].legend(fontsize=8)

    near_2d_results = results_by_aspect_ratio[1000]
    near_2d_angles = []
    induced_drag = []
    for row in near_2d_results:
        near_2d_angles.append(row["alpha_deg"])
        induced_drag.append(row["CD_induced"])
    axes[1, 1].plot(near_2d_angles, induced_drag,
                    label="AVL induced drag only")
    axes[1, 1].scatter(reference_angles, reference_drag,
                       color="black", label="Measured total drag")
    axes[1, 1].set_title("Different drag components, not a drag validation")
    axes[1, 1].set_ylabel("Drag coefficient")
    axes[1, 1].legend(fontsize=8)
    for axis in axes.flat:
        axis.set_xlabel("Angle of attack (deg)")
        axis.grid(alpha=0.25)
    figure.suptitle("NACA 0012 | Mach 0.15 | experimental Re 6 million | no tuning")
    figure.tight_layout()
    figure.savefig(results_folder / "comparison.png", dpi=180)
    plt.close(figure)


def compare_with_experiment(near_2d_results):
    """Compare AVL lift with the measured lift at each measured angle."""
    comparison_results = []
    for measured in EXPERIMENTAL_RESULTS:
        solved = None
        for row in near_2d_results:
            if abs(row["alpha_deg"] - measured["alpha_deg"]) < 1e-8:
                solved = row
                break
        if solved is None:
            raise RuntimeError("A measured angle is missing from the AVL results.")
        comparison_results.append({
            "alpha_deg": measured["alpha_deg"],
            "CL_experiment": measured["CL"],
            "CL_AVL": solved["CL"],
            "CL_error": solved["CL"] - measured["CL"],
        })
    squared_error_sum = 0.0
    comparison_angles = []
    avl_lift = []
    for row in comparison_results:
        squared_error_sum += row["CL_error"] ** 2
        comparison_angles.append(row["alpha_deg"])
        avl_lift.append(row["CL_AVL"])
    lift_rmse = sqrt(squared_error_sum / len(comparison_results))

    measured_angles = []
    measured_lift = []
    for row in EXPERIMENTAL_RESULTS:
        measured_angles.append(row["alpha_deg"])
        measured_lift.append(row["CL"])
    experimental_lift_slope = fit_lift_slope(measured_angles, measured_lift)
    avl_lift_slope = fit_lift_slope(comparison_angles, avl_lift)
    return comparison_results, lift_rmse, experimental_lift_slope, avl_lift_slope


def check_theory(near_2d_results):
    """Check symmetry and lift slope using the fixed input mesh."""
    zero_angle_result = None
    largest_moment = 0.0
    for row in near_2d_results:
        if row["alpha_deg"] == 0.0:
            zero_angle_result = row
        moment = abs(row["Cm"])
        if moment > largest_moment:
            largest_moment = moment
    if zero_angle_result is None:
        raise RuntimeError("The zero-degree result is missing.")
    theory_slope = 2 * pi / sqrt(1 - MACH ** 2)  # per radian, approximate 2D limit
    checks = {
        "zero_lift_at_zero_alpha": abs(zero_angle_result["CL"]) < 0.00001,
        "zero_moment_at_zero_alpha": abs(zero_angle_result["Cm"]) < 0.00001,
        "near_2D_quarter_chord_moment_small": largest_moment < 0.005,
        "near_2D_slope_within_2_percent_of_theory": abs(zero_angle_result["CL_alpha"] / theory_slope - 1) < 0.02,
    }
    return checks


def main():
    results_folder = PROGRAM_FOLDER / "results"
    results_folder.mkdir(exist_ok=True)
    if 1000 not in ASPECT_RATIOS:
        raise ValueError("Keep AR 1000 in ASPECT_RATIOS for the 2D comparison.")

    angles = []
    for alpha in ANGLES:
        if alpha not in angles:
            angles.append(alpha)
    for measured in EXPERIMENTAL_RESULTS:
        alpha = measured["alpha_deg"]
        if alpha not in angles:
            angles.append(alpha)
    angles.sort()

    results_by_aspect_ratio = {}
    all_results = []
    for aspect_ratio in ASPECT_RATIOS:
        input_folder = INPUT_FOLDER / f"ar_{aspect_ratio}"
        geometry_file = input_folder / "airfoil.avl"
        mass_file = input_folder / "airfoil.mass"

        wing_results = []
        for index in range(len(angles)):
            alpha = angles[index]
            case_folder = results_folder / "cases" / f"ar_{aspect_ratio}" / f"angle_{index:03d}"
            result = run_avl(geometry_file, mass_file, alpha, case_folder, debug=DEBUG)
            wing_results.append(result)

            saved_result = result.copy()
            saved_result["aspect_ratio"] = aspect_ratio
            all_results.append(saved_result)

        results_by_aspect_ratio[aspect_ratio] = wing_results
        print(f"Solved AR {aspect_ratio}: {len(wing_results)} angles", flush=True)

    near_2d_results = results_by_aspect_ratio[1000]
    comparison_results, lift_rmse, experimental_lift_slope, avl_lift_slope = compare_with_experiment(near_2d_results)
    checks = check_theory(near_2d_results)

    all_checks_passed = True
    for passed in checks.values():
        if not passed:
            all_checks_passed = False

    summary = {
        "CL_RMSE": lift_rmse,
        "experimental_slope_per_degree": experimental_lift_slope,
        "AVL_slope_per_degree": avl_lift_slope,
        "slope_difference_percent": 100 * (avl_lift_slope / experimental_lift_slope - 1),
        "checks": checks,
        "all_checks_passed": all_checks_passed,
    }
    summary_file = results_folder / "summary.json"
    summary_text = json.dumps(summary, indent=2)
    summary_file.write_text(summary_text)

    csv_file = results_folder / "coefficients.csv"
    with csv_file.open("w", newline="") as stream:
        column_names = list(all_results[0].keys())
        writer = csv.DictWriter(stream, fieldnames=column_names)
        writer.writeheader()
        for result in all_results:
            writer.writerow(result)

    comparison_file = results_folder / "experiment_comparison.csv"
    with comparison_file.open("w", newline="") as stream:
        column_names = list(comparison_results[0].keys())
        writer = csv.DictWriter(stream, fieldnames=column_names)
        writer.writeheader()
        for result in comparison_results:
            writer.writerow(result)

    # Plot and report the result. Real viscous drag and stall are not solved.
    make_plots(results_by_aspect_ratio, comparison_results, EXPERIMENTAL_RESULTS, results_folder)
    print(f"Experimental slope: {experimental_lift_slope:.6f} per degree")
    print(f"AVL slope: {avl_lift_slope:.6f} per degree")
    print(f"Lift-slope difference: {summary['slope_difference_percent']:+.2f}%")
    print(f"Lift RMSE: {lift_rmse:.6f}")
    print(f"Checks: {checks}")
    print(f"Plot and data saved in {results_folder}")
    if not all_checks_passed:
        raise RuntimeError("A theory check failed. Read results/summary.json.")


if __name__ == "__main__":
    main()

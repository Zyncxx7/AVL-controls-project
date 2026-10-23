"""Read existing inputs, loop through angles, run AVL, compare and plot.

Start reading at main() near the bottom, then follow its function calls.
This is an angle-of-attack sweep, not a time-stepped flight simulation.
"""
from pathlib import Path
import csv
import json

import matplotlib
matplotlib.use("Agg")  # Save images without needing a graphical window.
import matplotlib.pyplot as plt

from run_avl import run_avl


# Change the angle range here. Stay inside the four reference angles.
START_ANGLE_DEG = -2.5
END_ANGLE_DEG = 10.0
ANGLE_STEP_DEG = 0.25
DEBUG = True  # Print AVL commands, full output and parsed coefficients for each case.
PROGRAM_FOLDER = Path(__file__).resolve().parent
GEOMETRY_FILE = PROGRAM_FOLDER / "inputs" / "cessna.avl"
MASS_FILE = PROGRAM_FOLDER / "inputs" / "cessna.mass"

# Four original AVL results. Keeping this small table here means that the
# program does not need a separate reference-data file to run.
REFERENCE_RESULTS = [
    {
        "alpha_deg": -2.5,
        "CL": -0.07165,
        "CD": 0.00070,
        "Cm": 0.05001,
        "CL_alpha": 4.385781,
        "Cm_alpha": -1.439476,
        "Cm_q": -12.562618,
    },
    {
        "alpha_deg": 0.0,
        "CL": 0.11987,
        "CD": 0.00101,
        "Cm": -0.00901,
        "CL_alpha": 4.389099,
        "Cm_alpha": -1.264071,
        "Cm_q": -12.496427,
    },
    {
        "alpha_deg": 5.0,
        "CL": 0.50100,
        "CD": 0.01466,
        "Cm": -0.10306,
        "CL_alpha": 4.331571,
        "Cm_alpha": -0.885808,
        "Cm_q": -12.292855,
    },
    {
        "alpha_deg": 10.0,
        "CL": 0.87345,
        "CD": 0.04489,
        "Cm": -0.16283,
        "CL_alpha": 4.191019,
        "Cm_alpha": -0.480630,
        "Cm_q": -11.995727,
    },
]

# Dictionary keys are the names returned by run_avl().
COEFFICIENTS = ["CL", "CD", "Cm", "CL_alpha", "Cm_alpha", "Cm_q"]
LABELS = ["Lift coefficient CL", "AVL drag CD (no profile drag)", "Pitch moment Cm",
          "CL_alpha (per radian)", "Cm_alpha (per radian)", "Cm_q"]


def interpolate(alpha, reference, name):
    """Estimate a coefficient on the straight line between two saved angles."""
    for index in range(len(reference) - 1):
        lower_point = reference[index]
        upper_point = reference[index + 1]
        lower_angle = lower_point["alpha_deg"]
        upper_angle = upper_point["alpha_deg"]

        if lower_angle <= alpha <= upper_angle:
            angle_fraction = (alpha - lower_angle) / (upper_angle - lower_angle)
            coefficient_change = upper_point[name] - lower_point[name]
            interpolated_value = lower_point[name] + angle_fraction * coefficient_change
            return interpolated_value
    raise ValueError("The requested angle is outside the reference range.")


def make_plots(results, reference, results_folder):
    """Plot the AVL results alongside the reference comparison."""
    angles = []
    for row in results:
        angles.append(row["alpha_deg"])

    reference_angles = []
    for row in reference:
        reference_angles.append(row["alpha_deg"])
    figure, axes = plt.subplots(2, 3, figsize=(12, 8))
    for axis, name, label in zip(axes.flat, COEFFICIENTS, LABELS):
        direct_values = []
        interpolated_values = []
        reference_values = []
        for row in results:
            direct_values.append(row[name])
            interpolated_values.append(row[name + "_interpolated"])
        for row in reference:
            reference_values.append(row[name])

        axis.plot(angles, direct_values, "o-", markersize=3,
                  label="New AVL solves")
        axis.plot(angles, interpolated_values, "--",
                  label="Four-point interpolation")
        axis.scatter(reference_angles, reference_values,
                     color="black", marker="s", label="Original values", zorder=5)
        axis.set_xlabel("Angle of attack (deg)")
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    axes[0, 0].legend(fontsize=8)
    figure.suptitle("C172 direct AVL versus interpolation | Mach 0.16 | zero rates")
    figure.tight_layout()
    figure.savefig(results_folder / "comparison.png", dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(1, 3, figsize=(12, 4))
    for axis, name in zip(axes, COEFFICIENTS[:3]):
        errors = []
        for row in results:
            error = row[name + "_interpolated"] - row[name]
            errors.append(error)
        axis.plot(angles, errors)
        axis.axhline(0, color="black", linewidth=0.8)
        axis.set_xlabel("Angle of attack (deg)")
        axis.set_ylabel(f"Interpolated {name} minus direct AVL")
        axis.grid(alpha=0.25)
    figure.suptitle("How much the four-point interpolation misses")
    figure.tight_layout()
    figure.savefig(results_folder / "interpolation_error.png", dpi=180)
    plt.close(figure)


def compare_with_reference(results, reference):
    """Check the saved reference points and find the interpolation errors."""
    largest_reference_error = 0.0
    for reference_point in reference:
        solved_point = None
        for row in results:
            if abs(row["alpha_deg"] - reference_point["alpha_deg"]) < 1e-8:
                solved_point = row
                break
        if solved_point is None:
            raise RuntimeError("A reference angle is missing from the results.")
        for name in COEFFICIENTS:
            difference = abs(solved_point[name] - reference_point[name])
            if difference > largest_reference_error:
                largest_reference_error = difference
    errors = {}
    for name in COEFFICIENTS:
        largest_error = 0.0
        for row in results:
            difference = abs(row[name + "_interpolated"] - row[name])
            if difference > largest_error:
                largest_error = difference
        errors[name] = largest_error
    return {
        "case_count": len(results),
        "largest_original_point_error": largest_reference_error,
        "original_points_pass": largest_reference_error <= 0.00002,
        "maximum_interpolation_errors": errors,
    }


def main():
    results_folder = PROGRAM_FOLDER / "results"
    results_folder.mkdir(exist_ok=True)
    geometry_file, mass_file = GEOMETRY_FILE, MASS_FILE

    if ANGLE_STEP_DEG <= 0 or END_ANGLE_DEG < START_ANGLE_DEG:
        raise ValueError("Use a positive step and an increasing angle range.")
    if START_ANGLE_DEG < REFERENCE_RESULTS[0]["alpha_deg"]:
        raise ValueError("The start angle is below the reference range.")
    if END_ANGLE_DEG > REFERENCE_RESULTS[-1]["alpha_deg"]:
        raise ValueError("The end angle is above the reference range.")

    angle_count = round((END_ANGLE_DEG - START_ANGLE_DEG) / ANGLE_STEP_DEG) + 1
    angles = []
    for index in range(angle_count):
        alpha = START_ANGLE_DEG + index * ANGLE_STEP_DEG
        angles.append(alpha)

    # Include the original angles so their results can be checked.
    for reference_point in REFERENCE_RESULTS:
        alpha = reference_point["alpha_deg"]
        if alpha not in angles:
            angles.append(alpha)
    angles.sort()

    results = []
    for index in range(len(angles)):
        alpha = angles[index]
        case_folder = results_folder / "cases" / f"angle_{index:03d}"
        result = run_avl(geometry_file, mass_file, alpha, case_folder, debug=DEBUG)

        for coefficient in COEFFICIENTS:
            interpolated_value = interpolate(alpha, REFERENCE_RESULTS, coefficient)
            result[coefficient + "_interpolated"] = interpolated_value

        results.append(result)
        if index % 10 == 0 or index == len(angles) - 1:
            print(f"Solved {index + 1}/{len(angles)} angles", flush=True)

    csv_file = results_folder / "coefficients.csv"
    with csv_file.open("w", newline="") as stream:
        column_names = list(results[0].keys())
        writer = csv.DictWriter(stream, fieldnames=column_names)
        writer.writeheader()
        for result in results:
            writer.writerow(result)

    summary = compare_with_reference(results, REFERENCE_RESULTS)
    summary_file = results_folder / "summary.json"
    summary_text = json.dumps(summary, indent=2)
    summary_file.write_text(summary_text)

    # Plot direct results and estimates. Cm is AVL's total Cmtot; it already
    # includes alpha. Do not add Cm_alpha * alpha to this total a second time.
    make_plots(results, REFERENCE_RESULTS, results_folder)
    largest_reference_error = summary["largest_original_point_error"]
    print(f"Original-point maximum difference: {largest_reference_error:.7f}")
    print(f"Plots and data saved in {results_folder}")
    print("The original geometry has no elevator CONTROL, so this does not test elevator effectiveness.")
    if not summary["original_points_pass"]:
        raise RuntimeError("The original reference points do not match; inspect the case logs.")


if __name__ == "__main__":
    main()

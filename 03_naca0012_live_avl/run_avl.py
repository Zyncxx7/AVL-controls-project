"""Send text commands to AVL and read back one operating point.

Both standalone versions use this same small runner. It knows nothing about
the aircraft geometry or experimental data: those belong in the other files.
"""
from pathlib import Path
import math
import subprocess
import tempfile


AVL_EXE = Path(__file__).resolve().parent / "avl.exe"


def read_numbers(text):
    """Convert AVL's 'name = number' text into a Python dictionary."""
    coefficients = {}
    for line in text.splitlines():
        words = line.replace("=", " = ").split()
        for position in range(1, len(words) - 1):
            if words[position] != "=":
                continue
            coefficient_name = words[position - 1]
            number_text = words[position + 1]
            # Fortran sometimes uses D instead of E for scientific notation.
            number_text = number_text.replace("D", "E").replace("d", "e")
            try:
                coefficients[coefficient_name] = float(number_text)
            except ValueError:
                continue  # Ignore labels that are not numbers.
    return coefficients


def run_avl(geometry_file, mass_file, alpha_deg, case_folder, debug=False):
    """Save raw output; optionally print commands, reports and parsed results."""
    geometry_file = Path(geometry_file).resolve()
    mass_file = Path(mass_file).resolve()
    case_folder = Path(case_folder)
    case_folder.mkdir(parents=True, exist_ok=True)
    if not AVL_EXE.is_file():
        raise FileNotFoundError(f"Put the AVL executable here: {AVL_EXE}")
    for input_file in (geometry_file, mass_file):
        if not input_file.is_file():
            raise FileNotFoundError(f"AVL input file not found: {input_file}")
    if geometry_file.parent != mass_file.parent:
        raise ValueError("Keep the geometry and mass files in the same input folder.")
    if not math.isfinite(alpha_deg):
        raise ValueError("Angle of attack must be finite.")

    # A new temporary folder prevents an old result from being mistaken for
    # a successful new solve. Its short relative filename also avoids spaces.
    with tempfile.TemporaryDirectory(prefix="case_", dir=geometry_file.parent) as temporary:
        temporary = Path(temporary)
        # These are the commands we would type in AVL's console.
        # The blank lines leave the plotting and operating menus.
        commands = f"""PLOP
G

MASS {mass_file.name}
MSET 1
OPER
A A {alpha_deg}
B B 0
X
FT
{temporary.name}/forces.txt
ST
{temporary.name}/derivatives.txt

QUIT
"""
        (case_folder / "commands.txt").write_text(commands, encoding="ascii")
        if debug:
            print(f"\n[AVL debug] Alpha = {alpha_deg:g} deg", flush=True)
            print(f"Executable: {AVL_EXE}", flush=True)
            print(f"Geometry: {geometry_file}\nMass: {mass_file}", flush=True)
            print(f"Saved logs: {case_folder.resolve()}", flush=True)
            print(f"Commands sent to AVL:\n{commands}", flush=True)
            print("Running AVL (console output follows when this case finishes)...", flush=True)
        try:
            process = subprocess.run(
                [str(AVL_EXE), geometry_file.name],
                cwd=geometry_file.parent,
                input=commands,
                text=True,
                capture_output=True,
                errors="replace",
                timeout=30,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except subprocess.TimeoutExpired as error:
            # Timeout output can be bytes even when text=True.
            partial_output = error.stdout or ""
            if isinstance(partial_output, bytes):
                partial_output = partial_output.decode(errors="replace")
            partial_errors = error.stderr or ""
            if isinstance(partial_errors, bytes):
                partial_errors = partial_errors.decode(errors="replace")
            log = partial_output + partial_errors
            (case_folder / "console.txt").write_text(log, encoding="utf-8")
            if debug:
                print(f"[AVL timeout] Partial console output:\n{log}", flush=True)
            raise RuntimeError(f"AVL exceeded 30 seconds. Read {case_folder / 'console.txt'}") from error
        log = process.stdout + process.stderr
        (case_folder / "console.txt").write_text(log, encoding="utf-8")
        if debug:
            print(f"[AVL console] Exit code: {process.returncode}\n{log}", flush=True)
        forces_file = temporary / "forces.txt"
        derivatives_file = temporary / "derivatives.txt"
        if process.returncode != 0 or not forces_file.exists() or not derivatives_file.exists():
            raise RuntimeError(f"AVL failed. Read {case_folder / 'console.txt'}")
        if "read error on line" in log.lower():
            raise RuntimeError(f"AVL could not read an input. Read {case_folder / 'console.txt'}")

        force_text = forces_file.read_text(errors="replace")
        derivative_text = derivatives_file.read_text(errors="replace")
        (case_folder / "forces.txt").write_text(force_text, encoding="utf-8")
        (case_folder / "derivatives.txt").write_text(derivative_text, encoding="utf-8")
        if debug:
            print(f"[AVL forces.txt]\n{force_text}", flush=True)
            print(f"[AVL derivatives.txt]\n{derivative_text}", flush=True)
        values = read_numbers(derivative_text)
        values.update(read_numbers(force_text))

    required = ("Alpha", "CLtot", "CDtot", "CDind", "Cmtot", "CLa", "Cma", "Cmq")
    for name in required:
        if name not in values:
            raise RuntimeError("AVL returned incomplete or invalid coefficients.")
        if not math.isfinite(values[name]):
            raise RuntimeError("AVL returned incomplete or invalid coefficients.")
    if abs(values["Alpha"] - alpha_deg) > 0.00001:
        raise RuntimeError("AVL did not solve the requested angle.")
    result = {
        "alpha_deg": values["Alpha"],
        "CL": values["CLtot"],
        "CD": values["CDtot"],
        "Cm": values["Cmtot"],
        "CD_induced": values["CDind"],
        "CL_alpha": values["CLa"],
        "Cm_alpha": values["Cma"],
        "Cm_q": values["Cmq"],
    }
    if debug:
        print("[AVL parsed results] Alpha is degrees; alpha derivatives are per radian.", flush=True)
        for name, value in result.items():
            print(f"  {name} = {value:.8g}", flush=True)
    return result


if __name__ == "__main__":
    import argparse

    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Run AVL with existing geometry and mass files.")
    parser.add_argument("--geometry", type=Path, default=here / "inputs" / "airfoil.avl")
    parser.add_argument("--mass", type=Path, default=here / "inputs" / "airfoil.mass")
    parser.add_argument("--alpha", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=here / "results" / "single_point")
    args = parser.parse_args()
    print(run_avl(args.geometry, args.mass, args.alpha, args.output, debug=True))


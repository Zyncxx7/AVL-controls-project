"""Compare run_avl() with a separate, explicit LOAD/MASS AVL console session.

Uses existing inputs only. No builders or third-party Python packages needed.
Run: python tests/verify_avl.py
"""
from pathlib import Path
import hashlib
import importlib.util
import json
import math
import os
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output" / "verification"
MODEL_FOLDERS = {"cessna": "02_c172_live_avl", "airfoil": "03_naca0012_live_avl"}
FIELDS = {
    "alpha_deg": "Alpha", "CL": "CLtot", "CD": "CDtot", "Cm": "Cmtot",
    "CD_induced": "CDind", "CL_alpha": "CLa", "Cm_alpha": "Cma", "Cm_q": "Cmq",
}
NUMBER = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[EeDd][-+]?\d+)?"


def direct_avl(runner, geometry, mass, alpha, folder):
    """Start bare AVL, LOAD the geometry, then issue ordinary console commands."""
    folder.mkdir(parents=True, exist_ok=True)
    for filename in ("forces.txt", "derivatives.txt"):
        (folder / filename).unlink(missing_ok=True)
    force_path = Path(os.path.relpath(folder / "forces.txt", geometry.parent)).as_posix()
    derivative_path = Path(os.path.relpath(folder / "derivatives.txt", geometry.parent)).as_posix()
    commands = "\n".join([
        "PLOP", "G", "", f"LOAD {geometry.name}", f"MASS {mass.name}",
        "MSET 1", "OPER", f"A A {alpha}", "B B 0", "X",
        "FT", force_path, "ST", derivative_path, "", "QUIT", "",
    ])
    (folder / "commands.txt").write_text(commands, encoding="ascii")
    process = subprocess.run(
        [str(runner.AVL_EXE)], cwd=geometry.parent, input=commands,
        text=True, capture_output=True, timeout=30,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    log = process.stdout + process.stderr
    (folder / "console.txt").write_text(log, encoding="utf-8")
    if process.returncode or "read error on line" in log.lower():
        raise RuntimeError(f"Direct AVL failed: {folder}")
    # Independent regex parser: do not call the wrapper's read_numbers().
    values = {}
    for filename in ("derivatives.txt", "forces.txt"):
        report = (folder / filename).read_text()
        for name, number in re.findall(r"([A-Za-z][\w']*)\s*=\s*(" + NUMBER + r")(?![\w.])", report):
            values[name] = float(number.replace("D", "E").replace("d", "e"))
    return {name: values[label] for name, label in FIELDS.items()}


def main():
    input_files = sorted(ROOT.glob("*/inputs/**/*.avl")) + sorted(ROOT.glob("*/inputs/**/*.mass"))
    before = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in input_files}
    cases = [("cessna", "", a) for a in (-2.5, 0, 5, 10)]
    cases += [("airfoil", "", a) for a in (-4.04, 0, 4.04, 10.12)]
    cases += [("airfoil", variant, 4) for variant in ("ar_10", "ar_100", "ar_1000")]
    rows = []
    for model, variant, alpha in cases:
        model_folder = ROOT / MODEL_FOLDERS[model]
        spec = importlib.util.spec_from_file_location(f"{model}_runner", model_folder / "run_avl.py")
        runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)
        inputs = model_folder / "inputs" / variant
        geometry, mass = inputs / f"{model}.avl", inputs / f"{model}.mass"
        case = OUTPUT / model / (variant or "default") / f"alpha_{alpha:g}"
        actual = runner.run_avl(geometry, mass, alpha, case / "wrapper")
        expected = direct_avl(runner, geometry, mass, alpha, case / "direct")
        differences = {name: abs(actual[name] - expected[name]) for name in FIELDS}
        # Compare every number in each raw report as well as the returned dictionary.
        raw_matches = {}
        for filename in ("forces.txt", "derivatives.txt"):
            wrapper_numbers = re.findall(NUMBER, (case / "wrapper" / filename).read_text())
            direct_numbers = re.findall(NUMBER, (case / "direct" / filename).read_text())
            raw_matches[filename] = wrapper_numbers == direct_numbers
        passed = all(math.isfinite(d) and d <= 1e-10 for d in differences.values()) and all(raw_matches.values())
        rows.append({"model": model, "variant": variant or "default", "alpha_deg": alpha,
                     "wrapper": actual, "direct": expected, "absolute_differences": differences,
                     "raw_report_numbers_match": raw_matches, "passed": passed})
        print(f"{model:7s} {variant or 'default':8s} alpha={alpha:6g}: {'PASS' if passed else 'FAIL'}", flush=True)
    unchanged = all(hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == digest for p, digest in before.items())
    summary = {"case_count": len(rows), "all_passed": all(r["passed"] for r in rows) and unchanged,
               "inputs_unchanged": unchanged, "input_sha256": before,
               "maximum_absolute_difference": max(max(r["absolute_differences"].values()) for r in rows),
               "scope": "Same bundled AVL executable; wrapper versus explicit console LOAD/MASS commands. No aerodynamic accuracy claim.",
               "cases": rows}
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Summary: {OUTPUT / 'summary.json'}")
    if not summary["all_passed"]:
        raise RuntimeError("AVL verification failed; inspect output/verification/summary.json")


if __name__ == "__main__":
    main()

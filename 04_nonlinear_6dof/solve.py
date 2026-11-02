"""Compare trim elevator versus speed, exercise stall memory, integrate 6DOF.

Run python 04_nonlinear_6dof/solve.py from the project root.
Use --no-plots for a standard-library-only numerical run.
"""
import argparse
import csv
import hashlib
import json
from math import atan2, cos, isfinite, sin, sqrt
from pathlib import Path

import aerodynamics as aero
import model

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
SPEEDS_KT = range(40, 126)
RESPONSE_SPEED_KT = 100
RESPONSE_SECONDS = 15
TIME_STEP = 0.01
PULSE = {"control": "elevator", "size": -1.0, "start": 2.0, "end": 3.0}


def write_csv(filename, rows):
    RESULTS.mkdir(exist_ok=True)
    fields = []
    for row in rows:
        for name in row:
            if name not in fields:
                fields.append(name)
    with (RESULTS/filename).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def compare_trims():
    """Solve the original models first, then the two PDF stall states."""
    rows = []
    for name in ("DATCOM", "AVL"):
        for speed in SPEEDS_KT:
            try:
                controls, state = model.original.trim_level_flight(name, speed)
                rows.append({"model":"Original "+name,"airspeed_kt":speed,
                             "forward_speed_kt":state[0]/model.KNOT,
                             "alpha_deg":state[7]/aero.DEG,"elevator_deg":controls["elevator"],
                             "throttle":controls["throttle"],"feasible":True,"reason":""})
            except RuntimeError as error:
                rows.append({"model":"Original "+name,"airspeed_kt":speed,"feasible":False,"reason":str(error)})
    for hysteresis in (0, 1):
        if hysteresis == 0:
            name = "PDF attached"
        else:
            name = "PDF stalled"
        for speed in SPEEDS_KT:
            roots = model.trim_solutions(speed, hysteresis)
            if not roots:
                rows.append({"model":name,"airspeed_kt":speed,"hysteresis":hysteresis,
                             "feasible":False,"reason":"No force-balanced root consistent with this hysteresis state in the lift table."})
            for index,root in enumerate(roots):
                # Full state/control lists are useful to simulate(), but
                # scalar quantities are easier to read in a CSV table.
                row = {}
                for key, value in root.items():
                    if key not in ("state", "controls"):
                        row[key] = value
                row["model"] = name
                row["root_index"] = index
                row["root_region"] = "pre-peak"
                if root["alpha_rad"] > 0.28:
                    row["root_region"] = "post-peak"
                row["reason"] = ""
                if not root["feasible"]:
                    row["reason"] = "Required throttle is outside 0 to 1."
                rows.append(row)
    write_csv("trim_comparison.csv",rows)
    return rows


def hysteresis_cycle():
    """Prescribed alpha history, not a free-flight manoeuvre or spin.

    The same state-memory update and force evaluator used by simulate() are
    called at each sample. The 0.001-rad overshoot trips the strict >0.36 rule;
    its held-endpoint table samples are flagged in the CSV.
    """
    minimum_alpha = -0.05
    maximum_alpha = 0.361
    half_steps = 822
    dt = 0.02
    speed = 60 * model.KNOT
    controls = {"throttle": 0.5, "elevator": 0.0, "aileron": 0.0, "rudder": 0.0}
    hysteresis = 0
    rows = []
    for index in range(2*half_steps + 1):
        if index <= half_steps:
            direction = "increasing"
            fraction = index / half_steps
            alpha_dot = (maximum_alpha-minimum_alpha) / (half_steps*dt)
        else:
            direction = "decreasing"
            fraction = (2*half_steps-index) / half_steps
            alpha_dot = -(maximum_alpha-minimum_alpha) / (half_steps*dt)
        alpha = minimum_alpha + (maximum_alpha-minimum_alpha)*fraction
        state = [speed*cos(alpha), 0, speed*sin(alpha), 0, alpha_dot, 0,
                 0, alpha, 0, 0, 0, 0]
        hysteresis = aero.update_hysteresis(alpha, hysteresis)
        loads, coeff = model.forces(state, controls, hysteresis, alpha_dot)
        rows.append({"time_s":index*dt,"direction":direction,
                     "alpha_rad":alpha,"alpha_deg":alpha/aero.DEG,"hysteresis":hysteresis,
                     "CL_wing":aero.wing_lift(alpha,hysteresis),"CL_total":coeff["CL"],
                     "alpha_rate_rad_s":alpha_dot,"lift_N":0.5*model.original.density(0)*speed**2*aero.S*coeff["CL"],
                     "lift_table_endpoint_held":coeff["lift_table_endpoint_held"],
                     "drag_table_endpoint_held":coeff["drag_table_endpoint_held"]})
    write_csv("stall_cycle.csv",rows)
    return rows


def compare_responses():
    """Trim each model, apply the same elevator pulse, save every state."""
    rows = []
    state_names = ["u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s",
                   "roll_rad", "pitch_rad", "yaw_rad", "north_m", "east_m", "down_m"]
    for name in ("DATCOM", "AVL", "PDF"):
        if name == "PDF":
            for trim in model.trim_solutions(RESPONSE_SPEED_KT, 0):
                if trim["feasible"]:
                    break
            else:
                raise RuntimeError("No feasible PDF-model starting trim.")
            history = model.simulate(
                trim["controls"], trim["state"],
                seconds=RESPONSE_SECONDS, dt=TIME_STEP, pulse=PULSE)
        else:
            controls, state = model.original.trim_level_flight(name, RESPONSE_SPEED_KT)
            original_history = model.original.simulate(
                name, controls, state,
                seconds=RESPONSE_SECONDS, dt=TIME_STEP, pulse=PULSE)
            history = []
            for time, state in original_history:
                history.append((time, state, 0))

        for time, state, hysteresis in history:
            row = {"model": name, "time_s": time}
            for state_name, value in zip(state_names, state):
                row[state_name] = value
            row["speed_kt"] = sqrt(sum(value*value for value in state[:3])) / model.KNOT
            row["alpha_deg"] = atan2(state[2], state[0]) / aero.DEG
            row["pitch_deg"] = state[7] / aero.DEG
            row["altitude_m"] = -state[11]
            row["hysteresis"] = hysteresis
            rows.append(row)
    write_csv("time_response.csv",rows)
    return rows


def save_summary(trims, cycle, responses):
    """Save settings and assumptions separately from the simulation logic."""
    switches = []
    for index in range(1, len(cycle)):
        if cycle[index]["hysteresis"] != cycle[index-1]["hysteresis"]:
            switches.append(cycle[index])
    finite_states = True
    for row in responses:
        for value in row.values():
            if isinstance(value, (int, float)) and not isfinite(value):
                finite_states = False
    summary={
        "requested_work":{"trim_elevator_vs_speed":"implemented for original DATCOM, original AVL and PDF model",
                          "increasing_decreasing_stall_loop":"implemented with persistent memory, also used by 6DOF integration",
                          "spin_solution":"deferred as requested in the email; no spin solution claimed"},
        "setup":{"mass_kg":model.MASS,"inertia_kg_m2":[model.IXX,model.IYY,model.IZZ],
                 "wing_area_m2":aero.S,"span_m":aero.B,"chord_m":aero.C,
                 "flaps_deg":0,"density_kg_m3":model.original.density(0),"ground_effect":False,
                 "speeds_kt":[SPEEDS_KT[0],SPEEDS_KT[-1]],"speed_step_kt":SPEEDS_KT.step,"time_step_s":TIME_STEP,
                 "response_duration_s":RESPONSE_SECONDS,"response_trim_speed_kt":RESPONSE_SPEED_KT,"pulse":PULSE,
                 "hysteresis_reset_below_rad":aero.HYST_MIN,"hysteresis_set_above_rad":aero.HYST_MAX},
        "trim_results":{},
        "stall_cycle":{"time_samples":len(cycle),"max_alpha_rad":max(r["alpha_rad"] for r in cycle),
                       "switches":switches,
                       "prescribed_alpha_not_free_flight":True},
        "time_response":{"samples":len(responses),"finite_states":finite_states},
        "source_pdf_sha256":hashlib.sha256((HERE/'reference/aerodynamics.pdf').read_bytes()).hexdigest(),
        "assumptions":["Common original mass, inertia, atmosphere, thrust model and propeller diameter retained.",
                       "Clean configuration in free air; altitude zero denotes sea-level density, not ground clearance.",
                       "PDF aerodynamic moments treated at the common CG; the PDF supplies no loading-dependent CG.",
                       "Tail induced velocity estimated using actuator-disk momentum theory and the common thrust model.",
                       "Tables use endpoint holding; held lift/drag samples are explicitly flagged in stall_cycle.csv.",
                       "No unspecified post-stall roll-damping reversal was invented.",
                       "Original comparison uses its existing Euler integrator; nonlinear model uses RK4.",
                       "Trim roots are equilibria, not a guarantee of stability or a realised deceleration trajectory."]}
    for name in ("Original DATCOM","Original AVL","PDF attached","PDF stalled"):
        valid = []
        for row in trims:
            if row["model"] == name and row["feasible"]:
                valid.append(row)
        summary["trim_results"][name]={"feasible_equilibria":len(valid),
             "feasible_speed_range_kt":[min(r["airspeed_kt"] for r in valid),max(r["airspeed_kt"] for r in valid)] if valid else None,
             "at_100_kt":[r for r in valid if r["airspeed_kt"]==100]}
    (RESULTS/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    RESULTS.mkdir(exist_ok=True)

    # 1. Elevator, alpha and throttle needed for level flight at each speed.
    trims = compare_trims()

    # 2. Increasing/decreasing alpha, with a remembered stall state.
    cycle = hysteresis_cycle()

    # 3. Actual 6DOF response to an elevator pulse, starting from trim.
    responses = compare_responses()

    # 4. Save the settings, then create the same comparison plots.
    summary = save_summary(trims, cycle, responses)
    if not args.no_plots:
        from plots import make_plots
        make_plots(trims, cycle, responses)
    for name,data in summary["trim_results"].items():
        print(name,data["feasible_speed_range_kt"],"kt;",data["feasible_equilibria"],"feasible equilibria")
    print("Results:",RESULTS)


if __name__=="__main__":
    main()

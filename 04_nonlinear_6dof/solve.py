"""C172: a time-stepped response and trim-versus-speed comparison.

Everything for the new aircraft model is in this file.
Read the settings and tables, then main() at the bottom.
The original model is imported only to draw the comparison curves.
"""
from math import asin, atan2, cos, exp, isfinite, pi, sin, sqrt, tan
from pathlib import Path
import argparse
import csv
import sys


# SETTINGS: SI units, except speeds in knots and controls in degrees.
MASS = 1043.0
G = 9.81
IXX = 1285.0
IYY = 1825.0
IZZ = 2667.0
S = 174.0 * 0.09290304
B = 35.8 * 0.3048
C = 4.9 * 0.3048
DEG = pi / 180.0
KNOT = 0.514444
START_SPEED_KT = 100.0
SECONDS = 15.0
DT = 0.001
ELEVATOR_PULSE_DEG = -1.0
PULSE_START = 2.0
PULSE_END = 3.0
SPEEDS_KT = range(40, 126)
PROP_DIAMETER = 75.0 * 0.0254
PROP_RPM = 2400.0

# PDF tables. Matching positions in the two lists belong to the same row.
ALPHA = [-0.09, 0, 0.09, 0.10, 0.12, 0.14, 0.16, 0.17,
         0.19, 0.21, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36]
CL_ATTACHED = [-0.22, 0.25, 0.73, 0.83, 0.92, 1.02, 1.08, 1.13,
               1.19, 1.25, 1.35, 1.44, 1.47, 1.43, 1.38, 1.30, 1.15]
CL_STALLED = [-0.22, 0.25, 0.73, 0.78, 0.79, 0.81, 0.82, 0.83,
              0.85, 0.86, 0.88, 0.90, 0.92, 0.95, 0.99, 1.05, 1.15]
CD_ALPHA = [-0.0873, -0.0698, -0.0524, -0.0349, -0.0175, 0,
            0.0175, 0.0349, 0.0524, 0.0698, 0.0873, 0.1047, 0.1222,
            0.1396, 0.1571, 0.1745, 0.1920, 0.2094, 0.2269, 0.2443,
            0.2618, 0.2793, 0.2967, 0.3142, 0.3316, 0.3491]
CD_WING = [0.0041, 0.0013, 0.0001, 0.0003, 0.0020, 0.0052,
           0.0099, 0.0162, 0.0240, 0.0334, 0.0442, 0.0566, 0.0706,
           0.0860, 0.0962, 0.1069, 0.1180, 0.1298, 0.1424, 0.1565,
           0.1727, 0.1782, 0.1716, 0.1618, 0.1475, 0.1097]

# Propeller table retained from the original model, not supplied by the PDF.
PROP_J = [0, .05, .10, .15, .20, .25, .30, .35, .40, .45, .50, .55,
          .60, .65, .70, .75, .76, .77, .78, .79, .80, .81, .82,
          .83, .84, .85, .86, .87, .88, .89, .90, .91, .92, .93, .94]
PROP_CT = [.102122, .110970, .107621, .105191, .102446, .099470,
           .096775, .094706, .092341, .088912, .083878, .076336,
           .066669, .056342, .045688, .034716, .032492, .030253,
           .028001, .025735, .023453, .021159, .018852,
           .016529, .014194, .011843, .009479, .007100,
           .004686, .002278, -.000200, -.002638, -.005145, -.007641, -.010188]

# The longitudinal force and moment derivatives from the PDF.
CL_DE = 0.43
CL_Q = 3.9
CL_ADOT = 1.7
CD_ZERO = 0.027
CD_DE = 0.06
CD_BETA = 0.17
CM_ZERO = 0.1
CM_ALPHA = -1.8
CM_Q = -12.4
CM_ADOT = -7.27
CM_DE = -1.122


def interpolate(x, x_table, y_table):
    """Read between two table rows using a straight line."""
    if not isfinite(x):
        raise ValueError("Table input must be finite.")
    if x <= x_table[0]:
        return y_table[0]
    for i in range(len(x_table) - 1):
        if x <= x_table[i + 1]:
            fraction = (x - x_table[i]) / (x_table[i + 1] - x_table[i])
            return y_table[i] + fraction * (y_table[i + 1] - y_table[i])
    return y_table[-1]


def thrust(throttle, speed, rho):
    """Same fixed-RPM propeller calculation as the original model."""
    revolutions_per_second = PROP_RPM / 60.0
    advance_ratio = speed / (revolutions_per_second * PROP_DIAMETER)
    if advance_ratio < PROP_J[0] or advance_ratio > PROP_J[-1]:
        return 0.0
    ct = interpolate(advance_ratio, PROP_J, PROP_CT)
    return throttle * ct * rho * revolutions_per_second**2 * PROP_DIAMETER**4


def stall_state(alpha, stalled):
    """Keep the old branch unless alpha crosses a switching threshold."""
    if alpha > 0.36:
        stalled = 1
    elif alpha < 0.09:
        stalled = 0
    return stalled


def trim_balance(speed_kt, alpha, stalled):
    """At one trial alpha, calculate elevator, throttle and force error."""
    if not isfinite(speed_kt) or speed_kt <= 0:
        raise ValueError("Airspeed must be finite and positive.")
    speed = speed_kt * KNOT
    rho = 1.225
    qS = 0.5 * rho * speed**2 * S
    propeller_area = pi * PROP_DIAMETER**2 / 4
    base_cd = CD_ZERO + interpolate(alpha, CD_ALPHA, CD_WING)

    # Balance pitch: CM_ZERO + CM_ALPHA*alpha + CM_DE*elevator*tail_ratio = 0.
    # Elevator drag changes required thrust and therefore the tail pressure.
    # Substitution gives a quadratic in the magnitude of the elevator angle.
    tail_base = cos(alpha)**2 + S*base_cd / (propeller_area*cos(alpha))
    tail_elevator = S*CD_DE / (propeller_area*cos(alpha))
    required_effect = -(CM_ZERO + CM_ALPHA*alpha) / CM_DE
    discriminant = tail_base**2 + 4*tail_elevator*abs(required_effect)
    elevator = 2*abs(required_effect) / (tail_base + sqrt(discriminant))
    if required_effect < 0:
        elevator = -elevator

    # Balance forward force: thrust*cos(alpha) = drag.
    cd = base_cd + CD_DE*abs(elevator)
    drag = qS * cd
    required_thrust = drag / cos(alpha)
    full_thrust = thrust(1.0, speed, rho)
    throttle = float("inf")
    if full_thrust > 0:
        throttle = required_thrust / full_thrust

    if stalled == 0:
        cl = interpolate(alpha, ALPHA, CL_ATTACHED)
    else:
        cl = interpolate(alpha, ALPHA, CL_STALLED)
    cl = cl + CL_DE*elevator

    # Balance vertical force: lift + thrust*sin(alpha) = weight.
    force_error = qS*cl + drag*tan(alpha) - MASS*G
    return force_error, elevator/DEG, throttle


def find_trim(speed_kt, stalled=0):
    """Try alpha intervals, then halve intervals containing zero force error.

    Return three plain lists: alpha, elevator and throttle for every root.
    A throttle above 1 means that root cannot be flown with this engine.
    """
    trial_alphas = []
    for i in range(901):
        trial_alphas.append(ALPHA[0] + i*(ALPHA[-1] - ALPHA[0])/900)
    for alpha in ALPHA + CD_ALPHA:
        if alpha not in trial_alphas:
            trial_alphas.append(alpha)
    trial_alphas.sort()

    trim_alphas = []
    trim_elevators = []
    trim_throttles = []
    for i in range(len(trial_alphas) - 1):
        lower = trial_alphas[i]
        upper = trial_alphas[i + 1]
        lower_error, _, _ = trim_balance(speed_kt, lower, stalled)
        upper_error, _, _ = trim_balance(speed_kt, upper, stalled)
        if lower_error*upper_error > 0:
            continue
        if abs(lower_error) < 1e-8:
            alpha = lower
        elif abs(upper_error) < 1e-8:
            alpha = upper
        else:
            for j in range(50):
                alpha = (lower + upper) / 2
                error, _, _ = trim_balance(speed_kt, alpha, stalled)
                if lower_error*error <= 0:
                    upper = alpha
                else:
                    lower = alpha
                    lower_error = error
            alpha = (lower + upper) / 2
        if trim_alphas and abs(alpha - trim_alphas[-1]) < 1e-7:
            continue
        if stall_state(alpha, stalled) != stalled:
            continue
        error, elevator, throttle = trim_balance(speed_kt, alpha, stalled)
        trim_alphas.append(alpha)
        trim_elevators.append(elevator)
        trim_throttles.append(throttle)
    return trim_alphas, trim_elevators, trim_throttles


def calculate_rates(u, v, w, p, q, r, roll, pitch, yaw, down,
                    throttle, elevator_deg, aileron_deg, rudder_deg, stalled):
    """The six aerodynamic coefficients and all 12 aircraft equations.

    Every argument is a number. Every returned value is a rate of change.
    Angles/rates are radians internally; control inputs are degrees.
    """
    speed = sqrt(u*u + v*v + w*w)
    if speed < 1 or u <= 0 or abs(cos(pitch)) < 0.1:
        raise ValueError("The model left its forward-flight/Euler-angle range.")
    rho = 1.225 * exp(-max(-down, 0.0) / 8500.0)
    qS = 0.5 * rho * speed**2 * S
    alpha = atan2(w, u)
    beta = asin(max(-1, min(1, v/speed)))
    p_hat = p*B / (2*speed)
    q_hat = q*C / (2*speed)
    r_hat = r*B / (2*speed)
    elevator = elevator_deg*DEG
    aileron = aileron_deg*DEG
    rudder = rudder_deg*DEG

    engine_thrust = thrust(throttle, speed, rho)
    propeller_area = pi*PROP_DIAMETER**2 / 4
    axial_speed = speed*cos(alpha)
    induced_speed = (sqrt(axial_speed**2 + 2*max(engine_thrust, 0)/(rho*propeller_area)) - axial_speed)/2
    tail_ratio = ((axial_speed + 2*induced_speed)/speed)**2

    # LIFT AND DRAG. Add the alpha-rate term after solving its feedback below.
    if stalled == 0:
        CL = interpolate(alpha, ALPHA, CL_ATTACHED)
    else:
        CL = interpolate(alpha, ALPHA, CL_STALLED)
    CL = CL + CL_DE*elevator + CL_Q*q_hat
    CD = CD_ZERO + interpolate(alpha, CD_ALPHA, CD_WING)
    CD = CD + CD_DE*abs(elevator) + CD_BETA*abs(beta)

    # SIDE FORCE: sideslip, rudder, roll rate and yaw rate.
    CY_beta = interpolate(beta, [-0.349, 0, 0.349], [0.137, 0, -0.137])
    CY_p = interpolate(alpha, [0, 0.094], [-0.075, -0.145])
    CY_r = interpolate(alpha, [0, 0.094], [0.214, 0.267])
    CY = CY_beta + 0.187*rudder + CY_p*p_hat + CY_r*r_hat

    # Alpha-dot changes lift, but lift also changes alpha-dot. Solve both
    # together using the acceleration calculated without that extra lift.
    lift = qS*CL
    drag = qS*CD
    side = qS*CY
    Fx = engine_thrust - drag*cos(alpha)*cos(beta) - side*cos(alpha)*sin(beta) + lift*sin(alpha)
    Fz = -drag*sin(alpha)*cos(beta) - side*sin(alpha)*sin(beta) - lift*cos(alpha)
    u_dot = r*v - q*w + Fx/MASS - G*sin(pitch)
    w_dot = q*u - p*v + Fz/MASS + G*cos(roll)*cos(pitch)
    longitudinal_speed = sqrt(u*u + w*w)
    alpha_dot_free = (u*w_dot - w*u_dot) / longitudinal_speed**2
    extra_lift_per_alpha_rate = qS*CL_ADOT*C / (2*speed)
    alpha_dot = alpha_dot_free / (1 + extra_lift_per_alpha_rate/(MASS*longitudinal_speed))
    alpha_dot_hat = alpha_dot*C / (2*speed)
    CL = CL + CL_ADOT*alpha_dot_hat

    # ROLL MOMENT: sideslip, damping, yaw rate, aileron and rudder.
    Cl_beta = interpolate(beta, [-0.349, 0, 0.349], [0.0322, 0, -0.0322])
    Cl_r = interpolate(alpha, [0, 0.094], [0.0798, 0.1869])
    Cl = Cl_beta - 0.484*p_hat + Cl_r*r_hat + 0.229*aileron + 0.0147*rudder

    # PITCH MOMENT: alpha, pitch rate, alpha rate and elevator with propwash.
    Cm = CM_ZERO + CM_ALPHA*alpha + CM_Q*q_hat
    Cm = Cm + CM_ADOT*alpha_dot_hat + CM_DE*elevator*tail_ratio

    # YAW MOMENT: sideslip, roll/yaw damping, aileron and rudder with propwash.
    Cn_beta = interpolate(beta, [-0.349, 0, 0.349], [-0.0205, 0, 0.0205])
    Cn = Cn_beta - 0.0278*p_hat - 0.0937*r_hat - 0.0053*aileron - 0.043*rudder*tail_ratio

    # Convert coefficients to body forces (N) and moments (N m).
    lift = qS*CL
    Fx = engine_thrust - drag*cos(alpha)*cos(beta) - side*cos(alpha)*sin(beta) + lift*sin(alpha)
    Fy = -drag*sin(beta) + side*cos(beta)
    Fz = -drag*sin(alpha)*cos(beta) - side*sin(alpha)*sin(beta) - lift*cos(alpha)
    L = qS*B*Cl
    M = qS*C*Cm
    N = qS*B*Cn

    # Linear acceleration.
    u_dot = r*v - q*w + Fx/MASS - G*sin(pitch)
    v_dot = p*w - r*u + Fy/MASS + G*sin(roll)*cos(pitch)
    w_dot = q*u - p*v + Fz/MASS + G*cos(roll)*cos(pitch)

    # Angular acceleration.
    p_dot = (L - (IZZ-IYY)*q*r) / IXX
    q_dot = (M - (IXX-IZZ)*p*r) / IYY
    r_dot = (N - (IYY-IXX)*p*q) / IZZ

    # Attitude rates.
    roll_dot = p + (q*sin(roll) + r*cos(roll))*tan(pitch)
    pitch_dot = q*cos(roll) - r*sin(roll)
    yaw_dot = (q*sin(roll) + r*cos(roll)) / cos(pitch)

    # Position rates in North-East-Down coordinates.
    north_dot = (u*cos(pitch)*cos(yaw)
                 + v*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw))
                 + w*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)))
    east_dot = (u*cos(pitch)*sin(yaw)
                + v*(sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw))
                + w*(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw)))
    down_dot = -u*sin(pitch) + v*sin(roll)*cos(pitch) + w*cos(roll)*cos(pitch)
    return (u_dot, v_dot, w_dot, p_dot, q_dot, r_dot,
            roll_dot, pitch_dot, yaw_dot, north_dot, east_dot, down_dot)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-plots", action="store_true")
    no_plots = parser.parse_args().no_plots
    folder = Path(__file__).resolve().parent
    results = folder / "results"
    results.mkdir(exist_ok=True)

    # 1. FIND STARTING TRIM AND SET THE INITIAL AIRCRAFT VALUES.
    alphas, elevators, throttles = find_trim(START_SPEED_KT)
    for i in range(len(alphas)):
        if 0 <= throttles[i] <= 1:
            break
    else:
        raise RuntimeError("No feasible starting trim.")
    alpha = alphas[i]
    trim_elevator = elevators[i]
    throttle = throttles[i]
    u = START_SPEED_KT*KNOT*cos(alpha)
    v = 0.0
    w = START_SPEED_KT*KNOT*sin(alpha)
    p = q = r = 0.0
    roll = yaw = 0.0
    pitch = alpha
    north = east = down = 0.0
    stalled = 0
    time_history, speed_history, pitch_history, altitude_history = [], [], [], []

    # A fixed step must land exactly on the pulse edges and final time.
    if DT <= 0:
        raise ValueError("DT must be positive.")
    for boundary in (SECONDS, PULSE_START, PULSE_END):
        if abs(boundary/DT - round(boundary/DT)) > 1e-7:
            raise ValueError("Choose DT that divides the duration and pulse times exactly.")

    # 2. STEP THE AIRCRAFT THROUGH TIME AND SAVE ITS RESPONSE.
    with (results / "time_response.csv").open("w", newline="") as file:
        table = csv.writer(file)
        table.writerow(["time_s", "u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s",
                        "roll_rad", "pitch_rad", "yaw_rad", "north_m", "east_m", "down_m", "stalled"])
        for step in range(round(SECONDS/DT) + 1):
            time = step*DT
            alpha = atan2(w, u)
            stalled = stall_state(alpha, stalled)
            table.writerow([time, u, v, w, p, q, r, roll, pitch, yaw, north, east, down, stalled])
            time_history.append(time)
            speed_history.append(sqrt(u*u + v*v + w*w)/KNOT)
            pitch_history.append(pitch/DEG)
            altitude_history.append(-down)
            if step == round(SECONDS/DT):
                break

            elevator = trim_elevator
            if PULSE_START <= time < PULSE_END:
                elevator = elevator + ELEVATOR_PULSE_DEG

            u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, roll_dot, pitch_dot, yaw_dot, north_dot, east_dot, down_dot = calculate_rates(
                u, v, w, p, q, r, roll, pitch, yaw, down,
                throttle, elevator, 0.0, 0.0, stalled)

            # Forward Euler, exactly the form used in the original model:
            # new value = old value + timestep * rate of change.
            u = u + DT*u_dot
            v = v + DT*v_dot
            w = w + DT*w_dot
            p = p + DT*p_dot
            q = q + DT*q_dot
            r = r + DT*r_dot
            roll = roll + DT*roll_dot
            pitch = pitch + DT*pitch_dot
            yaw = yaw + DT*yaw_dot
            north = north + DT*north_dot
            east = east + DT*east_dot
            down = down + DT*down_dot
            if not all(isfinite(value) for value in (u, v, w, p, q, r, roll, pitch, yaw, north, east, down)):
                raise FloatingPointError("A state value is not finite.")

    print("Starting trim:", START_SPEED_KT, "kt; elevator", trim_elevator, "deg; throttle", throttle)

    # The original file is used ONLY for the reference comparison.
    sys.path.insert(0, str(folder.parent / "01_simple_6dof"))
    import simple_c172_6dof as original

    # 3. COMPARE LEVEL-FLIGHT TRIM AT DIFFERENT SPEEDS.
    datcom_speed, datcom_elevator, datcom_alpha, datcom_throttle = [], [], [], []
    avl_speed, avl_elevator, avl_alpha, avl_throttle = [], [], [], []
    pdf_speed, pdf_elevator, pdf_alpha, pdf_throttle = [], [], [], []
    post_speed, post_elevator, post_alpha, post_throttle = [], [], [], []
    stalled_speed, stalled_elevator, stalled_alpha, stalled_throttle = [], [], [], []

    with (results / "trim_comparison.csv").open("w", newline="") as file:
        table = csv.writer(file)
        table.writerow(["model", "airspeed_kt", "alpha_deg", "elevator_deg", "throttle", "feasible", "reason"])
        for speed_kt in SPEEDS_KT:
            for name in ("DATCOM", "AVL"):
                try:
                    controls, state = original.trim_level_flight(name, speed_kt)
                    alpha_deg = state[7]/DEG
                    elevator = controls["elevator"]
                    throttle = controls["throttle"]
                    table.writerow([name, speed_kt, alpha_deg, elevator, throttle, True, ""])
                    if name == "DATCOM":
                        datcom_speed.append(speed_kt)
                        datcom_elevator.append(elevator)
                        datcom_alpha.append(alpha_deg)
                        datcom_throttle.append(throttle)
                    else:
                        avl_speed.append(speed_kt)
                        avl_elevator.append(elevator)
                        avl_alpha.append(alpha_deg)
                        avl_throttle.append(throttle)
                except RuntimeError as error:
                    table.writerow([name, speed_kt, "", "", "", False, str(error)])

            for stalled in (0, 1):
                alphas, elevators, throttles = find_trim(speed_kt, stalled)
                name = "PDF attached" if stalled == 0 else "PDF stalled"
                if not alphas:
                    table.writerow([name, speed_kt, "", "", "", False, "No trim in the table range."])
                for i in range(len(alphas)):
                    alpha_deg = alphas[i]/DEG
                    feasible = 0 <= throttles[i] <= 1
                    reason = "" if feasible else "Required throttle is outside 0 to 1."
                    table.writerow([name, speed_kt, alpha_deg, elevators[i], throttles[i], feasible, reason])
                    if not feasible:
                        continue
                    if stalled == 1:
                        stalled_speed.append(speed_kt)
                        stalled_elevator.append(elevators[i])
                        stalled_alpha.append(alpha_deg)
                        stalled_throttle.append(throttles[i])
                    elif alphas[i] > 0.28:
                        post_speed.append(speed_kt)
                        post_elevator.append(elevators[i])
                        post_alpha.append(alpha_deg)
                        post_throttle.append(throttles[i])
                    else:
                        pdf_speed.append(speed_kt)
                        pdf_elevator.append(elevators[i])
                        pdf_alpha.append(alpha_deg)
                        pdf_throttle.append(throttles[i])

    print("Numerical results:", results)
    if no_plots:
        return

    # 4. PLOTS. Ordinary Matplotlib statements; no custom plotting framework.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    with PdfPages(results / "research_results.pdf") as report:
        figure, axis = plt.subplots(figsize=(10, 6))
        axis.plot(datcom_speed, datcom_elevator, label="Original DATCOM")
        axis.plot(avl_speed, avl_elevator, label="Original AVL tables")
        axis.plot(pdf_speed, pdf_elevator, label="PDF attached")
        axis.plot(post_speed, post_elevator, label="PDF attached, post-peak")
        axis.plot(stalled_speed, stalled_elevator, ".", label="PDF stalled")
        axis.set(xlabel="True airspeed (kt)", ylabel="Trim elevator (deg)",
                 title="Level-flight trim | clean configuration | sea-level density | mass 1043 kg")
        axis.grid(True, alpha=0.3)
        axis.legend()
        figure.tight_layout()
        figure.savefig(results / "trim_elevator.png", dpi=160)
        report.savefig(figure)
        plt.close(figure)

        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        axes[0].plot(datcom_speed, datcom_alpha, label="Original DATCOM")
        axes[0].plot(avl_speed, avl_alpha, label="Original AVL tables")
        axes[0].plot(pdf_speed, pdf_alpha, label="PDF attached")
        axes[0].plot(post_speed, post_alpha, label="PDF attached, post-peak")
        axes[0].plot(stalled_speed, stalled_alpha, ".", label="PDF stalled")
        axes[1].plot(datcom_speed, datcom_throttle)
        axes[1].plot(avl_speed, avl_throttle)
        axes[1].plot(pdf_speed, pdf_throttle)
        axes[1].plot(post_speed, post_throttle)
        axes[1].plot(stalled_speed, stalled_throttle, ".")
        axes[0].set_ylabel("Trim alpha (deg)")
        axes[1].set(xlabel="True airspeed (kt)", ylabel="Trim throttle")
        axes[0].legend()
        for axis in axes:
            axis.grid(True, alpha=0.3)
        figure.suptitle("Level-flight trim | clean configuration | sea-level density | mass 1043 kg")
        figure.tight_layout()
        figure.savefig(results / "trim_parameters.png", dpi=160)
        report.savefig(figure)
        plt.close(figure)

        figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        # Only this reference-comparison block uses the original file's API.
        for name in ("DATCOM", "AVL"):
            controls, state = original.trim_level_flight(name, START_SPEED_KT)
            pulse = {"control": "elevator", "size": ELEVATOR_PULSE_DEG,
                     "start": PULSE_START, "end": PULSE_END}
            history = original.simulate(name, controls, state, seconds=SECONDS, dt=DT, pulse=pulse)
            reference_time, reference_speed, reference_pitch, reference_altitude = [], [], [], []
            for time, state in history:
                reference_time.append(time)
                reference_speed.append(sqrt(state[0]**2 + state[1]**2 + state[2]**2)/KNOT)
                reference_pitch.append(state[7]/DEG)
                reference_altitude.append(-state[11])
            axes[0].plot(reference_time, reference_speed, label="Original " + name)
            axes[1].plot(reference_time, reference_pitch)
            axes[2].plot(reference_time, reference_altitude)
        axes[0].plot(time_history, speed_history, label="PDF model")
        axes[1].plot(time_history, pitch_history)
        axes[2].plot(time_history, altitude_history)
        axes[0].set_ylabel("Airspeed (kt)")
        axes[1].set_ylabel("Pitch (deg)")
        axes[2].set(xlabel="Time (s)", ylabel="Altitude (m)")
        axes[0].legend()
        for axis in axes:
            axis.grid(True, alpha=0.3)
        figure.suptitle(f"Each model trimmed at {START_SPEED_KT:g} kt | elevator {ELEVATOR_PULSE_DEG:g} deg from {PULSE_START:g}-{PULSE_END:g} s | dt {DT:g} s")
        figure.tight_layout()
        figure.savefig(results / "time_response.png", dpi=160)
        report.savefig(figure)
        plt.close(figure)
    print("Plots and report:", results / "research_results.pdf")


if __name__ == "__main__":
    main()

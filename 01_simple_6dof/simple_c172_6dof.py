"""Simple 6-DOF C172 model in one file.

Change DATASET at the bottom to compare AVL and DATCOM.
State: [u, v, w, p, q, r, roll, pitch, yaw, north, east, down]
The body axes are x forward, y right, and z down.
"""
from math import asin, atan2, cos, exp, isfinite, pi, sin, sqrt, tan
from pathlib import Path

PLOT_DIR = Path(__file__).resolve().parent / "output" / "plots"


# Aircraft values. Everything is in SI units.
G = 9.81
MASS = 1043.0
S = 16.2                 # wing area, m^2
B = 10.9                 # wing span, m
C = 1.49                 # mean chord, m
IXX, IYY, IZZ = 1285.0, 1825.0, 2667.0
DEG = pi / 180.0

# AVL and DATCOM values from the PDF you supplied.
# They are listed here so this script can run by itself.
ANGLES_DEG = [-2.5, 0.0, 5.0, 10.0]
TABLES = {
    "DATCOM": {
        "CL": [-0.08300, 0.14800, 0.65000, 1.19500],
        "CD": [0.03000, 0.03000, 0.04800, 0.09300],
        "Cm": [0.03140, 0.00750, -0.06800, -0.19270],
        "Cm_alpha": [-0.48460, -0.64420, -1.1220, -1.7770],
        "Cl_beta": [-0.19430, -0.20210, -0.21910, -0.23760],
        "CL_q": None, "Cm_q": None,
        "Cl_p": [-0.44890, -0.45950, -0.50850, -0.49160],
        "CY_p": [-0.04178, -0.04627, -0.05628, -0.06769],
        "Cn_p": [0.00262, -0.01077, -0.03849, -0.07082],
        "Cn_r": [-0.02666, -0.02746, -0.03246, -0.04287],
        "Cl_r": [-0.01087, 0.02986, 0.11930, 0.21520],
    },
    "AVL": {
        "CL": [-0.07165, 0.11987, 0.50100, 0.87345],
        # AVL gave induced drag, so 0.029 parasite drag was added from DATCOM.
        "CD": [0.02970, 0.03001, 0.04366, 0.07389],
        "Cm": [0.05001, -0.00901, -0.10306, -0.16283],
        "Cm_alpha": [-1.4395, -1.2641, -0.88581, -0.48063],
        "Cl_beta": [-0.00528, -0.00500, -0.00598, -0.00899],
        "CL_q": [9.1057, 9.3463, 9.7894, 10.170],
        "Cm_q": [-12.563, -12.496, -12.293, -11.996],
        "Cl_p": [-0.29574, -0.29675, -0.29470, -0.28700],
        "CY_p": [-0.01592, -0.00564, 0.01260, 0.02727],
        "Cn_p": [-0.00710, -0.01631, -0.03340, -0.04936],
        "Cn_r": [-0.06371, -0.05841, -0.04987, -0.04437],
        "Cl_r": [-0.01836, 0.00963, 0.06671, 0.12417],
    },
}


def available_coefficients(dataset, alpha_rad):
    """Read the coefficient table at the current angle of attack."""
    alpha_deg = alpha_rad / DEG
    table = TABLES[dataset]
    if alpha_deg <= ANGLES_DEG[0]:
        left = 0
    elif alpha_deg >= ANGLES_DEG[-1]:
        left = len(ANGLES_DEG) - 2
    else:
        left = next(i for i in range(3)
                    if ANGLES_DEG[i] <= alpha_deg <= ANGLES_DEG[i + 1])
    part = ((alpha_deg - ANGLES_DEG[left]) /
            (ANGLES_DEG[left + 1] - ANGLES_DEG[left]))
    data = {}
    for name, values in table.items():
        if values is None:
            data[name] = None
        else:
            data[name] = values[left] + part * (values[left + 1] - values[left])
    return data


def pitching_moment_baseline(dataset, alpha_rad, data=None):
    """Return Cm0 + Cm_alpha(alpha) * alpha at the reference elevator."""
    if data is None:
        data = available_coefficients(dataset, alpha_rad)
    cm0 = available_coefficients(dataset, 0.0)["Cm"]
    return cm0 + data["Cm_alpha"] * alpha_rad

# C172N propeller data. The C_T table is from the PyFME Cessna 172 JavaProp data:
# https://github.com/AeroPython/PyFME/blob/master/src/pyfme/aircrafts/cessna_172.py
PROP_DIAMETER = 75.0 * 0.0254      # 75 inches, converted to metres
PROP_RPM = 2400.0                  # full-throttle static RPM from the C172N POH
PROP_CT_TABLE = [
    (0.00, 0.102122), (0.05, 0.110970), (0.10, 0.107621),
    (0.15, 0.105191), (0.20, 0.102446), (0.25, 0.099470),
    (0.30, 0.096775), (0.35, 0.094706), (0.40, 0.092341),
    (0.45, 0.088912), (0.50, 0.083878), (0.55, 0.076336),
    (0.60, 0.066669), (0.65, 0.056342), (0.70, 0.045688),
    (0.75, 0.034716), (0.76, 0.032492), (0.77, 0.030253),
    (0.78, 0.028001), (0.79, 0.025735), (0.80, 0.023453),
    (0.81, 0.021159), (0.82, 0.018852), (0.83, 0.016529),
    (0.84, 0.014194), (0.85, 0.011843), (0.86, 0.009479),
    (0.87, 0.007100), (0.88, 0.004686), (0.89, 0.002278),
    (0.90, -0.000200), (0.91, -0.002638), (0.92, -0.005145),
    (0.93, -0.007641), (0.94, -0.010188),
]

# These control values were not in the supplied PDF. Their source is:
# Kasnakoglu (2016), Table 1, https://doi.org/10.1177/0142331216661760
# Positive elevator means trailing edge down, giving a nose-down moment.
CL_Q_MISSING = 3.9
CM_Q_MISSING = -12.4
CL_DE = 0.43
CD_DE = 0.06
CM_DE = -1.28
CY_BETA = -0.31
CY_R = 0.21
CY_DA = 0.0
CY_DR = 0.187
CN_BETA = 0.065
CL_DA = -0.178
CL_DR = 0.0147
CN_DA = -0.053
CN_DR = -0.0657

# This is the speed at which the code finds the starting trim condition.
INITIAL_SPEED_KT = 100.0
TRIM_CURVE_SPEEDS_KT = range(55, 126, 5)

# This is the control input used after the aircraft has been trimmed.
# A negative elevator pulse is trailing edge up and pitches the nose up.
PULSE = {"control": "elevator", "size": -1.0, "start": 2.0, "end": 3.0}
STABILITY_CHECK_SECONDS = 60.0
STABILITY_LIMITS = {
    "max_speed_deviation_kt": 5.0,
    "max_attitude_deviation_deg": 10.0,
    "final_speed_deviation_kt": 1.0,
    "final_pitch_deviation_deg": 0.5,
    "final_rate_deg_s": 0.1,
}


def density(altitude_m):
    """Use a simple atmosphere. Higher altitude means lower density."""
    return 1.225 * exp(-max(altitude_m, 0.0) / 8500.0)


def thrust(throttle, speed, rho):
    """Find propeller thrust from the C172 McCauley 7557 C_T curve."""
    n = PROP_RPM / 60.0
    advance_ratio = speed / (n * PROP_DIAMETER)

    # Find C_T between two points on the graph using straight-line interpolation.
    ct = 0.0
    for left, right in zip(PROP_CT_TABLE, PROP_CT_TABLE[1:]):
        if left[0] <= advance_ratio <= right[0]:
            fraction = (advance_ratio - left[0]) / (right[0] - left[0])
            ct = left[1] + fraction * (right[1] - left[1])
            break

    # T = C_T * rho * n^2 * D^4.  Here throttle is a simple thrust fraction.
    full_throttle_thrust = ct * rho * n * n * PROP_DIAMETER ** 4
    return throttle * full_throttle_thrust


def trim_level_flight(dataset, speed_kt):
    """Find the elevator and throttle needed for straight level flight."""
    if not isfinite(speed_kt) or speed_kt <= 0.0:
        raise ValueError("Trim airspeed must be finite and positive.")
    speed = speed_kt * 0.514444
    rho = density(0.0)
    qbar = 0.5 * rho * speed * speed

    def balance(alpha):
        data = available_coefficients(dataset, alpha)

        # Pick elevator that gives zero pitching moment.
        elevator_rad = -pitching_moment_baseline(dataset, alpha, data) / CM_DE
        cl = data["CL"] + CL_DE * elevator_rad
        cd = data["CD"] + CD_DE * abs(elevator_rad)
        drag = qbar * S * cd

        # At positive alpha, the propeller has a small upward component.
        return qbar * S * cl - MASS * G + drag * tan(alpha), elevator_rad, cd

    # First bracket a force-balance root within the supplied alpha range.
    # Then bisect it instead of accepting/rejecting a coarse 0.01-degree grid.
    lower = ANGLES_DEG[0] * DEG
    lower_error = balance(lower)[0]
    upper = lower
    for index in range(1, 251):
        upper = (ANGLES_DEG[0] +
                 (ANGLES_DEG[-1] - ANGLES_DEG[0]) * index / 250.0) * DEG
        upper_error = balance(upper)[0]
        if lower_error * upper_error <= 0.0:
            break
        lower, lower_error = upper, upper_error
    else:
        raise RuntimeError(
            f"{dataset} at {speed_kt:g} kt: no level-flight force balance "
            f"within alpha {ANGLES_DEG[0]:g} to {ANGLES_DEG[-1]:g} deg.")

    for _ in range(60):
        trim_alpha = (lower + upper) / 2.0
        error, trim_elevator, trim_cd = balance(trim_alpha)
        if abs(error) < 1.0e-7:
            break
        if lower_error * error <= 0.0:
            upper = trim_alpha
        else:
            lower, lower_error = trim_alpha, error

    # Thrust is along the body x-axis, so T cos(alpha) balances drag.
    drag = qbar * S * trim_cd
    full_thrust = thrust(1.0, speed, rho)
    if full_thrust <= 0.0:
        raise RuntimeError(f"{dataset} at {speed_kt:g} kt: no positive propeller thrust.")
    throttle = drag / (cos(trim_alpha) * full_thrust)
    if not 0.0 <= throttle <= 1.0:
        raise RuntimeError(
            f"{dataset} at {speed_kt:g} kt: trim requires throttle {throttle:.3f}, "
            "outside 0 to 1.")

    controls = {"throttle": throttle, "elevator": trim_elevator / DEG,
                "aileron": 0.0, "rudder": 0.0}
    state = [speed * cos(trim_alpha), 0.0, speed * sin(trim_alpha),
             0.0, 0.0, 0.0, 0.0, trim_alpha, 0.0, 0.0, 0.0, 0.0]
    return controls, state


def trim_curve(dataset, speeds_kt=TRIM_CURVE_SPEEDS_KT):
    """Solve level-flight trim over a speed sweep.

    Unavailable trims are recorded as gaps with a reason. Forward body-axis
    speed is reported separately from the requested airspeed.
    """
    points = []
    for speed_kt in speeds_kt:
        try:
            controls, state = trim_level_flight(dataset, speed_kt)
        except RuntimeError as error:
            points.append({"airspeed_kt": speed_kt, "error": str(error),
                           "forward_speed_kt": float("nan"),
                           "elevator_deg": float("nan")})
            continue
        points.append({
            "airspeed_kt": speed_kt,
            "forward_speed_kt": state[0] / 0.514444,
            "elevator_deg": controls["elevator"],
            "alpha_deg": state[7] / DEG,
            "throttle": controls["throttle"],
        })
    return points


def forces_and_moments(state, controls, dataset):
    """Calculates forces and moments from the current aircraft state."""
    u, v, w, p, q, r, roll, pitch, yaw, north, east, down = state
    speed = max(sqrt(u * u + v * v + w * w), 1.0)
    alpha = atan2(w, u)
    beta = asin(max(-1.0, min(1.0, v / speed)))
    rho = density(-down)
    qbar = 0.5 * rho * speed * speed
    data = available_coefficients(dataset, alpha)

    p_hat = p * B / (2.0 * speed)
    q_hat = q * C / (2.0 * speed)
    r_hat = r * B / (2.0 * speed)

    de = controls["elevator"] * DEG
    da = controls["aileron"] * DEG
    dr = controls["rudder"] * DEG

    # Use the PDF value when it exists. DATCOM does not give these two values,
    # so use the C172 reference value listed above for that case.
    cl_q_value = CL_Q_MISSING if data["CL_q"] is None else data["CL_q"]
    cm_q_value = CM_Q_MISSING if data["Cm_q"] is None else data["Cm_q"]
    cl_q = cl_q_value * q_hat
    cm_q = cm_q_value * q_hat

    cl = data["CL"] + cl_q + CL_DE * de
    cd = data["CD"] + CD_DE * abs(de)
    cm = pitching_moment_baseline(dataset, alpha, data) + cm_q + CM_DE * de
    cy = CY_BETA * beta + data["CY_p"] * p_hat + CY_R * r_hat + CY_DA * da + CY_DR * dr
    cl_roll = (data["Cl_beta"] * beta + data["Cl_p"] * p_hat
               + data["Cl_r"] * r_hat + CL_DA * da + CL_DR * dr)
    cn = (CN_BETA * beta + data["Cn_p"] * p_hat + data["Cn_r"] * r_hat
          + CN_DA * da + CN_DR * dr)

    lift = qbar * S * cl
    drag = qbar * S * cd
    x_force = thrust(controls["throttle"], speed, rho) - drag * cos(alpha) + lift * sin(alpha)
    y_force = qbar * S * cy
    z_force = -drag * sin(alpha) - lift * cos(alpha)
    roll_moment = qbar * S * B * cl_roll
    pitch_moment = qbar * S * C * cm
    yaw_moment = qbar * S * B * cn
    return x_force, y_force, z_force, roll_moment, pitch_moment, yaw_moment

def derivatives(state, controls, dataset):
    """Calculate how all 12 state values change at this instant."""
    u, v, w, p, q, r, roll, pitch, yaw, north, east, down = state
    x_force, y_force, z_force, roll_moment, pitch_moment, yaw_moment = (
        forces_and_moments(state, controls, dataset))
    u_dot = r * v - q * w + x_force / MASS - G * sin(pitch)
    v_dot = p * w - r * u + y_force / MASS + G * sin(roll) * cos(pitch)
    w_dot = q * u - p * v + z_force / MASS + G * cos(roll) * cos(pitch)
    p_dot = (roll_moment - (IZZ - IYY) * q * r) / IXX
    q_dot = (pitch_moment - (IXX - IZZ) * p * r) / IYY
    r_dot = (yaw_moment - (IYY - IXX) * p * q) / IZZ

    # Convert the body rotation rates into changes in the Euler angles.
    if abs(cos(pitch)) < 0.1:
        raise ValueError("Pitch is too close to 90 degrees for this simple Euler-angle model.")
    roll_dot = p + q * sin(roll) * tan(pitch) + r * cos(roll) * tan(pitch)
    pitch_dot = q * cos(roll) - r * sin(roll)
    yaw_dot = (q * sin(roll) + r * cos(roll)) / cos(pitch)

    # Rotate the velocity from body axes into North-East-Down axes.
    # Each row gives North, East, or Down.
    body_to_ned = [
        [cos(pitch) * cos(yaw),
         sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw),
         cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw)],
        [cos(pitch) * sin(yaw),
         sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw),
         cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw)],
        [-sin(pitch), sin(roll) * cos(pitch), cos(roll) * cos(pitch)],
    ]
    body_velocity = [u, v, w]
    ned_velocity = [
        sum(row[column] * body_velocity[column] for column in range(3))
        for row in body_to_ned
    ]
    north_dot, east_dot, down_dot = ned_velocity
    return [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot,
            roll_dot, pitch_dot, yaw_dot, north_dot, east_dot, down_dot]


def simulate(dataset, controls, initial_state, seconds=15.0, dt=0.01, pulse=None):
    if dataset not in ("DATCOM", "AVL"):
        raise ValueError("DATASET must be DATCOM or AVL.")
    if seconds <= 0.0 or dt <= 0.0:
        raise ValueError("seconds and dt must both be positive.")
    for name in ("throttle", "elevator", "aileron", "rudder"):
        if name not in controls or not isfinite(controls[name]):
            raise ValueError(f"Enter a finite value for {name}.")
    if not 0.0 <= controls["throttle"] <= 1.0:
        raise ValueError("throttle must be between 0 and 1.")

    state = initial_state[:]
    history = [(0.0, state)]

    for step_number in range(int(seconds / dt)):
        time = step_number * dt
        active_controls = controls.copy()
        if pulse is not None and pulse["start"] <= time < pulse["end"]:
            active_controls[pulse["control"]] += pulse["size"]
        change = derivatives(state, active_controls, dataset)
        state = [value + dt * rate for value, rate in zip(state, change)]

        # These checks catch a numerical failure instead of plotting bad data.
        if not all(isfinite(value) for value in state):
            raise FloatingPointError("The calculation produced an invalid number.")
        speed = sqrt(state[0] ** 2 + state[1] ** 2 + state[2] ** 2)
        if speed > 150.0:  # Far outside the range expected for this C172 example.
            raise RuntimeError("The model speed became unrealistic; check the inputs.")
        history.append((time + dt, state))
    return history


def check_trim_stability(dataset, controls, trim_state, pulse=PULSE,
                         seconds=STABILITY_CHECK_SECONDS, dt=0.01):
    """Check that a brief elevator pulse stays bounded and settles near trim."""
    history = simulate(dataset, controls, trim_state, seconds, dt, pulse)
    initial_speed = sqrt(sum(value * value for value in trim_state[:3])) / 0.514444
    speeds = [
        sqrt(sum(value * value for value in state[:3])) / 0.514444
        for _, state in history
    ]
    max_speed_deviation = max(abs(speed - initial_speed) for speed in speeds)
    max_attitude_deviation = max(
        max(abs(state[index] - trim_state[index]) for index in (6, 7, 8)) / DEG
        for _, state in history
    )

    final_state = history[-1][1]
    final_speed_deviation = abs(speeds[-1] - initial_speed)
    final_pitch_deviation = abs(final_state[7] - trim_state[7]) / DEG
    final_rate = max(abs(rate) for rate in final_state[3:6]) / DEG
    metrics = {
        "max_speed_deviation_kt": max_speed_deviation,
        "max_attitude_deviation_deg": max_attitude_deviation,
        "final_speed_deviation_kt": final_speed_deviation,
        "final_pitch_deviation_deg": final_pitch_deviation,
        "final_rate_deg_s": final_rate,
    }
    stable = all(metrics[name] <= limit
                 for name, limit in STABILITY_LIMITS.items())
    return stable, metrics


def plot(history, dataset, controls, pulse):
    """Show the main results and save the same plot as a PNG file."""
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    time = [item[0] for item in history]
    states = [item[1] for item in history]
    speed = [sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2) / 0.514444 for x in states]

    # The last plot uses North/East distance, so it cannot share the time axis.
    figure, axes = plt.subplots(5, 1, figsize=(8, 9))
    axes[0].plot(time, speed); axes[0].set_ylabel("Speed (kt)")
    axes[1].plot(time, [-x[11] for x in states]); axes[1].set_ylabel("Altitude (m)")
    axes[2].plot(time, [x[6] / DEG for x in states], label="roll")
    axes[2].plot(time, [x[7] / DEG for x in states], label="pitch")
    axes[2].plot(time, [x[8] / DEG for x in states], label="yaw")
    axes[2].set_ylabel("Angle (deg)"); axes[2].legend()
    axes[3].plot(time, [x[1] for x in states]); axes[3].set_ylabel("Side speed\n(m/s)")
    axes[3].set_xlabel("Time (s)")
    axes[4].plot([x[9] for x in states], [x[10] for x in states])
    axes[4].set_xlabel("North (m)"); axes[4].set_ylabel("East (m)")
    figure.suptitle(
        f"C172 {dataset}: level-flight trim, then {pulse['size']:+.1f} deg "
        f"{pulse['control']} pulse from {pulse['start']:.0f} to {pulse['end']:.0f} s")
    figure.tight_layout()
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    figure.savefig(PLOT_DIR / "simple_c172_6dof_response.png", dpi=160)
    plt.show()


def plot_trim_curve(dataset, speeds_kt=TRIM_CURVE_SPEEDS_KT):
    """Plot the elevator deflection required for each trimmed forward speed."""
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    points = trim_curve(dataset, speeds_kt)
    valid = [point for point in points if "error" not in point]
    if not valid:
        raise RuntimeError("No valid trim points in the requested speed sweep: "
                           + "; ".join(point["error"] for point in points))
    forward_speed = [point["forward_speed_kt"] for point in points]
    elevator = [point["elevator_deg"] for point in points]

    figure, axis = plt.subplots(figsize=(7, 4.5))
    axis.plot(forward_speed, elevator, marker="o")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xlabel("Forward body-axis speed, u (kt)")
    axis.set_ylabel("Trim elevator deflection (deg, positive trailing edge down)")
    axis.set_title(f"C172 {dataset}: elevator required for level-flight trim")
    axis.grid(True, alpha=0.3)
    failed = [point for point in points if "error" in point]
    if failed:
        figure.text(0.5, 0.01, "No trim at requested airspeeds (kt): " +
                    ", ".join(f"{point['airspeed_kt']:g}" for point in failed),
                    ha="center", fontsize=8)
    figure.tight_layout(rect=(0, 0.05, 1, 1))
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    figure.savefig(PLOT_DIR / "simple_c172_6dof_trim_curve.png", dpi=160)
    plt.show()
    return points


if __name__ == "__main__":
    DATASET = "DATCOM"          # Change this to AVL to compare the two sets of data.
    trim_controls, trim_state = trim_level_flight(DATASET, INITIAL_SPEED_KT)
    print(f"Trim speed: {INITIAL_SPEED_KT:.1f} kt")
    print(f"Trim angle of attack: {trim_state[7] / DEG:.2f} deg")
    print(f"Trim throttle: {trim_controls['throttle']:.3f}")
    print(f"Trim elevator: {trim_controls['elevator']:.2f} deg")
    trim_forces = forces_and_moments(trim_state, trim_controls, DATASET)
    print(f"Trim roll moment:  {trim_forces[3]:.3f} N m")
    print(f"Trim pitch moment: {trim_forces[4]:.3f} N m")
    print(f"Trim yaw moment:   {trim_forces[5]:.3f} N m")
    stable, metrics = check_trim_stability(DATASET, trim_controls, trim_state)
    print(f"{STABILITY_CHECK_SECONDS:.0f} s elevator-pulse stability check: "
          f"{'PASS' if stable else 'FAIL'}")
    for name, value in metrics.items():
        print(f"  {name.replace('_', ' ')}: {value:.3f} "
              f"(limit {STABILITY_LIMITS[name]:.3f})")
    results = simulate(DATASET, trim_controls, trim_state, pulse=PULSE)
    plot(results, DATASET, trim_controls, PULSE)
    trim_points = plot_trim_curve(DATASET)
    print("Trim curve saved as: output/plots/simple_c172_6dof_trim_curve.png")
    for point in trim_points:
        if "error" in point:
            print(f"  Skipped: {point['error']}")
            continue
        print(f"  u={point['forward_speed_kt']:.1f} kt: "
              f"elevator={point['elevator_deg']:+.2f} deg, "
              f"alpha={point['alpha_deg']:.2f} deg")

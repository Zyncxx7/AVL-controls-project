"""Aircraft forces, 6DOF equations, time stepping and level-flight trim.

Start with forces(), then derivatives(), then simulate(). For the trim
comparison, read trim_at_alpha() followed by trim_solutions().
State order: u, v, w, p, q, r, roll, pitch, yaw, north, east, down.
"""
from math import atan2, cos, isfinite, pi, sin, sqrt, tan
from pathlib import Path
import sys

import aerodynamics as aero

# Reuse the original aircraft properties and engine without changing that file.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "01_simple_6dof"))
import simple_c172_6dof as original

MASS = original.MASS
G = original.G
IXX, IYY, IZZ = original.IXX, original.IYY, original.IZZ
KNOT = 0.514444  # metres per second in one knot


def forces(state, controls, hysteresis, alpha_dot=0):
    """Turn the six aerodynamic coefficients into forces and moments."""
    u, v, w = state[:3]
    down = state[11]
    speed = sqrt(u*u + v*v + w*w)
    rho = original.density(-down)
    thrust = original.thrust(controls["throttle"], speed, rho)

    coefficients = aero.coefficients(
        state, controls, hysteresis, alpha_dot,
        rho, thrust, original.PROP_DIAMETER)
    alpha = coefficients["alpha"]
    beta = coefficients["beta"]
    qbar_area = 0.5 * rho * speed**2 * aero.S

    lift = qbar_area * coefficients["CL"]
    drag = qbar_area * coefficients["CD"]
    side = qbar_area * coefficients["CY"]

    # Rotate wind-axis forces into forward/right/down body axes.
    x_aero = (-drag * cos(alpha) * cos(beta)
              - side * cos(alpha) * sin(beta) + lift * sin(alpha))
    y_aero = -drag * sin(beta) + side * cos(beta)
    z_aero = (-drag * sin(alpha) * cos(beta)
              - side * sin(alpha) * sin(beta) - lift * cos(alpha))

    roll_moment = qbar_area * aero.B * coefficients["Cl"]
    pitch_moment = qbar_area * aero.C * coefficients["Cm"]
    yaw_moment = qbar_area * aero.B * coefficients["Cn"]

    # Moments are treated at the common CG; see the README assumption.
    loads = (thrust + x_aero, y_aero, z_aero,
             roll_moment, pitch_moment, yaw_moment)
    return loads, coefficients


def rigid_body_derivatives(state, loads):
    """Write each of the 12 equations separately, like the original model."""
    u, v, w, p, q, r, roll, pitch, yaw, north, east, down = state
    x_force, y_force, z_force, roll_moment, pitch_moment, yaw_moment = loads

    # Translational acceleration, including gravity and rotating body axes.
    u_dot = r*v - q*w + x_force/MASS - G*sin(pitch)
    v_dot = p*w - r*u + y_force/MASS + G*sin(roll)*cos(pitch)
    w_dot = q*u - p*v + z_force/MASS + G*cos(roll)*cos(pitch)

    # Rotational acceleration; inertias are about the body principal axes.
    p_dot = (roll_moment - (IZZ-IYY)*q*r) / IXX
    q_dot = (pitch_moment - (IXX-IZZ)*p*r) / IYY
    r_dot = (yaw_moment - (IYY-IXX)*p*q) / IZZ

    if abs(cos(pitch)) < 0.1:
        raise ValueError("Pitch is too close to 90 degrees for Euler angles.")
    roll_dot = p + (q*sin(roll) + r*cos(roll))*tan(pitch)
    pitch_dot = q*cos(roll) - r*sin(roll)
    yaw_dot = (q*sin(roll) + r*cos(roll)) / cos(pitch)

    # Rotate body velocity into North-East-Down position rates.
    north_dot = (
        u*cos(pitch)*cos(yaw)
        + v*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw))
        + w*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)))
    east_dot = (
        u*cos(pitch)*sin(yaw)
        + v*(sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw))
        + w*(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw)))
    down_dot = -u*sin(pitch) + v*sin(roll)*cos(pitch) + w*cos(roll)*cos(pitch)

    return [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot,
            roll_dot, pitch_dot, yaw_dot, north_dot, east_dot, down_dot]


def derivatives(state, controls, hysteresis):
    """Calculate the state rates, keeping the PDF's alpha-dot derivatives.

    Lift depends on alpha-dot, while alpha-dot depends on acceleration.
    Solve that feedback algebraically; do not drop the unsteady terms.
    """
    u, v, w = state[:3]
    longitudinal_speed = sqrt(u*u + w*w)
    if longitudinal_speed < 1:
        raise ValueError("Insufficient longitudinal airspeed.")

    # First calculate acceleration without the alpha-dot lift term.
    loads, _ = forces(state, controls, hysteresis, alpha_dot=0)
    rates = rigid_body_derivatives(state, loads)
    alpha_dot_without_feedback = (u*rates[2] - w*rates[0]) / longitudinal_speed**2

    # Extra lift per rad/s of alpha-dot. Its effect opposes alpha-dot.
    speed = sqrt(u*u + v*v + w*w)
    rho = original.density(-state[11])
    qbar_area = 0.5 * rho * speed**2 * aero.S
    lift_per_alpha_dot = qbar_area * aero.CL_ADOT * aero.C / (2*speed)
    feedback = lift_per_alpha_dot / (MASS * longitudinal_speed)
    alpha_dot = alpha_dot_without_feedback / (1 + feedback)

    loads, _ = forces(state, controls, hysteresis, alpha_dot)
    return rigid_body_derivatives(state, loads)


def shifted_state(state, rates, step):
    """Make a trial state: current value + timestep * rate of change."""
    trial = []
    for value, rate in zip(state, rates):
        trial.append(value + step*rate)
    return trial


def trial_rates(state, controls, previous_hysteresis):
    """Evaluate a trial state without changing the saved stall memory."""
    alpha = atan2(state[2], state[0])
    branch = aero.update_hysteresis(alpha, previous_hysteresis)
    return derivatives(state, controls, branch)


def rk4_step(state, controls, hysteresis, step):
    """One RK4 step: start, midpoint, midpoint, end; then average slopes."""
    k1 = trial_rates(state, controls, hysteresis)
    midpoint = shifted_state(state, k1, step/2)
    k2 = trial_rates(midpoint, controls, hysteresis)
    midpoint = shifted_state(state, k2, step/2)
    k3 = trial_rates(midpoint, controls, hysteresis)
    endpoint = shifted_state(state, k3, step)
    k4 = trial_rates(endpoint, controls, hysteresis)

    next_state = []
    for index in range(12):
        average_rate = (k1[index] + 2*k2[index] + 2*k3[index] + k4[index]) / 6
        next_state.append(state[index] + step*average_rate)
    return next_state


def simulate(controls, initial_state, seconds=15, dt=0.01, pulse=None, hysteresis=0):
    """Apply controls, advance the aircraft, update stall memory, save state."""
    if not isfinite(seconds) or not isfinite(dt) or seconds <= 0 or dt <= 0:
        raise ValueError("Duration and timestep must be finite and positive.")
    if len(initial_state) != 12 or not all(isfinite(x) for x in initial_state):
        raise ValueError("Supply 12 finite state values.")
    for name in ("throttle", "elevator", "aileron", "rudder"):
        if not isfinite(controls[name]):
            raise ValueError("Controls must be finite.")
    if not 0 <= controls["throttle"] <= 1:
        raise ValueError("Throttle must be between zero and one.")

    state = list(initial_state)
    alpha = atan2(state[2], state[0])
    hysteresis = aero.update_hysteresis(alpha, hysteresis)
    history = [(0.0, state[:], hysteresis)]
    time = 0.0

    while time < seconds - 1e-12:
        step = min(dt, seconds-time)

        # End a step at an input change, so a pulse is not applied too long.
        if pulse is not None:
            for edge in (pulse["start"], pulse["end"]):
                if time + 1e-12 < edge < time + step - 1e-12:
                    step = edge-time

        active_controls = controls.copy()
        if pulse is not None:
            if pulse["start"] - 1e-12 <= time < pulse["end"] - 1e-12:
                active_controls[pulse["control"]] += pulse["size"]

        state = rk4_step(state, active_controls, hysteresis, step)
        speed = sqrt(sum(value*value for value in state[:3]))
        if not all(isfinite(value) for value in state) or speed > 150:
            raise FloatingPointError("The aircraft left the supported flight range.")

        alpha = atan2(state[2], state[0])
        hysteresis = aero.update_hysteresis(alpha, hysteresis)
        time = min(seconds, time+step)
        history.append((time, state[:], hysteresis))
    return history


def trim_at_alpha(speed_kt, alpha, hysteresis):
    """For one trial alpha, balance pitch and thrust; return lift imbalance.

    The remaining imbalance is what trim_solutions() drives to zero.
    """
    if not isfinite(speed_kt) or speed_kt <= 0:
        raise ValueError("Trim speed must be finite and positive.")
    speed = speed_kt * KNOT
    rho = original.density(0)
    qbar_area = 0.5 * rho * speed**2 * aero.S
    propeller_area = pi * original.PROP_DIAMETER**2 / 4
    base_drag_coefficient = aero.CD_ZERO + aero.interpolate(alpha, aero.CD_ALPHA, aero.CD_WING)

    # At trim: thrust*cos(alpha) = drag. Therefore tail pressure / qbar
    # is base_tail_ratio + elevator_drag_factor*abs(elevator).
    base_tail_ratio = (cos(alpha)**2
                       + aero.S*base_drag_coefficient / (propeller_area*cos(alpha)))
    elevator_drag_factor = aero.S*aero.CD_DE / (propeller_area*cos(alpha))

    # Zero pitching moment gives a quadratic in elevator magnitude:
    # elevator_drag_factor*magnitude^2 + base_tail_ratio*magnitude
    #     = abs(-static_pitch / CM_DE).
    static_pitch = aero.CM_ZERO + aero.CM_ALPHA*alpha
    required_elevator_effect = -static_pitch / aero.CM_DE
    discriminant = (base_tail_ratio**2
                    + 4*elevator_drag_factor*abs(required_elevator_effect))
    magnitude = (2*abs(required_elevator_effect)
                 / (base_tail_ratio + sqrt(discriminant)))
    elevator = magnitude
    if required_elevator_effect < 0:
        elevator = -magnitude

    drag_coefficient = base_drag_coefficient + aero.CD_DE*abs(elevator)
    drag = qbar_area * drag_coefficient
    required_thrust = drag / cos(alpha)
    full_throttle_thrust = original.thrust(1, speed, rho)
    throttle = float("inf")
    if full_throttle_thrust > 0:
        throttle = required_thrust / full_throttle_thrust

    lift_coefficient = aero.wing_lift(alpha, hysteresis) + aero.CL_DE*elevator
    force_error = qbar_area*lift_coefficient + drag*tan(alpha) - MASS*G
    controls = {"throttle": throttle, "elevator": elevator/aero.DEG,
                "aileron": 0.0, "rudder": 0.0}
    state = [speed*cos(alpha), 0, speed*sin(alpha), 0, 0, 0,
             0, alpha, 0, 0, 0, 0]
    return force_error, controls, state


def bisect_trim(speed_kt, hysteresis, lower, upper):
    """Keep the half-interval that brackets zero force error."""
    lower_error = trim_at_alpha(speed_kt, lower, hysteresis)[0]
    upper_error = trim_at_alpha(speed_kt, upper, hysteresis)[0]
    if abs(lower_error) < 1e-8:
        return lower
    if abs(upper_error) < 1e-8:
        return upper
    for _ in range(50):
        midpoint = (lower+upper) / 2
        middle_error = trim_at_alpha(speed_kt, midpoint, hysteresis)[0]
        if lower_error*middle_error <= 0:
            upper = midpoint
        else:
            lower = midpoint
            lower_error = middle_error
    return (lower+upper) / 2


def trim_solutions(speed_kt, hysteresis=0):
    """Scan alpha, bracket every force-balance root, then refine each root."""
    if hysteresis not in (0, 1):
        raise ValueError("Hysteresis state must be zero or one.")

    # Include table corners as well as a fine grid: a post-stall root can
    # coexist with a lower-alpha root at the same speed.
    alpha_grid = []
    for index in range(901):
        alpha = aero.ALPHA[0] + index*(aero.ALPHA[-1]-aero.ALPHA[0])/900
        alpha_grid.append(alpha)
    alpha_grid.extend(aero.ALPHA)
    alpha_grid.extend(aero.CD_ALPHA)
    alpha_grid = sorted(set(alpha_grid))
    alpha_grid = [alpha for alpha in alpha_grid if aero.ALPHA[0] <= alpha <= aero.ALPHA[-1]]

    roots = []
    for lower, upper in zip(alpha_grid, alpha_grid[1:]):
        lower_error = trim_at_alpha(speed_kt, lower, hysteresis)[0]
        upper_error = trim_at_alpha(speed_kt, upper, hysteresis)[0]
        if lower_error*upper_error > 0:
            continue
        alpha = bisect_trim(speed_kt, hysteresis, lower, upper)
        if roots and abs(alpha-roots[-1]["alpha_rad"]) < 1e-7:
            continue
        if aero.update_hysteresis(alpha, hysteresis) != hysteresis:
            continue

        force_error, controls, state = trim_at_alpha(speed_kt, alpha, hysteresis)
        feasible = 0 <= controls["throttle"] <= 1
        acceleration_error = None
        if feasible:
            rates = derivatives(state, controls, hysteresis)
            acceleration_error = max(abs(rate) for rate in rates[:9])

        roots.append({
            "airspeed_kt": speed_kt,
            "forward_speed_kt": state[0]/KNOT,
            "alpha_rad": alpha, "alpha_deg": alpha/aero.DEG,
            "hysteresis": hysteresis,
            "elevator_deg": controls["elevator"],
            "throttle": controls["throttle"],
            "force_residual_N": force_error,
            "max_acceleration_residual": acceleration_error,
            "feasible": feasible, "controls": controls, "state": state})
    return roots

"""Clean C172P aerodynamics from Ranjith's aerodynamics.pdf, in SI units.

This module evaluates coefficients, not AVL. See README for the common mass,
propulsion and moment-reference assumptions used in the comparison.
"""
from math import asin, atan2, cos, isfinite, pi, sqrt

S = 174.0 * 0.09290304
B = 35.8 * 0.3048
C = 4.9 * 0.3048
DEG = pi / 180.0
HYST_MIN, HYST_MAX = 0.09, 0.36
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
# Derivatives are per radian. Rate derivatives multiply p_hat, q_hat,
# r_hat or alpha_dot_hat, rather than the dimensional rates directly.
CL_DE = 0.43
CL_Q = 3.9
CL_ADOT = 1.7
CM_ZERO = 0.1
CM_ALPHA = -1.8
CM_Q = -12.4
CM_ADOT = -7.27
CM_DE = -1.122
CD_ZERO = 0.027
CD_DE = 0.06
CD_BETA = 0.17

# Lateral derivatives: side force (CY), rolling moment (ROLL), yaw (YAW).
CY_DR = 0.187
ROLL_P = -0.484
ROLL_DA = 0.229
ROLL_DR = 0.0147
YAW_P = -0.0278
YAW_R = -0.0937
YAW_DA = -0.0053
YAW_DR = -0.043


def interpolate(x, grid, values):
    """Piecewise linear table with endpoint holding, as in JSBSim tables."""
    if not isfinite(x):
        raise ValueError("Table input must be finite.")
    if x <= grid[0]:
        return values[0]
    for a, b, ya, yb in zip(grid, grid[1:], values, values[1:]):
        if x <= b:
            return ya + (yb - ya) * (x - a) / (b - a)
    return values[-1]


def update_hysteresis(alpha, previous):
    """JSBSim FGAerodynamics: retain memory strictly between the thresholds.

    The 0.28-rad CL maximum is NOT the switching threshold. Do not choose a
    branch from the sign of alpha-dot or reset it at a reversal in alpha.
    """
    if previous not in (0, 1) or not isfinite(alpha):
        raise ValueError("Use finite alpha and a binary hysteresis state.")
    if alpha > HYST_MAX:
        return 1
    if alpha < HYST_MIN:
        return 0
    return previous


def wing_lift(alpha, hysteresis):
    if hysteresis not in (0, 1):
        raise ValueError("Hysteresis state must be zero or one.")
    if hysteresis == 1:
        return interpolate(alpha, ALPHA, CL_STALLED)
    return interpolate(alpha, ALPHA, CL_ATTACHED)


def tail_pressure_ratio(alpha, speed, rho, thrust, propeller_diameter):
    """PDF tail-pressure formula, with actuator-disk induced velocity.

    This induced-velocity closure is an explicit approximation; the common
    teaching-model propeller does not return JSBSim's induced velocity.
    """
    area = pi * propeller_diameter**2 / 4
    axial = speed * cos(alpha)
    induced = (sqrt(axial**2 + 2 * max(thrust, 0.0) / (rho * area)) - axial) / 2
    return ((axial + 2 * induced) / speed)**2


def coefficients(state, controls, hysteresis, alpha_dot, rho, thrust, propeller_diameter):
    """Read this function from top to bottom: flow, controls, then six axes."""
    u, v, w, p, q, r = state[:6]
    speed = sqrt(u*u + v*v + w*w)
    if speed < 1 or u <= 0:
        raise ValueError("The model requires forward flight above 1 m/s.")
    if controls.get("flaps", 0) != 0:
        raise ValueError("This comparison implements the clean (zero-flap) PDF configuration.")
    # 1. Flow angles and non-dimensional rotation rates.
    alpha = atan2(w, u)
    beta = asin(max(-1, min(1, v / speed)))
    p_hat = p * B / (2 * speed)
    q_hat = q * C / (2 * speed)
    r_hat = r * B / (2 * speed)
    alpha_dot_hat = alpha_dot * C / (2 * speed)

    # 2. Controls are entered in degrees; the equations use radians.
    elevator = controls["elevator"] * DEG
    aileron = controls["aileron"] * DEG
    rudder = controls["rudder"] * DEG
    tail_ratio = tail_pressure_ratio(alpha, speed, rho, thrust, propeller_diameter)

    # 3. Lift: static nonlinear table, elevator and unsteady terms.
    lift_static = wing_lift(alpha, hysteresis)
    lift_elevator = CL_DE * elevator
    lift_pitch_rate = CL_Q * q_hat
    lift_alpha_rate = CL_ADOT * alpha_dot_hat
    lift = lift_static + lift_elevator + lift_pitch_rate + lift_alpha_rate

    # 4. Drag: wing table, parasite drag and control/sideslip penalties.
    drag_wing = interpolate(alpha, CD_ALPHA, CD_WING)
    drag = CD_ZERO + drag_wing + CD_DE * abs(elevator) + CD_BETA * abs(beta)

    # 5. Pitching moment, including propwash on the elevator.
    pitch_static = CM_ZERO + CM_ALPHA * alpha
    pitch_rate = CM_Q * q_hat
    pitch_alpha_rate = CM_ADOT * alpha_dot_hat
    pitch_elevator = tail_ratio * CM_DE * elevator
    pitch = pitch_static + pitch_rate + pitch_alpha_rate + pitch_elevator

    # 6. Side force. The rate derivatives are also tables in alpha.
    side_beta = interpolate(beta, [-0.349, 0, 0.349], [0.137, 0, -0.137])
    side_p = interpolate(alpha, [0, 0.094], [-0.075, -0.145])
    side_r = interpolate(alpha, [0, 0.094], [0.214, 0.267])
    side = side_beta + CY_DR * rudder + side_p * p_hat + side_r * r_hat

    # 7. Rolling moment: sideslip, damping and control deflections.
    roll_beta = interpolate(beta, [-0.349, 0, 0.349], [0.0322, 0, -0.0322])
    roll_r = interpolate(alpha, [0, 0.094], [0.0798, 0.1869])
    roll = (roll_beta + ROLL_P * p_hat + roll_r * r_hat
            + ROLL_DA * aileron + ROLL_DR * rudder)

    # 8. Yawing moment, including propwash on the rudder.
    yaw_beta = interpolate(beta, [-0.349, 0, 0.349], [-0.0205, 0, 0.0205])
    yaw = (yaw_beta + YAW_P * p_hat + YAW_R * r_hat
           + YAW_DA * aileron + YAW_DR * rudder * tail_ratio)

    return {"CL": lift, "CD": drag, "Cm": pitch, "CY": side, "Cl": roll, "Cn": yaw,
            "alpha": alpha, "beta": beta, "tail_pressure_ratio": tail_ratio,
            "lift_table_endpoint_held": not ALPHA[0] <= alpha <= ALPHA[-1],
            "drag_table_endpoint_held": not CD_ALPHA[0] <= alpha <= CD_ALPHA[-1]}

# dynamics(x, u) = dotx

import numpy as np

MASS = 1.20
DX = 0.225
DY = 0.225
J = np.diag([1.25e-2, 1.25e-2, 2.20e-2])
J_INV = np.linalg.inv(J)
CR = 0.015
G = 9.81

def rotor_to_wrench(thrusts):
    T0, T1, T2, T3 = thrusts
    thrust_body = np.array([0, 0, -(T0 + T1 + T2 + T3)])
    torque_body = np.array(
        [
            DY * (-T0 - T1 + T2 + T3),
            DX * (-T0 + T1 + T2 - T3),
            CR * (-T0 + T1 - T2 + T3)
        ]
    )

    return thrust_body, torque_body

def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])
    return R

def quaternion_derivative(q, omega_body):
    qw, qx, qy, qz = q
    p, q_rate, r = omega_body
    Omega = np.array([
        [0, -p, -q_rate, -r],
        [p, 0, r, -q_rate],
        [q_rate, -r, 0, p],
        [r, q_rate, -p, 0]
    ])

    quaternion_dot = 0.5 * (Omega @ q)
    return quaternion_dot

def dynamics(state, thrusts):
    position = state[0:3]
    velocity = state[3:6]
    quaternion = state[6:10]
    omega_body = state[10:13]

    thrust_body, torque_body = rotor_to_wrench(thrusts)
    R_WB = quaternion_to_rotation_matrix(quaternion)
    position_dot = velocity
    velocity_dot = (R_WB @ thrust_body) / MASS + np.array([0, 0, G])
    quaternion_dot = quaternion_derivative(quaternion, omega_body)
    omega_dot = J_INV @ (torque_body - np.cross(omega_body, J @ omega_body))
    state_dot = np.concatenate([position_dot, velocity_dot, quaternion_dot, omega_dot])

    return state_dot

def step(state, thrusts, dt):
    # state_dot = dynamics(state, thrusts)
    # next_state = state + dt * state_dot

    # RK4
    k1 = dynamics(state, thrusts)
    k2 = dynamics(state + 0.5 * dt * k1, thrusts)
    k3 = dynamics(state + 0.5 * dt * k2, thrusts)
    k4 = dynamics(state + dt * k3, thrusts)

    next_state = state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

    # quaternion normalization
    q = next_state[6:10]
    q = q / np.linalg.norm(q)

    next_state[6:10] = q

    return next_state
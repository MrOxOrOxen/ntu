# kalman filter
# x_k|k, u, z(from sensor) -> x_k+1|k+1, P_k+1|k+1

import numpy as np
from dynamics import *
from sensors import *

# partial derivative, jacobian matrix
def numerical_jacobian(func, x, eps=1e-6):
    x = np.asarray(x, dtype=float)
    function = func(x)
    len_x = len(x)
    len_f0 = len(function)

    J = np.zeros((len_f0, len_x))

    for i in range(len_x):
        x_plus = x.copy()
        x_minus = x.copy()
        x_plus[i] += eps
        x_minus[i] -= eps

        f_plus = func(x_plus)
        f_minus = func(x_minus)

        J[:, i] = (f_plus - f_minus) / (2*eps)

    return J

# x -> IMU output (accel + gyro), function f(x,u) in kalman filter
def imu_measurement_model(state, thrusts):
    omega_body = state[10:13]
    state_dot = dynamics(state, thrusts)

    q = state[6:10]
    R_WB = quaternion_to_rotation_matrix(q)

    acceleration_world = state_dot[3:6]
    specific_force_body = R_WB.T @ (acceleration_world - np.array([0, 0, G]))

    measurement = np.concatenate([specific_force_body, omega_body])

    return measurement

# x -> GNSS output (position), function h(x) in kalman filter
def gnss_measurement_model(state):
    return state[0:3]

class EKF:
    def __init__(self, initial_state, initial_covariance=None, process_noise=None):
        self.x = np.array(initial_state, dtype=float)
        # P matrix: initial covariance, self-defined
        if initial_covariance is None:
            self.P = np.eye(13) * 0.1
        else:
            self.P = np.array(initial_covariance, dtype=float)

        # Q matrix: process noise, self-defined
        if process_noise is None:
            q_diag = np.array([
                1e-5, 1e-5, 1e-5,
                1e-4, 1e-4, 1e-4,
                1e-6, 1e-6, 1e-6, 1e-6,
                1e-4, 1e-4, 1e-4
            ]) # position, velocity, quat, angular velocity
            self.Q = np.diag(q_diag)

        else:
            self.Q = np.array(process_noise, dtype=float)

        # imu measurement covariance
        self.R_imu = np.diag([
            SIGMA_ACCEL**2, SIGMA_ACCEL**2, SIGMA_ACCEL**2,
            SIGMA_GYRO**2, SIGMA_GYRO**2, SIGMA_GYRO**2
        ])

        # gnss measurement covariance
        self.R_gnss = np.eye(3) * SIGMA_GNSS_POS**2

    def normalize_quaternion(self):
        q = self.x[6:10]
        norm = np.linalg.norm(q)
        if norm > 0:
            self.x[6:10] = q / norm

    # kalman filter: step 1, x: x_k|k -> x_k+1|k
    def predict(self, thrusts, dt):
        x_previous = self.x.copy()
        self.x = step(x_previous, thrusts, dt)
        self.normalize_quaternion()

        def transition_function(x):
            return step(x, thrusts, dt)

        F = numerical_jacobian(transition_function, x_previous)
        self.P = F @ self.P @ F.T + self.Q

    # kalman filter: step 2, x: k+1|k -> k+1|k+1, z: sensor measurement value
    def measurement_update(self, z, h_function, R):
        z_pred = h_function(self.x)
        innovation = z - z_pred
        H = numerical_jacobian(h_function, self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.normalize_quaternion()

        I = np.eye(13)
        self.P = (I - (K@H)) @ self.P @ (I - (K@H)).T + (K @ R @ K.T)

    # execute function measurement_update with imu data
    def update_imu(self, accel_measurement, gyro_measurement, thrusts):
        z = np.concatenate([accel_measurement, gyro_measurement])

        def h_imu(x):
            return imu_measurement_model(x, thrusts)

        self.measurement_update(z, h_imu, self.R_imu)

    # execute function measurement_update with gnss data
    def update_gnss(self, position_measurement):
        self.measurement_update(position_measurement, gnss_measurement_model, self.R_gnss)
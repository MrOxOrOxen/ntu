# random noise of IMU(gyro, accel) and GNSS(position)

import numpy as np
from dynamics import *

IMU_RATE = 100
IMU_DT = 1 / IMU_RATE
GNSS_RATE = 20
GNSS_DT = 1 / GNSS_RATE
SIGMA_ACCEL = 0.08
SIGMA_GYRO = 0.015
SIGMA_GNSS_POS = 0.02

class SensorModel:
    def __init__(self, sigma_accel_bias_rw=0, sigma_gyro_bias_rw=0, seed=None):
        self.sigma_accel_bias_rw = sigma_accel_bias_rw
        self.sigma_gyro_bias_rw = sigma_gyro_bias_rw
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.rng = np.random.default_rng(seed) # generate random number

    def update_biases(self, dt):
        accel_bias_noise = self.rng.normal(0, self.sigma_accel_bias_rw, size=3)
        gyro_bias_noise = self.rng.normal(0, self.sigma_gyro_bias_rw, size=3)
        self.accel_bias += accel_bias_noise * np.sqrt(dt)
        self.gyro_bias += gyro_bias_noise * np.sqrt(dt)

    def simulate_imu(self, state, thrusts, dt=IMU_DT):
        self.update_biases(dt)
        q = state[6:10]
        omega_body = state[10:13]
        state_dot = dynamics(state, thrusts)
        
        R_WB = quaternion_to_rotation_matrix(q)
        R_BW = R_WB.T

        acceleration_world = state_dot[3:6]
        gravity_world = np.array([0, 0, G])
        specific_force_world = acceleration_world - gravity_world
        spacific_force_body = R_BW @ specific_force_world

        accel_noise = self.rng.normal(0, SIGMA_ACCEL, size=3)
        accel_measured = spacific_force_body + self.accel_bias + accel_noise
        gyro_noise = self.rng.normal(0, SIGMA_GYRO, size=3)
        gyro_measured = omega_body + self.gyro_bias + gyro_noise

        return accel_measured, gyro_measured

    def simulate_gnss(self, state):
        true_position = state[0:3]
        position_noise = self.rng.normal(0, SIGMA_GNSS_POS, size=3)
        position_measured = true_position + position_noise
        return position_measured
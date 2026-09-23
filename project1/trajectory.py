# t -> p_r, v_r, a_r, yaw_r of the traj

import numpy as np

AX = 3
AY = 2
AZ = 0.5
Z0 = -2.5
OMEGA0 = 0.25
TF = 60

def ref_traj(t):
    x_r = AX * np.sin(OMEGA0 * t)
    y_r = AY * np.sin(2 * OMEGA0 * t)
    z_r = Z0 + AZ * np.cos(OMEGA0 * t)
    position_ref = np.array([x_r, y_r, z_r])

    x_dot_r = AX * OMEGA0 * np.cos(OMEGA0 * t)
    y_dot_r = 2 * AY * OMEGA0 * np.cos(2 * OMEGA0 * t)
    z_dot_r = -AZ * OMEGA0 * np.sin(OMEGA0 * t)
    velocity_ref = np.array([x_dot_r, y_dot_r, z_dot_r])

    x_ddot_r = -AX * OMEGA0 * OMEGA0 * np.sin(OMEGA0 * t)
    y_ddot_r = -4 * AY * OMEGA0 * OMEGA0 * np.sin(2 * OMEGA0 * t)
    z_ddot_r = -AZ * OMEGA0 * OMEGA0 * np.cos(OMEGA0 * t)
    acceleration_ref = np.array([x_ddot_r, y_ddot_r, z_ddot_r])

    yaw_ref = np.arctan2(y_dot_r, x_dot_r)

    return position_ref, velocity_ref, acceleration_ref, yaw_ref
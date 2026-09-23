import numpy as np
import matplotlib.pyplot as plt
import time
from dynamics import step
from sensors import SensorModel, IMU_DT, GNSS_DT
from ekf import EKF
from nmpc import NMPC, NMPC_DT, N, T_MAX, T_MIN
from trajectory import ref_traj, TF

DT = IMU_DT
GNSS_INTERVAL = int(round(GNSS_DT / DT))
NMPC_INTERVAL = int(round(NMPC_DT / DT))
NUM_STEPS = int(round(TF / DT))

# build ref traj used by nmpc: [x_ref, y_ref, z, vx, vy, vz, yaw]
def build_ref_horizon(current_time):
    ref = np.zeros((7, N+1))
    for j in range(N+1):
        t_ref = current_time + j * NMPC_DT
        position_ref, velocity_ref, acceleration_ref, yaw_ref = ref_traj(t_ref)
        ref[0:3, j] = position_ref
        ref[3:6, j] = velocity_ref
        ref[6, j] = yaw_ref
    return ref

def yaw_to_quat(yaw):
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])

def main():
    # initial reference
    position_ref_0, velocity_ref_0, acceleration_ref_0, yaw_ref_0 = ref_traj(0)

    # true initial state
    true_state = np.zeros(13)
    true_state[0:3] = position_ref_0
    true_state[3:6] = velocity_ref_0
    true_state[6:10] = yaw_to_quat(yaw_ref_0)
    true_state[10:13] = np.zeros(3)

    # ekf initial state
    initial_estimate = true_state.copy()
    initial_estimate[0:3] += np.array([0.05, -0.05, 0.03])
    initial_estimate[3:6] += np.array([0.02, -0.02, 0.01])

    # initial covariance P0
    initial_covariance = np.diag([
        0.2**2, 0.2**2, 0.2**2,
        0.2**2, 0.2**2, 0.2**2,
        0.05**2, 0.05**2, 0.05**2, 0.05**2,
        0.1**2, 0.1**2, 0.1**2
    ])

    # process noise Q
    process_noise = np.diag([
        1e-6, 1e-6, 1e-6,
        1e-4, 1e-4, 1e-4,
        1e-7, 1e-7, 1e-7, 1e-7,
        1e-5, 1e-5, 1e-5
    ])

    # sensor
    sensor = SensorModel(sigma_accel_bias_rw=0.005, sigma_gyro_bias_rw=0.0005, seed=42)

    # ekf
    ekf = EKF(initial_state=initial_estimate, initial_covariance=initial_covariance, process_noise=process_noise)

    # nmpc
    controller = NMPC(horizon=N, dt=NMPC_DT)

    # batch data
    time_history = []
    true_history = []
    estimate_history = []
    covariance_history = []
    reference_history = []
    thrust_history = []
    nmpc_solve_times = []
    nmpc_success_count = 0
    nmpc_solve_count = 0

    # initial nmpc solution
    ref_horizon = build_ref_horizon(0)
    start_time = time.perf_counter()
    thrust_command, predicted_states, predicted_controls = controller.solve(ekf.x, ref_horizon)
    solve_time = time.perf_counter() - start_time
    nmpc_solve_times.append(solve_time)
    nmpc_solve_count += 1
    if predicted_states is not None:
        nmpc_success_count += 1

    # save data of t=0
    time_history.append(0)
    true_history.append(true_state.copy())
    estimate_history.append(ekf.x.copy())
    covariance_history.append(np.diag(ekf.P).copy())
    reference_history.append(position_ref_0.copy())
    thrust_history.append(thrust_command.copy())

    # main simulation
    for k in range(NUM_STEPS):
        t = k * DT
        t_next = (k+1) * DT

        true_state = step(true_state, thrust_command, DT)
        ekf.predict(thrust_command, DT)
        accel_measurement, gyro_measurement = sensor.simulate_imu(true_state, thrust_command, DT)
        ekf.update_imu(accel_measurement, gyro_measurement, thrust_command)

        if (k+1) % GNSS_INTERVAL == 0:
            gnss_measurement = sensor.simulate_gnss(true_state)
            ekf.update_gnss(gnss_measurement)

        if (k+1) % NMPC_INTERVAL == 0:
            ref_horizon = build_ref_horizon(t_next)
            start_time = time.perf_counter()
            new_thrust_command, predicted_states, predicted_controls = controller.solve(ekf.x, ref_horizon)
            solve_time = time.perf_counter() - start_time
            nmpc_solve_times.append(solve_time)
            nmpc_solve_count += 1
            if predicted_states is not None:
                nmpc_success_count += 1
            thrust_command = new_thrust_command

        current_position_ref, current_velocity_ref, current_acceleration_ref, current_yaw_ref = ref_traj(t_next)

        time_history.append(t_next)
        true_history.append(true_state.copy())
        estimate_history.append(ekf.x.copy())
        reference_history.append(current_position_ref.copy())
        thrust_history.append(thrust_command.copy())
        covariance_history.append(np.diag(ekf.P).copy())

        if (k+1) % int(1 / DT) == 0:
            position_error = true_state[0:3] - current_position_ref
            estimation_error = true_state[0:3] - ekf.x[0:3]

            print(f"t = {t_next:.2f}s\n")
            print(f"true position = {true_state[0:3]}\n")
            print(f"EKF position = {ekf.x[0:3]}\n")
            print(f"tracking error = {position_error}\n")
            print(f"estimation_error = {estimation_error}\n")
            print(f"thrust = {thrust_command}\n")
            print("-" * 30)

    time_history = np.array(time_history)
    true_history = np.array(true_history)
    estimate_history = np.array(estimate_history)
    estimation_error_history = np.array(true_history - estimate_history)
    covariance_history = np.array(covariance_history)
    reference_history = np.array(reference_history)
    thrust_history = np.array(thrust_history)
    nmpc_solve_times = np.array(nmpc_solve_times)

    tracking_error = true_history[:, 0:3] - reference_history
    # estimation_error = true_history - estimate_history
    sigma_history = np.sqrt(np.maximum(covariance_history, 0))
    two_sigma = 2 * sigma_history

    tracking_rmse = np.sqrt(np.mean(tracking_error**2, axis=0))
    ekf_position_rmse = np.sqrt(np.mean(estimation_error_history[:, 0:3]**2, axis=0))

    print("\n")
    print("full gnc sim results\n")
    print(f"position tracking rmse(m): {tracking_rmse}\n")
    print(f"ekf position estimation rmse(m): {ekf_position_rmse}\n")
    print(f"nmpc solver success rate: {nmpc_success_count / nmpc_solve_count}\n")
    print(f"avg nmpc solve time(s): {np.mean(nmpc_solve_times)}\n")
    print(f"max nmpc solve time(s): {np.max(nmpc_solve_times)}\n")
    print(f"min rotor thrust(N): {np.min(thrust_history)}\n")
    print(f"max rotor thrust(N): {np.max(thrust_history)}\n")

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(reference_history[:, 0], reference_history[:, 1], reference_history[:, 2], label="Reference")
    ax.plot(true_history[:, 0], true_history[:, 1], true_history[:, 2], label="True trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("3D Trajectory Tracking")
    ax.legend()
    plt.tight_layout()

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes = np.asarray(axes).reshape(-1)
    names = ["x", "y", "z"]
    for i in range(3):
        axes[i].plot(time_history, tracking_error[:, i])
        axes[i].set_ylabel(f"e_{names[i]} (m)")
        axes[i].grid(True)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Position Tracking Error")
    plt.tight_layout()

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes = np.asarray(axes).reshape(-1)
    for i in range(3):
        axes[i].plot(time_history, estimation_error_history[:, i], label="Estimation error")
        axes[i].plot(time_history, two_sigma[:, i], "--", label="+2 sigma")
        axes[i].plot(time_history, -two_sigma[:, i], "--", label="-2 sigma")
        axes[i].set_ylabel(f"e_{names[i]} (m)")
        axes[i].grid(True)
        axes[i].legend()

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("EKF Position Estimation Error")
    plt.tight_layout()

    plt.figure(figsize=(10, 6))
    for i in range(4):
        plt.plot(time_history, thrust_history[:, i], label=f"T{i}")
    plt.axhline(T_MIN, linestyle="--", label="minimum thrust")
    plt.axhline(T_MAX, linestyle="--", label="maximum thrust")
    plt.xlabel("Time (s)")
    plt.ylabel("Rotor thrust (N)")
    plt.title("Rotor Thrust Commands")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
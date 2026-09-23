import json
import time
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

from dynamics import rotor_to_wrench
from ekf import EKF
from main_mujoco import (
    DT,
    GNSS_INTERVAL,
    NUM_STEPS,
    R_ENU_FROM_NED,
    advance_mujoco,
    get_true_state_from_mujoco,
    quat_to_yaw,
    read_mujoco_imu,
    set_mujuco_state,
    wrap_angle,
    yaw_to_quat,
)
from nmpc import T_MAX, T_MIN
from nmpc_acados import AcadosNMPC, N_ACADOS, NMPC_ACADOS_DT
from sensors_mujoco import SensorModel
from trajectory import TF, ref_traj


OUTPUT_DIR = Path("fig_acados")
REPLAY_JSON = OUTPUT_DIR / "mujoco_replay_acados.json"
NMPC_ACADOS_INTERVAL = int(round(NMPC_ACADOS_DT / DT))


def build_ref_horizon_acados(current_time):
    ref = np.zeros((7, N_ACADOS + 1))
    for j in range(N_ACADOS + 1):
        t_ref = current_time + j * NMPC_ACADOS_DT
        position_ref, velocity_ref, _, yaw_ref = ref_traj(t_ref)
        ref[0:3, j] = position_ref
        ref[3:6, j] = velocity_ref
        ref[6, j] = yaw_ref
    return ref


def save_replay_json(
    time_history,
    true_history,
    reference_history,
    nmpc_solve_times,
    nmpc_solve_count,
    nmpc_success_count,
):
    replay_data = {
        "metadata": {
            "description": "MuJoCo replay data saved from acados RTI-SQP NMPC simulation.",
            "method": "acados_sqp_rti",
            "state_frame": "NED",
            "state_layout": [
                "x", "y", "z",
                "vx", "vy", "vz",
                "qw", "qx", "qy", "qz",
                "p", "q", "r",
            ],
            "reference_frame": "NED",
            "reference_layout": ["x", "y", "z"],
            "dt": float(DT),
            "mission_time": float(time_history[-1]) if time_history.size > 0 else 0.0,
            "nmpc_dt": float(NMPC_ACADOS_DT),
            "nmpc_horizon": int(N_ACADOS),
            "nmpc_solve_count": int(nmpc_solve_count),
            "nmpc_success_count": int(nmpc_success_count),
            "avg_nmpc_solve_time": float(np.mean(nmpc_solve_times)) if nmpc_solve_times.size > 0 else None,
            "max_nmpc_solve_time": float(np.max(nmpc_solve_times)) if nmpc_solve_times.size > 0 else None,
        },
        "time_history": time_history.tolist(),
        "true_history": true_history.tolist(),
        "reference_history": reference_history.tolist(),
    }
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with REPLAY_JSON.open("w", encoding="utf-8") as f:
        json.dump(replay_data, f)


def save_figures(
    time_history,
    true_history,
    estimate_history,
    covariance_history,
    reference_history,
    reference_velocity_history,
    reference_yaw_history,
    thrust_history,
    torque_history,
):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    position_tracking_error = true_history[:, 0:3] - reference_history
    velocity_tracking_error = true_history[:, 3:6] - reference_velocity_history
    true_yaw_history = np.array([quat_to_yaw(q) for q in true_history[:, 6:10]])
    yaw_tracking_error = wrap_angle(true_yaw_history - reference_yaw_history)
    estimation_error_history = true_history - estimate_history
    sigma_history = np.sqrt(np.maximum(covariance_history, 0))
    two_sigma = 2 * sigma_history

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(reference_history[:, 0], reference_history[:, 1], reference_history[:, 2], label="Reference")
    ax.plot(true_history[:, 0], true_history[:, 1], true_history[:, 2], label="True trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("3D Trajectory Tracking - acados RTI-SQP")
    ax.legend()
    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / "3d_traj_acados.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    axes = np.asarray(axes).reshape(-1)
    position_names = ["x", "y", "z"]
    for i in range(3):
        axes[i].plot(time_history, position_tracking_error[:, i])
        axes[i].set_ylabel(f"e_{position_names[i]} (m)")
        axes[i].grid(True)
    velocity_names = ["vx", "vy", "vz"]
    for i in range(3):
        axes[i + 3].plot(time_history, velocity_tracking_error[:, i])
        axes[i + 3].set_ylabel(f"e_{velocity_names[i]} (m/s)")
        axes[i + 3].grid(True)
    axes[6].plot(time_history, np.rad2deg(yaw_tracking_error))
    axes[6].set_ylabel(r"$e_\psi$ (deg)")
    axes[6].set_xlabel("Time (s)")
    axes[6].grid(True)
    axes[7].axis("off")
    for ax in axes[:6]:
        ax.set_xlabel("Time (s)")
    fig.suptitle("State Tracking Error - acados RTI-SQP")
    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / "state_tracking_error_acados.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(3, 3, figsize=(13, 10), sharex=True)
    state_names = [
        ("Position x", 0, "m"),
        ("Position y", 1, "m"),
        ("Position z", 2, "m"),
        ("Velocity vx", 3, "m/s"),
        ("Velocity vy", 4, "m/s"),
        ("Velocity vz", 5, "m/s"),
        ("Body rate p", 10, "rad/s"),
        ("Body rate q", 11, "rad/s"),
        ("Body rate r", 12, "rad/s"),
    ]
    for ax, (title, idx, ylabel) in zip(axes.reshape(-1), state_names):
        ax.plot(time_history, estimation_error_history[:, idx], label="Error")
        ax.plot(time_history, two_sigma[:, idx], "--", label="+2 sigma")
        ax.plot(time_history, -two_sigma[:, idx], "--", label="-2 sigma")
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True)
    axes[0, 0].legend()
    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")
    fig.suptitle("EKF Estimation Error with 2 Sigma Bounds - acados RTI-SQP")
    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / "ekf_estimation_error_acados.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(10, 6))
    for i in range(4):
        plt.plot(time_history, thrust_history[:, i], label=f"T{i}")
    plt.axhline(T_MIN, linestyle="--", label="minimum thrust")
    plt.axhline(T_MAX, linestyle="--", label="maximum thrust")
    plt.xlabel("Time (s)")
    plt.ylabel("Rotor thrust (N)")
    plt.title("Rotor Thrust Commands - acados RTI-SQP")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / "rotor_thrust_acados.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    torque_limits = [2 * 0.225 * (T_MAX - T_MIN), 2 * 0.225 * (T_MAX - T_MIN), 2 * 0.015 * (T_MAX - T_MIN)]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    torque_names = [r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"]
    for i in range(3):
        axes[i].plot(time_history, torque_history[:, i], label=torque_names[i])
        axes[i].axhline(torque_limits[i], linestyle="--", label="Upper Limit")
        axes[i].axhline(-torque_limits[i], linestyle="--", label="Lower Limit")
        axes[i].set_ylabel("Torque (Nm)")
        axes[i].grid(True)
        axes[i].legend()
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Body Torque Commands - acados RTI-SQP")
    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / "body_torque_commands_acados.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def main():
    model = mujoco.MjModel.from_xml_path("quadrotor.xml")
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    accel_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
    gyro_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")

    position_ref_0, velocity_ref_0, _, yaw_ref_0 = ref_traj(0)
    true_state = np.zeros(13)
    true_state[0:3] = position_ref_0
    true_state[3:6] = velocity_ref_0
    true_state[6:10] = yaw_to_quat(yaw_ref_0)
    set_mujuco_state(model, data, joint_id, true_state)

    initial_estimate = true_state.copy()
    initial_estimate[0:3] += np.array([0.05, -0.05, 0.03])
    initial_estimate[3:6] += np.array([0.02, -0.02, 0.01])
    initial_covariance = np.diag([
        0.2**2, 0.2**2, 0.2**2,
        0.2**2, 0.2**2, 0.2**2,
        0.05**2, 0.05**2, 0.05**2, 0.05**2,
        0.1**2, 0.1**2, 0.1**2,
    ])
    process_noise = np.diag([
        1e-6, 1e-6, 1e-6,
        1e-4, 1e-4, 1e-4,
        1e-7, 1e-7, 1e-7, 1e-7,
        1e-4, 1e-4, 3e-5,
    ])

    sensor = SensorModel(sigma_accel_bias_rw=0.002, sigma_gyro_bias_rw=0.0002, seed=42)
    ekf = EKF(initial_state=initial_estimate, initial_covariance=initial_covariance, process_noise=process_noise)
    controller = AcadosNMPC(horizon=N_ACADOS, dt=NMPC_ACADOS_DT)

    time_history = []
    true_history = []
    estimate_history = []
    covariance_history = []
    reference_history = []
    reference_velocity_history = []
    reference_yaw_history = []
    thrust_history = []
    torque_history = []
    nmpc_solve_times = []
    nmpc_success_count = 0
    nmpc_solve_count = 0

    ref_horizon = build_ref_horizon_acados(0)
    start_time = time.perf_counter()
    thrust_command, predicted_states, _ = controller.solve(ekf.x, ref_horizon)
    solve_time = time.perf_counter() - start_time
    nmpc_solve_times.append(solve_time)
    nmpc_solve_count += 1
    nmpc_success_count += int(predicted_states is not None)
    print(f"initial acados thrust = {thrust_command}")
    print(f"initial acados solve time = {solve_time:.6f}s")

    for k in range(NUM_STEPS):
        t_next = (k + 1) * DT
        advance_mujoco(model, data, body_id, thrust_command, DT)
        true_state = get_true_state_from_mujoco(model, data, body_id, joint_id)

        ekf.predict(thrust_command, DT)
        ideal_accel, ideal_gyro = read_mujoco_imu(model, data, accel_sensor_id, gyro_sensor_id)
        accel_measurement, gyro_measurement = sensor.add_imu_noise(ideal_accel, ideal_gyro, DT)
        ekf.update_imu(accel_measurement, gyro_measurement, thrust_command)

        if (k + 1) % GNSS_INTERVAL == 0:
            gnss_measurement = sensor.simulate_gnss(true_state)
            ekf.update_gnss(gnss_measurement)

        if (k + 1) % NMPC_ACADOS_INTERVAL == 0:
            ref_horizon = build_ref_horizon_acados(t_next)
            start_time = time.perf_counter()
            new_thrust_command, predicted_states, _ = controller.solve(ekf.x, ref_horizon)
            solve_time = time.perf_counter() - start_time
            nmpc_solve_times.append(solve_time)
            nmpc_solve_count += 1
            if predicted_states is not None:
                nmpc_success_count += 1
            thrust_command = new_thrust_command

        current_position_ref, current_velocity_ref, _, current_yaw_ref = ref_traj(t_next)
        _, torque_body = rotor_to_wrench(thrust_command)

        time_history.append(t_next)
        true_history.append(true_state.copy())
        estimate_history.append(ekf.x.copy())
        covariance_history.append(np.diag(ekf.P).copy())
        reference_history.append(current_position_ref.copy())
        reference_velocity_history.append(current_velocity_ref.copy())
        reference_yaw_history.append(current_yaw_ref)
        thrust_history.append(thrust_command.copy())
        torque_history.append(torque_body.copy())

        if (k + 1) % int(1 / DT) == 0:
            print(f"t = {t_next:.2f}s, acados thrust = {thrust_command}")

    time_history = np.array(time_history)
    true_history = np.array(true_history)
    estimate_history = np.array(estimate_history)
    covariance_history = np.array(covariance_history)
    reference_history = np.array(reference_history)
    reference_velocity_history = np.array(reference_velocity_history)
    reference_yaw_history = np.array(reference_yaw_history)
    thrust_history = np.array(thrust_history)
    torque_history = np.array(torque_history)
    nmpc_solve_times = np.array(nmpc_solve_times)

    tracking_rmse = np.sqrt(np.mean((true_history[:, 0:3] - reference_history) ** 2, axis=0))
    ekf_position_rmse = np.sqrt(np.mean((true_history[:, 0:3] - estimate_history[:, 0:3]) ** 2, axis=0))
    print("\nacados mujoco simulation results")
    print(f"position tracking rmse(m): {tracking_rmse}")
    print(f"ekf position estimation rmse(m): {ekf_position_rmse}")
    print(f"acados nmpc solver success rate: {nmpc_success_count / nmpc_solve_count}")
    print(f"avg acados nmpc solve time(s): {np.mean(nmpc_solve_times)}")
    print(f"max acados nmpc solve time(s): {np.max(nmpc_solve_times)}")
    print(f"min rotor thrust(N): {np.min(thrust_history)}")
    print(f"max rotor thrust(N): {np.max(thrust_history)}")

    save_replay_json(
        time_history,
        true_history,
        reference_history,
        nmpc_solve_times,
        nmpc_solve_count,
        nmpc_success_count,
    )
    save_figures(
        time_history,
        true_history,
        estimate_history,
        covariance_history,
        reference_history,
        reference_velocity_history,
        reference_yaw_history,
        thrust_history,
        torque_history,
    )
    print(f"saved acados replay data to {REPLAY_JSON}")
    print(f"saved acados figures to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()

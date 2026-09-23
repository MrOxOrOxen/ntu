import numpy as np
import matplotlib.pyplot as plt
import time
import json
from pathlib import Path
from contextlib import nullcontext
from dynamics import step, rotor_to_wrench
from sensors_mujoco import SensorModel, IMU_DT, GNSS_DT
from ekf import EKF
from nmpc import NMPC, NMPC_DT, N, T_MAX, T_MIN
from trajectory import ref_traj, TF

import mujoco
import mujoco.viewer

DT = IMU_DT
GNSS_INTERVAL = int(round(GNSS_DT / DT))
NMPC_INTERVAL = int(round(NMPC_DT / DT))
NUM_STEPS = int(round(TF / DT))
LIVE_VIEWER = False

R_ENU_FROM_NED = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])

def quat_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    return np.array([
        [1-2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1-2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw),1-2*(qx*qx + qy*qy)]
    ])

def rotation_matrix_to_quat(R):
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qw, qx, qy, qz])
    q = q / np.linalg.norm(q)
    return q

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

def quat_to_yaw(q):
    qw, qx, qy, qz = q
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy**2 + qz**2))
    return yaw

def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def set_mujuco_state(model, data, joint_id, state_ned):
    qpos_adr = model.jnt_qposadr[joint_id]
    qvel_adr = model.jnt_dofadr[joint_id]
    position_ned = state_ned[0: 3]
    velocity_ned = state_ned[3: 6]
    quaternion_ned = state_ned[6: 10]
    omega_body = state_ned[10: 13]

    position_enu = R_ENU_FROM_NED @ position_ned
    velocity_enu = R_ENU_FROM_NED @ velocity_ned
    R_NB = quat_to_rotation_matrix(quaternion_ned)
    R_EB = R_ENU_FROM_NED @ R_NB
    quaternion_enu = rotation_matrix_to_quat(R_EB)

    data.qpos[qpos_adr: qpos_adr+3] = position_enu
    data.qpos[qpos_adr+3: qpos_adr+7] = quaternion_enu
    data.qvel[qvel_adr: qvel_adr+3] = velocity_enu
    data.qvel[qvel_adr+3: qvel_adr+6] = omega_body

    mujoco.mj_forward(model, data)

def get_true_state_from_mujoco(model, data, body_id, joint_id):
    qpos_adr = model.jnt_qposadr[joint_id]
    qvel_adr = model.jnt_dofadr[joint_id]
    position_enu = data.qpos[qpos_adr: qpos_adr+3].copy()
    velocity_enu = data.qvel[qvel_adr: qvel_adr+3].copy()
    position_ned = R_ENU_FROM_NED @ position_enu
    velocity_ned = R_ENU_FROM_NED @ velocity_enu

    R_EB = data.xmat[body_id].reshape(3, 3).copy()
    R_NB = R_ENU_FROM_NED @ R_EB
    quaternion_ned = rotation_matrix_to_quat(R_NB)
    omega_body = data.qvel[qvel_adr+3: qvel_adr+6].copy()
    state = np.concatenate([position_ned, velocity_ned, quaternion_ned, omega_body])
    return state

def apply_motor_forces(data, body_id, thrusts):
    force_body, torque_body = rotor_to_wrench(thrusts)
    R_EB = data.xmat[body_id].reshape(3, 3)
    force_world = R_EB @ force_body
    torque_world = R_EB @ torque_body
    data.xfrc_applied[body_id, :] = 0
    data.xfrc_applied[body_id, 0:3] = force_world
    data.xfrc_applied[body_id, 3:6] = torque_world

def advance_mujoco(model, data, body_id, thrusts, duration):
    physics_dt = model.opt.timestep
    substeps = int(round(duration / physics_dt))
    for _ in range(substeps):
        apply_motor_forces(data, body_id, thrusts)
        mujoco.mj_step(model, data)
    # refresh all derived quantities and sensors at the new state
    apply_motor_forces(data, body_id, thrusts)
    mujoco.mj_forward(model, data)

def read_mujoco_imu(model, data, accel_sensor_id, gyro_sensor_id):
    accel_adr = model.sensor_adr[accel_sensor_id]
    accel_dim = model.sensor_dim[accel_sensor_id]
    gyro_adr = model.sensor_adr[gyro_sensor_id]
    gyro_dim = model.sensor_dim[gyro_sensor_id]
    ideal_accel = data.sensordata[accel_adr: accel_adr + accel_dim].copy()
    ideal_gyro = data.sensordata[gyro_adr: gyro_adr + gyro_dim].copy()
    return ideal_accel, ideal_gyro

def draw_traj(viewer, points, rgba, max_segments=500):
    if len(points) < 2:
        return

    scene = viewer.user_scn
    start = max(0, len(points) - max_segments - 1)

    for i in range(start, len(points)-1):
        if scene.ngeom >= scene.maxgeom:
            break

        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom, type=mujoco.mjtGeom.mjGEOM_LINE, size=np.zeros(3),
            pos=np.zeros(3), mat=np.eye(3).flatten(), rgba=np.array(rgba, dtype=np.float32)
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 3, points[i], points[i+1])
        scene.ngeom += 1

def main():
    # load mujoco
    model = mujoco.MjModel.from_xml_path("quadrotor.xml")
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    accel_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
    gyro_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")

    # initial reference
    position_ref_0, velocity_ref_0, acceleration_ref_0, yaw_ref_0 = ref_traj(0)

    # true initial state
    true_state = np.zeros(13)
    true_state[0:3] = position_ref_0
    true_state[3:6] = velocity_ref_0
    true_state[6:10] = yaw_to_quat(yaw_ref_0)
    true_state[10:13] = np.zeros(3)

    set_mujuco_state(model, data, joint_id, true_state)
    print(f"mujoco gravity: {model.opt.gravity}")
    print(f"mujoco body mass: {model.body_mass[body_id]}")
    print(f"initial mujoco position: {data.qpos[0:3]}")

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
        1e-4, 1e-4, 3e-5
    ])

    # sensor
    sensor = SensorModel(sigma_accel_bias_rw=0.002, sigma_gyro_bias_rw=0.0002, seed=42)

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
    reference_velocity_history = []
    reference_yaw_history = []
    thrust_history = []
    torque_history = []
    nmpc_solve_times = []
    nmpc_success_count = 0
    nmpc_solve_count = 0

    rate_true_history = []
    rate_predict_history = []
    rate_gyro_history = []
    rate_update_history = []

    ideal_gyro_error_history = []

    # initial nmpc solution
    ref_horizon = build_ref_horizon(0)
    start_time = time.perf_counter()
    thrust_command, predicted_states, predicted_controls = controller.solve(ekf.x, ref_horizon)
    solve_time = time.perf_counter() - start_time
    nmpc_solve_times.append(solve_time)
    nmpc_solve_count += 1
    if predicted_states is not None:
        nmpc_success_count += 1
    print(f"initial thrust = {thrust_command}\n")

    # route draw: ref
    reference_path_mujoco = []
    actual_path_mujoco = []

    for t in np.arange(0, TF+0.2, 0.2):
        p_ref, _, _, _ = ref_traj(t)
        p_mj = R_ENU_FROM_NED @ p_ref
        reference_path_mujoco.append(p_mj)

    # The live viewer is disabled by default so data generation is decoupled
    # from the separate MuJoCo replay/recording step.
    viewer_context = mujoco.viewer.launch_passive(model, data) if LIVE_VIEWER else nullcontext(None)
    with viewer_context as viewer:


    # save data of t=0
    # time_history.append(0)
    # true_history.append(true_state.copy())
    # estimate_history.append(ekf.x.copy())
    # covariance_history.append(np.diag(ekf.P).copy())
    # reference_history.append(position_ref_0.copy())
    # thrust_history.append(thrust_command.copy())

    # main simulation
        for k in range(NUM_STEPS):
            t = k * DT
            t_next = (k+1) * DT

            advance_mujoco(model, data, body_id, thrust_command, DT)

            # true_state = step(true_state, thrust_command, DT)
            true_state = get_true_state_from_mujoco(model, data, body_id, joint_id)
            ekf.predict(thrust_command, DT)
            rate_predict = ekf.x[10:13].copy()
            ideal_accel, ideal_gyro = read_mujoco_imu(model, data, accel_sensor_id, gyro_sensor_id)
            ideal_gyro_error_history.append(ideal_gyro - true_state[10:13])
            accel_measurement, gyro_measurement = sensor.add_imu_noise(ideal_accel, ideal_gyro, DT)
            rate_gyro = gyro_measurement.copy()
            ekf.update_imu(accel_measurement, gyro_measurement, thrust_command)
            rate_update = ekf.x[10:13].copy()
            rate_true_history.append(true_state[10:13].copy())
            rate_predict_history.append(rate_predict)
            rate_gyro_history.append(rate_gyro)
            rate_update_history.append(rate_update)

            if k < 5:
                print(f"k: {k}")
                print(f"ideal accel: {ideal_accel}")
                print(f"ideal gyro: {ideal_gyro}")
                print(f"accel measurement: {accel_measurement}")
                print(f"gyro measurement: {gyro_measurement}")
                print("thrust:", thrust_command)
                print("EKF predicted state:", ekf.x)
            
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

            if (k+1) % 20 == 0:
                actual_position_mujoco = R_ENU_FROM_NED @ true_state[0:3]
                actual_path_mujoco.append(actual_position_mujoco.copy())

            current_position_ref, current_velocity_ref, current_acceleration_ref, current_yaw_ref = ref_traj(t_next)
            _, torque_body = rotor_to_wrench(thrust_command)

            time_history.append(t_next)
            true_history.append(true_state.copy())
            estimate_history.append(ekf.x.copy())
            reference_history.append(current_position_ref.copy())
            reference_velocity_history.append(current_velocity_ref.copy())
            reference_yaw_history.append(current_yaw_ref)
            thrust_history.append(thrust_command.copy())
            torque_history.append(torque_body.copy())
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

            if LIVE_VIEWER and viewer.is_running():
                viewer.user_scn.ngeom = 0
                draw_traj(viewer, reference_path_mujoco, rgba=[0.1, 0.8, 0.2, 1], max_segments=400)
                draw_traj(viewer, actual_path_mujoco, rgba=[0.9, 0.1, 0.1, 1], max_segments=400)
                viewer.sync()
            elif LIVE_VIEWER:
                break

        time_history = np.array(time_history)
        true_history = np.array(true_history)
        estimate_history = np.array(estimate_history)
        estimation_error_history = np.array(true_history - estimate_history)
        covariance_history = np.array(covariance_history)
        reference_history = np.array(reference_history)
        reference_velocity_history = np.array(reference_velocity_history)
        reference_yaw_history = np.array(reference_yaw_history)
        thrust_history = np.array(thrust_history)
        torque_history = np.array(torque_history)
        nmpc_solve_times = np.array(nmpc_solve_times)

        rate_true_history = np.array(rate_true_history)
        rate_predict_history = np.array(rate_predict_history)
        rate_gyro_history = np.array(rate_gyro_history)
        rate_update_history = np.array(rate_update_history)

        ideal_gyro_error_history = np.array(ideal_gyro_error_history)

        position_tracking_error = true_history[:, 0:3] - reference_history
        velocity_tracking_error = true_history[:, 3:6] - reference_velocity_history
        true_yaw_history = np.array([quat_to_yaw(q) for q in true_history[:, 6:10]])
        yaw_tracking_error = wrap_angle(true_yaw_history - reference_yaw_history)
        # estimation_error = true_history - estimate_history
        estimation_error = true_history[:, 0:3] - estimate_history[:, 0:3]
        sigma_history = np.sqrt(np.maximum(covariance_history, 0))
        two_sigma = 2 * sigma_history

        tracking_rmse = np.sqrt(np.mean(position_tracking_error**2, axis=0))
        ekf_position_rmse = np.sqrt(np.mean(estimation_error[:, 0:3]**2, axis=0))

        print("\n")
        print("mujoco simulation results\n")
        print(f"position tracking rmse(m): {tracking_rmse}\n")
        print(f"ekf position estimation rmse(m): {ekf_position_rmse}\n")
        print(f"nmpc solver success rate: {nmpc_success_count / nmpc_solve_count}\n")
        print(f"avg nmpc solve time(s): {np.mean(nmpc_solve_times)}\n")
        print(f"max nmpc solve time(s): {np.max(nmpc_solve_times)}\n")
        print(f"min rotor thrust(N): {np.min(thrust_history)}\n")
        print(f"max rotor thrust(N): {np.max(thrust_history)}\n")

        replay_data = {
            "metadata": {
                "description": "MuJoCo replay data saved from blocking NMPC simulation.",
                "state_frame": "NED",
                "state_layout": [
                    "x", "y", "z",
                    "vx", "vy", "vz",
                    "qw", "qx", "qy", "qz",
                    "p", "q", "r"
                ],
                "reference_frame": "NED",
                "reference_layout": ["x", "y", "z"],
                "dt": float(DT),
                "mission_time": float(time_history[-1]) if time_history.size > 0 else 0.0,
                "nmpc_dt": float(NMPC_DT),
                "nmpc_horizon": int(N),
                "nmpc_solve_count": int(nmpc_solve_count),
                "nmpc_success_count": int(nmpc_success_count),
                "avg_nmpc_solve_time": float(np.mean(nmpc_solve_times)) if nmpc_solve_times.size > 0 else None,
                "max_nmpc_solve_time": float(np.max(nmpc_solve_times)) if nmpc_solve_times.size > 0 else None,
            },
            "time_history": time_history.tolist(),
            "true_history": true_history.tolist(),
            "reference_history": reference_history.tolist(),
        }
        replay_path = Path("fig/mujoco_replay.json")
        replay_path.parent.mkdir(parents=True, exist_ok=True)
        with replay_path.open("w", encoding="utf-8") as f:
            json.dump(replay_data, f)
        print(f"saved MuJoCo replay data to {replay_path}\n")

    # figure
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
    fig.savefig("fig/3d_traj.png", dpi=300, bbox_inches="tight")

    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    axes = np.asarray(axes).reshape(-1)

    position_names = ["x", "y", "z"]
    for i in range(3):
        axes[i].plot(time_history, position_tracking_error[:, i])
        axes[i].set_ylabel(f"e_{position_names[i]} (m)")
        axes[i].grid(True)

    velocity_names = ["vx", "vy", "vz"]
    for i in range(3):
        axes[i+3].plot(time_history, velocity_tracking_error[:, i])
        axes[i+3].set_ylabel(f"e_{velocity_names[i]} (m/s)")
        axes[i+3].grid(True)

    axes[6].plot(time_history, np.rad2deg(yaw_tracking_error))
    axes[6].set_ylabel(r"$e_\psi$ (deg)")
    axes[6].set_xlabel("Time (s)")
    axes[6].grid(True)
    axes[7].axis("off")

    for ax in axes[:6]:
        ax.set_xlabel("Time (s)")
    fig.suptitle("State Tracking Error")
    plt.tight_layout()
    fig.savefig("fig/state_tracking_error.png", dpi=300, bbox_inches="tight")

    fig, axes = plt.subplots(3, 3, figsize=(13, 10), sharex=True)

    position_names = ["x", "y", "z"]
    # axes = np.asarray(axes).reshape(-1)
    for i in range(3):
        axes[0, i].plot(time_history, estimation_error_history[:, i], label="Error")
        axes[0, i].plot(time_history, two_sigma[:, i], "--", label="+2 sigma")
        axes[0, i].plot(time_history, -two_sigma[:, i], "--", label="-2 sigma")
        # axes[i].plot(time_history, two_sigma[:, i], "--", label="+2 sigma")
        # axes[i].plot(time_history, -two_sigma[:, i], "--", label="-2 sigma")
        # axes[i].set_ylabel(f"e_{names[i]} (m)")
        axes[0, i].set_title(f"Position {position_names[i]}")
        axes[0, i].set_ylabel("m")
        axes[0, i].grid(True)
        # axes[i].legend()

    velocity_names = ["vx", "vy", "vz"]
    for i in range(3):
        idx = 3 + i
        axes[1, i].plot(time_history, estimation_error_history[:, idx])
        axes[1, i].plot(time_history, two_sigma[:, idx], "--")
        axes[1, i].plot(time_history, -two_sigma[:, idx], "--")
        axes[1, i].set_title(f"Velocity {velocity_names[i]}")
        axes[1, i].set_ylabel("m/s")
        axes[1, i].grid(True)

    rate_names = ["p", "q", "r"]
    for i in range(3):
        idx = 10 + i
        mask = time_history > 2
        error = np.abs(estimation_error_history[mask, idx])
        sigma = sigma_history[mask, idx]
        coverage = np.mean(np.abs(error) <= 2 * sigma) * 100
        rmse = np.sqrt(np.mean(error**2))
        avg_sigma = np.mean(sigma)

        names = ["p", "q", "r"]
        true_rate = rate_true_history[mask, i]
        pred_rate = rate_predict_history[mask, i]
        gyro_rate = rate_gyro_history[mask, i]
        update_rate = rate_update_history[mask, i]
        pred_rmse = np.sqrt(np.mean((pred_rate - true_rate)**2))
        gyro_rmse = np.sqrt(np.mean((gyro_rate - true_rate)**2))
        update_rmse = np.sqrt(np.mean((update_rate - true_rate)**2))
        print(f"{names[i]}: prediction RMSE={pred_rmse:.5f}, gyro RMSE={gyro_rmse:.5f}, updated RMSE={update_rmse:.5f}")
        print(f"{rate_names[i]}: coverage={coverage:.2f}%, RMSE={rmse:.5f}, avg sigma={avg_sigma:.5f}, RMSE/sigma={rmse/avg_sigma:.2f}")
        print("ideal gyro RMSE vs true rate:", np.sqrt(np.mean(ideal_gyro_error_history[mask]**2,axis=0)))

        axes[2, i].plot(time_history, estimation_error_history[:, idx])
        axes[2, i].plot(time_history, two_sigma[:, idx], "--")
        axes[2, i].plot(time_history, -two_sigma[:, idx], "--")
        axes[2, i].set_title(f"Body rate {rate_names[i]}")
        axes[2, i].set_ylabel("rad/s")
        axes[2, i].set_xlabel("Time (s)")
        axes[2, i].grid(True)

    axes[0, 0].legend()

    # axes[-1].set_xlabel("Time (s)")
    fig.suptitle("EKF Estimation Error with 2 sigma bounds")
    plt.tight_layout()
    fig.savefig("fig/ekf_estimation_error.png", dpi=300, bbox_inches="tight")

    fig = plt.figure(figsize=(10, 6))
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
    fig.savefig("fig/rotor_thrust.png", dpi=300, bbox_inches="tight")

    TORQUE_LIMITS = [2*0.225*(T_MAX - T_MIN), 2*0.225*(T_MAX - T_MIN), 2*0.015*(T_MAX - T_MIN)]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    torque_names = [r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"]
    for i in range(3):
        axes[i].plot(time_history, torque_history[:, i], label=torque_names[i])
        axes[i].axhline(TORQUE_LIMITS[i], linestyle="--", label="Upper Limit")
        axes[i].axhline(-TORQUE_LIMITS[i], linestyle="--", label="Lower Limit")
        axes[i].set_ylabel("Torque (Nm)")
        axes[i].grid(True)
        axes[i].legend()

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Body Torque Commands")
    plt.tight_layout()
    fig.savefig("fig/body_torque_commands.png", dpi=300, bbox_inches="tight")

    plt.show()
    

if __name__ == "__main__":
    main()

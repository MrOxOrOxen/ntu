import argparse
import json
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


R_ENU_FROM_NED = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1],
])


def quat_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
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
    return q / np.linalg.norm(q)


def set_mujoco_state(model, data, joint_id, state_ned):
    qpos_adr = model.jnt_qposadr[joint_id]
    qvel_adr = model.jnt_dofadr[joint_id]

    position_enu = R_ENU_FROM_NED @ state_ned[0:3]
    velocity_enu = R_ENU_FROM_NED @ state_ned[3:6]
    R_NB = quat_to_rotation_matrix(state_ned[6:10])
    R_EB = R_ENU_FROM_NED @ R_NB
    quaternion_enu = rotation_matrix_to_quat(R_EB)

    data.qpos[qpos_adr:qpos_adr + 3] = position_enu
    data.qpos[qpos_adr + 3:qpos_adr + 7] = quaternion_enu
    data.qvel[qvel_adr:qvel_adr + 3] = velocity_enu
    data.qvel[qvel_adr + 3:qvel_adr + 6] = state_ned[10:13]
    mujoco.mj_forward(model, data)


def draw_traj(viewer, points, rgba, max_segments):
    if len(points) < 2:
        return

    scene = viewer.user_scn
    start = max(0, len(points) - max_segments - 1)
    for i in range(start, len(points) - 1):
        if scene.ngeom >= scene.maxgeom:
            break
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            type=mujoco.mjtGeom.mjGEOM_LINE,
            size=np.zeros(3),
            pos=np.zeros(3),
            mat=np.eye(3).flatten(),
            rgba=np.array(rgba, dtype=np.float32),
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 3, points[i], points[i + 1])
        scene.ngeom += 1


def load_replay(path):
    with Path(path).open("r", encoding="utf-8") as f:
        payload = json.load(f)

    time_history = np.array(payload["time_history"], dtype=float)
    true_history = np.array(payload["true_history"], dtype=float)
    reference_history = np.array(payload["reference_history"], dtype=float)
    if time_history.ndim != 1 or true_history.ndim != 2 or reference_history.ndim != 2:
        raise ValueError("Replay JSON has invalid array dimensions.")
    if len(time_history) != len(true_history) or len(time_history) != len(reference_history):
        raise ValueError("Replay JSON arrays must have the same length.")
    return payload.get("metadata", {}), time_history, true_history, reference_history


def main():
    parser = argparse.ArgumentParser(description="Replay saved quadrotor history in MuJoCo.")
    parser.add_argument("--input", default="fig/mujoco_replay.json", help="Replay JSON path.")
    parser.add_argument("--xml", default="quadrotor.xml", help="MuJoCo XML path.")
    parser.add_argument("--duration", type=float, default=90.0, help="Wall-clock playback duration in seconds.")
    parser.add_argument("--fps", type=float, default=60.0, help="Viewer update rate.")
    parser.add_argument("--trail-step", type=int, default=20, help="Draw one trail point every N saved samples.")
    parser.add_argument("--max-segments", type=int, default=900, help="Maximum line segments drawn per trajectory.")
    args = parser.parse_args()

    metadata, time_history, true_history, reference_history = load_replay(args.input)
    mission_time = float(time_history[-1])
    playback_duration = max(args.duration, 1e-6)
    playback_speed = mission_time / playback_duration

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")

    reference_path = [R_ENU_FROM_NED @ p for p in reference_history[::args.trail_step]]
    actual_path = []
    last_trail_index = -1

    print(f"Loaded replay: {args.input}")
    print(f"Mission time: {mission_time:.2f}s")
    print(f"Playback duration: {playback_duration:.2f}s")
    print(f"Playback speed: {playback_speed:.3f}x simulation time per wall time")
    if metadata:
        print(f"NMPC avg solve time: {metadata.get('avg_nmpc_solve_time')}")
        print(f"NMPC max solve time: {metadata.get('max_nmpc_solve_time')}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall_start = time.perf_counter()
        frame_dt = 1.0 / args.fps

        while viewer.is_running():
            elapsed_wall = time.perf_counter() - wall_start
            sim_t = min(elapsed_wall * playback_speed, mission_time)
            index = int(np.searchsorted(time_history, sim_t, side="left"))
            index = min(max(index, 0), len(time_history) - 1)

            set_mujoco_state(model, data, joint_id, true_history[index])

            trail_index = index // args.trail_step
            if trail_index != last_trail_index:
                actual_path = [R_ENU_FROM_NED @ p for p in true_history[:index + 1:args.trail_step, 0:3]]
                last_trail_index = trail_index

            viewer.user_scn.ngeom = 0
            draw_traj(viewer, reference_path, rgba=[0.1, 0.8, 0.2, 1], max_segments=args.max_segments)
            draw_traj(viewer, actual_path, rgba=[0.9, 0.1, 0.1, 1], max_segments=args.max_segments)
            viewer.sync()

            if sim_t >= mission_time:
                print("Replay finished. Close the MuJoCo viewer to exit.")
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(frame_dt)
                break

            target = wall_start + (elapsed_wall // frame_dt + 1) * frame_dt
            remaining = target - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()

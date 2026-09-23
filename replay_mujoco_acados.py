import argparse
import sys

import replay_mujoco


def main():
    parser = argparse.ArgumentParser(description="Replay saved acados RTI-SQP quadrotor history in MuJoCo.")
    parser.add_argument("--input", default="fig_acados/mujoco_replay_acados.json", help="acados replay JSON path.")
    parser.add_argument("--xml", default="quadrotor.xml", help="MuJoCo XML path.")
    parser.add_argument("--duration", type=float, default=90.0, help="Wall-clock playback duration in seconds.")
    parser.add_argument("--fps", type=float, default=60.0, help="Viewer update rate.")
    parser.add_argument("--trail-step", type=int, default=20, help="Draw one trail point every N saved samples.")
    parser.add_argument("--max-segments", type=int, default=900, help="Maximum line segments drawn per trajectory.")
    args = parser.parse_args()

    sys.argv = [
        "replay_mujoco.py",
        "--input", args.input,
        "--xml", args.xml,
        "--duration", str(args.duration),
        "--fps", str(args.fps),
        "--trail-step", str(args.trail_step),
        "--max-segments", str(args.max_segments),
    ]
    replay_mujoco.main()


if __name__ == "__main__":
    main()

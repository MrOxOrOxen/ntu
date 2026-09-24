# How to run the code

- If using casadi: run main_mujoco.py, then run replay_mujoco.py
- If using acados: run main_mujoco_acados.py, then run replay_mujoco_acados.py

# Structure

- main_mujoco.py: main for casadi/ipopt
- nmpc.py: nmpc for casadi/ipopt
- replay_mujoco.py: replay code for casadi/ipopt
- fig/: figures and json files of casadi/ipopt
- run.log: terminal output of casadi/ipopt

- main_mujoco_acados.py: main for acados/sqp-rti
- nmpc_acados.py: nmpc for acados/sqp-rti
- replay_mujoco_acados.py: replay code for acados/sqp-rti
- fig_acados/: figures and json files of acados/sqp-rti
- run_acados.log: terminal output of acados/sqp-rti

- ekf.py: kalman filter
- sensors_mujoco.py: imu and gnss
- trajectory.py: reference trajectory
- dynamics.py: dynamics
- quadrotor.xml: quadrotor model
- video/: videos
- main.tex: latex report file
- report_group8.pdf: report
- final_slide.pptx: slide
import numpy as np
import casadi as ca


MASS = 1.20
DX = 0.225
DY = 0.225
JXX = 1.25e-2
JYY = 1.25e-2
JZZ = 2.20e-2
CT = 0.015
G = 9.81

T_MIN = 0.2
T_MAX = 5.5
MAX_ROLL = np.deg2rad(35)
MAX_PITCH = np.deg2rad(35)
MAX_P = np.deg2rad(180)
MAX_Q = np.deg2rad(180)
MAX_R = np.deg2rad(90)

NMPC_ACADOS_DT = 0.05
N_ACADOS = 20


def quat_to_rotation_matrix_symbolic(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        ca.horzcat(2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)),
        ca.horzcat(2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)),
    )


def quat_derivative_symbolic(quat, omega_body):
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    p, q, r = omega_body[0], omega_body[1], omega_body[2]
    omega_matrix = ca.vertcat(
        ca.horzcat(0, -p, -q, -r),
        ca.horzcat(p, 0, r, -q),
        ca.horzcat(q, -r, 0, p),
        ca.horzcat(r, q, -p, 0),
    )
    return 0.5 * ca.mtimes(omega_matrix, quat)


def rotor_to_wrench_symbolic(u):
    T0, T1, T2, T3 = u[0], u[1], u[2], u[3]
    thrust_body = ca.vertcat(0, 0, -(T0 + T1 + T2 + T3))
    torque_body = ca.vertcat(
        DY * (-T0 - T1 + T2 + T3),
        DX * (-T0 + T1 + T2 - T3),
        CT * (-T0 + T1 - T2 + T3),
    )
    return thrust_body, torque_body


def dynamics_symbolic(x, u):
    velocity = x[3:6]
    quat = x[6:10]
    omega = x[10:13]

    thrust_body, torque_body = rotor_to_wrench_symbolic(u)
    R_WB = quat_to_rotation_matrix_symbolic(quat)

    position_dot = velocity
    velocity_dot = ca.mtimes(R_WB, thrust_body) / MASS + ca.vertcat(0, 0, G)
    quat_dot = quat_derivative_symbolic(quat, omega)
    angular_momentum = ca.vertcat(JXX * omega[0], JYY * omega[1], JZZ * omega[2])
    gyroscopic = ca.cross(omega, angular_momentum)
    omega_dot = ca.vertcat(
        (torque_body[0] - gyroscopic[0]) / JXX,
        (torque_body[1] - gyroscopic[1]) / JYY,
        (torque_body[2] - gyroscopic[2]) / JZZ,
    )
    return ca.vertcat(position_dot, velocity_dot, quat_dot, omega_dot)


def quaternion_to_euler_symbolic(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    roll = ca.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch_arg = ca.fmin(1.0, ca.fmax(-1.0, 2 * (qw * qy - qz * qx)))
    pitch = ca.asin(pitch_arg)
    yaw = ca.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return roll, pitch, yaw


class AcadosNMPC:
    def __init__(self, horizon=N_ACADOS, dt=NMPC_ACADOS_DT):
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        self.N = horizon
        self.dt = dt
        self.nx = 13
        self.nu = 4
        self.ny = 15
        self.ny_e = 11
        self.T_hover = MASS * G / 4
        self.previous_U = None
        self.previous_X = None

        x = ca.SX.sym("x", self.nx)
        xdot = ca.SX.sym("xdot", self.nx)
        u = ca.SX.sym("u", self.nu)
        f_expl = dynamics_symbolic(x, u)

        model = AcadosModel()
        model.name = "quadrotor_acados"
        model.x = x
        model.xdot = xdot
        model.u = u
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl

        roll, pitch, yaw = quaternion_to_euler_symbolic(x[6:10])
        model.con_h_expr = ca.vertcat(roll, pitch, x[10], x[11], x[12])
        model.cost_y_expr = ca.vertcat(x[0:3], x[3:6], ca.sin(yaw), ca.cos(yaw), x[10:13], u)
        model.cost_y_expr_e = ca.vertcat(x[0:3], x[3:6], ca.sin(yaw), ca.cos(yaw), x[10:13])

        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.N * self.dt
        ocp.code_export_directory = "acados_codegen_quadrotor"

        Q_POS = np.array([30, 30, 40])
        Q_VEL = np.array([5, 5, 8])
        Q_YAW = np.array([5, 5])
        Q_OMEGA = np.array([0.1, 0.1, 0.1])
        R_U = np.array([0.05, 0.05, 0.05, 0.05])
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag(np.concatenate([Q_POS, Q_VEL, Q_YAW, Q_OMEGA, R_U]))
        ocp.cost.W_e = np.diag(np.concatenate([50 * np.ones(3), 10 * np.ones(3), Q_YAW, Q_OMEGA]))
        ocp.cost.yref = self._stage_yref(np.zeros(3), np.zeros(3), 0.0)
        ocp.cost.yref_e = np.zeros(self.ny_e)

        ocp.constraints.x0 = np.zeros(self.nx)
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbu = np.ones(self.nu) * T_MIN
        ocp.constraints.ubu = np.ones(self.nu) * T_MAX
        ocp.constraints.lh = np.array([-MAX_ROLL, -MAX_PITCH, -MAX_P, -MAX_Q, -MAX_R])
        ocp.constraints.uh = np.array([MAX_ROLL, MAX_PITCH, MAX_P, MAX_Q, MAX_R])

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = 0

        self.solver = AcadosOcpSolver(
            ocp,
            json_file="acados_codegen_quadrotor/quadrotor_acados_ocp.json",
        )

    def _stage_yref(self, position_ref, velocity_ref, yaw_ref):
        return np.concatenate([
            position_ref,
            velocity_ref,
            np.array([np.sin(yaw_ref), np.cos(yaw_ref)]),
            np.zeros(3),
            np.ones(4) * self.T_hover,
        ])

    def _terminal_yref(self, position_ref, velocity_ref, yaw_ref):
        return np.concatenate([
            position_ref,
            velocity_ref,
            np.array([np.sin(yaw_ref), np.cos(yaw_ref)]),
            np.zeros(3),
        ])

    def _set_references(self, ref_horizon):
        for k in range(self.N):
            self.solver.set(
                k,
                "yref",
                self._stage_yref(ref_horizon[0:3, k], ref_horizon[3:6, k], ref_horizon[6, k]),
            )
        self.solver.set(
            self.N,
            "yref",
            self._terminal_yref(ref_horizon[0:3, self.N], ref_horizon[3:6, self.N], ref_horizon[6, self.N]),
        )

    def _set_initial_guess(self, current_state):
        if self.previous_U is None:
            U_guess = np.ones((self.nu, self.N)) * self.T_hover
        else:
            U_guess = np.hstack([self.previous_U[:, 1:], self.previous_U[:, -1:]])

        if self.previous_X is None:
            X_guess = np.tile(current_state.reshape(self.nx, 1), (1, self.N + 1))
        else:
            X_guess = np.hstack([self.previous_X[:, 1:], self.previous_X[:, -1:]])
            X_guess[:, 0] = current_state

        for k in range(self.N):
            self.solver.set(k, "x", X_guess[:, k])
            self.solver.set(k, "u", U_guess[:, k])
        self.solver.set(self.N, "x", X_guess[:, self.N])

    def solve(self, current_state, ref_horizon):
        self.solver.set(0, "lbx", current_state)
        self.solver.set(0, "ubx", current_state)
        self._set_references(ref_horizon)
        self._set_initial_guess(current_state)

        status = self.solver.solve()
        if status != 0:
            print(f"[ERROR] acados NMPC failed with status {status}")
            thrust_command = np.ones(4) * self.T_hover
            return thrust_command, None, None

        U_solution = np.column_stack([self.solver.get(k, "u") for k in range(self.N)])
        X_solution = np.column_stack([self.solver.get(k, "x") for k in range(self.N + 1)])
        self.previous_U = U_solution.copy()
        self.previous_X = X_solution.copy()
        thrust_command = U_solution[:, 0]
        return thrust_command, X_solution, U_solution

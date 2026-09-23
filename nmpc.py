# x_hat + reference traj -> T0, T1, T2, T3

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

NMPC_DT = 0.05
N = 20

def quat_to_rotation_matrix_symbolic(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    R = ca.vertcat(
        ca.horzcat(1-2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)),
        ca.horzcat(2*(qx*qy + qz*qw), 1-2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)),
        ca.horzcat(2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx**2 + qy**2))
    )

    return R

def quat_derivative_symbolic(quat, omega_body):
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    p, q, r = omega_body[0], omega_body[1], omega_body[2]

    Omega = ca.vertcat(
        ca.horzcat(0, -p, -q, -r),
        ca.horzcat(p, 0, r, -q),
        ca.horzcat(q, -r, 0, p),
        ca.horzcat(r, q, -p, 0)
    )

    quat_dot = 0.5 * ca.mtimes(Omega, quat)
    return quat_dot

def rotor_to_wrench_symbolic(u):
    T0, T1, T2, T3 = u[0], u[1], u[2], u[3]

    thrust_body = ca.vertcat(0, 0, -(T0 + T1 + T2 + T3))
    tau_x = DY * (-T0 - T1 + T2 + T3)
    tau_y = DX * (-T0 + T1 + T2 - T3)
    tau_z = CT * (-T0 + T1 - T2 + T3)

    torque_body = ca.vertcat(tau_x, tau_y, tau_z)
    return thrust_body, torque_body

def dynamics_symbolic(x, u):
    position = x[0:3]
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
        (torque_body[2] - gyroscopic[2]) / JZZ
    )

    x_dot = ca.vertcat(position_dot, velocity_dot, quat_dot, omega_dot)

    return x_dot

def rk4_symbolic(x, u, dt):
    k1 = dynamics_symbolic(x, u)
    k2 = dynamics_symbolic(x + 0.5 * dt * k1, u)
    k3 = dynamics_symbolic(x + 0.5 * dt * k2, u)
    k4 = dynamics_symbolic(x + dt * k3, u)
    x_next = x + dt / 6 * (k1 + 2*k2 + 2*k3 + k4)

    quat = x_next[6:10]
    quat_norm = ca.sqrt(ca.sumsqr(quat) + 1e-12)
    quat_normalized = quat / quat_norm

    x_next = ca.vertcat(x_next[0:6], quat_normalized, x_next[10:13])
    return x_next

def quaternion_to_euler_symbolic(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    roll = ca.atan2(2*(qw*qx + qy*qz), 1-2*(qx**2 + qy**2))
    pitch = ca.asin(2*(qw*qy - qz*qx))
    yaw = ca.atan2(2*(qw*qz + qx*qy), 1-2*(qy**2 + qz**2))
    return roll, pitch, yaw

def angle_error(angle, ref):
    return ca.atan2(ca.sin(angle - ref), ca.cos(angle - ref))

class NMPC:
    def __init__(self, horizon=N, dt=NMPC_DT):
        self.N = horizon
        self.dt = dt

        opti = ca.Opti()
        self.opti = opti

        self.X = opti.variable(13, self.N + 1)
        self.U = opti.variable(4, self.N)
        self.x0 = opti.parameter(13)
        self.ref = opti.parameter(7, self.N + 1)
        opti.subject_to(self.X[:, 0] == self.x0)

        cost = 0
        Q_POS = np.array([30, 30, 40])
        Q_VEL = np.array([5, 5, 8])
        Q_YAW = 5
        Q_OMEGA = np.array([0.1, 0.1, 0.1])
        R_U = np.array([0.05, 0.05, 0.05, 0.05])
        T_HOVER = MASS * G / 4

        for k in range(self.N):
            xk = self.X[:, k]
            uk = self.U[:, k]

            pos_ref = self.ref[0:3, k]
            vel_ref = self.ref[3:6, k]
            yaw_ref = self.ref[6, k]

            position = xk[0:3]
            velocity = xk[3:6]
            quat = xk[6:10]
            omega = xk[10:13]

            roll, pitch, yaw = quaternion_to_euler_symbolic(quat)

            if k > 0:
                pos_error = position - pos_ref
                for i in range(3):
                    cost += Q_POS[i] * pos_error[i]**2

                vel_error = velocity - vel_ref
                for i in range(3):
                    cost += Q_VEL[i] * vel_error[i]**2

                yaw_error = angle_error(yaw, yaw_ref)
                cost += Q_YAW * yaw_error**2

                for i in range(3):
                    cost += Q_OMEGA[i] * omega[i]**2

            for i in range(4):
                cost += R_U[i] * (uk[i] - T_HOVER)**2

            x_next = rk4_symbolic(xk, uk, self.dt)
            opti.subject_to(self.X[:, k+1] == x_next)
            opti.subject_to(opti.bounded(T_MIN, uk, T_MAX))
            opti.subject_to(opti.bounded(-MAX_ROLL, roll, MAX_ROLL))
            opti.subject_to(opti.bounded(-MAX_PITCH, pitch, MAX_PITCH))
            opti.subject_to(opti.bounded(-MAX_P, omega[0], MAX_P))
            opti.subject_to(opti.bounded(-MAX_Q, omega[1], MAX_Q))
            opti.subject_to(opti.bounded(-MAX_R, omega[2], MAX_R))

        final_state = self.X[:, self.N]
        final_ref = self.ref[:, self.N]
        final_pos_error = final_state[0:3] - final_ref[0:3]
        final_vel_error = final_state[3:6] - final_ref[3:6]
        for i in range(3):
            cost += 50 * final_pos_error[i]**2
            cost += 10 * final_vel_error[i]**2

        opti.minimize(cost)
        opti.solver("ipopt", 
            {
                "ipopt.print_level": 0,
                "print_time": False,
                "ipopt.max_iter": 100,
                "ipopt.tol": 1e-4
            }
        )

        self.previous_U = None
        self.previous_X = None

    def solve(self, current_state, ref_horizon):
        self.opti.set_value(self.x0, current_state)
        self.opti.set_value(self.ref, ref_horizon)

        T_hover = MASS * G / 4

        if self.previous_U is None:
            U_guess = np.ones((4, self.N)) * T_hover
        else:
            U_guess = self.previous_U
        self.opti.set_initial(self.U, U_guess)

        if self.previous_X is None:
            X_guess = np.tile(current_state.reshape(13, 1), (1, self.N + 1))
        else:
            X_guess = self.previous_X
        self.opti.set_initial(self.X, X_guess)

        try:
            solution = self.opti.solve()
            U_solution = np.array(solution.value(self.U))
            X_solution = np.array(solution.value(self.X))
            self.previous_U = U_solution.copy()
            self.previous_X = X_solution.copy()

            thrust_command = U_solution[:, 0]
            return thrust_command, X_solution, U_solution
        except RuntimeError:
            print("[ERROR] NMPC solver failed")
            thrust_command = np.ones(4) * T_hover
            return thrust_command, None, None

        
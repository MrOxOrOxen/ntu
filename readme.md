# MA6217 Advanced Linear Systems and Control

## PBA1

This assignment contains the implementation of an automobile cruise control system with automatic transmission modeling using MATLAB and Simulink Stateflow.

**Task 1: Simulink Model Architecture**

Build a Simulink closed-loop architecture with subsystems such as Controller, Actuator, Throttle and engine, Gears and wheels, Body, Disturbance forces and Driver Interface.

**Task 2: Open-loop Step Response**

Output the v-t figure with a step signal input in an open-loop architecture.

**Task 3: Closed-loop PI Control**

Design a PI controller based on error signal. Output the v-t figure with a step signal input.

**Task 4: Stateflow Finite State Machine for Automatic Gear Shifting**

Write an automatic gear switching logic using Stateflow. Add hysteresis to prevent dynamic gear hunting.

Monitor car velocity, engine rotational speed and gear states by time.

## PBA2

This assignment contains the analysis and implementation of a pendulum swing-up control system using Lyapunov energy-based control, MATLAB and Simulink.

**Task 1: Lyapunov Energy-Based Swing-Up Control**

Analyze the pendulum dynamics by treating the cart acceleration as the control input.

Use a Lyapunov function based on the pendulum energy error to show that the proposed control law drives the pendulum energy toward the desired energy level. Discuss the special cases where the Lyapunov derivative becomes zero.

**Task 2: Phase Portrait and Equilibrium Analysis**

Construct the closed-loop phase portrait of the pendulum system.

Identify the upright and downward equilibrium points and determine their stability by linearization and eigenvalue analysis.

Investigate the effect of the controller gain \(k\) and select an appropriate value based on the phase portraits and system response.

**Task 3: Nonlinear Cart-Pendulum Simulink Model**

Build the complete nonlinear cart-pendulum system in Simulink with the applied force \(F\) as the system input.

Validate the model using a known initial condition and constant applied force by comparing the analytical initial accelerations with the Simulink results.

Output the cart position and pendulum angle trajectories.

**Task 4: Lyapunov-Based Swing-Up Controller in Simulink**

Implement the Lyapunov energy-based swing-up controller on the nonlinear cart-pendulum model.

Convert the desired cart acceleration from the energy controller into the required applied force \(F\).

Demonstrate pendulum swing-up from the downward configuration and monitor the pendulum angle, pendulum energy and applied force.

Compare different controller gains \(k\) and analyze their effects on energy regulation speed, control force and cart displacement.

**## PBA3**

This assignment contains the linearization, state-feedback control and swing-up and catch implementation of a nonlinear cart-pendulum system using MATLAB, Simulink and Stateflow.

**Task 1: Jacobian Linearization**

Linearize the nonlinear cart-pendulum system around the upright equilibrium using the Jacobian method.

Derive the state-space matrices \(A\) and \(B\) analytically.

**Task 2: Numerical Linearization Using linmod**

Use the MATLAB `linmod` command to numerically linearize the nonlinear Simulink model at the upright equilibrium.

Compare the eigenvalues obtained from `linmod` with the analytical results from Task 1 and verify their agreement.

**Task 3: State-Feedback Controller Design**

Design a state-feedback controller using pole placement to stabilize the linearized cart-pendulum system.

Check system controllability, calculate the feedback gain matrix \(K\), and verify closed-loop stability through eigenvalue analysis.

**Task 4: Nonlinear Closed-Loop Simulation**

Implement the state-feedback controller on the nonlinear cart-pendulum model developed in PBA2.

Simulate different initial conditions and demonstrate local asymptotic stability.

Investigate the region of attraction by varying the initial pendulum angle and comparing convergent and non-convergent trajectories.

**Task 5: Swing-Up and Catch Controller Using Stateflow**

Combine the energy-based swing-up controller developed in PBA2 with the state-feedback balancing controller from Task 3.

Implement controller switching using Stateflow and apply a force saturation limit of \(\pm14\) N.

Evaluate the original PBA2 swing-up controller under force saturation and introduce a modified energy-based controller using a hyperbolic tangent function.

Demonstrate successful swing-up and catch, and monitor the pendulum angle, cart position, pendulum energy, applied force and Stateflow controller mode.
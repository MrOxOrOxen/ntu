% Physical parameter assignments from the supplied AAP5_setup.m.
% Kept separate from the optional visualizer and workspace resets.
% Variables
Lp = 0.33655;    % Length of pendulum (m)
mp = 0.127;      % Mass of pendulum (kg)
Jp = mp*Lp^2/12; % Pendulum Moment of Inertia about center (kg.m^2)

r = 0.2159;     % Length of arm (m)
mr = 0.257;      % Mass of arm (kg)
Jarm = mr*r^2/3; % Arm Moment of Inertia about pivot (kg.m^2)

% Transmission component inertias 
Jarmature = 3.462e-7; 
Jtachometer = 6.355e-8;
J1 = 5.444e-6;
J2 = 4.183e-5;
J3 = 1.008e-7;
J4 = 4.669e-7;
J5 = 2.382e-9;
Jencoder = 1.463e-13;
Jpotentiometer = 1.616e-10;
N1 = 72; N2 = 120; N3 = 24; N4 = 280; N5 = 20;
G = N4/N5 * N2/N3;
Jrotor_eff = 0.002087; % Confirm this in Task 1.
J = Jarm + Jrotor_eff;

Ra = 2.6;        % motor armature resistance
La = 0.18e-3;     % Motor inductance
kt = 7.68e-3;    % motor current-torque constant
kb = kt;         % back emf   
g = 9.81;        % gravity

Bp = 0.0024;     % Viscous damping coefficients as seen at the pendulum pivot
Br = 0.0024;

eta_m = 0.69;    % motor efficiency
eta_g = 0.9;     % gearbox efficiency


lp = Lp/2  ;    %half length of pendulum

%

% scalar constants
mass = 0.6; % kg

% define body frame as FRD front right down
% define inertial frame as NED north east down

% blade angle model
A_cut_in = 0.03; % modulation
K_coeff = 48.14 * pi / 180; % rad / b_xy command
phase_lag = 0; % rad    (0 for now since control mixing in sim does not account for alignment)

% prop model vars
torque_coeff_top = 0.1; % Nm / norm throttle
torque_coeff_bot = 0.1;
thrust_coeff_top = 10; % N / norm throttle^2
thrust_coeff_bot = 10; % N / norm throttle^2

% mav vars
mav_inertia = eye(3) * 0.003;
mav_mass = 0.6; %kg
top_motor_arm = [0, 0, 0.1]; % m from center of mass

% controller vars
att_ctrl_time = 1/250; % 4 ms
rate_ctrl_time = 1/1000; % 1 ms 

control_effectiveness_matrix = [
    0, 0, 1, 1;
    0, top_motor_arm(3), 0, 0;
    -top_motor_arm(3), 0, 0, 0;
    0, 0, -torque_coeff_top, torque_coeff_bot
];
control_allocator_matrix = inv(control_effectiveness_matrix)

% sim vars
sample_time = 0.001; % s -> 1000 hz

% initial conds
initial_euler = [0, 0.1, 0];

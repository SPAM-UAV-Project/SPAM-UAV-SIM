% scalar constants
mass = 0.631; % kg

% define body frame as FRD front right down
% define inertial frame as NED north east down

% blade angle model
A_cut_in = 0.16; % modulation
K_coeff = 19.95 * pi / 180; % rad / b_xy command
phase_lag = 0; % rad    (0 for now since control mixing in sim does not account for alignment)

% prop model vars
torque_coeff_top = 0.2051; % Nm / norm throttle: this is for the control allocator
torque_coeff_bot = 0.1455;
thrust_coeff_top = 15.9837; % N / norm throttle^2
thrust_coeff_bot = 9.3460; % N / norm throttle^2

% mav vars
mav_inertia = eye(3) * 0.004013;
mav_inertia(3, 3) = 0.0007873;
mav_mass = 0.631; %kg
top_motor_arm = [0, 0, 0.18]; % m from center of mass
f_xy_tf = load("f_xy_tf.mat").tf_force_y;
thrust_tf = load("thrust_tf.mat").thrust_tf;

% controller vars
att_ctrl_time = 1/250; % 4 ms
rate_ctrl_time = 1/1000; % 1 ms 

mav_inertia_ctrl = eye(3) * 0.004013;
mav_inertia_ctrl(3, 3) = 0.0007873;
torque_coeff_thrust_top = 0.0187; % Nm / Force for control allocator
torque_coeff_thrust_bot = 0.0155;

control_effectiveness_matrix = [
    0, 0, -1, -1;
    0, top_motor_arm(3), 0, 0;
    -top_motor_arm(3), 0, 0, 0;
    0, 0, torque_coeff_thrust_top, -torque_coeff_thrust_bot
]
control_allocator_matrix = inv(control_effectiveness_matrix)

% normalize the allocator matrix based on motors
% thrust_scale = (max(control_allocator_matrix(:, 1)));
% control_allocator_matrix(:, 1) = control_allocator_matrix(:, 1) / thrust_scale;
% x_scale = sum(control_allocator_matrix(:, 2));
% control_allocator_matrix(:, 2) = control_allocator_matrix(:, 2) / x_scale;
% y_scale = sum(control_allocator_matrix(:, 3));
% control_allocator_matrix(:, 3) = control_allocator_matrix(:, 3) / y_scale;
% z_scale = max(control_allocator_matrix(:, 4));
% control_allocator_matrix(:, 4) = control_allocator_matrix(:, 4) / z_scale


% normalize the thrust column

% sim vars
sample_time = 0.001; % s -> 1000 hz

% initial conds
initial_euler = [0, 0, 0];


%% test
% format long g
% forces = control_allocator_matrix * [-3, 0, 0, 0.0078]'
% sqrt(forces(3) / thrust_coeff_top)
% sqrt(forces(4) / thrust_coeff_bot)
% forces(3) * torque_coeff_thrust_top
% forces(4) * torque_coeff_thrust_bot
% 

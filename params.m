% scalar constants
mass = 0.6; % kg

% define body frame as FRD front right down
% define inertial frame as NED north east down

% blade angle model
A_cut_in = 0.03; % modulation
K_coeff = 48.14 * pi / 180; % rad / b_xy command
phase_lag = 0; % rad    (0 for now since control mixing in sim does not account for alignment)

% prop model vars
torque_coeff = 0.1; % Nm / norm throttle
thrust_coeff = 0.5; % N / norm throttle^2

% mav vars
mav_inertia = eye(3) * 0.003;
mav_mass = 0.6; %kg
top_motor_arm = [0, 0, 0.1]; % m from center of mass

% sim vars
sample_time = 0.002; % s -> 500 hz
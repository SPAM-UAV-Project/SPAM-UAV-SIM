% scalar constants
mass = 0.6; % kg

% define body frame as FRD front right down
% define inertial frame as NED north east down

% propeller vars
A_cut_in = 0.15; % modulation
K_coeff = 0.2; % Nm / % modulation
phase_lag = 0; % rad    (0 for now since control mixing in sim does not account for alignment)
torque_coeff = 0.1; % Nm / % throttle

% mav vars
mav_inertia = eye(3);
mav_mass = 0.6; %kg
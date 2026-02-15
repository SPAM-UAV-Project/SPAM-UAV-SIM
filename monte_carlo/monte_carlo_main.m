N_trials = 1000;

bx_offsets = zeros(1, N_trials);
by_offsets = zeros(1, N_trials);
yaw_diffs = zeros(1, N_trials);
inertias = zeros(3,3,N_trials);

simIn = Simulink.SimulationInput.empty(0, N_trials);

for i=1:N_trials
    simIn(i) = Simulink.SimulationInput('main_sim');

    % set initial conditions
    initial_pqr = [0.5, 0.5, 0.5];

    % distribution of torque_coeff offsets
    U_torque_coeff_top = 0.2551;
    U_torque_coeff_bot = 0.1255;
    U_torque_deviation = 0.01; % 

    Cq_top_trial = U_torque_coeff_top + (-U_torque_deviation + (2 * U_torque_deviation) * rand());
    Cq_bot_trial = U_torque_coeff_bot + (-U_torque_deviation + (2 * U_torque_deviation) * rand());

    yaw_diffs(i) = Cq_top_trial - Cq_bot_trial;

    % distribution of angular offset - assume uniform distribution from -0.5 deg
    % to 0.5 deg offset in both mount angles
    U_rad_dev = 0.5 * pi / 180;
    bx_offset_trial = -U_rad_dev + (2 * U_rad_dev) * rand(); % from -0.5 to 0.5
    by_offset_trial = -U_rad_dev + (2 * U_rad_dev) * rand();
    
    % distribution of propeller coefficients (gaussian from thrust stand
    % tests)

    % distribution of inertia uncertainty (gaussian from repeated tests)
    mu_inertia = 0.004013;
    mu_inertia_z = 0.0007873;
    std_inertia = 0.1 * mu_inertia / 3;
    ixx_trial = max(mu_inertia + std_inertia * randn(), 1e-10);
    iyy_trial = max(mu_inertia + std_inertia * randn(), 1e-10);
    izz_trial = max(mu_inertia + std_inertia * randn(), 1e-10);

    % randomize cross coupling centered around 0 - guess a small std of 1e-6
    iyz_trial = 1e-6 * randn();
    ixz_trial = 1e-6 * randn();

    inertia_trial = [ixx_trial, 0,    ixz_trial;
                     0,   iyy_trial, iyz_trial;
                     ixz_trial,    iyz_trial,    izz_trial];

    inertias(:, :, i) = inertia_trial;
    
    % assign to sim variables
    bx_offsets(i) = bx_offset_trial;
    by_offsets(i) = by_offset_trial;

    % set sim variables
    simIn(i) = simIn(i).setModelParameter('SimulationMode', 'normal');
    simIn(i) = simIn(i).setModelParameter('FastRestart', 'on');

    simIn(i) = simIn(i).setVariable("initial_pqr", initial_pqr);
    simIn(i) = simIn(i).setVariable('mav_inertia', inertia_trial);
    simIn(i) = simIn(i).setVariable('bx_offset', bx_offset_trial);
    simIn(i) = simIn(i).setVariable('by_offset', by_offset_trial);
    simIn(i) = simIn(i).setVariable('torque_coeff_top', Cq_top_trial);
    simIn(i) = simIn(i).setVariable('torque_coeff_bot', Cq_bot_trial);
    simIn(i) = simIn(i).setModelParameter('StopTime', '5');
end
%%
%% Run sim
tic;
simOut = parsim(simIn, ... %[output:group:146efdcd] %[output:6e4739b0]
    'UseParallel', false, ... %[output:6e4739b0]
    'UseFastRestart', 'on', ... %[output:6e4739b0]
    'ShowProgress', 'on', ... %[output:6e4739b0]
    'TransferBaseWorkspaceVariables', 'on'); %[output:group:146efdcd] %[output:6e4739b0]
toc;
beep;
%%
%% Extract results
IAE_roll = zeros(1, N_trials);
IAE_pitch = zeros(1, N_trials);
IAE_yaw  = zeros(1, N_trials);
end_rate_x = zeros(1, N_trials);
end_rate_y = zeros(1, N_trials);
end_rate_z = zeros(1, N_trials);

stored_time = cell(1, N_trials);
stored_rate = cell(1, N_trials);

for i = 1:N_trials %[output:group:32ff6d3c]
    % [p, q, r]
    ts = simOut(i).logsout.get('Wb').Values;  %[output:1b9f3d4a]
    stored_time{i} = ts.Time;
    stored_rate{i} = ts.Data;
    
    % calculate IAE with a target rate of 0
    IAE_roll(i)  = trapz(ts.Time, abs(ts.Data(:, 1)));
    IAE_pitch(i) = trapz(ts.Time, abs(ts.Data(:, 2)));
    IAE_yaw(i)   = trapz(ts.Time, abs(ts.Data(:, 3)));
    
    % Store end rates if needed
    end_rate_x(i) = ts.Data(end, 1);
    end_rate_y(i) = ts.Data(end, 2);
    end_rate_z(i) = ts.Data(end, 3);
end %[output:group:32ff6d3c]
%%
% plotting the time response
[max_val_p, worst_idx_p] = max(IAE_roll);
[max_val_q, worst_idx_q] = max(IAE_pitch);
[max_val_r, worst_idx_r] = max(IAE_yaw);

num_samples = 1000; 
t_common = linspace(0, 5, num_samples); % Adjust '5' to your StopTime

% Pre-allocate matrices: [N_trials x Time]
roll_data_interp  = zeros(N_trials, num_samples);
pitch_data_interp = zeros(N_trials, num_samples);
yaw_data_interp   = zeros(N_trials, num_samples);

for i = 1:N_trials
    t_raw = stored_time{i};
    r_raw = stored_rate{i};
    
    % Interpolate this trial onto the common time vector
    roll_data_interp(i, :)  = interp1(t_raw, r_raw(:,1), t_common, 'linear', 'extrap');
    pitch_data_interp(i, :) = interp1(t_raw, r_raw(:,2), t_common, 'linear', 'extrap');
    yaw_data_interp(i, :)   = interp1(t_raw, r_raw(:,3), t_common, 'linear', 'extrap');
end

% --- 2. Calculate Percentiles (The "Frequency" Bands) ---
% We want to find boundaries for 90% of data and 50% of data
% Rows: 1=5%, 2=25%, 3=50%(Median), 4=75%, 5=95%
p_roll  = prctile(roll_data_interp,  [5, 25, 50, 75, 95], 1);
p_pitch = prctile(pitch_data_interp, [5, 25, 50, 75, 95], 1);
p_yaw   = prctile(yaw_data_interp,   [5, 25, 50, 75, 95], 1);

% --- 3. Plotting Function ---
figure('Color', 'w', 'Name', 'Probabilistic Rate Envelopes');
t_layout = tiledlayout(3,1);
title(t_layout, 'Monte Carlo Rate Envelopes');

% Helper function to plot bands (defined at bottom or nested)
plot_fan_chart(nexttile, t_common, p_roll,  'Roll Rate (rad/s)',  worst_idx_p, stored_time, stored_rate, 1);
plot_fan_chart(nexttile, t_common, p_pitch, 'Pitch Rate (rad/s)', worst_idx_q, stored_time, stored_rate, 2);
plot_fan_chart(nexttile, t_common, p_yaw,   'Yaw Rate (rad/s)',   worst_idx_r, stored_time, stored_rate, 3);


% --- Local Function for Plotting ---
function plot_fan_chart(ax, t, p_data, y_lab, worst_idx, all_t, all_r, axis_col)
    hold(ax, 'on'); grid(ax, 'on'); ylabel(ax, y_lab);
    
    % Define Colors
    color_90 = [0.8, 0.8, 0.9]; % Light Blue (Outer 90% band)
    color_50 = [0.6, 0.6, 0.8]; % Darker Blue (Inner 50% band)
    
    % Plot 5% to 95% Band (Lightest)
    % We create a polygon by going Forward along the top line, then Backward along bottom
    fill(ax, [t, fliplr(t)], [p_data(5,:), fliplr(p_data(1,:))], ...
        color_90, 'EdgeColor', 'none', 'DisplayName', '90% of Trials');
    
    % Plot 25% to 75% Band (Darker)
    fill(ax, [t, fliplr(t)], [p_data(4,:), fliplr(p_data(2,:))], ...
        color_50, 'EdgeColor', 'none', 'DisplayName', '50% of Trials');
    
    % Plot Median Line
    plot(ax, t, p_data(3,:), 'b', 'LineWidth', 1, 'DisplayName', 'Median');
    
    % Plot Worst Case (Red Line)
    t_bad = all_t{worst_idx};
    r_bad = all_r{worst_idx};
    plot(ax, t_bad, r_bad(:, axis_col), 'r', 'LineWidth', 1.5, 'DisplayName', 'Worst Case');
    
    legend(ax, 'show');
end

fprintf('\n========================================\n');
fprintf('   MONTE CARLO WORST-CASE ANALYSIS\n');
fprintf('========================================\n');

% --- Analyze Worst Roll Case ---
fprintf('\n[ROLL AXIS] Worst Trial: #%d (IAE: %.4f)\n', worst_idx_p, max_val_p);
fprintf('   - bx_offset:   %.4f deg\n', bx_offsets(worst_idx_p) * 180/pi);
fprintf('   - by_offset:   %.4f deg\n', by_offsets(worst_idx_p) * 180/pi);
fprintf('   - Ixx Inertia: %.6f\n', inertias(1,1,worst_idx_p));

% --- Analyze Worst Pitch Case ---
fprintf('\n[PITCH AXIS] Worst Trial: #%d (IAE: %.4f)\n', worst_idx_q, max_val_q);
fprintf('   - bx_offset:   %.4f deg\n', bx_offsets(worst_idx_q) * 180/pi);
fprintf('   - by_offset:   %.4f deg\n', by_offsets(worst_idx_q) * 180/pi);
fprintf('   - Iyy Inertia: %.6f\n', inertias(2,2,worst_idx_q));

% --- Analyze Worst Yaw Case ---
fprintf('\n[YAW AXIS] Worst Trial: #%d (IAE: %.4f)\n', worst_idx_r, max_val_r);
fprintf('   - Torque Coeff Diff: %.4f\n', yaw_diffs(worst_idx_r));
fprintf('   - Izz Inertia:       %.6f\n', inertias(3,3,worst_idx_r));
fprintf('========================================\n');
%%
%% Plot IAE

% Figure 1: Sensitivity (Scatter Plots)
figure('Name', 'IAE Sensitivity Analysis', 'Color', 'w');
t = tiledlayout(1, 3);
title(t, 'Rates Integrated Error (IAE) Sensitivity Analysis');

% Plot 1: Roll Offset vs Roll IAE
nexttile;
scatter(bx_offsets * (180/pi), IAE_roll, 30, 'filled', 'MarkerFaceColor', '#0072BD');
xlabel('Mount Angle Offset $\beta_x$ (deg)', 'Interpreter', 'latex');
ylabel('Roll Rate Error (IAE)')
grid on; title('Roll Sensitivity');

% Plot 2: Pitch Offset vs Pitch IAE
nexttile;
scatter(by_offsets * (180/pi), IAE_pitch, 30, 'filled', 'MarkerFaceColor', '#D95319');
xlabel('Mount Angle Offset $\beta_y$ (deg)', 'Interpreter', 'latex');
ylabel('Pitch Rate Error (IAE)')
grid on; title('Pitch Sensitivity');

% Plot 3: Torque Mismatch vs Yaw IAE
nexttile;
scatter(yaw_diffs, IAE_yaw, 30, 'filled', 'MarkerFaceColor', '#EDB120');
xlabel('Torque Coeff Diff ($C_{Q_{top}} - C_{Q_{bot}}$)', 'Interpreter', 'latex');
ylabel('Yaw Rate Error (IAE)')
grid on; title('Yaw Sensitivity');


% Figure 2: Performance Distribution (Histograms)
figure('Name', 'Controller IAE Distribution', 'Color', 'w');
t2 = tiledlayout(3, 1);

nexttile;
histogram(IAE_roll, 20, 'FaceColor', '#0072BD');
title('Roll Rate Performance Distribution'); ylabel('Count');

nexttile;
histogram(IAE_pitch, 20, 'FaceColor', '#D95319');
title('Pitch Rate Performance Distribution'); ylabel('Count');

nexttile;
histogram(IAE_yaw, 20, 'FaceColor', '#EDB120');
title('Yaw Rate Performance Distribution'); xlabel('Integrated Absolute Error (rad)'); ylabel('Count');
%%
%% Ploting

% Create a new figure for Parameter Sensitivity (Relationships)
figure('Name', 'Steady-State Rate Sensitivity Analysis', 'Color', 'w');
tiledlayout(1, 3); % Layout for 3 side-by-side plots

% 1. Roll Offset vs. Final Roll Rate
nexttile;
scatter(bx_offsets * (180/pi), end_rate_x * (180/pi), 25, 'filled');
xlabel('Roll Offset $b_x$ (deg)', 'Interpreter', 'latex');
ylabel('Final Roll Rate $p$ (deg/s)', 'Interpreter', 'latex');
title('Roll Sensitivity');
grid on;

% 2. Pitch Offset vs. Final Pitch Rate
nexttile;
scatter(by_offsets * (180/pi), end_rate_y * (180/pi), 25, 'filled');
xlabel('Pitch Offset $b_y$ (deg)', 'Interpreter', 'latex');
ylabel('Final Pitch Rate $q$ (deg/s)', 'Interpreter', 'latex');
title('Pitch Sensitivity');
grid on;

% 3. Torque/Thrust Diff vs. Yaw (Placeholder)
nexttile;
scatter(yaw_diffs, end_rate_z * (180/pi), 25, 'filled');
xlabel('Coeff Diff (Unitless)');
ylabel('Final Yaw Rate $r$ (deg/s)', 'Interpreter', 'latex');
title('Yaw Sensitivity');
grid on;

% --- Create a new figure for Error Distributions (Histograms) ---
figure('Name', 'Steady-State Rate Distribution', 'Color', 'w');
tiledlayout(3, 1);

% 1. Histogram of Roll Rate
nexttile;
histogram(end_rate_x * (180/pi), 20); % 20 bins
xlabel('Final Roll Rate (deg/s)');
ylabel('Frequency');
title('Distribution of SS Roll Rate');
grid on;

% 2. Histogram of Pitch Rate
nexttile;
histogram(end_rate_y * (180/pi), 20); 
xlabel('Final Pitch Rate (deg/s)');
ylabel('Frequency');
title('Distribution of SS Pitch Rate');
grid on;

% 2. Histogram of Pitch Rate
nexttile;
histogram(end_rate_z * (180/pi), 20); 
xlabel('Final Yaw Rate (deg/s)');
ylabel('Frequency');
title('Distribution of SS Yaw Rate');
grid on;
%%
%% Plot vs Time
% [max_val_p, worst_idx_p] = max(IAE_roll);
% [max_val_q, worst_idx_q] = max(IAE_pitch);
% [max_val_r, worst_idx_r] = max(IAE_yaw);

% figure('Color', 'w');
% t = tiledlayout(3,1);
% title(t, 'Monte Carlo Rate Response (Gray: All Trials, Red: Worst Case)');
% 
% ax1 = nexttile; hold(ax1,'on'); ylabel(ax1,'Roll (rad/s)'); grid on;
% ax2 = nexttile; hold(ax2,'on'); ylabel(ax2,'Pitch (rad/s)'); grid on;
% ax3 = nexttile; hold(ax3,'on'); ylabel(ax3,'Yaw (rad/s)'); xlabel(ax3,'Time (s)'); grid on;
% 
% for i = 1:N_trials
%     t_plot = stored_time{i};
%     r_plot = stored_rate{i};
% 
%     if i == worst_idx_p || i == worst_idx_q || i == worst_idx_r
%         % skip worse one
%         continue; 
%     end
% 
%     color_grey = [0.6, 0.6, 0.6, 0.2]; 
% 
%     p1 = plot(ax1, t_plot, r_plot(:,1), 'Color', color_grey, 'LineWidth', 0.5);
%     p2 = plot(ax2, t_plot, r_plot(:,2), 'Color', color_grey, 'LineWidth', 0.5);
%     p3 = plot(ax3, t_plot, r_plot(:,3), 'Color', color_grey, 'LineWidth', 0.5);
% 
%     % Disable legend for these background lines
%     p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
%     p2.Annotation.LegendInformation.IconDisplayStyle = 'off';
%     p3.Annotation.LegendInformation.IconDisplayStyle = 'off';
% end
% 
% % Plot Worst ROLL Trial on Axis 1
% t_bad_p = stored_time{worst_idx_p};
% r_bad_p = stored_rate{worst_idx_p};
% plot(ax1, t_bad_p, r_bad_p(:,1), 'Color', 'r', 'LineWidth', 1.5, ...
%     'DisplayName', sprintf('Worst Roll (Trial %d)', worst_idx_p));
% legend(ax1, 'show', 'Location', 'best');
% 
% % Plot Worst PITCH Trial on Axis 2
% t_bad_q = stored_time{worst_idx_q};
% r_bad_q = stored_rate{worst_idx_q};
% plot(ax2, t_bad_q, r_bad_q(:,2), 'Color', 'r', 'LineWidth', 1.5, ...
%     'DisplayName', sprintf('Worst Pitch (Trial %d)', worst_idx_q));
% legend(ax2, 'show', 'Location', 'best');
% 
% % Plot Worst YAW Trial on Axis 3
% t_bad_r = stored_time{worst_idx_r};
% r_bad_r = stored_rate{worst_idx_r};
% plot(ax3, t_bad_r, r_bad_r(:,3), 'Color', 'r', 'LineWidth', 1.5, ...
%     'DisplayName', sprintf('Worst Yaw (Trial %d)', worst_idx_r));
% legend(ax3, 'show', 'Location', 'best');

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:6e4739b0]
%   data: {"dataType":"text","outputData":{"text":"[15-Feb-2026 02:50:22] Checking for availability of parallel pool...\n[15-Feb-2026 02:50:22] Starting Simulink on parallel workers...\n[15-Feb-2026 02:50:22] Configuring simulation cache folder on parallel workers...\n[15-Feb-2026 02:50:23] Transferring base workspace variables used in the model to parallel workers...\n[15-Feb-2026 02:50:23] Total size of base workspace variables to send to parallel workers is 2.61 MB.\n[15-Feb-2026 02:50:24] Loading model on parallel workers...\n[15-Feb-2026 02:50:46] Running simulations...\n[15-Feb-2026 02:52:51] Completed 1 of 1000 simulation runs\n[15-Feb-2026 02:52:51] Received simulation output (size: 162.31 KB) for run 1 from parallel worker.\n[15-Feb-2026 02:52:51] Completed 2 of 1000 simulation runs\n[15-Feb-2026 02:52:51] Received simulation output (size: 162.31 KB) for run 2 from parallel worker.\n[15-Feb-2026 02:52:51] Completed 3 of 1000 simulation runs\n[15-Feb-2026 02:52:51] Received simulation output (size: 162.31 KB) for run 3 from parallel worker.\n[15-Feb-2026 02:52:51] Completed 4 of 1000 simulation runs\n[15-Feb-2026 02:52:51] Received simulation output (size: 162.31 KB) for run 4 from parallel worker.\n[15-Feb-2026 02:52:51] Completed 5 of 1000 simulation runs\n[15-Feb-2026 02:52:51] Received simulation output (size: 162.31 KB) for run 5 from parallel worker.\n[15-Feb-2026 02:52:51] Completed 6 of 1000 simulation runs\n[15-Feb-2026 02:52:51] Received simulation output (size: 162.31 KB) for run 6 from parallel worker.\n[15-Feb-2026 02:53:04] Completed 7 of 1000 simulation runs\n[15-Feb-2026 02:53:04] Received simulation output (size: 162.31 KB) for run 7 from parallel worker.\n[15-Feb-2026 02:53:04] Completed 8 of 1000 simulation runs\n[15-Feb-2026 02:53:05] Received simulation output (size: 162.31 KB) for run 8 from parallel worker.\n[15-Feb-2026 02:53:05] Completed 9 of 1000 simulation runs\n[15-Feb-2026 02:53:05] Received simulation output (size: 162.31 KB) for run 9 from parallel worker.\n[15-Feb-2026 02:53:05] Completed 10 of 1000 simulation runs\n[15-Feb-2026 02:53:05] Received simulation output (size: 162.31 KB) for run 10 from parallel worker.\n[15-Feb-2026 02:53:05] Completed 11 of 1000 simulation runs\n[15-Feb-2026 02:53:05] Received simulation output (size: 162.31 KB) for run 11 from parallel worker.\n[15-Feb-2026 02:53:05] Completed 12 of 1000 simulation runs\n[15-Feb-2026 02:53:05] Received simulation output (size: 162.31 KB) for run 12 from parallel worker.\n[15-Feb-2026 02:53:17] Completed 13 of 1000 simulation runs\n[15-Feb-2026 02:53:17] Received simulation output (size: 162.31 KB) for run 13 from parallel worker.\n[15-Feb-2026 02:53:17] Completed 14 of 1000 simulation runs\n[15-Feb-2026 02:53:17] Received simulation output (size: 162.31 KB) for run 14 from parallel worker.\n[15-Feb-2026 02:53:17] Completed 15 of 1000 simulation runs\n[15-Feb-2026 02:53:17] Received simulation output (size: 162.31 KB) for run 15 from parallel worker.\n[15-Feb-2026 02:53:17] Completed 16 of 1000 simulation runs\n[15-Feb-2026 02:53:17] Received simulation output (size: 162.31 KB) for run 16 from parallel worker.\n[15-Feb-2026 02:53:17] Completed 17 of 1000 simulation runs\n[15-Feb-2026 02:53:17] Received simulation output (size: 162.31 KB) for run 17 from parallel worker.\n[15-Feb-2026 02:53:18] Completed 18 of 1000 simulation runs\n[15-Feb-2026 02:53:18] Received simulation output (size: 162.31 KB) for run 18 from parallel worker.\n[15-Feb-2026 02:53:29] Completed 19 of 1000 simulation runs\n[15-Feb-2026 02:53:29] Received simulation output (size: 162.31 KB) for run 19 from parallel worker.\n[15-Feb-2026 02:53:29] Completed 20 of 1000 simulation runs\n[15-Feb-2026 02:53:29] Received simulation output (size: 162.31 KB) for run 20 from parallel worker.\n[15-Feb-2026 02:53:29] Completed 21 of 1000 simulation runs\n[15-Feb-2026 02:53:29] Received simulation output (size: 162.31 KB) for run 21 from parallel worker.\n[15-Feb-2026 02:53:29] Completed 22 of 1000 simulation runs\n[15-Feb-2026 02:53:29] Received simulation output (size: 162.31 KB) for run 22 from parallel worker.\n[15-Feb-2026 02:53:30] Completed 23 of 1000 simulation runs\n[15-Feb-2026 02:53:30] Received simulation output (size: 162.31 KB) for run 23 from parallel worker.\n[15-Feb-2026 02:53:32] Completed 24 of 1000 simulation runs\n[15-Feb-2026 02:53:32] Received simulation output (size: 162.31 KB) for run 24 from parallel worker.\n[15-Feb-2026 02:53:40] Completed 25 of 1000 simulation runs\n[15-Feb-2026 02:53:40] Received simulation output (size: 162.31 KB) for run 25 from parallel worker.\n[15-Feb-2026 02:53:40] Completed 26 of 1000 simulation runs\n[15-Feb-2026 02:53:40] Received simulation output (size: 162.31 KB) for run 26 from parallel worker.\n[15-Feb-2026 02:53:41] Completed 27 of 1000 simulation runs\n[15-Feb-2026 02:53:41] Received simulation output (size: 162.31 KB) for run 27 from parallel worker.\n[15-Feb-2026 02:53:41] Completed 28 of 1000 simulation runs\n[15-Feb-2026 02:53:41] Received simulation output (size: 162.31 KB) for run 28 from parallel worker.\n[15-Feb-2026 02:53:41] Completed 29 of 1000 simulation runs\n[15-Feb-2026 02:53:41] Received simulation output (size: 162.31 KB) for run 29 from parallel worker.\n[15-Feb-2026 02:53:44] Completed 30 of 1000 simulation runs\n[15-Feb-2026 02:53:44] Received simulation output (size: 162.31 KB) for run 30 from parallel worker.\n[15-Feb-2026 02:53:51] Completed 31 of 1000 simulation runs\n[15-Feb-2026 02:53:51] Received simulation output (size: 162.31 KB) for run 31 from parallel worker.\n[15-Feb-2026 02:53:51] Completed 32 of 1000 simulation runs\n[15-Feb-2026 02:53:51] Received simulation output (size: 162.31 KB) for run 32 from parallel worker.\n[15-Feb-2026 02:53:52] Completed 33 of 1000 simulation runs\n[15-Feb-2026 02:53:52] Received simulation output (size: 162.31 KB) for run 33 from parallel worker.\n[15-Feb-2026 02:53:52] Completed 34 of 1000 simulation runs\n[15-Feb-2026 02:53:52] Received simulation output (size: 162.31 KB) for run 34 from parallel worker.\n[15-Feb-2026 02:53:52] Completed 35 of 1000 simulation runs\n[15-Feb-2026 02:53:52] Received simulation output (size: 162.31 KB) for run 35 from parallel worker.\n[15-Feb-2026 02:53:54] Completed 36 of 1000 simulation runs\n[15-Feb-2026 02:53:54] Received simulation output (size: 162.31 KB) for run 36 from parallel worker.\n[15-Feb-2026 02:54:01] Completed 37 of 1000 simulation runs\n[15-Feb-2026 02:54:01] Received simulation output (size: 162.31 KB) for run 37 from parallel worker.\n[15-Feb-2026 02:54:01] Completed 38 of 1000 simulation runs\n[15-Feb-2026 02:54:01] Received simulation output (size: 162.31 KB) for run 38 from parallel worker.\n[15-Feb-2026 02:54:03] Completed 39 of 1000 simulation runs\n[15-Feb-2026 02:54:03] Received simulation output (size: 162.31 KB) for run 39 from parallel worker.\n[15-Feb-2026 02:54:03] Completed 40 of 1000 simulation runs\n[15-Feb-2026 02:54:03] Received simulation output (size: 162.31 KB) for run 40 from parallel worker.\n[15-Feb-2026 02:54:03] Completed 41 of 1000 simulation runs\n[15-Feb-2026 02:54:03] Received simulation output (size: 162.31 KB) for run 41 from parallel worker.\n[15-Feb-2026 02:54:05] Completed 42 of 1000 simulation runs\n[15-Feb-2026 02:54:05] Received simulation output (size: 162.31 KB) for run 42 from parallel worker.\n[15-Feb-2026 02:54:13] Completed 43 of 1000 simulation runs\n[15-Feb-2026 02:54:13] Received simulation output (size: 162.31 KB) for run 43 from parallel worker.\n[15-Feb-2026 02:54:13] Completed 44 of 1000 simulation runs\n[15-Feb-2026 02:54:13] Received simulation output (size: 162.31 KB) for run 44 from parallel worker.\n[15-Feb-2026 02:54:14] Completed 45 of 1000 simulation runs\n[15-Feb-2026 02:54:14] Received simulation output (size: 162.31 KB) for run 45 from parallel worker.\n[15-Feb-2026 02:54:14] Completed 46 of 1000 simulation runs\n[15-Feb-2026 02:54:14] Received simulation output (size: 162.31 KB) for run 46 from parallel worker.\n[15-Feb-2026 02:54:14] Completed 47 of 1000 simulation runs\n[15-Feb-2026 02:54:14] Received simulation output (size: 162.31 KB) for run 47 from parallel worker.\n[15-Feb-2026 02:54:18] Completed 48 of 1000 simulation runs\n[15-Feb-2026 02:54:18] Received simulation output (size: 162.31 KB) for run 48 from parallel worker.\n[15-Feb-2026 02:54:24] Completed 49 of 1000 simulation runs\n[15-Feb-2026 02:54:24] Received simulation output (size: 162.31 KB) for run 49 from parallel worker.\n[15-Feb-2026 02:54:24] Completed 50 of 1000 simulation runs\n[15-Feb-2026 02:54:24] Received simulation output (size: 162.31 KB) for run 50 from parallel worker.\n[15-Feb-2026 02:54:25] Completed 51 of 1000 simulation runs\n[15-Feb-2026 02:54:25] Received simulation output (size: 162.31 KB) for run 51 from parallel worker.\n[15-Feb-2026 02:54:25] Completed 52 of 1000 simulation runs\n[15-Feb-2026 02:54:25] Received simulation output (size: 162.31 KB) for run 52 from parallel worker.\n[15-Feb-2026 02:54:26] Completed 53 of 1000 simulation runs\n[15-Feb-2026 02:54:26] Received simulation output (size: 162.31 KB) for run 53 from parallel worker.\n[15-Feb-2026 02:54:30] Completed 54 of 1000 simulation runs\n[15-Feb-2026 02:54:30] Received simulation output (size: 162.31 KB) for run 54 from parallel worker.\n[15-Feb-2026 02:54:35] Completed 55 of 1000 simulation runs\n[15-Feb-2026 02:54:35] Received simulation output (size: 162.31 KB) for run 55 from parallel worker.\n[15-Feb-2026 02:54:35] Completed 56 of 1000 simulation runs\n[15-Feb-2026 02:54:35] Received simulation output (size: 162.31 KB) for run 56 from parallel worker.\n[15-Feb-2026 02:54:36] Completed 57 of 1000 simulation runs\n[15-Feb-2026 02:54:36] Received simulation output (size: 162.31 KB) for run 57 from parallel worker.\n[15-Feb-2026 02:54:36] Completed 58 of 1000 simulation runs\n[15-Feb-2026 02:54:36] Received simulation output (size: 162.31 KB) for run 58 from parallel worker.\n[15-Feb-2026 02:54:38] Completed 59 of 1000 simulation runs\n[15-Feb-2026 02:54:38] Received simulation output (size: 162.31 KB) for run 59 from parallel worker.\n[15-Feb-2026 02:54:42] Completed 60 of 1000 simulation runs\n[15-Feb-2026 02:54:42] Received simulation output (size: 162.31 KB) for run 60 from parallel worker.\n[15-Feb-2026 02:54:45] Completed 61 of 1000 simulation runs\n[15-Feb-2026 02:54:46] Received simulation output (size: 162.31 KB) for run 61 from parallel worker.\n[15-Feb-2026 02:54:47] Completed 62 of 1000 simulation runs\n[15-Feb-2026 02:54:47] Received simulation output (size: 162.31 KB) for run 62 from parallel worker.\n[15-Feb-2026 02:54:47] Completed 63 of 1000 simulation runs\n[15-Feb-2026 02:54:47] Received simulation output (size: 162.31 KB) for run 63 from parallel worker.\n[15-Feb-2026 02:54:47] Completed 64 of 1000 simulation runs\n[15-Feb-2026 02:54:47] Received simulation output (size: 162.31 KB) for run 64 from parallel worker.\n[15-Feb-2026 02:54:49] Completed 65 of 1000 simulation runs\n[15-Feb-2026 02:54:49] Received simulation output (size: 162.31 KB) for run 65 from parallel worker.\n[15-Feb-2026 02:54:53] Completed 66 of 1000 simulation runs\n[15-Feb-2026 02:54:53] Received simulation output (size: 162.31 KB) for run 66 from parallel worker.\n[15-Feb-2026 02:54:57] Completed 67 of 1000 simulation runs\n[15-Feb-2026 02:54:57] Received simulation output (size: 162.31 KB) for run 67 from parallel worker.\n[15-Feb-2026 02:54:58] Completed 68 of 1000 simulation runs\n[15-Feb-2026 02:54:58] Received simulation output (size: 162.31 KB) for run 68 from parallel worker.\n[15-Feb-2026 02:54:59] Completed 69 of 1000 simulation runs\n[15-Feb-2026 02:54:59] Received simulation output (size: 162.31 KB) for run 69 from parallel worker.\n[15-Feb-2026 02:54:59] Completed 70 of 1000 simulation runs\n[15-Feb-2026 02:54:59] Received simulation output (size: 162.31 KB) for run 70 from parallel worker.\n[15-Feb-2026 02:55:00] Completed 71 of 1000 simulation runs\n[15-Feb-2026 02:55:01] Received simulation output (size: 162.31 KB) for run 71 from parallel worker.\n[15-Feb-2026 02:55:04] Completed 72 of 1000 simulation runs\n[15-Feb-2026 02:55:04] Received simulation output (size: 162.31 KB) for run 72 from parallel worker.\n[15-Feb-2026 02:55:08] Completed 73 of 1000 simulation runs\n[15-Feb-2026 02:55:08] Received simulation output (size: 162.31 KB) for run 73 from parallel worker.\n[15-Feb-2026 02:55:09] Completed 74 of 1000 simulation runs\n[15-Feb-2026 02:55:09] Received simulation output (size: 162.31 KB) for run 74 from parallel worker.\n[15-Feb-2026 02:55:10] Completed 75 of 1000 simulation runs\n[15-Feb-2026 02:55:10] Received simulation output (size: 162.31 KB) for run 75 from parallel worker.\n[15-Feb-2026 02:55:10] Completed 76 of 1000 simulation runs\n[15-Feb-2026 02:55:10] Received simulation output (size: 162.31 KB) for run 76 from parallel worker.\n[15-Feb-2026 02:55:13] Completed 77 of 1000 simulation runs\n[15-Feb-2026 02:55:13] Received simulation output (size: 162.31 KB) for run 77 from parallel worker.\n[15-Feb-2026 02:55:16] Completed 78 of 1000 simulation runs\n[15-Feb-2026 02:55:16] Received simulation output (size: 162.31 KB) for run 78 from parallel worker.\n[15-Feb-2026 02:55:21] Completed 79 of 1000 simulation runs\n[15-Feb-2026 02:55:21] Received simulation output (size: 162.31 KB) for run 79 from parallel worker.\n[15-Feb-2026 02:55:22] Completed 80 of 1000 simulation runs\n[15-Feb-2026 02:55:22] Received simulation output (size: 162.31 KB) for run 80 from parallel worker.\n[15-Feb-2026 02:55:22] Completed 81 of 1000 simulation runs\n[15-Feb-2026 02:55:22] Received simulation output (size: 162.31 KB) for run 81 from parallel worker.\n[15-Feb-2026 02:55:22] Completed 82 of 1000 simulation runs\n[15-Feb-2026 02:55:22] Received simulation output (size: 162.31 KB) for run 82 from parallel worker.\n[15-Feb-2026 02:55:24] Completed 83 of 1000 simulation runs\n[15-Feb-2026 02:55:25] Received simulation output (size: 162.31 KB) for run 83 from parallel worker.\n[15-Feb-2026 02:55:28] Completed 84 of 1000 simulation runs\n[15-Feb-2026 02:55:28] Received simulation output (size: 162.31 KB) for run 84 from parallel worker.\n[15-Feb-2026 02:55:32] Completed 85 of 1000 simulation runs\n[15-Feb-2026 02:55:32] Received simulation output (size: 162.31 KB) for run 85 from parallel worker.\n[15-Feb-2026 02:55:32] Completed 86 of 1000 simulation runs\n[15-Feb-2026 02:55:32] Received simulation output (size: 162.31 KB) for run 86 from parallel worker.\n[15-Feb-2026 02:55:32] Completed 87 of 1000 simulation runs\n[15-Feb-2026 02:55:32] Received simulation output (size: 162.31 KB) for run 87 from parallel worker.\n[15-Feb-2026 02:55:33] Completed 88 of 1000 simulation runs\n[15-Feb-2026 02:55:33] Received simulation output (size: 162.31 KB) for run 88 from parallel worker.\n[15-Feb-2026 02:55:35] Completed 89 of 1000 simulation runs\n[15-Feb-2026 02:55:35] Received simulation output (size: 162.31 KB) for run 89 from parallel worker.\n[15-Feb-2026 02:55:40] Completed 90 of 1000 simulation runs\n[15-Feb-2026 02:55:40] Received simulation output (size: 162.31 KB) for run 90 from parallel worker.\n[15-Feb-2026 02:55:44] Completed 91 of 1000 simulation runs\n[15-Feb-2026 02:55:44] Received simulation output (size: 162.31 KB) for run 91 from parallel worker.\n[15-Feb-2026 02:55:44] Completed 92 of 1000 simulation runs\n[15-Feb-2026 02:55:44] Received simulation output (size: 162.31 KB) for run 92 from parallel worker.\n[15-Feb-2026 02:55:44] Completed 93 of 1000 simulation runs\n[15-Feb-2026 02:55:44] Received simulation output (size: 162.31 KB) for run 93 from parallel worker.\n[15-Feb-2026 02:55:46] Completed 94 of 1000 simulation runs\n[15-Feb-2026 02:55:46] Received simulation output (size: 162.31 KB) for run 94 from parallel worker.\n[15-Feb-2026 02:55:47] Completed 95 of 1000 simulation runs\n[15-Feb-2026 02:55:47] Received simulation output (size: 162.31 KB) for run 95 from parallel worker.\n[15-Feb-2026 02:55:52] Completed 96 of 1000 simulation runs\n[15-Feb-2026 02:55:52] Received simulation output (size: 162.31 KB) for run 96 from parallel worker.\n[15-Feb-2026 02:55:55] Completed 97 of 1000 simulation runs\n[15-Feb-2026 02:55:56] Received simulation output (size: 162.31 KB) for run 97 from parallel worker.\n[15-Feb-2026 02:55:56] Completed 98 of 1000 simulation runs\n[15-Feb-2026 02:55:57] Received simulation output (size: 162.31 KB) for run 98 from parallel worker.\n[15-Feb-2026 02:55:57] Completed 99 of 1000 simulation runs\n[15-Feb-2026 02:55:57] Received simulation output (size: 162.31 KB) for run 99 from parallel worker.\n[15-Feb-2026 02:55:58] Completed 100 of 1000 simulation runs\n[15-Feb-2026 02:55:58] Received simulation output (size: 162.31 KB) for run 100 from parallel worker.\n[15-Feb-2026 02:56:00] Completed 101 of 1000 simulation runs\n[15-Feb-2026 02:56:00] Received simulation output (size: 162.31 KB) for run 101 from parallel worker.\n[15-Feb-2026 02:56:04] Completed 102 of 1000 simulation runs\n[15-Feb-2026 02:56:05] Received simulation output (size: 162.31 KB) for run 102 from parallel worker.\n[15-Feb-2026 02:56:09] Completed 103 of 1000 simulation runs\n[15-Feb-2026 02:56:09] Received simulation output (size: 162.31 KB) for run 103 from parallel worker.\n[15-Feb-2026 02:56:09] Completed 104 of 1000 simulation runs\n[15-Feb-2026 02:56:09] Received simulation output (size: 162.31 KB) for run 104 from parallel worker.\n[15-Feb-2026 02:56:09] Completed 105 of 1000 simulation runs\n[15-Feb-2026 02:56:09] Received simulation output (size: 162.31 KB) for run 105 from parallel worker.\n[15-Feb-2026 02:56:11] Completed 106 of 1000 simulation runs\n[15-Feb-2026 02:56:11] Received simulation output (size: 162.31 KB) for run 106 from parallel worker.\n[15-Feb-2026 02:56:13] Completed 107 of 1000 simulation runs\n[15-Feb-2026 02:56:13] Received simulation output (size: 162.31 KB) for run 107 from parallel worker.\n[15-Feb-2026 02:56:17] Completed 108 of 1000 simulation runs\n[15-Feb-2026 02:56:17] Received simulation output (size: 162.31 KB) for run 108 from parallel worker.\n[15-Feb-2026 02:56:21] Completed 109 of 1000 simulation runs\n[15-Feb-2026 02:56:21] Received simulation output (size: 162.31 KB) for run 109 from parallel worker.\n[15-Feb-2026 02:56:21] Completed 110 of 1000 simulation runs\n[15-Feb-2026 02:56:21] Received simulation output (size: 162.31 KB) for run 110 from parallel worker.\n[15-Feb-2026 02:56:21] Completed 111 of 1000 simulation runs\n[15-Feb-2026 02:56:21] Received simulation output (size: 162.31 KB) for run 111 from parallel worker.\n[15-Feb-2026 02:56:23] Completed 112 of 1000 simulation runs\n[15-Feb-2026 02:56:23] Received simulation output (size: 162.31 KB) for run 112 from parallel worker.\n[15-Feb-2026 02:56:24] Completed 113 of 1000 simulation runs\n[15-Feb-2026 02:56:24] Received simulation output (size: 162.31 KB) for run 113 from parallel worker.\n[15-Feb-2026 02:56:29] Completed 114 of 1000 simulation runs\n[15-Feb-2026 02:56:30] Received simulation output (size: 162.31 KB) for run 114 from parallel worker.\n[15-Feb-2026 02:56:30] Cleaning up parallel workers...\n","truncated":false}}
%---
%[output:1b9f3d4a]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'simOut'."}}
%---

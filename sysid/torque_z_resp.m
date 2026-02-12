T = readtable("thrust_response.csv");

head(T)

u = T.thrust_sp;
y = T.torque_z * 0.13;
t_ms = T.time_ms;

Ts = (t_ms(3) - t_ms(2)) / 1000;

% filter y
fs = 100;                % Sampling frequency (Hz)
fc = 4;                 % Cutoff frequency (Hz) - Starting point
nyquist = fs / 2;       % Nyquist frequency (5 Hz)
Wn = fc / nyquist; 

[b, a] = butter(1, Wn, 'low');
y_filt = filtfilt(b, a, y);

t = (0:length(y)-1) / fs;
plot(t, y, 'Color', [0.7 0.7 0.7]); 
hold on;
plot(t, y_filt, 'LineWidth', 2);
legend('Raw Sensor Data', 'filtfilt (1st order)');
xlabel('Time (s)'); ylabel('Thrust');

sys = iddata(y_filt, u, Ts);

%% get sys e object and then normalize so the TF always reaches desired values
est = syse;

u_norm = (1 * (est.InputData - min(est.InputData)) / (max(est.InputData) - min(est.InputData)));
% y_norm = (1 * (est.OutputData - min(est.OutputData)) / (max(est.OutputData) - min(est.OutputData)));

% remove means on u_norm
y = est.OutputData * 0.13;
% u = est.InputData


G = load("thrust_tf.mat").thrust_tf;
t = (0:length(u_norm)-1) / fs;
% subtract the response to isolate the unmodelled inertia term
step_resp = (0.191 + (0.314-0.191)*lsim(G, u_norm, t));
% shift step_resp back 0.15 seconds
step_resp_shifted = circshift(step_resp, round(-0.1 * fs)); % Shift step response by 0.15 seconds
y_unmodelled = y - step_resp_shifted;
u_der = gradient(step_resp_shifted, 0.01);
t_u = (0:length(u_new)-1) / fs;
plot(t, y)
hold on;
% plot(t_u, u_der)
% plot(t_u, y_unmodelled)
% %plot(t_u, step_resp_shifted)

hold off;
figure
plot(u_new, y_unmodelled)

sysee = iddata(y_unmodelled, u, Ts);
legend()
est = sysv;

u_norm = (1 * (est.InputData - min(est.InputData)) / (max(est.InputData) - min(est.InputData)));
y_norm = (1 * (est.OutputData - min(est.OutputData)) / (max(est.OutputData) - min(est.OutputData)));

t = (0:length(u_norm)-1) / fs;
y_unmodelled_v = y_norm - lsim(G, u_norm, t);
sysvv = iddata(y_unmodelled_v, u_norm, Ts);

%% save
thrust_tf = tf16;
save("thrust_tf.mat", "thrust_tf");

%% modify thrust_tf
thrust_tf = load("thrust_tf.mat").thrust_tf;

num_0 = thrust_tf.Numerator(2)
den_0 = thrust_tf.Denominator(3)
thrust_tf.numerator = thrust_tf.numerator * (den_0 / num_0)
save("thrust_tf.mat", "thrust_tf");
T = readtable("f_y_response.csv");

head(T)

u = T.B_y;
y = T.force_y;

% filter y
fs = 10;                % Sampling frequency (Hz)
fc = 0.3;                 % Cutoff frequency (Hz) - Starting point
nyquist = fs / 2;       % Nyquist frequency (5 Hz)
Wn = fc / nyquist; 

[b, a] = butter(1, Wn, 'low');
y_filt = filtfilt(b, a, y);

t = (0:length(y)-1) / fs;
plot(t, y, 'Color', [0.7 0.7 0.7]); 
hold on;
plot(t, y_filt, 'LineWidth', 2);
legend('Raw Sensor Data', 'filtfilt (1st order)');
xlabel('Time (s)'); ylabel('Force_y');
% end filter

t_ms = T.time_ms;
Ts = (t_ms(2) - t_ms(1)) / 1000;
sys = iddata(y_filt, u, Ts);

%% normalize values between 0 and 1
u_norm = (1 * (syse.InputData - min(syse.InputData)) / (max(syse.InputData) - min(syse.InputData)));
y_norm = (1 * (syse.OutputData - min(syse.OutputData)) / (max(syse.OutputData) - min(syse.OutputData)));

sysee = iddata(y_norm, u_norm, Ts);

v = sysv1;
u_norm_v1 = (2 * (v.InputData - min(v.InputData)) / (max(v.InputData) - min(v.InputData))) - 1;
y_norm_v1 = (2 * (v.OutputData - min(v.OutputData)) / (max(v.OutputData) - min(v.OutputData))) - 1;
sysvv1 = iddata(y_norm_v1, u_norm_v1, Ts);

v = sysv2;
u_norm_v2 = (1 * (v.InputData - min(v.InputData)) / (max(v.InputData) - min(v.InputData)));
y_norm_v2 = (1 * (v.OutputData - min(v.OutputData)) / (max(v.OutputData) - min(v.OutputData)));
sysvv2 = iddata(y_norm_v2, u_norm_v2, Ts);

%% save tf
tf_force_y = tf14;
save('f_xy_tf.mat', 'tf_force_y');

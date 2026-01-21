T = readtable("y_force_resp_step.csv");

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
xlabel('Time (s)'); ylabel('Position');
% end filter

t_ms = T.time_ms;
Ts = (t_ms(2) - t_ms(1)) / 1000;
sys = iddata(y_filt, u, Ts);

%% normalize values between 0 and 1
u_norm = (1 * (syse.InputData - min(syse.InputData)) / (max(syse.InputData) - min(syse.InputData)));
y_norm = (1 * (syse.OutputData - min(syse.OutputData)) / (max(syse.OutputData) - min(syse.OutputData)));

sysee = iddata(y_norm, u_norm, Ts);

%% save tf
save('f_xy_tf.mat', 'tf1');

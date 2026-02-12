T = readtable("thrust_response.csv");

head(T)

u = T.thrust_sp;
y = -T.thrust;
t_ms = T.time_ms;

Ts = (t_ms(3) - t_ms(2)) / 1000

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
xlabel('Time (s)'); ylabel('Thrust');

sys = iddata(y_filt, u, Ts);

%% get sys e object and then normalize so the TF always reaches desired values
est = syse;

u_norm = (1 * (est.InputData - min(est.InputData)) / (max(est.InputData) - min(est.InputData)));
y_norm = (1 * (est.OutputData - min(est.OutputData)) / (max(est.OutputData) - min(est.OutputData)));

sysee = iddata(y_norm, u_norm, Ts);

est = sysv;

u_norm = (1 * (est.InputData - min(est.InputData)) / (max(est.InputData) - min(est.InputData)));
y_norm = (1 * (est.OutputData - min(est.OutputData)) / (max(est.OutputData) - min(est.OutputData)));

sysvv = iddata(y_norm, u_norm, Ts);

%% save
thrust_tf = tf4;
save("thrust_tf.mat", "thrust_tf");

T = readtable("thrust_resp_step.csv");

head(T)

u = T.thrust_sp;
y = -T.thrust;
t_ms = T.time_ms;

Ts = (t_ms(2) - t_ms(1)) / 1000;

% normalize thrust setpoint and values between 0 and 1
u_norm = (u - min(u)) / (max(u) - min(u));
y_norm = (y - min(y)) / (max(y) - min(y));

u_norm = filtfilt(u_norm);
y_norm = filtfilt(y_norm);

sys = iddata(y_norm, u_norm, Ts);
systemIdentification;
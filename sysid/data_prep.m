T = readtable("thrust_step.csv");

head(T)

u = T.thrust_sp;
y = T.thrust;
t_ms = T.time_ms;

Ts = (t_ms(2) - t_ms(1)) / 1000;

sys = iddata(y, u, Ts);

sys.OutputName = 'Measured Thrust';
sys.InputName = 'Commanded Thrust';
sys.OutputUnit = 'N';
sys.InputUnit = '%'; 

systemIdentification

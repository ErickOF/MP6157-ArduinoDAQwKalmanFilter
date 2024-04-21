% Constants
g = 9.8;
%% Sampling data time
delta_t = 0.1;
%% Position standar deviation
stdev_x = 0.2;
%% Velocity standar deviation
stdev_v = 0.2 / delta_t;
%% Measurement deviation
R = stdev_x^2;
%% Process deviaton
Q = [0 0; 0 0];

% Initial conditions
x0 = [0; 0];
x0e = [5; 3];
P0 = (2 * stdev_x)^2 * eye(2);
y0 = 0;

% Continuos system model
A = [0 1; 0 0];
B = [0; 1];
H = [1 0];
D = 0;

sys = ss(A, B, H, D);

% Discrete system model
dsys = c2d(sys, T);
Phi = dsys.A;
Gamma = dsys.B;

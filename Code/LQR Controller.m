% GATHERING LONGITUDINAL CONTROLS
% System matrix
A = [-0.890 -0.343 0;
     -0.106 -0.117 0;
      0      83.86 0];

% Input matrix
B = [-1.57;
     -0.002;
      0];

% Output matrix for theta
C = [0 0 1];
D = 0;

% Choose Q and R
Q = diag([10, 1, 100]);
R = 1;

% LQR gain
K = lqr(A, B, Q, R);

% State-space system
sys = ss(A, B, C, D);

% Compute Nbar for reference tracking
Nbar = rscale(sys, K);

% Desired pitch command (5 degrees in radians)
r = 5 * pi / 180;

% Simulate from initial state x0 = [0; 0; 0]
x0 = [0; 0; 0];
t = 0:0.01:5;

% Closed-loop system with reference input
Acl = A - B*K;
Bcl = B * Nbar;

sys_cl = ss(Acl, Bcl, eye(3), 0);  % Outputs = all states

% Simulate
[y, t, x] = lsim(sys_cl, r * ones(size(t)), t, x0);

% Plot states
figure;
plot(t, x);
legend('\alpha', 'q', '\theta');
xlabel('Time (s)');
ylabel('State values');
title('LQR with Pitch Command Reference (5°)');

% GATHERING LATERAL CONTROLS
% Lateral-directional system matrix (from ASU lecture)
A_lat = [
   -0.322,  0.052,  0.028, -1.12,   0.002;
    0,     -0.429,  0.804,  0,     -0.001;
   -10.6,   0,     -2.87,   0,      0.46;
    6.87,   0,    -0.04,   -0.32,  -0.02;
    0,      0,      1,      0,      0
];

% Inputs: [aileron, rudder]
B_lat = [
    0,      0.002;
    0.001,  0;
   -0.65,   0.13;
   -0.02,   0.0001;
    0,      0
];

% Choose Q and R
Q_lat = [  10     0     0     0     0;
            0   300     0     0     0;
            0     0   300     0     0;
            0     0     0   1000     0;
            0     0     0     0    100 ];

R_lat = diag([1, 1]);

% Compute LQR gain
K_lat = lqr(A_lat, B_lat, Q_lat, R_lat)

% Output matrix
C_lat = [0 0 0 1 0];
D_lat = [0 0];

% Create state-space system for computing Nbar
sys_lat = ss(A_lat, B_lat, C_lat, D_lat);

% Compute reference gain Nbar
Nbar_lat = rscale(sys_lat, K_lat);

% Desired roll command at 10 degrees
r_phi = 10 * pi / 180;

% Simulation setup
Acl = A_lat - B_lat * K_lat;
Bcl = B_lat * Nbar_lat;
sys_cl = ss(Acl, Bcl, eye(5), 0);

x0 = [0; 0; 0; 0; 0];
t = 0:0.01:20;
u = r_phi * ones(size(t)); 

% Simulate system response
[y, t, x] = lsim(sys_cl, u, t, x0);

% Plot states
figure;
plot(t, x);
legend('v', 'p', 'r', '\phi', '\psi');
xlabel('Time (s)');
ylabel('States');
title('Lateral LQR with Roll Angle Tracking (10° Step)');
grid on;


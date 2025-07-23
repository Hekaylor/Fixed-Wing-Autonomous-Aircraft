% A, B, C, and D from State Space Model
A = [-0.890  -0.343   0;
     -0.106  -0.117   0;
      0      83.86    0];

B = [-1.57;
     -0.002;
      0];

C = [0 0 1];
D = 0;

% LQR gain and feedforward
Q = diag([10, 1, 100]);
R = 1;
K = lqr(A, B, Q, R);

sys = ss(A, B, C, D);

% Simulate closed-loop system with reference input
x0 = [0; 0; 0];
r = 5 * pi/180;
t = 0:0.01:5;

Acl = A - B * K;
Bcl = B * Nbar;
sys_cl = ss(Acl, Bcl, eye(3), 0);

[y, t, x] = lsim(sys_cl, r * ones(size(t)), t, x0);

% Create discrete system for Kalman filter
Ts = 0.01;
sysd = c2d(sys, Ts);

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

% Add measurement noise to theta (pitch angle)
theta_true = x(:,3); 
noise_std = 0.5 * pi/180;  % noise std dev in radians
theta_measured = theta_true + noise_std * randn(size(theta_true));

% Process noise covariance and measurement noise variance
Q_kf = diag([1e-4, 1e-4, 1e-6]); 
R_kf = noise_std^2;                

% Initialize Kalman filter variables
x_hat = zeros(3, length(t));  
P = eye(3); 

% Run discrete Kalman Filter with control input from estimated states
for k = 1:length(t)-1
    uk = -K * x_hat(:,k) + Nbar * r;

    x_pred = Ad * x_hat(:,k) + Bd * uk;
    P_pred = Ad * P * Ad' + Q_kf;

    y_meas = theta_measured(k);
    Kk = P_pred * Cd' / (Cd * P_pred * Cd' + R_kf);
    x_hat(:,k+1) = x_pred + Kk * (y_meas - Cd * x_pred);
    P = (eye(3) - Kk * Cd) * P_pred;
end

% Compute control input from estimated states over entire time
u_hat = zeros(1,length(t));
for k = 1:length(t)
    u_hat(k) = -K * x_hat(:,k) + Nbar * r;
end

% Plot true states vs estimated states
figure;
subplot(3,1,1);
plot(t, x(:,1), t, x_hat(1,:));
ylabel('\alpha');
legend('True', 'Estimated');

subplot(3,1,2);
plot(t, x(:,2), t, x_hat(2,:));
ylabel('q');

subplot(3,1,3);
plot(t, x(:,3), t, x_hat(3,:));
ylabel('\theta');
xlabel('Time (s)');
sgtitle('Kalman Filter State Estimation');

% Compare control inputs
figure;
plot(t, -K * x' + Nbar * r, 'k', t, u_hat, 'r--');
legend('LQR w/ true state', 'LQR w/ estimated state');
xlabel('Time (s)');
ylabel('Elevator input');
title('Control Input Comparison');
s = tf('s');

% Plant model (pitch dynamics)
P_pitch = (-0.168*s + 14.0886) / (s^3 + 1.007*s^2 + 0.0682*s);

% Actuator dynamics (1st-order lag)
tau = 0.05;  % seconds
Actuator = 1 / (tau*s + 1);

% Combined plant
P_combined = P_pitch * Actuator;

% PID tuning with target bandwidth
target_bandwidth = 0.88;  % rad/s
[C_opt, info] = pidtune(P_combined, 'PID', target_bandwidth);

% Closed-loop system
sys_cl = feedback(C_opt * P_combined, 1);

% Step response
figure;
step(0.2*sys_cl, 0:0.01:20);
ylabel('Pitch Angle (rad)');
title('Closed-loop Step Response (Tuned PID)');
grid on;

% Display poles and step response info
disp('Closed-loop poles:');
disp(pole(sys_cl));
disp('Step response info:');
disp(stepinfo(sys_cl));

% Show tuned PID gains
disp('Tuned PID gains:');
C_opt

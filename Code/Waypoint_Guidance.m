% Define Parameters
L1_dist = 50;
waypoints = [0, 0; 500, 200; 1000, 0];

dt = 0.1;      
T = 60;                    
N = round(T/dt);          
V = 25;                    
k_heading = 1.5;           
switch_dist = 20;           

% Define position and error variables
pos = zeros(2, N);         
psi = zeros(1, N);        
psi_cmd = zeros(1, N);     
xt_err = zeros(1, N);      
wp_index = 1;

% Define initial conditions (East)
pos(:,1) = [0; 0];
psi(1) = deg2rad(0);

% Simulation loop
for k = 1:N-1
    % Check for waypoint switching
    if wp_index < size(waypoints,1) && ...
       norm(pos(:,k) - waypoints(wp_index+1,:)') < switch_dist
        wp_index = wp_index + 1;
    end

    % Compute L1 heading if valid leg
    if wp_index < size(waypoints,1)
        p1 = waypoints(wp_index,:)';
        p2 = waypoints(wp_index+1,:)';
        pos_k = pos(:,k);
        vel_k = V * [cos(psi(k)); sin(psi(k))];

        path_vec = p2 - p1;
        path_unit = path_vec / norm(path_vec);
        s = dot(pos_k - p1, path_unit);           
        e = (pos_k - p1) - s * path_unit;         
        xt_err(k) = norm(e) * sign(path_unit(1)*e(2) - path_unit(2)*e(1));

        L1_vec = s * path_unit + L1_dist * path_unit - (pos_k - p1);
        psi_cmd(k) = atan2(L1_vec(2), L1_vec(1));
    else
        psi_cmd(k) = psi_cmd(k-1);
        xt_err(k) = xt_err(k-1);
    end

    % Heading controller (P)
    psi_error = wrapToPi(psi_cmd(k) - psi(k));
    psi_dot = k_heading * psi_error;

    % Integrate heading
    psi(k+1) = psi(k) + psi_dot * dt;

    % Integrate position
    pos(:,k+1) = pos(:,k) + V * [cos(psi(k+1)); sin(psi(k+1))] * dt;
end

% Final values
psi_cmd(end) = psi_cmd(end-1);
xt_err(end) = xt_err(end-1);

% Animate flight trajectory simulation
figure; hold on; axis equal; grid on;
plot(waypoints(:,1), waypoints(:,2), 'ko--', 'LineWidth', 2);
traj = plot(pos(1,1), pos(2,1), 'b-', 'LineWidth', 1.5);
aircraft = scatter(pos(1,1), pos(2,1), 100, 'r', 'filled');
heading_arrow = quiver(pos(1,1), pos(2,1), cos(psi(1)), sin(psi(1)), 20, 'g', 'LineWidth', 2);
title('Aircraft Path Following via L1 Guidance');
legend('Waypoints', 'Trajectory', 'Aircraft', 'Heading');

for k = 1:5:N
    set(traj, 'XData', pos(1,1:k), 'YData', pos(2,1:k));
    set(aircraft, 'XData', pos(1,k), 'YData', pos(2,k));
    set(heading_arrow, 'XData', pos(1,k), 'YData', pos(2,k), ...
        'UData', cos(psi(k)), 'VData', sin(psi(k)));
    drawnow;
end

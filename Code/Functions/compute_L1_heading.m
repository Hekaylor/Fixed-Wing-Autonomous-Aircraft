function [psi_cmd, crosstrack_error] = compute_L1_heading(pos, vel, wp_prev, wp_next, L1_distance)

% Path unit vector and segment length
path_vec = wp_next - wp_prev;
path_unit = path_vec / norm(path_vec);

% Vector from previous WP to aircraft
vec_to_pos = pos - wp_prev;

% Project position onto path
proj_len = dot(vec_to_pos, path_unit);
proj_point = wp_prev + proj_len * path_unit;

% Crosstrack error and direction
vec_crosstrack = pos - proj_point;
crosstrack_error = norm(vec_crosstrack);
turn_dir = sign(path_unit(1)*vec_crosstrack(2) - path_unit(2)*vec_crosstrack(1));

% Normalize velocity
vel_norm = vel / max(norm(vel), 1e-3);

% Angle between path and aircraft track
eta_raw = turn_dir * crosstrack_error / max(L1_distance, 1e-6);
eta = asin(max(-1, min(1, eta_raw)));

% Heading command
psi_path = atan2(path_unit(2), path_unit(1));
psi_cmd = psi_path + atan2(2 * L1_distance * sin(eta), crosstrack_error + 1e-3);
psi_cmd = wrapToPi(psi_cmd);
end

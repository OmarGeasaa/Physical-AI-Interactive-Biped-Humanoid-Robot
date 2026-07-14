% LIPM ZMP Preview Control 

% 1. System Parameters
zc = 0.65188;             % CoM constant height (m) 
com_offset_X = -0.095;    % Example: CoM is 2cm behind the pelvis origin
com_offset_Y =  -0.00397; % Example: CoM is 0.5cm to the left
g = 9.81;                 % Gravity (m/s^2)
Ts = 0.01;                % Control loop sample time (100 Hz)
preview_time = 1.5;       % Preview window (1.5s to see the next step)
N = round(preview_time / Ts); 

%% 2. Discrete State-Space Matrices (Same for both X and Y)
A = [1, Ts, Ts^2/2; 0, 1, Ts; 0, 0, 1];
B = [Ts^3/6; Ts^2/2; Ts];
C = [1, 0, -zc/g];

% Augmented System for Tracking Error
C_A = C * A;
C_B = C * B;
A_tilde = [1, C_A; zeros(3,1), A];
B_tilde = [C_B; B];
I_tilde = [1; 0; 0; 0]; 

%% 3. LQR Cost Weights & Gain Calculation
Q_error = 1e8;      % High penalty on ZMP tracking error
R = 1.0;            % Penalty on control effort (jerk)
Q = blkdiag(Q_error, 0, 0, 0);

[P, ~, K_tilde] = dare(A_tilde, B_tilde, Q, R);
Gi = K_tilde(1);
Gx = K_tilde(2:4);
Ac = A_tilde - B_tilde * K_tilde;
f = zeros(1, N);
for j = 1:N
    f(j) = (R + B_tilde' * P * B_tilde)^(-1) * B_tilde' * (Ac')^(j-1) * P * I_tilde;
end

%% 4. Generate 2D ZMP Reference Trajectories (Footprints)
sim_time = 15;       % Time allocated for actual walking
T_step = 3.33;       % Matches your SEA cadence
step_width = 0.20;   % 20cm stance width

% Simulated Nav2 command
cmd_vel_x = 0.06;    % Walking forward at 6 cm/s
cmd_vel_y = 0.0;     % No lateral drift

% Generate the dynamic arrays using the new function
[pref_X_base, pref_Y_base] = plan_footsteps(cmd_vel_x, cmd_vel_y, T_step, Ts, sim_time, step_width);

% --- NEW: The Preparation Phase ---
% Create an array of zeros representing 1.5 seconds of standing still
startup_ticks = round(1.5 / Ts); 
startup_pad_X = zeros(1, startup_ticks);
startup_pad_Y = zeros(1, startup_ticks);

% Pad the arrays at the START (Preparation) and the END (Cooldown)
pref_X = [startup_pad_X, pref_X_base, pref_X_base(end) * ones(1, N)]; 
pref_Y = [startup_pad_Y, pref_Y_base, pref_Y_base(end) * ones(1, N)];

% IMPORTANT: Update 'steps' so the for-loop runs the correct number of times
% We subtract N because the simulation loop stops right before the preview window ends
total_trajectory_length = length(pref_X) - N; 
steps = total_trajectory_length;

%% 5. Simulation Loop & Translation
X_state = [0; 0; 0]; % Initial CoM state X [pos; vel; acc]
Y_state = [0; 0; 0]; % Initial CoM state Y [pos; vel; acc]

error_sum_X = 0;
error_sum_Y = 0;

% Output Logs
log_com_X = zeros(1, steps);
log_com_Y = zeros(1, steps);
log_pitch = zeros(1, steps);
log_roll  = zeros(1, steps);

for k = 1:steps
    % --- X-Axis Physics ---
    zmp_X = C * X_state;
    log_com_X(k) = X_state(1);
    error_sum_X = error_sum_X + (zmp_X - pref_X(k));
    prev_X = sum(f .* pref_X(k+1 : k+N));
    u_X = -Gi * error_sum_X - Gx * X_state + prev_X;
    X_state = A * X_state + B * u_X;
    
    % --- Y-Axis Physics ---
    zmp_Y = C * Y_state;
    log_com_Y(k) = Y_state(1);
    error_sum_Y = error_sum_Y + (zmp_Y - pref_Y(k));
    prev_Y = sum(f .* pref_Y(k+1 : k+N));
    u_Y = -Gi * error_sum_Y - Gx * Y_state + prev_Y;
    Y_state = A * Y_state + B * u_Y;
    
%% 6. KINEMATIC TRANSLATION (LIPM -> 3-UPR)
    % Translating horizontal CoM travel into joint angles.
    % atan2(opposite, adjacent) handles safety and quadrants.

    % Shift the reference frame to the current stance foot
    pelvis_X = X_state(1) - com_offset_X;
    pelvis_Y = Y_state(1) - com_offset_Y;

    % 3. Shift the reference frame to the current stance foot
    relative_X = pelvis_X - pref_X(k);
    relative_Y = pelvis_Y - pref_Y(k);
    
    % 4. Calculate angle based on the new relative distance
    log_pitch(k) = atan2(relative_X, zc); 
    log_roll(k)  = atan2(relative_Y, zc);
end

%% 7. Plotting the Results
figure('Name', 'LIPM Trajectories & Kinematics', 'Position', [50, 100, 1400, 500]);

% --- Plot 1: The X/Y Spatial Path (Top-Down) ---
subplot(1,3,1);
plot(pref_Y(1:steps), pref_X(1:steps), 'ks', 'MarkerSize', 10); hold on;
plot(log_com_Y, log_com_X, 'b-', 'LineWidth', 2);
title('Top-Down View (X-Y Plane)');
xlabel('Lateral Sway - Y (m)');
ylabel('Forward Travel - X (m)');
legend('Foot Placement', 'CoM Trajectory', 'Location', 'northwest');
grid on; axis equal;

% --- Plot 2: The 3D Trajectory (Showing Zc Height) ---
subplot(1,3,2);
% Plot footprints on the floor (Z = 0)
plot3(pref_Y(1:steps), pref_X(1:steps), zeros(1,steps), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); hold on;
% Plot the CoM trajectory at Z = zc
plot3(log_com_Y, log_com_X, zc * ones(1,steps), 'b-', 'LineWidth', 2);
% Draw vertical lines connecting the floor to the CoM (Pendulum visualization)
for i = 1:50:steps
    plot3([log_com_Y(i), log_com_Y(i)], [log_com_X(i), log_com_X(i)], [0, zc], 'k:', 'Color', [0.7 0.7 0.7]);
end
title('3D Walking Physics (LIPM)');
xlabel('Y (m)'); ylabel('X (m)'); zlabel('Z (m) - CoM Height');
zlim([0, zc + 0.2]);
view(45, 25); % Set a nice isometric camera angle
grid on;

% --- Plot 3: The 3-UPR Angles (IK Inputs) ---
subplot(1,3,3);
time = (0:steps-1) * Ts;
plot(time, log_pitch, 'r-', 'LineWidth', 2); hold on;
plot(time, log_roll, 'g-', 'LineWidth', 2);
title('IK Node Command Inputs');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Pitch (Forward Lean)', 'Roll (Side Sway)', 'Location', 'northwest');
grid on;
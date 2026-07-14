% ==========================================
% 3D-LIPM ZMP Preview Control (X-Axis)
% ==========================================

%% 1. System Parameters
zc = 0.6;           % CoM constant height (meters)
g = 9.81;           % Gravity (m/s^2)
Ts = 0.01;          % Control loop sample time (100 Hz)
preview_time = 0.5; % How far ahead to look (seconds)
N = round(preview_time / Ts); % Number of preview steps

% 2. Discrete State-Space Matrices
A = [1, Ts, Ts^2/2; 
     0, 1,  Ts; 
     0, 0,  1];
 
B = [Ts^3/6; 
     Ts^2/2; 
     Ts];
 
C = [1, 0, -zc/g];

%% 3. Augmented System for Tracking Error
% We augment the state with the ZMP tracking error
C_A = C * A;
C_B = C * B;

A_tilde = [1,   C_A; 
           zeros(3,1), A];
       
B_tilde = [C_B; 
           B];
       
I_tilde = [1; 0; 0; 0]; 

% 4. LQR Cost Weights
Q_error = 1e8;      % Penalty on ZMP tracking error (make this high)
R = 1.0;            % Penalty on control effort (jerk)
Q = blkdiag(Q_error, 0, 0, 0);

% 5. Solve Discrete Algebraic Riccati Equation (DARE)
[P, ~, K_tilde] = dare(A_tilde, B_tilde, Q, R);

% Extract Integral Gain (Gi) and State Feedback Gain (Gx)
Gi = K_tilde(1);
Gx = K_tilde(2:4);

% 6. Calculate Preview Feedforward Gains (f)
Ac = A_tilde - B_tilde * K_tilde;
f = zeros(1, N);
for j = 1:N
    % Gain decays as it looks further into the future
    f(j) = (R + B_tilde' * P * B_tilde)^(-1) * B_tilde' * (Ac')^(j-1) * P * I_tilde;
end

% 7. Generate a Dummy ZMP Reference Trajectory (Footsteps)
sim_time = 6; % seconds
steps = sim_time / Ts;
pref = zeros(1, steps + N); 

% Simple step pattern: stance for 1s, step 20cm, stance for 1s, etc.
pref(100:200) = 0.2;
pref(201:300) = 0.4;
pref(301:400) = 0.6;
pref(401:end) = 0.8;

% 8. Simulation Loop
X = [0; 0; 0]; % Initial CoM state [pos; vel; acc]
com_pos = zeros(1, steps);
zmp_actual = zeros(1, steps);
error_sum = 0;

for k = 1:steps
    % Current ZMP
    p = C * X;
    zmp_actual(k) = p;
    com_pos(k) = X(1);
    
    % Tracking Error
    e = p - pref(k);
    error_sum = error_sum + e;
    
    % Calculate Preview Feedforward component
    preview_term = 0;
    for j = 1:N
        preview_term = preview_term + f(j) * pref(k + j);
    end
    
    % Control Law: Jerk command
    u = -Gi * error_sum - Gx * X + preview_term;
    
    % Update State
    X = A * X + B * u;
end

% 9. Plotting Results
time = (0:steps-1) * Ts;
figure;
plot(time, pref(1:steps), 'k--', 'LineWidth', 2); hold on;
plot(time, zmp_actual, 'r', 'LineWidth', 1.5);
plot(time, com_pos, 'b', 'LineWidth', 2);
grid on;
legend('Desired ZMP (Footprints)', 'Actual ZMP', 'CoM Trajectory');
xlabel('Time (s)');
ylabel('Position (m)');
title('Kajita Preview Control: CoM tracking ZMP');
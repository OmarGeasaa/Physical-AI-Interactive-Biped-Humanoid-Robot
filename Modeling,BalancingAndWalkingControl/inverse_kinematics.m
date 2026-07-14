%% 1. Define Robot Geometry (Physical Measurements in meters)

Rb=83.14/1000;
Rp=30/1000;
A = [ Rb,             0             , 0.34; 
      Rb*cos(2*pi/3), Rb*sin(2*pi/3), 0.34;  
      Rb*cos(4*pi/3), Rb*sin(4*pi/3), 0.34]'; 

B = [ Rp,  0, 0.0;
      Rp*cos(2*pi/3),  Rp*sin(2*pi/3), 0.0;
      Rp*cos(4*pi/3), Rp*sin(4*pi/3), 0.0]';

%% 2. The Inverse Kinematics (The "True" State)
true_Z = 0;     
true_Roll = 0;   % Tilt side-to-side (Abduction)
true_Pitch = 0;  % Tilt forward (Flexion)

% Run IK to find what the physical sensor lengths would be
true_lengths = calculate_IK(true_Z, true_Roll, true_Pitch, A, B);
fprintf('Physical Actuator Lengths from Sensors: [%.4f, %.4f, %.4f] m\n\n', true_lengths);

%% 3. The Forward Kinematics (Newton-Raphson Solver)
guess_q = [0; 0.0; 0.0]; % Initial guess [Z; Roll; Pitch]
max_iterations = 20;
tolerance = 1e-5; % Stop when error is less than 0.01 mm

fprintf('--- Starting Newton-Raphson FK Solver ---\n');
for iter = 1:max_iterations
    
    % Step A: Calculate lengths based on our current guess
    guess_lengths = calculate_IK(guess_q(1), guess_q(2), guess_q(3), A, B);
    
    % Step B: Calculate the error (Actual - Guess)
    error_L = true_lengths - guess_lengths;
    
    if norm(error_L) < tolerance
        fprintf('Solver converged in %d iterations!\n', iter);
        break;
    end
    
    % Step C: Calculate the 3x3 Analytical Jacobian for [Z, Roll, Pitch]
    J = calculate_Jacobian(guess_q(1), guess_q(2), guess_q(3), A, B);
    
    % Step D: The Newton-Raphson Update
    delta_q = J \ error_L; 
    guess_q = guess_q + delta_q;
    
    fprintf('Iter %d: Error = %.5f\n', iter, norm(error_L));
end

fprintf('\n--- Final Results ---\n');
fprintf('True Pose:  Z = %.4f, Roll = %.4f, Pitch = %.4f\n', true_Z, true_Roll, true_Pitch);
fprintf('Solved FK:  Z = %.4f, Roll = %.4f, Pitch = %.4f\n', guess_q(1), guess_q(2), guess_q(3));

%% ================= LOCAL FUNCTIONS ================= %%

function L = calculate_IK(z, roll, pitch, A, B)
    P = [0; 0; z];                                               % X and Y are locked to 0
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R = Ry * Rx;                                                 % Ensure multiplication matches your notebook's sequence
    
    L = zeros(3,1);
    for i = 1:3
        L_vec = P + R * B(:,i) - A(:,i);
        L(i) = norm(L_vec);
    end
end

function J = calculate_Jacobian(z, roll, pitch, A, B)
    P = [0; 0; z];
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R = Ry * Rx;
    
    J = zeros(3,3);
    for i = 1:3
        L_vec = P + R * B(:,i) - A(:,i);
        u_i = L_vec / norm(L_vec); 
        r_i = R * B(:,i);
        
        % Full 1x6 spatial Jacobian row: [vx, vy, vz, wx, wy, wz]
        cross_prod = cross(r_i, u_i);
        J_full_row = [u_i', cross_prod'];
        
        % Col 3 = Translation in Z
        % Col 4 = Rotation about X (Roll)
        % Col 5 = Rotation about Y (Pitch)
        J(i, :) = [J_full_row(3), J_full_row(4), J_full_row(5)];
    end
end

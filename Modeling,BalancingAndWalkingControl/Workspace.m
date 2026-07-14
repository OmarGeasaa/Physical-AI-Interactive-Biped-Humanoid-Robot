%% 1. Define Geometry and Hardware Limits
Rb = 83.14/1000;
Rp = 30/1000;
A = [ Rb,             0,              0.34; 
      Rb*cos(2*pi/3), Rb*sin(2*pi/3), 0.34;  
      Rb*cos(4*pi/3), Rb*sin(4*pi/3), 0.34]'; 
B = [ Rp,             0,              0.0;
      Rp*cos(2*pi/3), Rp*sin(2*pi/3), 0.0;
      Rp*cos(4*pi/3), Rp*sin(4*pi/3), 0.0]';

%% Hardware limits
L_MIN = 0.30; % Fully retracted length in meters
L_MAX = 0.40; % Fully extended length in meters

% 2. Define the Search Grid (Resolution dictates speed vs accuracy)
z_vals     = linspace(-0.10, 0.05, 30);  % Test Z from -10cm to +5cm
roll_vals  = linspace(-0.8,  0.8,  30);  % Test Roll from -30 to +30 deg
pitch_vals = linspace(-0.8,  0.8,  30);  % Test Pitch from -30 to +30 deg

% Prepare arrays to hold the valid poses
valid_Z = [];
valid_Roll = [];
valid_Pitch = [];

fprintf('Mapping Workspace...\n');

%% 3. The Grid Search Loop
for z = z_vals
    for r = roll_vals
        for p = pitch_vals
            
            % Use our Phase 0 IK function
            L = calculate_IK(z, r, p, A, B);
            
            % 4. The Safety Check
            % If ALL three actuators are within limits, save the pose
            if (L(1) >= L_MIN && L(1) <= L_MAX) && ...
               (L(2) >= L_MIN && L(2) <= L_MAX) && ...
               (L(3) >= L_MIN && L(3) <= L_MAX)
               
                valid_Z(end+1) = z;
                valid_Roll(end+1) = r;
                valid_Pitch(end+1) = p;
            end
        end
    end
end

%% 5. Visualize the Workspace
figure;
scatter3(valid_Roll, valid_Pitch, valid_Z, 10, valid_Z, 'filled');
xlabel('Roll (rad)');
ylabel('Pitch (rad)');
zlabel('Z Translation (m)');
title('Safe 3-UPR Operational Workspace');
colorbar;
grid on;

fprintf('Found %d safe poses out of %d tested.\n', length(valid_Z), length(z_vals)*length(roll_vals)*length(pitch_vals));

% --- Local Function from Phase 0 ---
function L = calculate_IK(z, roll, pitch, A, B)
    P = [0; 0; z];
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R = Ry * Rx;
    L = zeros(3,1);
    for i = 1:3
        L_vec = P + R * B(:,i) - A(:,i);
        L(i) = norm(L_vec);
    end
end
% 2. Define the Search Grid
% ndgrid generates perfect 3D matrices for our three variables
z_vals     = linspace(-0.15, 0.05, 40);  % Z limits
roll_vals  = linspace(-0.5,  0.5,  40);  % Roll limits
pitch_vals = linspace(-0.5,  0.5,  40);  % Pitch limits

[Roll_grid, Pitch_grid, Z_grid] = ndgrid(roll_vals, pitch_vals, z_vals);

% Pre-allocate the Volume Matrix (0 = Unsafe/Out of bounds)
V = zeros(size(Roll_grid)); 

fprintf('Mapping 3D Workspace Volume...\n');

% 3. The Grid Search Loop
% We use numel to loop through all 64,000 combinations instantly
for i = 1:numel(Roll_grid)
    % Calculate IK for this specific grid coordinate
    L = calculate_IK(Z_grid(i), Roll_grid(i), Pitch_grid(i), A, B);
    
    % 4. The Safety Check
    if (L(1) >= L_MIN && L(1) <= L_MAX) && ...
       (L(2) >= L_MIN && L(2) <= L_MAX) && ...
       (L(3) >= L_MIN && L(3) <= L_MAX)
       
        V(i) = 1; % Mark this specific voxel as Safe
    end
end

% 5. Visualize the Solid 3D Workspace
figure('Color', 'white');

% Isosurface finds the boundary between 0 and 1 (at 0.5) and builds a mesh
[faces, vertices] = isosurface(Roll_grid, Pitch_grid, Z_grid, V, 0.5);

% Patch renders the mesh as a solid object
h = patch('Faces', faces, 'Vertices', vertices);

% Styling for a professional engineering render
h.FaceColor = [0, 0.4470, 0.7410]; % MATLAB blue
h.EdgeColor = 'none';              % Remove wireframe lines
h.FaceAlpha = 0.6;                 % 60% transparency 

% Lighting and Camera
view(3);
grid on;
camlight('headlight'); 
lighting gouraud; % Smooths the shadows across the 3D surface

xlabel('Roll (rad)');
ylabel('Pitch (rad)');
zlabel('Z Translation (m)');
title('3-UPR Kinematic Workspace');
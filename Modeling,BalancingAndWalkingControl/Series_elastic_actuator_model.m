%% ============================================================
%  HUMANOID ROBOT ANKLE SEA DESIGN
%  Steps 1 to 8
%  Graduation Project – Hardware-Oriented Design
% ============================================================

clear; clc; close all;

%% =========================
% STEP 1 — ANKLE KINEMATICS
% =========================

% User-defined ankle range (given)
ankle_angle_deg = 30;                % [deg] ± range
ankle_angle_rad = deg2rad(ankle_angle_deg);  % [rad]

% Total angular excursion
theta_total = 2 * ankle_angle_rad;   % [rad]

% Comment:
% Neutral ankle angle assumed at 0 deg for symmetry and simplicity.
% To calculate neutral angle from gait:
%   -> Need gait kinematics and stance phase analysis.

%% =========================
% STEP 2 — WALKING SPEED
% =========================

% Assumption A2:
% Slow outdoor walking cadence
f_step = 0.25;                         % [Hz]
T_step = 1 / f_step;                  % [s]

% Assumption A3:
% Push-off duration ≈ 20% of step
t_push = 0.20 * T_step;               % [s]

% Angular excursion during push-off
% Assumed 10° dorsiflexion to 20° plantarflexion
theta_push_deg = 30;                  % [deg]
theta_push = deg2rad(theta_push_deg); % [rad]

% Average ankle angular velocity
omega_avg = theta_push / t_push;      % [rad/s]

% Peak angular velocity (sinusoidal motion)
omega_max = omega_avg * (pi/2);       % [rad/s]

% Comment:
% If you want to compute omega from gait:
%   -> Need step length, COM trajectory, and foot contact timing.

%% =========================
% STEP 3 — LOAD & TORQUE
% =========================

% Robot parameters (given)
mass = 35;                            % [kg]
g = 9.81;                             % [m/s^2]

% Weight
W = mass * g;                         % [N]

% Assumption A4:
% Outdoor walking GRF factor
GRF_factor = 1.5;

F_ground = W * GRF_factor;            % [N]

% Foot geometry (given)
foot_length = 0.18;                   % [m]

% Assumption A5:
% Ankle joint at 25%, COP at 75% of foot length
lever_ground = 0.5 * foot_length;     % [m]

% Peak ankle torque
tau_ankle_peak = F_ground * lever_ground; % [Nm]

% Continuous torque assumption (30–60% of peak)
tau_ankle_cont = 0.5 * tau_ankle_peak;   % [Nm]

% Comment:
% For exact GRF and COP:
%   -> Need force plate data or multibody simulation.

%% =========================
% STEP 4 — ANKLE → LINEAR
% =========================

% Assumption A6:
% Ankle lever arm to linkage
r_ankle = 0.045;                       % [m]

% Linear forces
F_rod_peak = tau_ankle_peak / r_ankle; % [N]
F_rod_cont = tau_ankle_cont / r_ankle; % [N]

% Linear displacement due to rotation
x_theta = r_ankle * ankle_angle_rad; % [m]

%% =========================
% STEP 5 — SEA SPRING
% =========================

% Assumption A7:
% Spring stiffness
k_spring = 10.4e3;    % [N/m]

% Number of parallel springs
n = 6;

% Medium stiffness → 30% of stroke allowed as spring deflection
delta_max = F_rod_peak / (n * k_spring); 

% Continuous deflection
delta_cont = F_rod_cont / k_spring;   % [m]

% Stored elastic energy
E_spring = 0.5 * k_spring * delta_max^2; % [J]

% Assumption A8:
% Preload = 20% of continuous force
F_preload = 0.2 * F_rod_cont;         % [N]
delta_preload = F_preload / k_spring; % [m]

% Comment:
% To compute stiffness from impedance requirements:
%   -> Need desired ankle impedance and control bandwidth.

%% =========================
% STEP 6 — BALL SCREW
% =========================

% Required stroke (rotation + spring)
stroke_one_dir = x_theta + delta_max; % [m]

% Total stroke
stroke_total = 2 * stroke_one_dir;    % [m]

% Assumption A9:
% 15% margin for tolerances
stroke_design = 1.15 * stroke_total; % [m]

% Assumption A10:
% Ball screw pitch (balanced)
pitch = 0.004;                        % [m/rev]

% Linear velocity
v_max = r_ankle * omega_max;          % [m/s]

% Screw speed
n_screw = v_max / pitch;              % [rev/s]
rpm_screw = n_screw * 60;             % [RPM]

% Assumption A11:
% Ball screw efficiency
eta_screw = 0.85;

% Screw torque
tau_screw_peak = (F_rod_peak * pitch) / (2*pi*eta_screw); % [Nm]
tau_screw_cont = (F_rod_cont * pitch) / (2*pi*eta_screw); % [Nm]

%% =========================
% STEP 7 — MOTOR & GEARBOX
% =========================

% Assumption A12:
% Balanced gearbox ratio
G = 10;

% Motor speed
rpm_motor = G * rpm_screw;

% Motor torque
tau_motor_peak = tau_screw_peak / G;
tau_motor_cont = tau_screw_cont / G;

% Assumption A13:
% Safety factor
SF = 3;

tau_motor_rated = SF * tau_motor_cont;
tau_motor_peak_req = SF * tau_motor_peak;

% Motor power
omega_motor = rpm_motor * 2*pi/60;    % [rad/s]
P_peak = tau_motor_peak_req * omega_motor;
P_cont = tau_motor_rated * (omega_motor/2);

%% =========================
% STEP 8 — STRUCTURAL CHECKS
% =========================

% Rod parameters
rod_diameter = 0.005;                 % [m]
E_steel = 200e9;                      % [Pa]
L_rod = 0.07;                         % [m]
K = 1;                                % pinned-pinned

I_rod = pi * rod_diameter^4 / 64;
F_cr = (pi^2 * E_steel * I_rod) / (K*L_rod)^2;

sigma_axial = F_rod_peak / (pi*(rod_diameter^2)/4);

%% =========================
% RESULTS SUMMARY
% =========================

fprintf('\n==== DESIGN SUMMARY ====\n');
fprintf('Peak ankle torque        : %.1f Nm\n', tau_ankle_peak);
fprintf('Peak rod force           : %.0f N\n', F_rod_peak);
fprintf('Spring stiffness         : %.1f N/mm\n', k_spring/1000);
fprintf('Ball screw stroke        : %.1f mm\n', stroke_design*1000);
fprintf('Screw max speed          : %.0f RPM\n', rpm_screw);
fprintf('Motor rated torque       : %.3f Nm\n', tau_motor_rated);
fprintf('Motor peak power         : %.1f W\n', P_peak);
fprintf('Rod buckling safety fact : %.1f\n', F_cr / F_rod_peak);



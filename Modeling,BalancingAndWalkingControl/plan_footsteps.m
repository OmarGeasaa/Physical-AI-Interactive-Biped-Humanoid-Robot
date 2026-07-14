function [pref_X, pref_Y] = plan_footsteps(v_x, v_y, T_step, Ts, sim_time, step_width)
    % Inputs:
    % v_x, v_y   : Commanded velocity in m/s
    % T_step     : Duration of one step in seconds
    % Ts         : Control loop sample time (e.g., 0.01s for 100 Hz)
    % sim_time   : Total simulation time in seconds
    % step_width : Physical distance between left and right foot (m)
    
    % Calculate array dimensions
    total_ticks = round(sim_time / Ts);
    ticks_per_step = round(T_step / Ts);
    
    % Initialize reference arrays
    pref_X = zeros(1, total_ticks);
    pref_Y = zeros(1, total_ticks);
    
    % Initial position
    current_x = 0;
    current_y = 0;
    
    % Start with the Left Leg (1 = Left, -1 = Right)
    current_leg = 1; 
    
    for k = 1:ticks_per_step:total_ticks
        % Determine the end index for the current step
        end_idx = min(k + ticks_per_step - 1, total_ticks);
        
        % 1. Calculate forward progression (X)
        delta_x = v_x * T_step;
        current_x = current_x + delta_x;
        
        % 2. Calculate lateral progression (Y)
        % Shift based on velocity, plus the physical stance offset
        delta_y = v_y * T_step;
        current_y = current_y + delta_y;
        stance_y = current_y + (current_leg * (step_width / 2.0));
        
        % 3. Populate the ZMP reference array for this step duration
        pref_X(k:end_idx) = current_x;
        pref_Y(k:end_idx) = stance_y;
        
        % 4. Swap legs for the next step
        current_leg = current_leg * -1;
    end
end
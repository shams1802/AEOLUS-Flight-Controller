%% CodeDroneDIY - Complete MATLAB Implementation
% Single monolithic script replicating the entire C++ drone controller
% Author: Generated from C++ CodeDroneDIY repository
% Date: 2024

function CodeDroneDIY_Complete()
    %% ========================================================================
    %% INITIALIZATION AND PARAMETERS
    %% ========================================================================
    
    % Clear workspace and close figures
    clear all; close all; clc;
    
    % Initialize global parameters (from ControlLoopConstants.h)
    global params;
    params = struct();
    
    % PID Gains (from your C++ code)
    params.anglePos = struct('G', 0.010, 'Kp', 268, 'Kd', 0.5, 'Ki', 0.0);
    params.angleSpeed = struct('G', 0.010, 'Kp', 192, 'Kd', 0.0, 'Ki', 0.0);
    params.accroSpeed = struct('G', 0.010, 'Kp', 192, 'Kd', 0.0, 'Ki', 0.0);
    params.yawSpeed = struct('G', 0.010, 'Kp', 150, 'Kd', 0.0, 'Ki', 0.0);
    
    % Filter Parameters
    params.HighPassFilterCoeff = 0.9995;
    params.mixing = 0.5;
    
    % Limits (from RadioReception.h)
    params.MAX_ANGLE = 45;        % degrees
    params.MAX_ROT_SPEED = 135;   % degrees/second
    params.MAX_YAW_SPEED = 135;   % degrees/second
    
    % Motor Configuration
    params.MOTOR_MIN_POWER = 1000;
    params.MOTOR_MAX_POWER = 2000;
    params.MOTOR_IDLE_THRESHOLD = 20; % Percentage threshold (not PWM)
    
    % Timing
    params.LOOP_TIME_TARGET = 0.0025; % 400Hz (2.5ms like your C++ code)
    params.SAFETY_TIMEOUT = 5.0; % seconds
    
    % State Machine States (from RadioReception.h enum Mode)
    params.states = struct('INITIALIZING', 1, 'SAFETY', 2, 'DISARMED', 3, 'ACCRONODE', 4, 'ANGLEMODE', 5);
    params.currentState = params.states.INITIALIZING;
    
    %% ========================================================================
    %% GLOBAL VARIABLES AND PERSISTENT STORAGE
    %% ========================================================================
    
    % Initialize persistent variables for controllers
    persistent rollPosPID pitchPosPID rollSpeedPID_Angle pitchSpeedPID_Angle;
    persistent rollSpeedPID_Accro pitchSpeedPID_Accro yawPID;
    persistent roll_angle_filt pitch_angle_filt;
    persistent throttle_was_high throttle_low_start_time;
    persistent loop_counter mean_loop_time;
    
    % Initialize PID controllers if first run
    if isempty(rollPosPID)
        rollPosPID = initPID(params.anglePos);
        pitchPosPID = initPID(params.anglePos);
        rollSpeedPID_Angle = initPID(params.angleSpeed);
        pitchSpeedPID_Angle = initPID(params.angleSpeed);
        rollSpeedPID_Accro = initPID(params.accroSpeed);
        pitchSpeedPID_Accro = initPID(params.accroSpeed);
        yawPID = initPID(params.yawSpeed);
        
        % Initialize attitude filter
        roll_angle_filt = 0;
        pitch_angle_filt = 0;
        
        % Initialize safety logic
        throttle_was_high = true;
        throttle_low_start_time = 0;
        
        % Initialize timing
        loop_counter = 0;
        mean_loop_time = 0;
    end
    
    %% ========================================================================
    %% SIMULATION SETUP
    %% ========================================================================
    
    % Simulation parameters
    simulation_time = 30; % seconds
    dt = params.LOOP_TIME_TARGET;
    time_steps = round(simulation_time / dt);
    
    % Pre-allocate data storage
    data = struct();
    data.time = zeros(time_steps, 1);
    data.roll_angle = zeros(time_steps, 1);
    data.pitch_angle = zeros(time_steps, 1);
    data.roll_rate = zeros(time_steps, 1);
    data.pitch_rate = zeros(time_steps, 1);
    data.yaw_rate = zeros(time_steps, 1);
    data.roll_cmd = zeros(time_steps, 1);
    data.pitch_cmd = zeros(time_steps, 1);
    data.throttle = zeros(time_steps, 1);
    data.yaw_cmd = zeros(time_steps, 1);
    data.motor0 = zeros(time_steps, 1);
    data.motor1 = zeros(time_steps, 1);
    data.motor2 = zeros(time_steps, 1);
    data.motor3 = zeros(time_steps, 1);
    data.state = zeros(time_steps, 1);
    data.loop_time = zeros(time_steps, 1);
    
    % Create test scenario (replace with real inputs later)
    test_scenario = createTestScenario(time_steps, dt);
    
    fprintf('Starting CodeDroneDIY simulation...\n');
    fprintf('Simulation time: %.1f seconds\n', simulation_time);
    fprintf('Loop frequency: %.0f Hz\n', 1/dt);
    fprintf('Total iterations: %d\n\n', time_steps);
    
    %% ========================================================================
    %% MAIN SIMULATION LOOP
    %% ========================================================================
    
    tic; % Start timing
    
    for step = 1:time_steps
        loop_start_time = tic;
        
        % Current time
        current_time = (step - 1) * dt;
        data.time(step) = current_time;
        
        %% ========================================================================
        %% INPUT PROCESSING (Radio Reception + IMU)
        %% ========================================================================
        
        % Get radio commands (from test scenario or real inputs)
        [radio_cmds, imu_data] = getInputs(test_scenario, step);
        
        % Process radio inputs (from RadioReception.cpp)
        [roll_cmd, pitch_cmd, throttle, yaw_cmd, mode_switch, safety_switch] = processRadioInputs(radio_cmds);
        
        % Process IMU data (from InertialMeasurementUnit.cpp + Stabilization.cpp)
        [roll_angle, pitch_angle, roll_rate, pitch_rate, yaw_rate] = processIMUData(imu_data, dt, roll_angle_filt, pitch_angle_filt);
        
        % Update filtered angles
        roll_angle_filt = roll_angle;
        pitch_angle_filt = pitch_angle;
        
        % Store data
        data.roll_angle(step) = roll_angle;
        data.pitch_angle(step) = pitch_angle;
        data.roll_rate(step) = roll_rate;
        data.pitch_rate(step) = pitch_rate;
        data.yaw_rate(step) = roll_rate;
        data.roll_cmd(step) = roll_cmd;
        data.pitch_cmd(step) = pitch_cmd;
        data.throttle(step) = throttle;
        data.yaw_cmd(step) = yaw_cmd;
        
        %% ========================================================================
        %% STATE MACHINE (from StateMachine.cpp)
        %% ========================================================================
        
        % Check safety conditions (from StateMachine::IsSafetyStateNeeded)
        is_safety_needed = checkSafetyState(throttle, current_time, throttle_was_high, throttle_low_start_time);
        
        % State machine logic
        [new_state, throttle_was_high, throttle_low_start_time] = runStateMachine(params.currentState, throttle, is_safety_needed, safety_switch, throttle_was_high, throttle_low_start_time, current_time);
        
        % Debug state transitions
        if new_state ~= params.currentState
            fprintf('State transition: %d -> %d at time %.1fs (throttle=%.1f%%, safety=%d)\n', params.currentState, new_state, current_time, throttle, safety_switch);
        end
        
        params.currentState = new_state;
        
        data.state(step) = params.currentState;
        
        %% ========================================================================
        %% CONTROL SYSTEM (from Stabilization.cpp)
        %% ========================================================================
        
        % Initialize motor powers
        roll_motor_power = 0;
        pitch_motor_power = 0;
        yaw_motor_power = 0;
        
        % Control logic based on current state
        switch params.currentState
            case params.states.ANGLEMODE
                % Angle mode - cascaded PID (position + speed)
                [roll_motor_power, pitch_motor_power, yaw_motor_power] = angleModeControl(roll_cmd, pitch_cmd, yaw_cmd, roll_angle, pitch_angle, roll_rate, pitch_rate, yaw_rate, dt);
                
            case params.states.ACCRONODE
                % Accro mode - single PID (speed only)
                [roll_motor_power, pitch_motor_power, yaw_motor_power] = accroModeControl(roll_cmd, pitch_cmd, yaw_cmd, roll_rate, pitch_rate, yaw_rate, dt);
                
            case {params.states.SAFETY, params.states.DISARMED, params.states.INITIALIZING}
                % Safety states - reset all controllers
                [roll_motor_power, pitch_motor_power, yaw_motor_power] = safetyControl();
        end
        
        %% ========================================================================
        %% MOTOR MIXING (from Stabilization::SetMotorsPwrXConfig)
        %% ========================================================================
        
        % X-configuration motor mixing
        [motor0, motor1, motor2, motor3] = motorMixing(throttle, roll_motor_power, pitch_motor_power, yaw_motor_power);
        
        % Store motor outputs
        data.motor0(step) = motor0;
        data.motor1(step) = motor1;
        data.motor2(step) = motor2;
        data.motor3(step) = motor3;
        
        %% ========================================================================
        %% TIMING AND PERFORMANCE MONITORING
        %% ========================================================================
        
        % Calculate actual loop time
        actual_loop_time = toc(loop_start_time);
        data.loop_time(step) = actual_loop_time;
        
        % Compute mean loop time (like in main.cpp)
        if (params.currentState == params.states.ANGLEMODE || params.currentState == params.states.ACCRONODE)
            if throttle > params.MOTOR_IDLE_THRESHOLD
                loop_counter = loop_counter + 1;
                mean_loop_time = ((loop_counter - 1) * mean_loop_time + actual_loop_time) / loop_counter;
            end
        end
        
        % Progress indicator
        if mod(step, round(time_steps/20)) == 0
            progress = step / time_steps * 100;
            fprintf('Progress: %.0f%% (Time: %.1fs, State: %d, Loop: %.3fms)\n', progress, current_time, params.currentState, actual_loop_time*1000);
        end
        
        % Maintain real-time simulation (optional)
        % pause(max(0, dt - actual_loop_time));
    end
    
    %% ========================================================================
    %% SIMULATION COMPLETE - ANALYSIS AND VISUALIZATION
    %% ========================================================================
    
    total_time = toc;
    fprintf('\nSimulation completed!\n');
    fprintf('Total simulation time: %.2f seconds\n', total_time);
    fprintf('Mean loop time: %.3f ms\n', mean_loop_time * 1000);
    fprintf('Target loop time: %.3f ms\n', dt * 1000);
    fprintf('Real-time factor: %.2fx\n', (data.time(end) / total_time));
    
    % Generate plots and analysis
    analyzeResults(data, params);
    
    % Save data
    save('CodeDroneDIY_Results.mat', 'data', 'params');
    fprintf('Results saved to CodeDroneDIY_Results.mat\n');
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================

function pid = initPID(constants)
    % Initialize PID controller structure
    pid = struct();
    pid.G = constants.G;
    pid.Kp = constants.Kp;
    pid.Kd = constants.Kd;
    pid.Ki = constants.Ki;
    pid.integrator = 0;
    pid.prev_error = 0;
end

function [radio_cmds, imu_data] = getInputs(test_scenario, step)
    % Get inputs from test scenario (replace with real sensor data)
    radio_cmds = struct();
    radio_cmds.roll = test_scenario.roll_raw(step);
    radio_cmds.pitch = test_scenario.pitch_raw(step);
    radio_cmds.throttle = test_scenario.throttle_raw(step);
    radio_cmds.yaw = test_scenario.yaw_raw(step);
    radio_cmds.mode = test_scenario.mode_raw(step);
    radio_cmds.safety = test_scenario.safety_raw(step);
    
    imu_data = struct();
    imu_data.acc_x = test_scenario.acc_x(step);
    imu_data.acc_y = test_scenario.acc_y(step);
    imu_data.acc_z = test_scenario.acc_z(step);
    imu_data.gyro_x = test_scenario.gyro_x(step);
    imu_data.gyro_y = test_scenario.gyro_y(step);
    imu_data.gyro_z = test_scenario.gyro_z(step);
end

function [roll_cmd, pitch_cmd, throttle, yaw_cmd, mode_switch, safety_switch] = processRadioInputs(radio_cmds)
    % Process radio inputs (from RadioReception.cpp)
    global params;
    
    % Scale inputs to match your C++ code
    roll_cmd = (radio_cmds.roll - 1500) * params.MAX_ANGLE / 400;      % ±45 degrees
    pitch_cmd = (radio_cmds.pitch - 1500) * params.MAX_ANGLE / 400;    % ±45 degrees
    throttle = (radio_cmds.throttle - 1090) * 100 / 810; % 0-100%
    yaw_cmd = (radio_cmds.yaw - 1500) * params.MAX_YAW_SPEED / 400;       % ±135 deg/s
    
    % Mode selection
    if radio_cmds.mode > 1700
        mode_switch = 1; % Angle mode
    elseif radio_cmds.mode < 1300
        mode_switch = 0; % Accro mode
    else
        mode_switch = 0; % Default to Accro
    end
    
    % Safety switch
    safety_switch = (radio_cmds.safety > 1500);
    
    % Apply limits
    roll_cmd = max(-params.MAX_ANGLE, min(params.MAX_ANGLE, roll_cmd));
    pitch_cmd = max(-params.MAX_ANGLE, min(params.MAX_ANGLE, pitch_cmd));
    yaw_cmd = max(-params.MAX_YAW_SPEED, min(params.MAX_YAW_SPEED, yaw_cmd));
    throttle = max(0, min(100, throttle));
end

function [roll_angle, pitch_angle, roll_rate, pitch_rate, yaw_rate] = processIMUData(imu_data, dt, prev_roll, prev_pitch)
    % Process IMU data with complementary filter (from Stabilization.cpp)
    global params;
    
    % Convert gyro to degrees/second
    roll_rate = imu_data.gyro_x;
    pitch_rate = imu_data.gyro_y;
    yaw_rate = imu_data.gyro_z;
    
    % Normalize accelerometer
    acc_mag = sqrt(imu_data.acc_x^2 + imu_data.acc_y^2 + imu_data.acc_z^2);
    acc_x_norm = imu_data.acc_x / acc_mag;
    acc_y_norm = imu_data.acc_y / acc_mag;
    acc_z_norm = imu_data.acc_z / acc_mag;
    
    % Calculate angles from accelerometer
    roll_angle_acc = atan2(acc_y_norm, acc_z_norm) * 180/pi;
    pitch_angle_acc = atan2(-acc_x_norm, acc_z_norm) * 180/pi;
    
    % Complementary filter (from Stabilization::ApplyComplementaryFilter)
    roll_angle = params.HighPassFilterCoeff * (prev_roll + roll_rate * dt) + (1 - params.HighPassFilterCoeff) * roll_angle_acc;
    pitch_angle = params.HighPassFilterCoeff * (prev_pitch + pitch_rate * dt) + (1 - params.HighPassFilterCoeff) * pitch_angle_acc;
end

function is_safety_needed = checkSafetyState(throttle, current_time, throttle_was_high, throttle_low_start_time)
    % Safety state logic (from StateMachine::IsSafetyStateNeeded)
    global params;
    
    if throttle_was_high && throttle < params.MOTOR_IDLE_THRESHOLD
        is_safety_needed = false;
    elseif ~throttle_was_high && (current_time - throttle_low_start_time > params.SAFETY_TIMEOUT)
        is_safety_needed = true;
    else
        is_safety_needed = false;
    end
end

function [new_state, throttle_was_high_out, throttle_low_start_time_out] = runStateMachine(current_state, throttle, is_safety_needed, safety_switch, throttle_was_high, throttle_low_start_time, current_time)
    % State machine logic (from StateMachine.cpp and state implementations)
    global params;
    
    new_state = current_state;
    throttle_was_high_out = throttle_was_high;
    throttle_low_start_time_out = throttle_low_start_time;
    
    switch current_state
        case params.states.INITIALIZING
            % Transition to Disarmed when IMU is calibrated (from Initializing.cpp)
            % In the C++ code, this transitions to Disarmed, then to AngleMode when throttle high
            new_state = params.states.DISARMED;
            
        case params.states.ANGLEMODE
            if is_safety_needed
                new_state = params.states.SAFETY;
            end
            
        case params.states.ACCRONODE
            if is_safety_needed
                new_state = params.states.SAFETY;
            end
            
        case params.states.SAFETY
            % Automatic transition to disarmed after 1 second
            new_state = params.states.DISARMED;
            
        case params.states.DISARMED
            % Transition back to armed when throttle high and safety switch on
            if throttle > params.MOTOR_IDLE_THRESHOLD && safety_switch
                new_state = params.states.ANGLEMODE;
            end
    end
    
    % Update throttle tracking
    if throttle_was_high && throttle < params.MOTOR_IDLE_THRESHOLD
        throttle_was_high_out = false;
        throttle_low_start_time_out = current_time;
    elseif throttle > params.MOTOR_IDLE_THRESHOLD
        throttle_was_high_out = true;
    end
end

function [roll_power, pitch_power, yaw_power] = angleModeControl(roll_cmd, pitch_cmd, yaw_cmd, roll_angle, pitch_angle, roll_rate, pitch_rate, yaw_rate, dt)
    % Angle mode control - cascaded PID (from Stabilization::Angle)
    persistent rollPosPID pitchPosPID rollSpeedPID_Angle pitchSpeedPID_Angle yawPID;
    global params;
    
    if isempty(rollPosPID)
        rollPosPID = initPID(params.anglePos);
        pitchPosPID = initPID(params.anglePos);
        rollSpeedPID_Angle = initPID(params.angleSpeed);
        pitchSpeedPID_Angle = initPID(params.angleSpeed);
        yawPID = initPID(params.yawSpeed);
    end
    
    % Roll control - cascaded PID
    roll_pos_cmd = computePID(rollPosPID, roll_cmd, roll_angle, dt);
    roll_power = computePID(rollSpeedPID_Angle, roll_pos_cmd, roll_rate, dt);
    
    % Pitch control - cascaded PID
    pitch_pos_cmd = computePID(pitchPosPID, pitch_cmd, pitch_angle, dt);
    pitch_power = computePID(pitchSpeedPID_Angle, pitch_pos_cmd, pitch_rate, dt);
    
    % Yaw control - single PID (always rate control)
    yaw_power = computePID(yawPID, yaw_cmd, yaw_rate, dt);
end

function [roll_power, pitch_power, yaw_power] = accroModeControl(roll_cmd, pitch_cmd, yaw_cmd, roll_rate, pitch_rate, yaw_rate, dt)
    % Accro mode control - single PID (from Stabilization::Accro)
    persistent rollSpeedPID_Accro pitchSpeedPID_Accro yawPID;
    global params;
    
    if isempty(rollSpeedPID_Accro)
        rollSpeedPID_Accro = initPID(params.accroSpeed);
        pitchSpeedPID_Accro = initPID(params.accroSpeed);
        yawPID = initPID(params.yawSpeed);
    end
    
    % Direct rate control
    roll_power = computePID(rollSpeedPID_Accro, roll_cmd, roll_rate, dt);
    pitch_power = computePID(pitchSpeedPID_Accro, pitch_cmd, pitch_rate, dt);
    yaw_power = computePID(yawPID, yaw_cmd, yaw_rate, dt);
end

function [roll_power, pitch_power, yaw_power] = safetyControl()
    % Safety control - all motors idle (from Stabilization::Idle)
    roll_power = 0;
    pitch_power = 0;
    yaw_power = 0;
    
    % Reset all PID controllers
    resetAllPIDs();
end

function output = computePID(pid, command, feedback, dt)
    % Generic PID computation (from ControlLoop::ComputeCorrection)
    error = command - feedback;
    pid.integrator = pid.integrator + error;
    
    output = pid.G * (pid.Kp * error + pid.Kd * (error - pid.prev_error) / dt + pid.Ki * pid.integrator);
    
    pid.prev_error = error;
end

function [motor0, motor1, motor2, motor3] = motorMixing(throttle, roll_power, pitch_power, yaw_power)
    % X-configuration motor mixing (from Stabilization::SetMotorsPwrXConfig)
    global params;
    
    % Convert throttle percentage to PWM
    throttle_pwm = params.MOTOR_MIN_POWER + (throttle / 100) * (params.MOTOR_MAX_POWER - params.MOTOR_MIN_POWER);
    
    % X configuration mixing
    motor0 = throttle_pwm - pitch_power * params.mixing + roll_power * params.mixing - yaw_power * params.mixing;
    motor1 = throttle_pwm - pitch_power * params.mixing - roll_power * params.mixing + yaw_power * params.mixing;
    motor2 = throttle_pwm + pitch_power * params.mixing - roll_power * params.mixing - yaw_power * params.mixing;
    motor3 = throttle_pwm + pitch_power * params.mixing + roll_power * params.mixing + yaw_power * params.mixing;
    
    % Apply PWM limits
    motor0 = max(params.MOTOR_MIN_POWER, min(params.MOTOR_MAX_POWER, motor0));
    motor1 = max(params.MOTOR_MIN_POWER, min(params.MOTOR_MAX_POWER, motor1));
    motor2 = max(params.MOTOR_MIN_POWER, min(params.MOTOR_MAX_POWER, motor2));
    motor3 = max(params.MOTOR_MIN_POWER, min(params.MOTOR_MAX_POWER, motor3));
end

function resetAllPIDs()
    % Reset all PID controllers (from Stabilization::ResetPID)
    persistent rollPosPID pitchPosPID rollSpeedPID_Angle pitchSpeedPID_Angle;
    persistent rollSpeedPID_Accro pitchSpeedPID_Accro yawPID;
    
    if ~isempty(rollPosPID)
        rollPosPID.integrator = 0;
        rollPosPID.prev_error = 0;
    end
    if ~isempty(pitchPosPID)
        pitchPosPID.integrator = 0;
        pitchPosPID.prev_error = 0;
    end
    if ~isempty(rollSpeedPID_Angle)
        rollSpeedPID_Angle.integrator = 0;
        rollSpeedPID_Angle.prev_error = 0;
    end
    if ~isempty(pitchSpeedPID_Angle)
        pitchSpeedPID_Angle.integrator = 0;
        pitchSpeedPID_Angle.prev_error = 0;
    end
    if ~isempty(rollSpeedPID_Accro)
        rollSpeedPID_Accro.integrator = 0;
        rollSpeedPID_Accro.prev_error = 0;
    end
    if ~isempty(pitchSpeedPID_Accro)
        pitchSpeedPID_Accro.integrator = 0;
        pitchSpeedPID_Accro.prev_error = 0;
    end
    if ~isempty(yawPID)
        yawPID.integrator = 0;
        yawPID.prev_error = 0;
    end
end

function test_scenario = createTestScenario(time_steps, dt)
    % Create test scenario with realistic inputs
    time = (0:time_steps-1) * dt;
    
    % Radio commands
    test_scenario.roll_raw = 1500 + 200 * sin(2*pi*0.1*time);     % Slow roll oscillation
    test_scenario.pitch_raw = 1500 + 150 * sin(2*pi*0.15*time);   % Pitch oscillation
    test_scenario.throttle_raw = 1090 + 0 * (time < 5) + 200 * (time >= 5 & time < 15) + 400 * (time >= 15); % Step throttle: 1090->1090->1290->1490
    test_scenario.yaw_raw = 1500 + 100 * sin(2*pi*0.05*time);     % Slow yaw
    test_scenario.mode_raw = 1900 * ones(size(time));             % Angle mode
    test_scenario.safety_raw = 1900 * ones(size(time));           % Armed
    
    % IMU data (simulated)
    test_scenario.acc_x = 0.1 * sin(2*pi*0.1*time);              % Small accelerations
    test_scenario.acc_y = 0.15 * sin(2*pi*0.15*time);
    test_scenario.acc_z = 9.81 + 0.05 * sin(2*pi*0.05*time);     % Gravity + noise
    
    test_scenario.gyro_x = 2 * sin(2*pi*0.1*time);               % Gyro rates
    test_scenario.gyro_y = 1.5 * sin(2*pi*0.15*time);
    test_scenario.gyro_z = 0.5 * sin(2*pi*0.05*time);
end

function analyzeResults(data, params)
    % Analyze and plot simulation results
    figure('Name', 'CodeDroneDIY Simulation Results', 'Position', [100, 100, 1200, 800]);
    
    % Time vector
    time = data.time;
    
    % Plot 1: Attitude
    subplot(3, 3, 1);
    plot(time, data.roll_angle, 'r-', time, data.pitch_angle, 'b-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Angle (deg)');
    title('Attitude Angles'); legend('Roll', 'Pitch'); grid on;
    
    % Plot 2: Angular Rates
    subplot(3, 3, 2);
    plot(time, data.roll_rate, 'r-', time, data.pitch_rate, 'b-', time, data.yaw_rate, 'g-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Rate (deg/s)');
    title('Angular Rates'); legend('Roll', 'Pitch', 'Yaw'); grid on;
    
    % Plot 3: Commands
    subplot(3, 3, 3);
    plot(time, data.roll_cmd, 'r-', time, data.pitch_cmd, 'b-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Command (deg)');
    title('Control Commands'); legend('Roll Cmd', 'Pitch Cmd'); grid on;
    
    % Plot 4: Throttle
    subplot(3, 3, 4);
    plot(time, data.throttle, 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Throttle (%)');
    title('Throttle'); grid on;
    
    % Plot 5: Motor Outputs
    subplot(3, 3, 5);
    plot(time, data.motor0, 'r-', time, data.motor1, 'g-', time, data.motor2, 'b-', time, data.motor3, 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('PWM (us)');
    title('Motor Outputs'); legend('Motor0', 'Motor1', 'Motor2', 'Motor3'); grid on;
    
    % Plot 6: State Machine
    subplot(3, 3, 6);
    plot(time, data.state, 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('State');
    title('State Machine'); ylim([0, 6]); grid on;
    
    % Plot 7: Loop Timing
    subplot(3, 3, 7);
    plot(time, data.loop_time * 1000, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, params.LOOP_TIME_TARGET * 1000 * ones(size(time)), 'r--', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Loop Time (ms)');
    title('Loop Timing'); legend('Actual', 'Target'); grid on;
    
    % Plot 8: Control Performance
    subplot(3, 3, 8);
    plot(time, data.roll_cmd - data.roll_angle, 'r-', time, data.pitch_cmd - data.pitch_angle, 'b-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Error (deg)');
    title('Control Errors'); legend('Roll Error', 'Pitch Error'); grid on;
    
    % Plot 9: Power Analysis
    subplot(3, 3, 9);
    total_power = data.motor0 + data.motor1 + data.motor2 + data.motor3;
    plot(time, total_power, 'k-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Total Power (us)');
    title('Total Motor Power'); grid on;
    
    % Print performance summary
    fprintf('\n=== PERFORMANCE ANALYSIS ===\n');
    fprintf('RMS Roll Error: %.2f degrees\n', rms(data.roll_cmd - data.roll_angle));
    fprintf('RMS Pitch Error: %.2f degrees\n', rms(data.pitch_cmd - data.pitch_angle));
    fprintf('Mean Loop Time: %.3f ms (Target: %.3f ms)\n', mean(data.loop_time)*1000, params.LOOP_TIME_TARGET*1000);
    fprintf('Loop Time Std: %.3f ms\n', std(data.loop_time)*1000);
    fprintf('Max Loop Time: %.3f ms\n', max(data.loop_time)*1000);
end
%% Drone Simulink Block Functions
% This file contains all the MATLAB Function block implementations
% for the CodeDroneDIY Simulink model
% Author: Auto-generated for CodeDroneDIY Simulink conversion

%% ========================================================================
%% RADIO INPUT PROCESSING BLOCK
%% ========================================================================

function [processed_cmds] = RadioInputProcessor(radio_cmds)
    % MATLAB Function Block: Radio Input Processing
    % Converts raw radio PWM signals to engineering units
    
    %#codegen
    
    % Constants (should match original MATLAB code)
    MAX_ANGLE = 45;        % degrees
    MAX_YAW_SPEED = 135;   % degrees/second
    
    % Initialize output structure
    processed_cmds = struct();
    
    % Scale inputs to engineering units
    processed_cmds.roll_cmd = (radio_cmds.roll - 1500) * MAX_ANGLE / 400;      % ±45 degrees
    processed_cmds.pitch_cmd = (radio_cmds.pitch - 1500) * MAX_ANGLE / 400;    % ±45 degrees
    processed_cmds.throttle = (radio_cmds.throttle - 1090) * 100 / 810;        % 0-100%
    processed_cmds.yaw_cmd = (radio_cmds.yaw - 1500) * MAX_YAW_SPEED / 400;    % ±135 deg/s
    
    % Mode selection
    if radio_cmds.mode > 1700
        processed_cmds.mode_switch = true;  % Angle mode
    elseif radio_cmds.mode < 1300
        processed_cmds.mode_switch = false; % Accro mode
    else
        processed_cmds.mode_switch = false; % Default to Accro
    end
    
    % Safety switch
    processed_cmds.safety_switch = (radio_cmds.safety > 1500);
    
    % Apply limits
    processed_cmds.roll_cmd = max(-MAX_ANGLE, min(MAX_ANGLE, processed_cmds.roll_cmd));
    processed_cmds.pitch_cmd = max(-MAX_ANGLE, min(MAX_ANGLE, processed_cmds.pitch_cmd));
    processed_cmds.yaw_cmd = max(-MAX_YAW_SPEED, min(MAX_YAW_SPEED, processed_cmds.yaw_cmd));
    processed_cmds.throttle = max(0, min(100, processed_cmds.throttle));
end

%% ========================================================================
%% IMU DATA PROCESSING BLOCK
%% ========================================================================

function [attitude, attitude_prev] = IMUProcessor(imu_data, attitude_prev, dt)
    % MATLAB Function Block: IMU Data Processing
    % Processes raw IMU data using complementary filter
    
    %#codegen
    
    % Constants
    HighPassFilterCoeff = 0.9995;
    
    % Initialize output structure
    attitude = struct();
    
    % Convert gyro to degrees/second (assuming already in correct units)
    attitude.roll_rate = imu_data.gyro_x;
    attitude.pitch_rate = imu_data.gyro_y;
    attitude.yaw_rate = imu_data.gyro_z;
    
    % Normalize accelerometer
    acc_mag = sqrt(imu_data.acc_x^2 + imu_data.acc_y^2 + imu_data.acc_z^2);
    
    % Avoid division by zero
    if acc_mag < 1e-6
        acc_mag = 1;
    end
    
    acc_x_norm = imu_data.acc_x / acc_mag;
    acc_y_norm = imu_data.acc_y / acc_mag;
    acc_z_norm = imu_data.acc_z / acc_mag;
    
    % Calculate angles from accelerometer
    roll_angle_acc = atan2(acc_y_norm, acc_z_norm) * 180/pi;
    pitch_angle_acc = atan2(-acc_x_norm, acc_z_norm) * 180/pi;
    
    % Complementary filter
    attitude.roll_angle = HighPassFilterCoeff * (attitude_prev.roll_angle + attitude.roll_rate * dt) + ...
                         (1 - HighPassFilterCoeff) * roll_angle_acc;
    attitude.pitch_angle = HighPassFilterCoeff * (attitude_prev.pitch_angle + attitude.pitch_rate * dt) + ...
                          (1 - HighPassFilterCoeff) * pitch_angle_acc;
end

%% ========================================================================
%% SAFETY STATE CHECKER BLOCK
%% ========================================================================

function [is_safety_needed, throttle_was_high_out, throttle_low_start_time_out] = SafetyStateChecker(throttle, current_time, throttle_was_high, throttle_low_start_time)
    % MATLAB Function Block: Safety State Checker
    % Determines if safety state is needed based on throttle behavior
    
    %#codegen
    
    % Constants
    MOTOR_IDLE_THRESHOLD = 20;  % Percentage threshold
    SAFETY_TIMEOUT = 5.0;       % seconds
    
    % Pass through the tracking variables
    throttle_was_high_out = throttle_was_high;
    throttle_low_start_time_out = throttle_low_start_time;
    
    % Safety logic
    if throttle_was_high && throttle < MOTOR_IDLE_THRESHOLD
        is_safety_needed = false;
    elseif ~throttle_was_high && (current_time - throttle_low_start_time > SAFETY_TIMEOUT)
        is_safety_needed = true;
    else
        is_safety_needed = false;
    end
end

%% ========================================================================
%% ANGLE MODE CONTROLLER BLOCK
%% ========================================================================

function [control_powers, pid_states_out] = AngleModeController(processed_cmds, attitude, pid_states_in, dt)
    % MATLAB Function Block: Angle Mode Controller
    % Cascaded PID control for angle mode
    
    %#codegen
    
    % PID Gains (from original code)
    anglePos_G = 0.010;
    anglePos_Kp = 268;
    anglePos_Kd = 0.5;
    anglePos_Ki = 0.0;
    
    angleSpeed_G = 0.010;
    angleSpeed_Kp = 192;
    angleSpeed_Kd = 0.0;
    angleSpeed_Ki = 0.0;
    
    yawSpeed_G = 0.010;
    yawSpeed_Kp = 150;
    yawSpeed_Kd = 0.0;
    yawSpeed_Ki = 0.0;
    
    % Initialize output structures
    control_powers = struct();
    pid_states_out = pid_states_in;  % Pass through and update
    
    % Roll control - cascaded PID
    % Position loop
    roll_pos_error = processed_cmds.roll_cmd - attitude.roll_angle;
    pid_states_out.roll_pos_integrator = pid_states_out.roll_pos_integrator + roll_pos_error;
    roll_pos_cmd = anglePos_G * (anglePos_Kp * roll_pos_error + ...
                                anglePos_Kd * (roll_pos_error - pid_states_out.roll_pos_prev_error) / dt + ...
                                anglePos_Ki * pid_states_out.roll_pos_integrator);
    pid_states_out.roll_pos_prev_error = roll_pos_error;
    
    % Speed loop
    roll_speed_error = roll_pos_cmd - attitude.roll_rate;
    pid_states_out.roll_speed_angle_integrator = pid_states_out.roll_speed_angle_integrator + roll_speed_error;
    control_powers.roll_power = angleSpeed_G * (angleSpeed_Kp * roll_speed_error + ...
                                               angleSpeed_Kd * (roll_speed_error - pid_states_out.roll_speed_angle_prev_error) / dt + ...
                                               angleSpeed_Ki * pid_states_out.roll_speed_angle_integrator);
    pid_states_out.roll_speed_angle_prev_error = roll_speed_error;
    
    % Pitch control - cascaded PID
    % Position loop
    pitch_pos_error = processed_cmds.pitch_cmd - attitude.pitch_angle;
    pid_states_out.pitch_pos_integrator = pid_states_out.pitch_pos_integrator + pitch_pos_error;
    pitch_pos_cmd = anglePos_G * (anglePos_Kp * pitch_pos_error + ...
                                 anglePos_Kd * (pitch_pos_error - pid_states_out.pitch_pos_prev_error) / dt + ...
                                 anglePos_Ki * pid_states_out.pitch_pos_integrator);
    pid_states_out.pitch_pos_prev_error = pitch_pos_error;
    
    % Speed loop
    pitch_speed_error = pitch_pos_cmd - attitude.pitch_rate;
    pid_states_out.pitch_speed_angle_integrator = pid_states_out.pitch_speed_angle_integrator + pitch_speed_error;
    control_powers.pitch_power = angleSpeed_G * (angleSpeed_Kp * pitch_speed_error + ...
                                                angleSpeed_Kd * (pitch_speed_error - pid_states_out.pitch_speed_angle_prev_error) / dt + ...
                                                angleSpeed_Ki * pid_states_out.pitch_speed_angle_integrator);
    pid_states_out.pitch_speed_angle_prev_error = pitch_speed_error;
    
    % Yaw control - single PID (always rate control)
    yaw_error = processed_cmds.yaw_cmd - attitude.yaw_rate;
    pid_states_out.yaw_integrator = pid_states_out.yaw_integrator + yaw_error;
    control_powers.yaw_power = yawSpeed_G * (yawSpeed_Kp * yaw_error + ...
                                            yawSpeed_Kd * (yaw_error - pid_states_out.yaw_prev_error) / dt + ...
                                            yawSpeed_Ki * pid_states_out.yaw_integrator);
    pid_states_out.yaw_prev_error = yaw_error;
end

%% ========================================================================
%% ACCRO MODE CONTROLLER BLOCK
%% ========================================================================

function [control_powers, pid_states_out] = AccroModeController(processed_cmds, attitude, pid_states_in, dt)
    % MATLAB Function Block: Accro Mode Controller
    % Single PID control for rate mode
    
    %#codegen
    
    % PID Gains
    accroSpeed_G = 0.010;
    accroSpeed_Kp = 192;
    accroSpeed_Kd = 0.0;
    accroSpeed_Ki = 0.0;
    
    yawSpeed_G = 0.010;
    yawSpeed_Kp = 150;
    yawSpeed_Kd = 0.0;
    yawSpeed_Ki = 0.0;
    
    % Initialize output structures
    control_powers = struct();
    pid_states_out = pid_states_in;
    
    % Direct rate control
    roll_error = processed_cmds.roll_cmd - attitude.roll_rate;
    pid_states_out.roll_speed_accro_integrator = pid_states_out.roll_speed_accro_integrator + roll_error;
    control_powers.roll_power = accroSpeed_G * (accroSpeed_Kp * roll_error + ...
                                               accroSpeed_Kd * (roll_error - pid_states_out.roll_speed_accro_prev_error) / dt + ...
                                               accroSpeed_Ki * pid_states_out.roll_speed_accro_integrator);
    pid_states_out.roll_speed_accro_prev_error = roll_error;
    
    pitch_error = processed_cmds.pitch_cmd - attitude.pitch_rate;
    pid_states_out.pitch_speed_accro_integrator = pid_states_out.pitch_speed_accro_integrator + pitch_error;
    control_powers.pitch_power = accroSpeed_G * (accroSpeed_Kp * pitch_error + ...
                                                accroSpeed_Kd * (pitch_error - pid_states_out.pitch_speed_accro_prev_error) / dt + ...
                                                accroSpeed_Ki * pid_states_out.pitch_speed_accro_integrator);
    pid_states_out.pitch_speed_accro_prev_error = pitch_error;
    
    yaw_error = processed_cmds.yaw_cmd - attitude.yaw_rate;
    pid_states_out.yaw_integrator = pid_states_out.yaw_integrator + yaw_error;
    control_powers.yaw_power = yawSpeed_G * (yawSpeed_Kp * yaw_error + ...
                                            yawSpeed_Kd * (yaw_error - pid_states_out.yaw_prev_error) / dt + ...
                                            yawSpeed_Ki * pid_states_out.yaw_integrator);
    pid_states_out.yaw_prev_error = yaw_error;
end

%% ========================================================================
%% SAFETY CONTROLLER BLOCK
%% ========================================================================

function [control_powers, pid_states_out] = SafetyController(pid_states_in)
    % MATLAB Function Block: Safety Controller
    % Sets all control powers to zero and resets PID states
    
    %#codegen
    
    % Initialize output structures
    control_powers = struct();
    control_powers.roll_power = 0;
    control_powers.pitch_power = 0;
    control_powers.yaw_power = 0;
    
    % Reset all PID states
    pid_states_out = pid_states_in;
    pid_states_out.roll_pos_integrator = 0;
    pid_states_out.roll_pos_prev_error = 0;
    pid_states_out.pitch_pos_integrator = 0;
    pid_states_out.pitch_pos_prev_error = 0;
    pid_states_out.roll_speed_angle_integrator = 0;
    pid_states_out.roll_speed_angle_prev_error = 0;
    pid_states_out.pitch_speed_angle_integrator = 0;
    pid_states_out.pitch_speed_angle_prev_error = 0;
    pid_states_out.roll_speed_accro_integrator = 0;
    pid_states_out.roll_speed_accro_prev_error = 0;
    pid_states_out.pitch_speed_accro_integrator = 0;
    pid_states_out.pitch_speed_accro_prev_error = 0;
    pid_states_out.yaw_integrator = 0;
    pid_states_out.yaw_prev_error = 0;
end

%% ========================================================================
%% MOTOR MIXING BLOCK
%% ========================================================================

function [motor_cmds] = MotorMixer(throttle, control_powers)
    % MATLAB Function Block: Motor Mixing
    % X-configuration motor mixing
    
    %#codegen
    
    % Constants
    MOTOR_MIN_POWER = 1000;  % us
    MOTOR_MAX_POWER = 2000;  % us
    mixing = 0.5;
    
    % Initialize output structure
    motor_cmds = struct();
    
    % Convert throttle percentage to PWM
    throttle_pwm = MOTOR_MIN_POWER + (throttle / 100) * (MOTOR_MAX_POWER - MOTOR_MIN_POWER);
    
    % X configuration mixing
    motor_cmds.motor0 = throttle_pwm - control_powers.pitch_power * mixing + control_powers.roll_power * mixing - control_powers.yaw_power * mixing;
    motor_cmds.motor1 = throttle_pwm - control_powers.pitch_power * mixing - control_powers.roll_power * mixing + control_powers.yaw_power * mixing;
    motor_cmds.motor2 = throttle_pwm + control_powers.pitch_power * mixing - control_powers.roll_power * mixing - control_powers.yaw_power * mixing;
    motor_cmds.motor3 = throttle_pwm + control_powers.pitch_power * mixing + control_powers.roll_power * mixing + control_powers.yaw_power * mixing;
    
    % Apply PWM limits
    motor_cmds.motor0 = max(MOTOR_MIN_POWER, min(MOTOR_MAX_POWER, motor_cmds.motor0));
    motor_cmds.motor1 = max(MOTOR_MIN_POWER, min(MOTOR_MAX_POWER, motor_cmds.motor1));
    motor_cmds.motor2 = max(MOTOR_MIN_POWER, min(MOTOR_MAX_POWER, motor_cmds.motor2));
    motor_cmds.motor3 = max(MOTOR_MIN_POWER, min(MOTOR_MAX_POWER, motor_cmds.motor3));
end

%% ========================================================================
%% TEST SCENARIO GENERATOR BLOCK
%% ========================================================================

function [radio_cmds, imu_data] = TestScenarioGenerator(current_time)
    % MATLAB Function Block: Test Scenario Generator
    % Generates test inputs for simulation
    
    %#codegen
    
    % Initialize output structures
    radio_cmds = struct();
    imu_data = struct();
    
    % Radio commands (sinusoidal test signals)
    radio_cmds.roll = 1500 + 200 * sin(2*pi*0.1*current_time);     % Slow roll oscillation
    radio_cmds.pitch = 1500 + 150 * sin(2*pi*0.15*current_time);   % Pitch oscillation
    
    % Throttle profile: idle -> low -> medium -> high
    if current_time < 5
        radio_cmds.throttle = 1090;  % Idle
    elseif current_time < 15
        radio_cmds.throttle = 1290;  % Low throttle
    else
        radio_cmds.throttle = 1490;  % Medium throttle
    end
    
    radio_cmds.yaw = 1500 + 100 * sin(2*pi*0.05*current_time);     % Slow yaw
    radio_cmds.mode = 1900;                                         % Angle mode
    radio_cmds.safety = 1900;                                       % Armed
    
    % IMU data (simulated)
    imu_data.acc_x = 0.1 * sin(2*pi*0.1*current_time);              % Small accelerations
    imu_data.acc_y = 0.15 * sin(2*pi*0.15*current_time);
    imu_data.acc_z = 9.81 + 0.05 * sin(2*pi*0.05*current_time);     % Gravity + noise
    
    imu_data.gyro_x = 2 * sin(2*pi*0.1*current_time);               % Gyro rates
    imu_data.gyro_y = 1.5 * sin(2*pi*0.15*current_time);
    imu_data.gyro_z = 0.5 * sin(2*pi*0.05*current_time);
end

%% ========================================================================
%% THROTTLE TRACKER BLOCK
%% ========================================================================

function [throttle_was_high_out, throttle_low_start_time_out] = ThrottleTracker(throttle, current_time, throttle_was_high_in, throttle_low_start_time_in)
    % MATLAB Function Block: Throttle Tracker
    % Tracks throttle state changes for safety logic
    
    %#codegen
    
    % Constants
    MOTOR_IDLE_THRESHOLD = 20;  % Percentage threshold
    
    % Initialize outputs
    throttle_was_high_out = throttle_was_high_in;
    throttle_low_start_time_out = throttle_low_start_time_in;
    
    % Update throttle tracking
    if throttle_was_high_in && throttle < MOTOR_IDLE_THRESHOLD
        throttle_was_high_out = false;
        throttle_low_start_time_out = current_time;
    elseif throttle > MOTOR_IDLE_THRESHOLD
        throttle_was_high_out = true;
    end
end

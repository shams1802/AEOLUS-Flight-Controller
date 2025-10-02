% initialize_drone_parameters.m

% Clear workspace to ensure a clean start
clear; clc;

% Create a structure 'params' to hold all configuration data
params = struct();

% PID Gains
params.anglePos   = struct('G', 0.010, 'Kp', 268, 'Kd', 0.5, 'Ki', 0.0);
params.angleSpeed = struct('G', 0.010, 'Kp', 192, 'Kd', 0.0, 'Ki', 0.0);
params.accroSpeed = struct('G', 0.010, 'Kp', 192, 'Kd', 0.0, 'Ki', 0.0);
params.yawSpeed   = struct('G', 0.010, 'Kp', 150, 'Kd', 0.0, 'Ki', 0.0);

% Filter Parameters
params.HighPassFilterCoeff = 0.9995;
params.mixing = 0.5;

% Limits
params.MAX_ANGLE      = 45;   % degrees
params.MAX_ROT_SPEED  = 135;  % degrees/second
params.MAX_YAW_SPEED  = 135;  % degrees/second

% Motor Configuration
params.MOTOR_MIN_POWER = 1000;
params.MOTOR_MAX_POWER = 2000;
params.MOTOR_IDLE_THRESHOLD = 20; % Percentage

% Timing
params.LOOP_TIME_TARGET = 0.0025; % 400Hz

% State Machine States
params.states = struct('INITIALIZING', 1, 'SAFETY', 2, 'DISARMED', 3, 'ACCRONODE', 4, 'ANGLEMODE', 5);

fprintf('Drone parameters loaded into workspace.\n');
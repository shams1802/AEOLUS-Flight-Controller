%% Bus Definitions for CodeDroneDIY Simulink Model
% This script defines all the bus objects needed for the drone control system
% Author: Auto-generated for CodeDroneDIY Simulink conversion
% Date: 2024

function bus_definitions()
    % Define all bus objects for the drone control system
    
    %% Clear any existing bus definitions
    clear_bus_definitions();
    
    %% IMU Bus Definition
    IMU_Bus = Simulink.Bus;
    IMU_Bus.Description = 'Inertial Measurement Unit data bus';
    
    % Accelerometer data (m/s^2)
    IMU_Bus.Elements(1) = Simulink.BusElement;
    IMU_Bus.Elements(1).Name = 'acc_x';
    IMU_Bus.Elements(1).DataType = 'double';
    IMU_Bus.Elements(1).Unit = 'm/s^2';
    IMU_Bus.Elements(1).Description = 'X-axis acceleration';
    
    IMU_Bus.Elements(2) = Simulink.BusElement;
    IMU_Bus.Elements(2).Name = 'acc_y';
    IMU_Bus.Elements(2).DataType = 'double';
    IMU_Bus.Elements(2).Unit = 'm/s^2';
    IMU_Bus.Elements(2).Description = 'Y-axis acceleration';
    
    IMU_Bus.Elements(3) = Simulink.BusElement;
    IMU_Bus.Elements(3).Name = 'acc_z';
    IMU_Bus.Elements(3).DataType = 'double';
    IMU_Bus.Elements(3).Unit = 'm/s^2';
    IMU_Bus.Elements(3).Description = 'Z-axis acceleration';
    
    % Gyroscope data (deg/s)
    IMU_Bus.Elements(4) = Simulink.BusElement;
    IMU_Bus.Elements(4).Name = 'gyro_x';
    IMU_Bus.Elements(4).DataType = 'double';
    IMU_Bus.Elements(4).Unit = 'deg/s';
    IMU_Bus.Elements(4).Description = 'X-axis angular rate (roll rate)';
    
    IMU_Bus.Elements(5) = Simulink.BusElement;
    IMU_Bus.Elements(5).Name = 'gyro_y';
    IMU_Bus.Elements(5).DataType = 'double';
    IMU_Bus.Elements(5).Unit = 'deg/s';
    IMU_Bus.Elements(5).Description = 'Y-axis angular rate (pitch rate)';
    
    IMU_Bus.Elements(6) = Simulink.BusElement;
    IMU_Bus.Elements(6).Name = 'gyro_z';
    IMU_Bus.Elements(6).DataType = 'double';
    IMU_Bus.Elements(6).Unit = 'deg/s';
    IMU_Bus.Elements(6).Description = 'Z-axis angular rate (yaw rate)';
    
    % Save to base workspace
    assignin('base', 'IMU_Bus', IMU_Bus);
    
    %% Radio Commands Bus Definition
    RadioCmds_Bus = Simulink.Bus;
    RadioCmds_Bus.Description = 'Radio receiver command signals';
    
    RadioCmds_Bus.Elements(1) = Simulink.BusElement;
    RadioCmds_Bus.Elements(1).Name = 'roll';
    RadioCmds_Bus.Elements(1).DataType = 'double';
    RadioCmds_Bus.Elements(1).Unit = 'us';
    RadioCmds_Bus.Elements(1).Description = 'Roll command PWM (1100-1900us)';
    
    RadioCmds_Bus.Elements(2) = Simulink.BusElement;
    RadioCmds_Bus.Elements(2).Name = 'pitch';
    RadioCmds_Bus.Elements(2).DataType = 'double';
    RadioCmds_Bus.Elements(2).Unit = 'us';
    RadioCmds_Bus.Elements(2).Description = 'Pitch command PWM (1100-1900us)';
    
    RadioCmds_Bus.Elements(3) = Simulink.BusElement;
    RadioCmds_Bus.Elements(3).Name = 'throttle';
    RadioCmds_Bus.Elements(3).DataType = 'double';
    RadioCmds_Bus.Elements(3).Unit = 'us';
    RadioCmds_Bus.Elements(3).Description = 'Throttle command PWM (1090-1900us)';
    
    RadioCmds_Bus.Elements(4) = Simulink.BusElement;
    RadioCmds_Bus.Elements(4).Name = 'yaw';
    RadioCmds_Bus.Elements(4).DataType = 'double';
    RadioCmds_Bus.Elements(4).Unit = 'us';
    RadioCmds_Bus.Elements(4).Description = 'Yaw command PWM (1100-1900us)';
    
    RadioCmds_Bus.Elements(5) = Simulink.BusElement;
    RadioCmds_Bus.Elements(5).Name = 'mode';
    RadioCmds_Bus.Elements(5).DataType = 'double';
    RadioCmds_Bus.Elements(5).Unit = 'us';
    RadioCmds_Bus.Elements(5).Description = 'Mode switch PWM (1100-1900us)';
    
    RadioCmds_Bus.Elements(6) = Simulink.BusElement;
    RadioCmds_Bus.Elements(6).Name = 'safety';
    RadioCmds_Bus.Elements(6).DataType = 'double';
    RadioCmds_Bus.Elements(6).Unit = 'us';
    RadioCmds_Bus.Elements(6).Description = 'Safety switch PWM (1100-1900us)';
    
    assignin('base', 'RadioCmds_Bus', RadioCmds_Bus);
    
    %% Processed Commands Bus Definition
    ProcessedCmds_Bus = Simulink.Bus;
    ProcessedCmds_Bus.Description = 'Processed radio commands';
    
    ProcessedCmds_Bus.Elements(1) = Simulink.BusElement;
    ProcessedCmds_Bus.Elements(1).Name = 'roll_cmd';
    ProcessedCmds_Bus.Elements(1).DataType = 'double';
    ProcessedCmds_Bus.Elements(1).Unit = 'deg';
    ProcessedCmds_Bus.Elements(1).Description = 'Roll angle command (-45 to +45 deg)';
    
    ProcessedCmds_Bus.Elements(2) = Simulink.BusElement;
    ProcessedCmds_Bus.Elements(2).Name = 'pitch_cmd';
    ProcessedCmds_Bus.Elements(2).DataType = 'double';
    ProcessedCmds_Bus.Elements(2).Unit = 'deg';
    ProcessedCmds_Bus.Elements(2).Description = 'Pitch angle command (-45 to +45 deg)';
    
    ProcessedCmds_Bus.Elements(3) = Simulink.BusElement;
    ProcessedCmds_Bus.Elements(3).Name = 'throttle';
    ProcessedCmds_Bus.Elements(3).DataType = 'double';
    ProcessedCmds_Bus.Elements(3).Unit = '%';
    ProcessedCmds_Bus.Elements(3).Description = 'Throttle command (0-100%)';
    
    ProcessedCmds_Bus.Elements(4) = Simulink.BusElement;
    ProcessedCmds_Bus.Elements(4).Name = 'yaw_cmd';
    ProcessedCmds_Bus.Elements(4).DataType = 'double';
    ProcessedCmds_Bus.Elements(4).Unit = 'deg/s';
    ProcessedCmds_Bus.Elements(4).Description = 'Yaw rate command (-135 to +135 deg/s)';
    
    ProcessedCmds_Bus.Elements(5) = Simulink.BusElement;
    ProcessedCmds_Bus.Elements(5).Name = 'mode_switch';
    ProcessedCmds_Bus.Elements(5).DataType = 'boolean';
    ProcessedCmds_Bus.Elements(5).Unit = '';
    ProcessedCmds_Bus.Elements(5).Description = 'Mode switch (0=Accro, 1=Angle)';
    
    ProcessedCmds_Bus.Elements(6) = Simulink.BusElement;
    ProcessedCmds_Bus.Elements(6).Name = 'safety_switch';
    ProcessedCmds_Bus.Elements(6).DataType = 'boolean';
    ProcessedCmds_Bus.Elements(6).Unit = '';
    ProcessedCmds_Bus.Elements(6).Description = 'Safety switch (0=Disarmed, 1=Armed)';
    
    assignin('base', 'ProcessedCmds_Bus', ProcessedCmds_Bus);
    
    %% Attitude Bus Definition
    Attitude_Bus = Simulink.Bus;
    Attitude_Bus.Description = 'Vehicle attitude information';
    
    Attitude_Bus.Elements(1) = Simulink.BusElement;
    Attitude_Bus.Elements(1).Name = 'roll_angle';
    Attitude_Bus.Elements(1).DataType = 'double';
    Attitude_Bus.Elements(1).Unit = 'deg';
    Attitude_Bus.Elements(1).Description = 'Roll angle';
    
    Attitude_Bus.Elements(2) = Simulink.BusElement;
    Attitude_Bus.Elements(2).Name = 'pitch_angle';
    Attitude_Bus.Elements(2).DataType = 'double';
    Attitude_Bus.Elements(2).Unit = 'deg';
    Attitude_Bus.Elements(2).Description = 'Pitch angle';
    
    Attitude_Bus.Elements(3) = Simulink.BusElement;
    Attitude_Bus.Elements(3).Name = 'roll_rate';
    Attitude_Bus.Elements(3).DataType = 'double';
    Attitude_Bus.Elements(3).Unit = 'deg/s';
    Attitude_Bus.Elements(3).Description = 'Roll angular rate';
    
    Attitude_Bus.Elements(4) = Simulink.BusElement;
    Attitude_Bus.Elements(4).Name = 'pitch_rate';
    Attitude_Bus.Elements(4).DataType = 'double';
    Attitude_Bus.Elements(4).Unit = 'deg/s';
    Attitude_Bus.Elements(4).Description = 'Pitch angular rate';
    
    Attitude_Bus.Elements(5) = Simulink.BusElement;
    Attitude_Bus.Elements(5).Name = 'yaw_rate';
    Attitude_Bus.Elements(5).DataType = 'double';
    Attitude_Bus.Elements(5).Unit = 'deg/s';
    Attitude_Bus.Elements(5).Description = 'Yaw angular rate';
    
    assignin('base', 'Attitude_Bus', Attitude_Bus);
    
    %% Control Powers Bus Definition
    ControlPowers_Bus = Simulink.Bus;
    ControlPowers_Bus.Description = 'Control system power commands';
    
    ControlPowers_Bus.Elements(1) = Simulink.BusElement;
    ControlPowers_Bus.Elements(1).Name = 'roll_power';
    ControlPowers_Bus.Elements(1).DataType = 'double';
    ControlPowers_Bus.Elements(1).Unit = '';
    ControlPowers_Bus.Elements(1).Description = 'Roll control power';
    
    ControlPowers_Bus.Elements(2) = Simulink.BusElement;
    ControlPowers_Bus.Elements(2).Name = 'pitch_power';
    ControlPowers_Bus.Elements(2).DataType = 'double';
    ControlPowers_Bus.Elements(2).Unit = '';
    ControlPowers_Bus.Elements(2).Description = 'Pitch control power';
    
    ControlPowers_Bus.Elements(3) = Simulink.BusElement;
    ControlPowers_Bus.Elements(3).Name = 'yaw_power';
    ControlPowers_Bus.Elements(3).DataType = 'double';
    ControlPowers_Bus.Elements(3).Unit = '';
    ControlPowers_Bus.Elements(3).Description = 'Yaw control power';
    
    assignin('base', 'ControlPowers_Bus', ControlPowers_Bus);
    
    %% Motor Commands Bus Definition
    MotorCmds_Bus = Simulink.Bus;
    MotorCmds_Bus.Description = 'Motor PWM commands';
    
    MotorCmds_Bus.Elements(1) = Simulink.BusElement;
    MotorCmds_Bus.Elements(1).Name = 'motor0';
    MotorCmds_Bus.Elements(1).DataType = 'double';
    MotorCmds_Bus.Elements(1).Unit = 'us';
    MotorCmds_Bus.Elements(1).Description = 'Motor 0 PWM command (1000-2000us)';
    
    MotorCmds_Bus.Elements(2) = Simulink.BusElement;
    MotorCmds_Bus.Elements(2).Name = 'motor1';
    MotorCmds_Bus.Elements(2).DataType = 'double';
    MotorCmds_Bus.Elements(2).Unit = 'us';
    MotorCmds_Bus.Elements(2).Description = 'Motor 1 PWM command (1000-2000us)';
    
    MotorCmds_Bus.Elements(3) = Simulink.BusElement;
    MotorCmds_Bus.Elements(3).Name = 'motor2';
    MotorCmds_Bus.Elements(3).DataType = 'double';
    MotorCmds_Bus.Elements(3).Unit = 'us';
    MotorCmds_Bus.Elements(3).Description = 'Motor 2 PWM command (1000-2000us)';
    
    MotorCmds_Bus.Elements(4) = Simulink.BusElement;
    MotorCmds_Bus.Elements(4).Name = 'motor3';
    MotorCmds_Bus.Elements(4).DataType = 'double';
    MotorCmds_Bus.Elements(4).Unit = 'us';
    MotorCmds_Bus.Elements(4).Description = 'Motor 3 PWM command (1000-2000us)';
    
    assignin('base', 'MotorCmds_Bus', MotorCmds_Bus);
    
    %% System Status Bus Definition
    SystemStatus_Bus = Simulink.Bus;
    SystemStatus_Bus.Description = 'System status information';
    
    SystemStatus_Bus.Elements(1) = Simulink.BusElement;
    SystemStatus_Bus.Elements(1).Name = 'current_state';
    SystemStatus_Bus.Elements(1).DataType = 'uint8';
    SystemStatus_Bus.Elements(1).Unit = '';
    SystemStatus_Bus.Elements(1).Description = 'Current state machine state (1-5)';
    
    SystemStatus_Bus.Elements(2) = Simulink.BusElement;
    SystemStatus_Bus.Elements(2).Name = 'is_safety_needed';
    SystemStatus_Bus.Elements(2).DataType = 'boolean';
    SystemStatus_Bus.Elements(2).Unit = '';
    SystemStatus_Bus.Elements(2).Description = 'Safety state trigger flag';
    
    SystemStatus_Bus.Elements(3) = Simulink.BusElement;
    SystemStatus_Bus.Elements(3).Name = 'loop_time';
    SystemStatus_Bus.Elements(3).DataType = 'double';
    SystemStatus_Bus.Elements(3).Unit = 's';
    SystemStatus_Bus.Elements(3).Description = 'Actual loop execution time';
    
    assignin('base', 'SystemStatus_Bus', SystemStatus_Bus);
    
    %% PID State Bus Definition (for persistent state storage)
    PIDState_Bus = Simulink.Bus;
    PIDState_Bus.Description = 'PID controller internal state';
    
    PIDState_Bus.Elements(1) = Simulink.BusElement;
    PIDState_Bus.Elements(1).Name = 'integrator';
    PIDState_Bus.Elements(1).DataType = 'double';
    PIDState_Bus.Elements(1).Unit = '';
    PIDState_Bus.Elements(1).Description = 'PID integrator state';
    
    PIDState_Bus.Elements(2) = Simulink.BusElement;
    PIDState_Bus.Elements(2).Name = 'prev_error';
    PIDState_Bus.Elements(2).DataType = 'double';
    PIDState_Bus.Elements(2).Unit = '';
    PIDState_Bus.Elements(2).Description = 'Previous error for derivative';
    
    assignin('base', 'PIDState_Bus', PIDState_Bus);
    
    fprintf('✓ All bus definitions created successfully!\n');
    fprintf('  - IMU_Bus: Inertial measurement data\n');
    fprintf('  - RadioCmds_Bus: Raw radio commands\n');
    fprintf('  - ProcessedCmds_Bus: Processed commands\n');
    fprintf('  - Attitude_Bus: Vehicle attitude\n');
    fprintf('  - ControlPowers_Bus: Control outputs\n');
    fprintf('  - MotorCmds_Bus: Motor PWM commands\n');
    fprintf('  - SystemStatus_Bus: System status\n');
    fprintf('  - PIDState_Bus: PID controller states\n');
end

function clear_bus_definitions()
    % Clear any existing bus definitions from workspace
    bus_names = {'IMU_Bus', 'RadioCmds_Bus', 'ProcessedCmds_Bus', ...
                 'Attitude_Bus', 'ControlPowers_Bus', 'MotorCmds_Bus', ...
                 'SystemStatus_Bus', 'PIDState_Bus'};
    
    for i = 1:length(bus_names)
        try
            evalin('base', ['clear ' bus_names{i}]);
        catch
            % Bus doesn't exist, continue
        end
    end
end

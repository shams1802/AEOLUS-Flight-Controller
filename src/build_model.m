%% Build CodeDroneDIY Simulink Model
% This script programmatically creates the complete Simulink model
% for the CodeDroneDIY flight controller
% Author: Auto-generated for CodeDroneDIY Simulink conversion
% Date: 2024

function build_model()
    % Main function to build the complete drone control Simulink model
    
    %% Model Configuration
    modelName = 'CodeDroneDIY_FlightController';
    
    % Close any existing model with the same name
    try
        close_system(modelName, 0);
    catch
        % Model not open, continue
    end
    
    % Create new model
    new_system(modelName);
    open_system(modelName);
    
    fprintf('Building CodeDroneDIY Flight Controller Simulink Model...\n');
    
    %% Load Bus Definitions
    fprintf('Loading bus definitions...\n');
    run('bus_definitions.m');
    
    %% Configure Model Properties
    configure_model(modelName);
    
    %% Create Subsystems and Blocks
    create_input_subsystem(modelName);
    create_imu_processing_subsystem(modelName);
    create_safety_subsystem(modelName);
    create_state_machine_subsystem(modelName);
    create_control_subsystem(modelName);
    create_motor_mixing_subsystem(modelName);
    create_output_subsystem(modelName);
    
    %% Connect the blocks
    connect_model_blocks(modelName);
    
    %% Add scopes and displays
    add_visualization_blocks(modelName);
    
    %% Layout the model
    layout_model(modelName);
    
    %% Save the model
    save_system(modelName);
    
    fprintf('\n✓ CodeDroneDIY Simulink model created successfully!\n');
    fprintf('Model name: %s.slx\n', modelName);
    fprintf('To run simulation: sim(''%s'')\n', modelName);
    
    %% Generate configuration script
    generate_config_script(modelName);
end

%% ========================================================================
%% MODEL CONFIGURATION
%% ========================================================================

function configure_model(modelName)
    % Configure model solver and simulation parameters
    
    fprintf('Configuring model parameters...\n');
    
    % Get model configuration parameters
    cs = getActiveConfigSet(modelName);
    
    % Solver configuration for real-time control
    set_param(cs, 'Solver', 'ode4');                    % Fixed-step Runge-Kutta
    set_param(cs, 'FixedStep', '0.0025');               % 400 Hz (2.5ms)
    set_param(cs, 'StartTime', '0');
    set_param(cs, 'StopTime', '30');                    % 30 second simulation
    
    % Data import/export
    set_param(cs, 'LoadExternalInput', 'off');
    set_param(cs, 'SaveOutput', 'on');
    set_param(cs, 'OutputSaveName', 'yout');
    set_param(cs, 'SaveTime', 'on');
    set_param(cs, 'TimeSaveName', 'tout');
    
    % Code generation settings (for deployment)
    set_param(cs, 'SystemTargetFile', 'ert.tlc');       % Embedded real-time target
    set_param(cs, 'TargetLang', 'C');
    set_param(cs, 'GenerateReport', 'on');
    
    % Optimization settings
    set_param(cs, 'OptimizeBlockIOStorage', 'on');
    set_param(cs, 'LocalBlockOutputs', 'on');
    set_param(cs, 'BufferReuse', 'on');
    
    % Diagnostics
    set_param(cs, 'AlgebraicLoopMsg', 'error');
    set_param(cs, 'CheckSSInitialOutputMsg', 'error');
    
    fprintf('  - Solver: Fixed-step ODE4 @ 400Hz\n');
    fprintf('  - Target: Embedded Real-Time (C code)\n');
end

%% ========================================================================
%% SUBSYSTEM CREATION FUNCTIONS
%% ========================================================================

function create_input_subsystem(modelName)
    % Create input processing subsystem
    
    fprintf('Creating input subsystem...\n');
    
    subsysName = [modelName '/Input_Processing'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add test scenario generator
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/TestScenarioGenerator']);
    set_param([subsysName '/TestScenarioGenerator'], 'Script', ...
        ['function [radio_cmds, imu_data] = fcn(current_time)\n' ...
         'run(''drone_simulink_blocks.m'');\n' ...
         '[radio_cmds, imu_data] = TestScenarioGenerator(current_time);']);
    
    % Add clock
    add_block('simulink/Sources/Clock', [subsysName '/Clock']);
    
    % Add outputs
    add_block('simulink/Sinks/Out1', [subsysName '/RadioCmds_Out']);
    set_param([subsysName '/RadioCmds_Out'], 'BusObject', 'RadioCmds_Bus');
    
    add_block('simulink/Sinks/Out1', [subsysName '/IMU_Out']);
    set_param([subsysName '/IMU_Out'], 'BusObject', 'IMU_Bus');
    set_param([subsysName '/IMU_Out'], 'Port', '2');
    
    % Connect within subsystem
    add_line(subsysName, 'Clock/1', 'TestScenarioGenerator/1');
    add_line(subsysName, 'TestScenarioGenerator/1', 'RadioCmds_Out/1');
    add_line(subsysName, 'TestScenarioGenerator/2', 'IMU_Out/1');
end

function create_imu_processing_subsystem(modelName)
    % Create IMU data processing subsystem
    
    fprintf('Creating IMU processing subsystem...\n');
    
    subsysName = [modelName '/IMU_Processing'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add inputs
    add_block('simulink/Sources/In1', [subsysName '/IMU_In']);
    set_param([subsysName '/IMU_In'], 'BusObject', 'IMU_Bus');
    
    add_block('simulink/Sources/In1', [subsysName '/Clock_In']);
    set_param([subsysName '/Clock_In'], 'Port', '2');
    
    % Add IMU processor
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/IMUProcessor']);
    set_param([subsysName '/IMUProcessor'], 'Script', ...
        ['function [attitude, attitude_prev_out] = fcn(imu_data, attitude_prev_in, dt)\n' ...
         'persistent attitude_prev;\n' ...
         'if isempty(attitude_prev)\n' ...
         '    attitude_prev.roll_angle = 0;\n' ...
         '    attitude_prev.pitch_angle = 0;\n' ...
         'end\n' ...
         'run(''drone_simulink_blocks.m'');\n' ...
         '[attitude, attitude_prev] = IMUProcessor(imu_data, attitude_prev, dt);\n' ...
         'attitude_prev_out = attitude_prev;']);
    
    % Add dt calculation
    add_block('simulink/Math Operations/Derivative', [subsysName '/dt_calc']);
    add_block('simulink/Math Operations/Abs', [subsysName '/dt_abs']);
    add_block('simulink/Math Operations/MinMax', [subsysName '/dt_limit']);
    set_param([subsysName '/dt_limit'], 'Function', 'max');
    set_param([subsysName '/dt_limit'], 'Inputs', '2');
    
    add_block('simulink/Sources/Constant', [subsysName '/dt_min']);
    set_param([subsysName '/dt_min'], 'Value', '1e-6');
    
    % Add outputs
    add_block('simulink/Sinks/Out1', [subsysName '/Attitude_Out']);
    set_param([subsysName '/Attitude_Out'], 'BusObject', 'Attitude_Bus');
    
    % Connect within subsystem
    add_line(subsysName, 'Clock_In/1', 'dt_calc/1');
    add_line(subsysName, 'dt_calc/1', 'dt_abs/1');
    add_line(subsysName, 'dt_abs/1', 'dt_limit/1');
    add_line(subsysName, 'dt_min/1', 'dt_limit/2');
    add_line(subsysName, 'IMU_In/1', 'IMUProcessor/1');
    add_line(subsysName, 'dt_limit/1', 'IMUProcessor/3');
    add_line(subsysName, 'IMUProcessor/1', 'Attitude_Out/1');
    % Note: attitude_prev feedback would need a unit delay
end

function create_safety_subsystem(modelName)
    % Create safety monitoring subsystem
    
    fprintf('Creating safety subsystem...\n');
    
    subsysName = [modelName '/Safety_Monitoring'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add inputs
    add_block('simulink/Sources/In1', [subsysName '/ProcessedCmds_In']);
    set_param([subsysName '/ProcessedCmds_In'], 'BusObject', 'ProcessedCmds_Bus');
    
    add_block('simulink/Sources/In1', [subsysName '/Clock_In']);
    set_param([subsysName '/Clock_In'], 'Port', '2');
    
    % Add safety checker
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/SafetyChecker']);
    set_param([subsysName '/SafetyChecker'], 'Script', ...
        ['function [is_safety_needed, throttle_was_high_out, throttle_low_start_time_out] = fcn(throttle, current_time, throttle_was_high_in, throttle_low_start_time_in)\n' ...
         'persistent throttle_was_high throttle_low_start_time;\n' ...
         'if isempty(throttle_was_high)\n' ...
         '    throttle_was_high = true;\n' ...
         '    throttle_low_start_time = 0;\n' ...
         'end\n' ...
         'run(''drone_simulink_blocks.m'');\n' ...
         '[is_safety_needed, throttle_was_high, throttle_low_start_time] = SafetyStateChecker(throttle, current_time, throttle_was_high, throttle_low_start_time);\n' ...
         'throttle_was_high_out = throttle_was_high;\n' ...
         'throttle_low_start_time_out = throttle_low_start_time;']);
    
    % Add output
    add_block('simulink/Sinks/Out1', [subsysName '/SafetyFlag_Out']);
    
    % Connect within subsystem
    add_line(subsysName, 'ProcessedCmds_In/1', 'SafetyChecker/1');
    add_line(subsysName, 'Clock_In/1', 'SafetyChecker/2');
    add_line(subsysName, 'SafetyChecker/1', 'SafetyFlag_Out/1');
end

function create_state_machine_subsystem(modelName)
    % Create state machine subsystem (using MATLAB Function for simplicity)
    
    fprintf('Creating state machine subsystem...\n');
    
    subsysName = [modelName '/State_Machine'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add inputs
    add_block('simulink/Sources/In1', [subsysName '/ProcessedCmds_In']);
    set_param([subsysName '/ProcessedCmds_In'], 'BusObject', 'ProcessedCmds_Bus');
    
    add_block('simulink/Sources/In1', [subsysName '/SafetyFlag_In']);
    set_param([subsysName '/SafetyFlag_In'], 'Port', '2');
    
    add_block('simulink/Sources/In1', [subsysName '/Clock_In']);
    set_param([subsysName '/Clock_In'], 'Port', '3');
    
    % Add state machine
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/StateMachine']);
    set_param([subsysName '/StateMachine'], 'Script', ...
        ['function [new_state, throttle_was_high_out, throttle_low_start_time_out] = fcn(processed_cmds, is_safety_needed, current_time)\n' ...
         'persistent current_state throttle_was_high throttle_low_start_time;\n' ...
         'if isempty(current_state)\n' ...
         '    current_state = uint8(1);  % INITIALIZING\n' ...
         '    throttle_was_high = true;\n' ...
         '    throttle_low_start_time = 0;\n' ...
         'end\n' ...
         'run(''drone_stateflow_chart.m'');\n' ...
         '[new_state, throttle_was_high, throttle_low_start_time] = StateMachineBlock(current_state, processed_cmds.throttle, is_safety_needed, processed_cmds.safety_switch, processed_cmds.mode_switch, throttle_was_high, throttle_low_start_time, current_time);\n' ...
         'current_state = new_state;\n' ...
         'throttle_was_high_out = throttle_was_high;\n' ...
         'throttle_low_start_time_out = throttle_low_start_time;']);
    
    % Add output
    add_block('simulink/Sinks/Out1', [subsysName '/CurrentState_Out']);
    
    % Connect within subsystem
    add_line(subsysName, 'ProcessedCmds_In/1', 'StateMachine/1');
    add_line(subsysName, 'SafetyFlag_In/1', 'StateMachine/2');
    add_line(subsysName, 'Clock_In/1', 'StateMachine/3');
    add_line(subsysName, 'StateMachine/1', 'CurrentState_Out/1');
end

function create_control_subsystem(modelName)
    % Create control system subsystem
    
    fprintf('Creating control subsystem...\n');
    
    subsysName = [modelName '/Control_System'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add inputs
    add_block('simulink/Sources/In1', [subsysName '/ProcessedCmds_In']);
    set_param([subsysName '/ProcessedCmds_In'], 'BusObject', 'ProcessedCmds_Bus');
    
    add_block('simulink/Sources/In1', [subsysName '/Attitude_In']);
    set_param([subsysName '/Attitude_In'], 'BusObject', 'Attitude_Bus');
    set_param([subsysName '/Attitude_In'], 'Port', '2');
    
    add_block('simulink/Sources/In1', [subsysName '/CurrentState_In']);
    set_param([subsysName '/CurrentState_In'], 'Port', '3');
    
    add_block('simulink/Sources/In1', [subsysName '/Clock_In']);
    set_param([subsysName '/Clock_In'], 'Port', '4');
    
    % Add control selector (switches between angle/accro/safety modes)
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/ControlSelector']);
    set_param([subsysName '/ControlSelector'], 'Script', ...
        ['function [control_powers] = fcn(processed_cmds, attitude, current_state, dt)\n' ...
         'persistent pid_states;\n' ...
         'if isempty(pid_states)\n' ...
         '    pid_states = init_pid_states();\n' ...
         'end\n' ...
         'run(''drone_simulink_blocks.m'');\n' ...
         'switch current_state\n' ...
         '    case 5  % ANGLEMODE\n' ...
         '        [control_powers, pid_states] = AngleModeController(processed_cmds, attitude, pid_states, dt);\n' ...
         '    case 4  % ACCRONODE\n' ...
         '        [control_powers, pid_states] = AccroModeController(processed_cmds, attitude, pid_states, dt);\n' ...
         '    otherwise  % SAFETY, DISARMED, INITIALIZING\n' ...
         '        [control_powers, pid_states] = SafetyController(pid_states);\n' ...
         'end\n' ...
         '\n' ...
         'function pid_states = init_pid_states()\n' ...
         '    pid_states.roll_pos_integrator = 0;\n' ...
         '    pid_states.roll_pos_prev_error = 0;\n' ...
         '    pid_states.pitch_pos_integrator = 0;\n' ...
         '    pid_states.pitch_pos_prev_error = 0;\n' ...
         '    pid_states.roll_speed_angle_integrator = 0;\n' ...
         '    pid_states.roll_speed_angle_prev_error = 0;\n' ...
         '    pid_states.pitch_speed_angle_integrator = 0;\n' ...
         '    pid_states.pitch_speed_angle_prev_error = 0;\n' ...
         '    pid_states.roll_speed_accro_integrator = 0;\n' ...
         '    pid_states.roll_speed_accro_prev_error = 0;\n' ...
         '    pid_states.pitch_speed_accro_integrator = 0;\n' ...
         '    pid_states.pitch_speed_accro_prev_error = 0;\n' ...
         '    pid_states.yaw_integrator = 0;\n' ...
         '    pid_states.yaw_prev_error = 0;']);
    
    % Add dt calculation
    add_block('simulink/Math Operations/Derivative', [subsysName '/dt_calc']);
    add_block('simulink/Math Operations/Abs', [subsysName '/dt_abs']);
    add_block('simulink/Math Operations/MinMax', [subsysName '/dt_limit']);
    set_param([subsysName '/dt_limit'], 'Function', 'max');
    set_param([subsysName '/dt_limit'], 'Inputs', '2');
    
    add_block('simulink/Sources/Constant', [subsysName '/dt_min']);
    set_param([subsysName '/dt_min'], 'Value', '1e-6');
    
    % Add output
    add_block('simulink/Sinks/Out1', [subsysName '/ControlPowers_Out']);
    set_param([subsysName '/ControlPowers_Out'], 'BusObject', 'ControlPowers_Bus');
    
    % Connect within subsystem
    add_line(subsysName, 'Clock_In/1', 'dt_calc/1');
    add_line(subsysName, 'dt_calc/1', 'dt_abs/1');
    add_line(subsysName, 'dt_abs/1', 'dt_limit/1');
    add_line(subsysName, 'dt_min/1', 'dt_limit/2');
    add_line(subsysName, 'ProcessedCmds_In/1', 'ControlSelector/1');
    add_line(subsysName, 'Attitude_In/1', 'ControlSelector/2');
    add_line(subsysName, 'CurrentState_In/1', 'ControlSelector/3');
    add_line(subsysName, 'dt_limit/1', 'ControlSelector/4');
    add_line(subsysName, 'ControlSelector/1', 'ControlPowers_Out/1');
end

function create_motor_mixing_subsystem(modelName)
    % Create motor mixing subsystem
    
    fprintf('Creating motor mixing subsystem...\n');
    
    subsysName = [modelName '/Motor_Mixing'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add inputs
    add_block('simulink/Sources/In1', [subsysName '/ProcessedCmds_In']);
    set_param([subsysName '/ProcessedCmds_In'], 'BusObject', 'ProcessedCmds_Bus');
    
    add_block('simulink/Sources/In1', [subsysName '/ControlPowers_In']);
    set_param([subsysName '/ControlPowers_In'], 'BusObject', 'ControlPowers_Bus');
    set_param([subsysName '/ControlPowers_In'], 'Port', '2');
    
    % Add motor mixer
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/MotorMixer']);
    set_param([subsysName '/MotorMixer'], 'Script', ...
        ['function [motor_cmds] = fcn(processed_cmds, control_powers)\n' ...
         'run(''drone_simulink_blocks.m'');\n' ...
         'motor_cmds = MotorMixer(processed_cmds.throttle, control_powers);']);
    
    % Add output
    add_block('simulink/Sinks/Out1', [subsysName '/MotorCmds_Out']);
    set_param([subsysName '/MotorCmds_Out'], 'BusObject', 'MotorCmds_Bus');
    
    % Connect within subsystem
    add_line(subsysName, 'ProcessedCmds_In/1', 'MotorMixer/1');
    add_line(subsysName, 'ControlPowers_In/1', 'MotorMixer/2');
    add_line(subsysName, 'MotorMixer/1', 'MotorCmds_Out/1');
end

function create_output_subsystem(modelName)
    % Create output processing subsystem
    
    fprintf('Creating output subsystem...\n');
    
    subsysName = [modelName '/Output_Processing'];
    add_block('built-in/Subsystem', subsysName);
    
    % Delete default blocks
    delete_line(subsysName, 'In1/1', 'Out1/1');
    delete_block([subsysName '/In1']);
    delete_block([subsysName '/Out1']);
    
    % Add radio processor
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [subsysName '/RadioProcessor']);
    set_param([subsysName '/RadioProcessor'], 'Script', ...
        ['function [processed_cmds] = fcn(radio_cmds)\n' ...
         'run(''drone_simulink_blocks.m'');\n' ...
         'processed_cmds = RadioInputProcessor(radio_cmds);']);
    
    % Add inputs
    add_block('simulink/Sources/In1', [subsysName '/RadioCmds_In']);
    set_param([subsysName '/RadioCmds_In'], 'BusObject', 'RadioCmds_Bus');
    
    % Add outputs  
    add_block('simulink/Sinks/Out1', [subsysName '/ProcessedCmds_Out']);
    set_param([subsysName '/ProcessedCmds_Out'], 'BusObject', 'ProcessedCmds_Bus');
    
    % Connect within subsystem
    add_line(subsysName, 'RadioCmds_In/1', 'RadioProcessor/1');
    add_line(subsysName, 'RadioProcessor/1', 'ProcessedCmds_Out/1');
end

%% ========================================================================
%% MODEL CONNECTION AND LAYOUT
%% ========================================================================

function connect_model_blocks(modelName)
    % Connect all the subsystems in the main model
    
    fprintf('Connecting model blocks...\n');
    
    % Add clock for timing
    add_block('simulink/Sources/Clock', [modelName '/System_Clock']);
    
    % Connect the data flow
    add_line(modelName, 'System_Clock/1', 'Input_Processing/1');
    add_line(modelName, 'Input_Processing/1', 'Output_Processing/1');
    add_line(modelName, 'Input_Processing/2', 'IMU_Processing/1');
    add_line(modelName, 'System_Clock/1', 'IMU_Processing/2');
    add_line(modelName, 'Output_Processing/1', 'Safety_Monitoring/1');
    add_line(modelName, 'System_Clock/1', 'Safety_Monitoring/2');
    add_line(modelName, 'Output_Processing/1', 'State_Machine/1');
    add_line(modelName, 'Safety_Monitoring/1', 'State_Machine/2');
    add_line(modelName, 'System_Clock/1', 'State_Machine/3');
    add_line(modelName, 'Output_Processing/1', 'Control_System/1');
    add_line(modelName, 'IMU_Processing/1', 'Control_System/2');
    add_line(modelName, 'State_Machine/1', 'Control_System/3');
    add_line(modelName, 'System_Clock/1', 'Control_System/4');
    add_line(modelName, 'Output_Processing/1', 'Motor_Mixing/1');
    add_line(modelName, 'Control_System/1', 'Motor_Mixing/2');
end

function add_visualization_blocks(modelName)
    % Add scopes and displays for monitoring
    
    fprintf('Adding visualization blocks...\n');
    
    % Add scope for attitude
    add_block('simulink/Sinks/Scope', [modelName '/Attitude_Scope']);
    set_param([modelName '/Attitude_Scope'], 'NumInputPorts', '3');
    set_param([modelName '/Attitude_Scope'], 'ScopeType', 'Floating');
    
    % Add scope for motor outputs
    add_block('simulink/Sinks/Scope', [modelName '/Motor_Scope']);
    set_param([modelName '/Motor_Scope'], 'NumInputPorts', '4');
    set_param([modelName '/Motor_Scope'], 'ScopeType', 'Floating');
    
    % Add display for current state
    add_block('simulink/Sinks/Display', [modelName '/State_Display']);
    
    % Add bus selectors for visualization
    add_block('simulink/Signal Routing/Bus Selector', [modelName '/Attitude_BusSelect']);
    set_param([modelName '/Attitude_BusSelect'], 'OutputSignals', 'roll_angle,pitch_angle,yaw_rate');
    
    add_block('simulink/Signal Routing/Bus Selector', [modelName '/Motor_BusSelect']);
    set_param([modelName '/Motor_BusSelect'], 'OutputSignals', 'motor0,motor1,motor2,motor3');
    
    % Connect visualization
    add_line(modelName, 'IMU_Processing/1', 'Attitude_BusSelect/1');
    add_line(modelName, 'Motor_Mixing/1', 'Motor_BusSelect/1');
    add_line(modelName, 'State_Machine/1', 'State_Display/1');
    
    add_line(modelName, 'Attitude_BusSelect/1', 'Attitude_Scope/1');
    add_line(modelName, 'Attitude_BusSelect/2', 'Attitude_Scope/2');
    add_line(modelName, 'Attitude_BusSelect/3', 'Attitude_Scope/3');
    
    add_line(modelName, 'Motor_BusSelect/1', 'Motor_Scope/1');
    add_line(modelName, 'Motor_BusSelect/2', 'Motor_Scope/2');
    add_line(modelName, 'Motor_BusSelect/3', 'Motor_Scope/3');
    add_line(modelName, 'Motor_BusSelect/4', 'Motor_Scope/4');
end

function layout_model(modelName)
    % Auto-layout the model for better visualization
    
    fprintf('Layouting model...\n');
    
    % Use Simulink's auto-layout feature
    try
        Simulink.BlockDiagram.arrangeSystem(modelName);
    catch
        % If auto-layout fails, continue with manual positioning
        fprintf('Auto-layout not available, using default positioning\n');
    end
end

function generate_config_script(modelName)
    % Generate configuration script for the model
    
    fprintf('Generating configuration script...\n');
    
    configScript = [
        '%% Configuration Script for ' modelName '\n' ...
        '% Auto-generated configuration for CodeDroneDIY Flight Controller\n\n' ...
        '%% Load Dependencies\n' ...
        'run(''bus_definitions.m'');  % Load bus objects\n' ...
        'addpath(pwd);                % Ensure current path is included\n\n' ...
        '%% Simulation Commands\n' ...
        'fprintf(''Running CodeDroneDIY Flight Controller Simulation...\\n'');\n' ...
        'sim(''' modelName ''');      % Run simulation\n' ...
        'fprintf(''Simulation completed!\\n'');\n\n' ...
        '%% Analysis Commands\n' ...
        '% Extract simulation results\n' ...
        '% plot(tout, yout);           % Plot results\n' ...
        '% analyze_drone_performance;  % Custom analysis function\n'
    ];
    
    configFileName = [modelName '_config.m'];
    fid = fopen(configFileName, 'w');
    fprintf(fid, configScript);
    fclose(fid);
    
    fprintf('Configuration script saved as: %s\n', configFileName);
end

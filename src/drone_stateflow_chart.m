%% Stateflow Chart Definition for Drone State Machine
% This file contains the Stateflow chart logic for the drone state machine
% Converted from runStateMachine function in original MATLAB code

%% STATE MACHINE OVERVIEW
% States:
% 1. INITIALIZING - System startup and calibration
% 2. SAFETY       - Emergency safety mode
% 3. DISARMED     - Motors off, safe state
% 4. ACCRONODE    - Acrobatic (rate) mode
% 5. ANGLEMODE    - Angle (stabilized) mode

%% STATEFLOW CHART ACTIONS AND CONDITIONS

function stateflow_actions = drone_stateflow_chart()
    % Returns the Stateflow chart definition structure
    
    stateflow_actions = struct();
    
    %% Chart Data Definition
    stateflow_actions.chart_data = {
        % Inputs
        'throttle',             'double', 'Input',  'Throttle percentage (0-100%)';
        'is_safety_needed',     'boolean','Input',  'Safety trigger flag';
        'safety_switch',        'boolean','Input',  'Safety switch state';
        'current_time',         'double', 'Input',  'Current simulation time';
        
        % Outputs  
        'current_state',        'uint8',  'Output', 'Current state (1-5)';
        
        % Local Data
        'entry_time',           'double', 'Local',  'State entry time';
        'prev_state',           'uint8',  'Local',  'Previous state for debugging';
    };
    
    %% State Definitions
    stateflow_actions.states = {
        'INITIALIZING', 1, 'Initialize system and calibrate sensors';
        'SAFETY',       2, 'Emergency safety mode - all systems disabled';
        'DISARMED',     3, 'Motors disabled, waiting for arm command';
        'ACCRONODE',    4, 'Acrobatic mode - direct rate control';
        'ANGLEMODE',    5, 'Angle mode - stabilized attitude control';
    };
    
    %% Transition Conditions and Actions
    stateflow_actions.transitions = {
        % From INITIALIZING
        'INITIALIZING', 'DISARMED', 'after(2,sec)', 'Calibration complete after 2 seconds';
        
        % From DISARMED
        'DISARMED', 'ANGLEMODE', 'throttle > 20 && safety_switch', 'Arm to angle mode';
        
        % From ANGLEMODE
        'ANGLEMODE', 'SAFETY', 'is_safety_needed', 'Safety trigger activated';
        'ANGLEMODE', 'ACCRONODE', '~mode_switch && throttle > 20', 'Switch to acro mode';
        'ANGLEMODE', 'DISARMED', 'throttle <= 20 && ~safety_switch', 'Disarm command';
        
        % From ACCRONODE
        'ACCRONODE', 'SAFETY', 'is_safety_needed', 'Safety trigger activated';
        'ACCRONODE', 'ANGLEMODE', 'mode_switch && throttle > 20', 'Switch to angle mode';
        'ACCRONODE', 'DISARMED', 'throttle <= 20 && ~safety_switch', 'Disarm command';
        
        % From SAFETY
        'SAFETY', 'DISARMED', 'after(1,sec)', 'Auto-transition to disarmed after 1 second';
    };
    
    %% Entry Actions for each state
    stateflow_actions.entry_actions = {
        'INITIALIZING', 'entry_time = current_time; prev_state = current_state; current_state = 1;';
        'SAFETY',       'entry_time = current_time; prev_state = current_state; current_state = 2;';
        'DISARMED',     'entry_time = current_time; prev_state = current_state; current_state = 3;';
        'ACCRONODE',    'entry_time = current_time; prev_state = current_state; current_state = 4;';
        'ANGLEMODE',    'entry_time = current_time; prev_state = current_state; current_state = 5;';
    };
    
    %% During Actions for each state
    stateflow_actions.during_actions = {
        'INITIALIZING', '% Perform sensor calibration';
        'SAFETY',       '% All control outputs disabled';
        'DISARMED',     '% Motors disabled, waiting for commands';
        'ACCRONODE',    '% Rate control mode active';
        'ANGLEMODE',    '% Angle control mode active';
    };
    
    %% Exit Actions for each state
    stateflow_actions.exit_actions = {
        'INITIALIZING', '% Initialization complete';
        'SAFETY',       '% Exiting safety mode';
        'DISARMED',     '% Exiting disarmed state';
        'ACCRONODE',    '% Exiting acro mode';
        'ANGLEMODE',    '% Exiting angle mode';
    };
    
end

%% MATLAB FUNCTION BLOCK VERSION OF STATE MACHINE
% This can be used as an alternative to Stateflow if preferred

function [new_state, throttle_was_high_out, throttle_low_start_time_out] = StateMachineBlock(...
    current_state, throttle, is_safety_needed, safety_switch, mode_switch, ...
    throttle_was_high, throttle_low_start_time, current_time)
    % MATLAB Function Block: State Machine
    % Implements the drone state machine logic
    
    %#codegen
    
    % State enumeration
    INITIALIZING = uint8(1);
    SAFETY = uint8(2);
    DISARMED = uint8(3);
    ACCRONODE = uint8(4);
    ANGLEMODE = uint8(5);
    
    % Constants
    MOTOR_IDLE_THRESHOLD = 20;  % Percentage threshold
    
    % Initialize outputs
    new_state = current_state;
    throttle_was_high_out = throttle_was_high;
    throttle_low_start_time_out = throttle_low_start_time;
    
    % State machine logic
    switch current_state
        case INITIALIZING
            % Transition to Disarmed when IMU is calibrated
            % In real system, this would wait for calibration complete
            new_state = DISARMED;
            
        case ANGLEMODE
            if is_safety_needed
                new_state = SAFETY;
            elseif ~mode_switch && throttle > MOTOR_IDLE_THRESHOLD
                new_state = ACCRONODE;  % Switch to acro mode
            elseif throttle <= MOTOR_IDLE_THRESHOLD && ~safety_switch
                new_state = DISARMED;   % Disarm
            end
            
        case ACCRONODE
            if is_safety_needed
                new_state = SAFETY;
            elseif mode_switch && throttle > MOTOR_IDLE_THRESHOLD
                new_state = ANGLEMODE;  % Switch to angle mode
            elseif throttle <= MOTOR_IDLE_THRESHOLD && ~safety_switch
                new_state = DISARMED;   % Disarm
            end
            
        case SAFETY
            % Automatic transition to disarmed after safety condition
            new_state = DISARMED;
            
        case DISARMED
            % Transition back to armed when throttle high and safety switch on
            if throttle > MOTOR_IDLE_THRESHOLD && safety_switch
                if mode_switch
                    new_state = ANGLEMODE;
                else
                    new_state = ACCRONODE;
                end
            end
    end
    
    % Update throttle tracking for safety logic
    if throttle_was_high && throttle < MOTOR_IDLE_THRESHOLD
        throttle_was_high_out = false;
        throttle_low_start_time_out = current_time;
    elseif throttle > MOTOR_IDLE_THRESHOLD
        throttle_was_high_out = true;
    end
end

%% STATEFLOW CHART CREATION HELPER
function create_stateflow_chart(modelName, chartName)
    % Helper function to programmatically create Stateflow chart
    % This function can be called from the build script
    
    % Add Stateflow chart to model
    chart_block = [modelName '/' chartName];
    add_block('sflib/Chart', chart_block);
    
    % Get chart object
    rt = sfroot;
    model = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', modelName);
    chart = model.find('-isa', 'Stateflow.Chart', 'Name', chartName);
    
    % Configure chart properties
    chart.ActionLanguage = 'MATLAB';
    
    % Add data to chart
    add_chart_data(chart, 'throttle', 'double', 'Input');
    add_chart_data(chart, 'is_safety_needed', 'boolean', 'Input');
    add_chart_data(chart, 'safety_switch', 'boolean', 'Input');
    add_chart_data(chart, 'mode_switch', 'boolean', 'Input');
    add_chart_data(chart, 'current_time', 'double', 'Input');
    add_chart_data(chart, 'current_state', 'uint8', 'Output');
    add_chart_data(chart, 'entry_time', 'double', 'Local');
    
    % Create states
    states = create_states(chart);
    
    % Create transitions
    create_transitions(chart, states);
    
    fprintf('Stateflow chart "%s" created successfully!\n', chartName);
end

function data = add_chart_data(chart, name, dataType, scope)
    % Helper to add data to Stateflow chart
    data = Stateflow.Data(chart);
    data.Name = name;
    data.DataType = dataType;
    data.Scope = scope;
end

function states = create_states(chart)
    % Create all states for the drone state machine
    
    % INITIALIZING state
    states.initializing = Stateflow.State(chart);
    states.initializing.Name = 'INITIALIZING';
    states.initializing.Position = [50, 50, 150, 100];
    states.initializing.EntryAction = 'entry_time = current_time; current_state = 1;';
    
    % SAFETY state  
    states.safety = Stateflow.State(chart);
    states.safety.Name = 'SAFETY';
    states.safety.Position = [250, 50, 150, 100];
    states.safety.EntryAction = 'entry_time = current_time; current_state = 2;';
    
    % DISARMED state
    states.disarmed = Stateflow.State(chart);
    states.disarmed.Name = 'DISARMED';
    states.disarmed.Position = [50, 200, 150, 100];
    states.disarmed.EntryAction = 'entry_time = current_time; current_state = 3;';
    
    % ACCRONODE state
    states.accro = Stateflow.State(chart);
    states.accro.Name = 'ACCRONODE';
    states.accro.Position = [250, 200, 150, 100];
    states.accro.EntryAction = 'entry_time = current_time; current_state = 4;';
    
    % ANGLEMODE state
    states.angle = Stateflow.State(chart);
    states.angle.Name = 'ANGLEMODE';
    states.angle.Position = [450, 200, 150, 100];
    states.angle.EntryAction = 'entry_time = current_time; current_state = 5;';
end

function create_transitions(chart, states)
    % Create transitions between states
    
    % INITIALIZING -> DISARMED
    t1 = Stateflow.Transition(chart);
    t1.Source = states.initializing;
    t1.Destination = states.disarmed;
    t1.Condition = 'after(2,sec)';
    t1.ConditionAction = '';
    
    % DISARMED -> ANGLEMODE
    t2 = Stateflow.Transition(chart);
    t2.Source = states.disarmed;
    t2.Destination = states.angle;
    t2.Condition = 'throttle > 20 && safety_switch && mode_switch';
    
    % DISARMED -> ACCRONODE  
    t3 = Stateflow.Transition(chart);
    t3.Source = states.disarmed;
    t3.Destination = states.accro;
    t3.Condition = 'throttle > 20 && safety_switch && ~mode_switch';
    
    % ANGLEMODE -> SAFETY
    t4 = Stateflow.Transition(chart);
    t4.Source = states.angle;
    t4.Destination = states.safety;
    t4.Condition = 'is_safety_needed';
    
    % ANGLEMODE -> ACCRONODE
    t5 = Stateflow.Transition(chart);
    t5.Source = states.angle;
    t5.Destination = states.accro;
    t5.Condition = '~mode_switch && throttle > 20';
    
    % ANGLEMODE -> DISARMED
    t6 = Stateflow.Transition(chart);
    t6.Source = states.angle;
    t6.Destination = states.disarmed;
    t6.Condition = 'throttle <= 20 || ~safety_switch';
    
    % ACCRONODE -> SAFETY
    t7 = Stateflow.Transition(chart);
    t7.Source = states.accro;
    t7.Destination = states.safety;
    t7.Condition = 'is_safety_needed';
    
    % ACCRONODE -> ANGLEMODE
    t8 = Stateflow.Transition(chart);
    t8.Source = states.accro;
    t8.Destination = states.angle;
    t8.Condition = 'mode_switch && throttle > 20';
    
    % ACCRONODE -> DISARMED
    t9 = Stateflow.Transition(chart);
    t9.Source = states.accro;
    t9.Destination = states.disarmed;
    t9.Condition = 'throttle <= 20 || ~safety_switch';
    
    % SAFETY -> DISARMED
    t10 = Stateflow.Transition(chart);
    t10.Source = states.safety;
    t10.Destination = states.disarmed;
    t10.Condition = 'after(1,sec)';
end

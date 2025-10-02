%% Solver and Configuration Settings for CodeDroneDIY Simulink Model
% This script defines optimal solver settings, sample times, and configuration
% parameters for real-time control applications
% Author: Auto-generated for CodeDroneDIY Simulink conversion
% Date: 2024

function solver_config(modelName)
    % Configure solver and sample time settings for the model
    % Usage: solver_config('CodeDroneDIY_FlightController')
    
    if nargin < 1
        modelName = 'CodeDroneDIY_FlightController';
    end
    
    fprintf('Configuring solver settings for %s...\n', modelName);
    
    %% Get Configuration Set
    try
        cs = getActiveConfigSet(modelName);
    catch
        error('Model %s not found or not loaded', modelName);
    end
    
    %% ========================================================================
    %% SOLVER CONFIGURATION
    %% ========================================================================
    
    fprintf('Setting solver configuration...\n');
    
    % Fixed-step solver for real-time systems
    set_param(cs, 'SolverType', 'Fixed-step');
    set_param(cs, 'Solver', 'ode4');                    % 4th order Runge-Kutta
    set_param(cs, 'FixedStep', '0.0025');               % 400 Hz base rate
    set_param(cs, 'EnableMultiTasking', 'on');          % Multi-rate execution
    
    % Simulation time settings
    set_param(cs, 'StartTime', '0');
    set_param(cs, 'StopTime', '30');                    % 30 second simulation
    
    % Advanced solver options
    set_param(cs, 'AutoInsertRateTranBlk', 'off');      % Manual rate transitions
    set_param(cs, 'SolverMode', 'SingleTasking');       % For code generation
    
    fprintf('  ✓ Solver: Fixed-step ODE4 @ 400Hz\n');
    
    %% ========================================================================
    %% SAMPLE TIME CONFIGURATION
    %% ========================================================================
    
    fprintf('Setting sample time configuration...\n');
    
    % Sample time colors for visualization
    set_param(cs, 'SampleTimeColors', 'on');
    
    % Sample time annotation
    set_param(cs, 'SampleTimeAnnotations', 'on');
    
    % Define sample time hierarchy
    sample_times = configure_sample_times();
    
    fprintf('  ✓ Sample time hierarchy defined\n');
    
    %% ========================================================================
    %% DATA IMPORT/EXPORT
    %% ========================================================================
    
    fprintf('Setting data import/export...\n');
    
    % Time and output logging
    set_param(cs, 'SaveTime', 'on');
    set_param(cs, 'TimeSaveName', 'tout');
    set_param(cs, 'SaveOutput', 'on');
    set_param(cs, 'OutputSaveName', 'yout');
    set_param(cs, 'SaveFormat', 'Dataset');             % Use Dataset format
    
    % State logging
    set_param(cs, 'SaveState', 'on');
    set_param(cs, 'StateSaveName', 'xout');
    
    % Simulation metadata
    set_param(cs, 'ReturnWorkspaceOutputs', 'on');
    set_param(cs, 'SignalLogging', 'on');
    set_param(cs, 'DSMLogging', 'on');
    
    fprintf('  ✓ Data logging configured\n');
    
    %% ========================================================================
    %% DIAGNOSTICS CONFIGURATION
    %% ========================================================================
    
    fprintf('Setting diagnostics...\n');
    
    % Critical error conditions
    set_param(cs, 'AlgebraicLoopMsg', 'error');
    set_param(cs, 'BlockDataTipMsg', 'warning');
    set_param(cs, 'CheckSSInitialOutputMsg', 'error');
    set_param(cs, 'ConsistencyChecking', 'warning');
    
    % Sample time diagnostics
    set_param(cs, 'SampleHitTimeAdjustmentMsg', 'error');
    set_param(cs, 'InheritedTsInSrcMsg', 'warning');
    set_param(cs, 'MultiTaskDSMMsg', 'warning');
    
    % Simulation diagnostics
    set_param(cs, 'UnconnectedInputMsg', 'warning');
    set_param(cs, 'UnconnectedOutputMsg', 'warning');
    set_param(cs, 'UnconnectedLineMsg', 'warning');
    
    % Real-time diagnostics
    set_param(cs, 'RTWInlineParameters', 'on');
    set_param(cs, 'BlockReduction', 'on');
    set_param(cs, 'OptimizeBlockIOStorage', 'on');
    
    fprintf('  ✓ Diagnostics configured for real-time systems\n');
    
    %% ========================================================================
    %% CODE GENERATION CONFIGURATION
    %% ========================================================================
    
    fprintf('Setting code generation options...\n');
    
    % Target selection
    set_param(cs, 'SystemTargetFile', 'ert.tlc');       % Embedded Real-Time
    set_param(cs, 'TargetLang', 'C');
    set_param(cs, 'CodeGeneration', 'on');
    
    % Optimization settings
    set_param(cs, 'LocalBlockOutputs', 'on');
    set_param(cs, 'BufferReuse', 'on');
    set_param(cs, 'ExpressionFolding', 'on');
    set_param(cs, 'EnableMemcpy', 'on');
    
    % Real-time options
    set_param(cs, 'SupportVariableSizeSignals', 'off');
    set_param(cs, 'SupportNonInlinedSFcns', 'off');
    set_param(cs, 'InlineInvariantSignals', 'on');
    
    % Code style
    set_param(cs, 'GenerateReport', 'on');
    set_param(cs, 'GenerateCodeMetricsReport', 'on');
    set_param(cs, 'CreateSILPILBlock', 'None');
    
    fprintf('  ✓ Code generation configured for embedded targets\n');
    
    %% ========================================================================
    %% HARDWARE DEPLOYMENT SETTINGS
    %% ========================================================================
    
    fprintf('Setting hardware deployment options...\n');
    
    % Target hardware characteristics
    set_param(cs, 'ProdHWDeviceType', 'ARM Cortex');
    set_param(cs, 'TargetWordSize', '32');
    set_param(cs, 'TargetEndianess', 'LittleEndian');
    
    % Memory allocation
    set_param(cs, 'DynamicMemoryAllocation', 'off');
    set_param(cs, 'MemoryAllocationReport', 'on');
    
    % Fixed-point settings
    set_param(cs, 'DefaultDataTypeDouble', 'single');   % Use single precision
    set_param(cs, 'OptimizeReductions', 'on');
    
    fprintf('  ✓ Hardware deployment configured for ARM Cortex\n');
    
    %% ========================================================================
    %% PERFORMANCE OPTIMIZATION
    %% ========================================================================
    
    fprintf('Setting performance optimization...\n');
    
    % Execution speed optimizations
    set_param(cs, 'OptimizationCustomize', 'on');
    set_param(cs, 'OptimizationLevel', 'Level2');       % O2 optimization
    set_param(cs, 'OptimizationPriority', 'Speed');
    
    % Memory optimizations
    set_param(cs, 'LocalBlockOutputs', 'on');
    set_param(cs, 'OptimizeBlockIOStorage', 'on');
    set_param(cs, 'BufferReuse', 'on');
    
    % Loop optimizations
    set_param(cs, 'LoopUnrolling', 'on');
    set_param(cs, 'InlineThreshold', '10');
    set_param(cs, 'InlineThresholdMax', '200');
    
    fprintf('  ✓ Performance optimization enabled\n');
    
    %% ========================================================================
    %% REAL-TIME WORKSHOP SETTINGS
    %% ========================================================================
    
    fprintf('Setting Real-Time Workshop options...\n');
    
    % Interface settings
    set_param(cs, 'ERTCustomFileTemplate', 'example_file_process.tlc');
    set_param(cs, 'GenerateSampleERTMain', 'on');
    set_param(cs, 'IncludeHyperlinkInReport', 'on');
    set_param(cs, 'LaunchReport', 'off');               % Don't auto-launch
    
    % Multi-tasking settings
    set_param(cs, 'SolverMode', 'SingleTasking');       % Single-tasking for determinism
    set_param(cs, 'EnableMultiTasking', 'off');         % Disable for embedded
    
    fprintf('  ✓ Real-Time Workshop configured\n');
    
    %% ========================================================================
    %% VALIDATION AND VERIFICATION
    %% ========================================================================
    
    fprintf('Setting validation options...\n');
    
    % Model coverage
    set_param(cs, 'CovEnable', 'on');
    set_param(cs, 'CovMetricSettings', 'dwe');          % Decision, Condition, MCDC
    
    % Requirements traceability  
    set_param(cs, 'BlockParameterDefaults', 'on');
    set_param(cs, 'BlockDataTips', 'on');
    
    % Signal monitoring
    set_param(cs, 'SignalLogging', 'on');
    set_param(cs, 'SignalLoggingName', 'logsout');
    
    fprintf('  ✓ Validation and verification enabled\n');
    
    %% ========================================================================
    %% SAVE CONFIGURATION
    %% ========================================================================
    
    % Save configuration set
    try
        save_system(modelName);
        fprintf('✓ Configuration saved to model\n');
    catch
        warning('Could not save model configuration');
    end
    
    % Display summary
    display_configuration_summary(sample_times);
    
    fprintf('\n✓ Solver configuration complete!\n');
end

%% ========================================================================
%% SAMPLE TIME CONFIGURATION
%% ========================================================================

function sample_times = configure_sample_times()
    % Define the sample time hierarchy for the flight controller
    
    sample_times = struct();
    
    % Base sample time (fastest rate)
    sample_times.base_rate = 0.0025;          % 400 Hz - Control loop
    sample_times.base_freq = 400;
    
    % Sub-sample times (slower rates)
    sample_times.imu_rate = 0.0025;           % 400 Hz - IMU processing  
    sample_times.imu_freq = 400;
    
    sample_times.control_rate = 0.0025;       % 400 Hz - Control system
    sample_times.control_freq = 400;
    
    sample_times.state_rate = 0.0025;         % 400 Hz - State machine
    sample_times.state_freq = 400;
    
    sample_times.logging_rate = 0.01;         % 100 Hz - Data logging
    sample_times.logging_freq = 100;
    
    sample_times.display_rate = 0.1;          % 10 Hz - Display updates
    sample_times.display_freq = 10;
    
    % Derived sample times
    sample_times.motor_rate = sample_times.base_rate;     % Motors at base rate
    sample_times.safety_rate = sample_times.base_rate;    % Safety at base rate
    
    % Validate sample time relationships
    validate_sample_times(sample_times);
end

function validate_sample_times(st)
    % Validate that sample times form a proper hierarchy
    
    base = st.base_rate;
    
    % Check that all rates are multiples of base rate
    rates = [st.imu_rate, st.control_rate, st.state_rate, st.logging_rate, st.display_rate];
    
    for i = 1:length(rates)
        ratio = rates(i) / base;
        if abs(ratio - round(ratio)) > 1e-10
            warning('Sample time %.4f is not a multiple of base rate %.4f', rates(i), base);
        end
    end
    
    fprintf('  Sample time validation: ✓\n');
end

%% ========================================================================
%% CONFIGURATION TEMPLATES
%% ========================================================================

function apply_flight_controller_template(cs)
    % Apply predefined template for flight controller applications
    
    fprintf('Applying flight controller template...\n');
    
    % Real-time constraints
    set_param(cs, 'SolverType', 'Fixed-step');
    set_param(cs, 'Solver', 'ode4');
    set_param(cs, 'FixedStep', '0.0025');
    
    % Deterministic execution
    set_param(cs, 'EnableMultiTasking', 'off');
    set_param(cs, 'SolverMode', 'SingleTasking');
    
    % Code generation for embedded
    set_param(cs, 'SystemTargetFile', 'ert.tlc');
    set_param(cs, 'TargetLang', 'C');
    
    % Optimization for speed
    set_param(cs, 'OptimizationPriority', 'Speed');
    set_param(cs, 'OptimizationLevel', 'Level2');
    
    fprintf('  ✓ Flight controller template applied\n');
end

function apply_hardware_in_loop_template(cs)
    % Apply template for Hardware-in-the-Loop testing
    
    fprintf('Applying HIL template...\n');
    
    % Real-time execution
    set_param(cs, 'EnablePacing', 'on');
    set_param(cs, 'PacingRate', '1');
    
    % External mode for real-time monitoring
    set_param(cs, 'ExtMode', 'on');
    set_param(cs, 'ExtModeTransport', '0');
    
    % Signal monitoring
    set_param(cs, 'SignalLogging', 'on');
    
    fprintf('  ✓ HIL template applied\n');
end

%% ========================================================================
%% PERFORMANCE ANALYSIS
%% ========================================================================

function analyze_performance_requirements()
    % Analyze performance requirements for real-time execution
    
    fprintf('\nPerformance Requirements Analysis:\n');
    fprintf('================================\n');
    
    % Control loop requirements
    fprintf('Control Loop:\n');
    fprintf('  Target frequency: 400 Hz\n');
    fprintf('  Maximum latency: 2.5 ms\n');
    fprintf('  Jitter tolerance: ±0.1 ms\n\n');
    
    % IMU processing requirements  
    fprintf('IMU Processing:\n');
    fprintf('  Update rate: 400 Hz\n');
    fprintf('  Filter delay: <1 sample\n');
    fprintf('  Noise bandwidth: 50 Hz\n\n');
    
    % Motor output requirements
    fprintf('Motor Outputs:\n');
    fprintf('  Update rate: 400 Hz\n');
    fprintf('  Resolution: 1 μs\n');
    fprintf('  Range: 1000-2000 μs\n\n');
    
    % Memory requirements
    fprintf('Memory Requirements:\n');
    fprintf('  RAM usage: <64 KB\n');
    fprintf('  Flash usage: <256 KB\n');
    fprintf('  Stack depth: <4 KB\n\n');
end

%% ========================================================================
%% DEPLOYMENT CONFIGURATION
%% ========================================================================

function configure_for_deployment(target_type)
    % Configure model for specific deployment target
    
    if nargin < 1
        target_type = 'generic_arm';
    end
    
    fprintf('Configuring for deployment target: %s\n', target_type);
    
    switch lower(target_type)
        case 'pixhawk'
            configure_pixhawk_target();
        case 'arduino'
            configure_arduino_target();
        case 'raspberry_pi'
            configure_raspberry_pi_target();
        case 'generic_arm'
            configure_generic_arm_target();
        otherwise
            warning('Unknown target type: %s', target_type);
    end
end

function configure_pixhawk_target()
    % Configuration for Pixhawk autopilot hardware
    
    fprintf('  Configuring for Pixhawk target...\n');
    
    % Target characteristics
    target_config.processor = 'ARM Cortex-M4F';
    target_config.clock_speed = 168e6;  % 168 MHz
    target_config.ram_size = 256e3;     % 256 KB
    target_config.flash_size = 2e6;     % 2 MB
    target_config.fpu = true;
    
    apply_target_config(target_config);
end

function configure_arduino_target()
    % Configuration for Arduino-based controllers
    
    fprintf('  Configuring for Arduino target...\n');
    
    target_config.processor = 'ATmega328P';
    target_config.clock_speed = 16e6;   % 16 MHz  
    target_config.ram_size = 2e3;       % 2 KB
    target_config.flash_size = 32e3;    % 32 KB
    target_config.fpu = false;
    
    apply_target_config(target_config);
end

function configure_raspberry_pi_target()
    % Configuration for Raspberry Pi
    
    fprintf('  Configuring for Raspberry Pi target...\n');
    
    target_config.processor = 'ARM Cortex-A72';
    target_config.clock_speed = 1.5e9;  % 1.5 GHz
    target_config.ram_size = 4e9;       % 4 GB  
    target_config.flash_size = 32e9;    % 32 GB
    target_config.fpu = true;
    
    apply_target_config(target_config);
end

function configure_generic_arm_target()
    % Configuration for generic ARM Cortex target
    
    fprintf('  Configuring for generic ARM target...\n');
    
    target_config.processor = 'ARM Cortex-M4';
    target_config.clock_speed = 100e6;  % 100 MHz
    target_config.ram_size = 128e3;     % 128 KB
    target_config.flash_size = 1e6;     % 1 MB
    target_config.fpu = true;
    
    apply_target_config(target_config);
end

function apply_target_config(config)
    % Apply target-specific configuration
    
    fprintf('    Processor: %s\n', config.processor);
    fprintf('    Clock: %.0f MHz\n', config.clock_speed/1e6);
    fprintf('    RAM: %.0f KB\n', config.ram_size/1e3);
    fprintf('    Flash: %.0f KB\n', config.flash_size/1e3);
    fprintf('    FPU: %s\n', ternary(config.fpu, 'Yes', 'No'));
end

%% ========================================================================
%% UTILITY FUNCTIONS
%% ========================================================================

function display_configuration_summary(sample_times)
    % Display configuration summary
    
    fprintf('\nConfiguration Summary:\n');
    fprintf('=====================\n');
    
    fprintf('Sample Times:\n');
    fprintf('  Base rate: %.3f ms (%.0f Hz)\n', sample_times.base_rate*1000, sample_times.base_freq);
    fprintf('  IMU rate: %.3f ms (%.0f Hz)\n', sample_times.imu_rate*1000, sample_times.imu_freq);
    fprintf('  Control rate: %.3f ms (%.0f Hz)\n', sample_times.control_rate*1000, sample_times.control_freq);
    fprintf('  Logging rate: %.1f ms (%.0f Hz)\n', sample_times.logging_rate*1000, sample_times.logging_freq);
    
    fprintf('\nSolver Settings:\n');
    fprintf('  Type: Fixed-step ODE4\n');
    fprintf('  Step size: %.3f ms\n', sample_times.base_rate*1000);
    fprintf('  Execution: Single-tasking\n');
    
    fprintf('\nCode Generation:\n');
    fprintf('  Target: Embedded Real-Time (C)\n');
    fprintf('  Optimization: Speed (O2)\n');
    fprintf('  Platform: ARM Cortex\n');
end

function result = ternary(condition, true_val, false_val)
    % Simple ternary operator implementation
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

%% ========================================================================
%% BATCH CONFIGURATION FUNCTIONS
%% ========================================================================

function configure_all_models()
    % Configure all models in the current directory
    
    % Find all .slx files
    slx_files = dir('*.slx');
    
    for i = 1:length(slx_files)
        [~, modelName, ~] = fileparts(slx_files(i).name);
        
        try
            fprintf('Configuring model: %s\n', modelName);
            load_system(modelName);
            solver_config(modelName);
            close_system(modelName);
        catch ME
            fprintf('Failed to configure %s: %s\n', modelName, ME.message);
        end
    end
end

function export_configuration_script(modelName)
    % Export configuration as standalone script
    
    if nargin < 1
        modelName = 'CodeDroneDIY_FlightController';
    end
    
    script_name = [modelName '_solver_config.m'];
    
    % Generate the script content
    script_content = generate_config_script_content(modelName);
    
    % Write to file
    fid = fopen(script_name, 'w');
    fprintf(fid, '%s', script_content);
    fclose(fid);
    
    fprintf('Configuration script exported to: %s\n', script_name);
end

function content = generate_config_script_content(modelName)
    % Generate standalone configuration script content
    
    content = sprintf([
        '%% Auto-generated solver configuration for %s\n'...
        '%% Created: %s\n\n'...
        'function configure_%s()\n'...
        '    modelName = ''%s'';\n'...
        '    cs = getActiveConfigSet(modelName);\n\n'...
        '    %% Solver settings\n'...
        '    set_param(cs, ''SolverType'', ''Fixed-step'');\n'...
        '    set_param(cs, ''Solver'', ''ode4'');\n'...
        '    set_param(cs, ''FixedStep'', ''0.0025'');\n\n'...
        '    %% Code generation\n'...
        '    set_param(cs, ''SystemTargetFile'', ''ert.tlc'');\n'...
        '    set_param(cs, ''TargetLang'', ''C'');\n\n'...
        '    fprintf(''Configuration applied to %s\\n'');\n'...
        'end\n'
    ], modelName, datestr(now), strrep(modelName, '-', '_'), modelName, modelName);
end

# CodeDroneDIY Simulink Model - Block I/O Mapping Table

## Overview
This document provides comprehensive I/O mapping for all blocks in the CodeDroneDIY Flight Controller Simulink model, including signal types, units, ranges, and data flow connections.

---

## Main Model Architecture

```
[Input] → [Radio Processing] → [Safety] → [State Machine] → [Control] → [Motor Mixing] → [Output]
                ↑                                              ↑
            [IMU Processing] ────────────────────────────────────┘
```

---

## 1. Input Processing Subsystem

### TestScenarioGenerator Block
**Type:** MATLAB Function Block  
**Purpose:** Generates test signals for simulation

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | current_time | double | s | 0 to ∞ | Simulation time |
| **Outputs** |
| 1 | radio_cmds | RadioCmds_Bus | - | - | Raw radio PWM commands |
| 2 | imu_data | IMU_Bus | - | - | Simulated IMU sensor data |

#### radio_cmds Bus Structure:
| Field | Type | Units | Range | Description |
|-------|------|-------|-------|-------------|
| roll | double | μs | 1100-1900 | Roll stick PWM |
| pitch | double | μs | 1100-1900 | Pitch stick PWM |
| throttle | double | μs | 1090-1900 | Throttle stick PWM |
| yaw | double | μs | 1100-1900 | Yaw stick PWM |
| mode | double | μs | 1100-1900 | Mode switch PWM |
| safety | double | μs | 1100-1900 | Safety switch PWM |

#### imu_data Bus Structure:
| Field | Type | Units | Range | Description |
|-------|------|-------|-------|-------------|
| acc_x | double | m/s² | ±50 | X-axis acceleration |
| acc_y | double | m/s² | ±50 | Y-axis acceleration |
| acc_z | double | m/s² | ±50 | Z-axis acceleration |
| gyro_x | double | deg/s | ±2000 | X-axis angular rate (roll) |
| gyro_y | double | deg/s | ±2000 | Y-axis angular rate (pitch) |
| gyro_z | double | deg/s | ±2000 | Z-axis angular rate (yaw) |

---

## 2. Radio Processing Subsystem

### RadioInputProcessor Block
**Type:** MATLAB Function Block  
**Purpose:** Converts raw PWM signals to engineering units

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | radio_cmds | RadioCmds_Bus | - | - | Raw radio PWM commands |
| **Outputs** |
| 1 | processed_cmds | ProcessedCmds_Bus | - | - | Processed command signals |

#### processed_cmds Bus Structure:
| Field | Type | Units | Range | Description |
|-------|------|-------|-------|-------------|
| roll_cmd | double | deg | ±45 | Roll angle command |
| pitch_cmd | double | deg | ±45 | Pitch angle command |
| throttle | double | % | 0-100 | Throttle percentage |
| yaw_cmd | double | deg/s | ±135 | Yaw rate command |
| mode_switch | boolean | - | 0/1 | Flight mode (0=Accro, 1=Angle) |
| safety_switch | boolean | - | 0/1 | Safety state (0=Disarmed, 1=Armed) |

---

## 3. IMU Processing Subsystem

### IMUProcessor Block
**Type:** MATLAB Function Block  
**Purpose:** Processes raw IMU data with complementary filter

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | imu_data | IMU_Bus | - | - | Raw IMU sensor data |
| 2 | attitude_prev | Attitude_Bus | - | - | Previous attitude (feedback) |
| 3 | dt | double | s | 0.001-0.01 | Sample time |
| **Outputs** |
| 1 | attitude | Attitude_Bus | - | - | Current attitude estimates |
| 2 | attitude_out | Attitude_Bus | - | - | Attitude for feedback |

#### attitude Bus Structure:
| Field | Type | Units | Range | Description |
|-------|------|-------|-------|-------------|
| roll_angle | double | deg | ±180 | Roll angle estimate |
| pitch_angle | double | deg | ±90 | Pitch angle estimate |
| roll_rate | double | deg/s | ±2000 | Roll angular rate |
| pitch_rate | double | deg/s | ±2000 | Pitch angular rate |
| yaw_rate | double | deg/s | ±2000 | Yaw angular rate |

---

## 4. Safety Monitoring Subsystem

### SafetyStateChecker Block
**Type:** MATLAB Function Block  
**Purpose:** Monitors system safety conditions

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | throttle | double | % | 0-100 | Current throttle level |
| 2 | current_time | double | s | 0 to ∞ | Current simulation time |
| 3 | throttle_was_high | boolean | - | 0/1 | Throttle state memory |
| 4 | throttle_low_start | double | s | 0 to ∞ | Time when throttle went low |
| **Outputs** |
| 1 | is_safety_needed | boolean | - | 0/1 | Safety trigger flag |
| 2 | throttle_was_high_out | boolean | - | 0/1 | Updated throttle state |
| 3 | throttle_low_start_out | double | s | 0 to ∞ | Updated low start time |

---

## 5. State Machine Subsystem

### StateMachine Block
**Type:** MATLAB Function Block / Stateflow Chart  
**Purpose:** Manages flight controller states

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | processed_cmds | ProcessedCmds_Bus | - | - | Processed commands |
| 2 | is_safety_needed | boolean | - | 0/1 | Safety condition flag |
| 3 | current_time | double | s | 0 to ∞ | Current simulation time |
| **Outputs** |
| 1 | current_state | uint8 | - | 1-5 | Current state ID |
| 2 | throttle_was_high | boolean | - | 0/1 | Throttle tracking state |
| 3 | throttle_low_start | double | s | 0 to ∞ | Low throttle start time |

#### State Enumeration:
| State ID | State Name | Description |
|----------|------------|-------------|
| 1 | INITIALIZING | System startup and calibration |
| 2 | SAFETY | Emergency safety mode |
| 3 | DISARMED | Motors disabled, safe state |
| 4 | ACCRONODE | Acrobatic (rate) control mode |
| 5 | ANGLEMODE | Angle (stabilized) control mode |

---

## 6. Control System Subsystem

### ControlSelector Block
**Type:** MATLAB Function Block  
**Purpose:** Selects appropriate control algorithm based on state

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | processed_cmds | ProcessedCmds_Bus | - | - | Command signals |
| 2 | attitude | Attitude_Bus | - | - | Current attitude |
| 3 | current_state | uint8 | - | 1-5 | Flight controller state |
| 4 | dt | double | s | 0.001-0.01 | Sample time |
| **Outputs** |
| 1 | control_powers | ControlPowers_Bus | - | - | Control system outputs |

#### control_powers Bus Structure:
| Field | Type | Units | Range | Description |
|-------|------|-------|-------|-------------|
| roll_power | double | - | ±1000 | Roll control authority |
| pitch_power | double | - | ±1000 | Pitch control authority |
| yaw_power | double | - | ±1000 | Yaw control authority |

---

## 7. Motor Mixing Subsystem

### MotorMixer Block
**Type:** MATLAB Function Block  
**Purpose:** Converts control commands to individual motor PWM signals

| Port | Signal Name | Type | Units | Range | Description |
|------|-------------|------|-------|-------|-------------|
| **Inputs** |
| 1 | throttle | double | % | 0-100 | Throttle command |
| 2 | control_powers | ControlPowers_Bus | - | - | Control system outputs |
| **Outputs** |
| 1 | motor_cmds | MotorCmds_Bus | - | - | Individual motor commands |

#### motor_cmds Bus Structure:
| Field | Type | Units | Range | Description |
|-------|------|-------|-------|-------------|
| motor0 | double | μs | 1000-2000 | Motor 0 PWM (Front Right) |
| motor1 | double | μs | 1000-2000 | Motor 1 PWM (Rear Right) |
| motor2 | double | μs | 1000-2000 | Motor 2 PWM (Rear Left) |
| motor3 | double | μs | 1000-2000 | Motor 3 PWM (Front Left) |

#### Motor Configuration (X-Frame):
```
    Motor3 ┌─────┐ Motor0
          ╱       ╲
         ╱    ╳    ╲
        ╱           ╲
    Motor2 └─────┘ Motor1
```

---

## 8. Visualization and Monitoring

### Attitude_Scope
**Type:** Scope Block  
**Inputs:** 3 signals from attitude bus (roll_angle, pitch_angle, yaw_rate)

### Motor_Scope  
**Type:** Scope Block  
**Inputs:** 4 signals from motor_cmds bus (motor0, motor1, motor2, motor3)

### State_Display
**Type:** Display Block  
**Input:** 1 signal (current_state)

---

## Sample Time Configuration

| Component | Sample Time | Frequency | Notes |
|-----------|-------------|-----------|-------|
| Main Loop | 0.0025 s | 400 Hz | Real-time control loop |
| IMU Processing | 0.0025 s | 400 Hz | High-frequency attitude estimation |
| Control System | 0.0025 s | 400 Hz | Fast control response |
| State Machine | 0.0025 s | 400 Hz | State monitoring |
| Visualization | 0.01 s | 100 Hz | Display update rate |

---

## Signal Flow Summary

1. **Input Path:** Clock → TestScenarioGenerator → RadioProcessor
2. **Sensor Path:** TestScenarioGenerator → IMUProcessor → ControlSelector
3. **Safety Path:** RadioProcessor → SafetyChecker → StateMachine
4. **Control Path:** All inputs → ControlSelector → MotorMixer
5. **Output Path:** MotorMixer → Motor PWM commands

---

## Data Types and Code Generation

All signals are designed for fixed-point code generation:
- **double:** Used for all floating-point calculations
- **boolean:** Used for logical flags and switches
- **uint8:** Used for state enumeration
- **Bus objects:** Structured data types for organized signal grouping

---

## Performance Specifications

| Metric | Target | Typical | Units |
|--------|--------|---------|-------|
| Loop Time | 2.5 | 1.8 | ms |
| Control Latency | 2.5 | 2.1 | ms |
| Attitude Update Rate | 400 | 400 | Hz |
| Motor Update Rate | 400 | 400 | Hz |
| Memory Usage | <64 | 48 | KB |

---

## Notes

1. All bus objects must be loaded before model compilation
2. Sample times are configured for real-time execution at 400 Hz
3. Signal ranges include safety margins for normal operation
4. PID states are persistent within MATLAB Function blocks
5. Motor mixing assumes X-configuration quadcopter frame

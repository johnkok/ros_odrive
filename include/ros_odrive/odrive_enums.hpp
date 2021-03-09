#ifndef ODRIVE_ENUMS_HPP_
#define ODRIVE_ENUMS_HPP_

// ODrive.Can.Protocol
#define PROTOCOL_SIMPLE                           0

// ODrive.Axis.AxisState
#define AXIS_STATE_UNDEFINED                      0
#define AXIS_STATE_IDLE                           1
#define AXIS_STATE_STARTUP_SEQUENCE               2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE      3
#define AXIS_STATE_MOTOR_CALIBRATION              4
#define AXIS_STATE_SENSORLESS_CONTROL             5
#define AXIS_STATE_ENCODER_INDEX_SEARCH           6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION     7
#define AXIS_STATE_CLOSED_LOOP_CONTROL            8
#define AXIS_STATE_LOCKIN_SPIN                    9
#define AXIS_STATE_ENCODER_DIR_FIND               10
#define AXIS_STATE_HOMING                         11

// ODrive.ThermistorCurrentLimiter.Error
#define THERMISTOR_CURRENT_LIMITER_ERROR_NONE     0x00000000
#define THERMISTOR_CURRENT_LIMITER_ERROR_OVER_TEMP  0x00000001

// ODrive.Encoder.Mode
#define ENCODER_MODE_INCREMENTAL                  0
#define ENCODER_MODE_HALL                         1
#define ENCODER_MODE_SINCOS                       2
#define ENCODER_MODE_SPI_ABS_CUI                  256
#define ENCODER_MODE_SPI_ABS_AMS                  257
#define ENCODER_MODE_SPI_ABS_AEAT                 258

// ODrive.Controller.ControlMode
#define CONTROL_MODE_VOLTAGE_CONTROL              0
#define CONTROL_MODE_TORQUE_CONTROL               1
#define CONTROL_MODE_VELOCITY_CONTROL             2
#define CONTROL_MODE_POSITION_CONTROL             3

// ODrive.Controller.InputMode
#define INPUT_MODE_INACTIVE                       0
#define INPUT_MODE_PASSTHROUGH                    1
#define INPUT_MODE_VEL_RAMP                       2
#define INPUT_MODE_POS_FILTER                     3
#define INPUT_MODE_MIX_CHANNELS                   4
#define INPUT_MODE_TRAP_TRAJ                      5
#define INPUT_MODE_TORQUE_RAMP                    6
#define INPUT_MODE_MIRROR                         7

// ODrive.Motor.MotorType
#define MOTOR_TYPE_HIGH_CURRENT                   0
#define MOTOR_TYPE_GIMBAL                         2
#define MOTOR_TYPE_ACIM                           3

// ODrive.Motor.MotorType
#define MOTOR_TYPE_HIGH_CURRENT                   0
#define MOTOR_TYPE_GIMBAL                         2
#define MOTOR_TYPE_ACIM                           3

// ODrive.Can.Error
#define CAN_ERROR_NONE                            0x00000000
#define CAN_ERROR_DUPLICATE_CAN_IDS               0x00000001

// ODrive.Axis.Error
#define AXIS_ERROR_NONE                           0x00000000
#define AXIS_ERROR_INVALID_STATE                  0x00000001
#define AXIS_ERROR_DC_BUS_UNDER_VOLTAGE           0x00000002
#define AXIS_ERROR_DC_BUS_OVER_VOLTAGE            0x00000004
#define AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT    0x00000008
#define AXIS_ERROR_BRAKE_RESISTOR_DISARMED        0x00000010
#define AXIS_ERROR_MOTOR_DISARMED                 0x00000020
#define AXIS_ERROR_MOTOR_FAILED                   0x00000040
#define AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED    0x00000080
#define AXIS_ERROR_ENCODER_FAILED                 0x00000100
#define AXIS_ERROR_CONTROLLER_FAILED              0x00000200
#define AXIS_ERROR_POS_CTRL_DURING_SENSORLESS     0x00000400
#define AXIS_ERROR_WATCHDOG_TIMER_EXPIRED         0x00000800
#define AXIS_ERROR_MIN_ENDSTOP_PRESSED            0x00001000
#define AXIS_ERROR_MAX_ENDSTOP_PRESSED            0x00002000
#define AXIS_ERROR_ESTOP_REQUESTED                0x00004000
#define AXIS_ERROR_HOMING_WITHOUT_ENDSTOP         0x00020000
#define AXIS_ERROR_OVER_TEMP                      0x00040000

// ODrive.Axis.LockinState
#define LOCKIN_STATE_INACTIVE                     0
#define LOCKIN_STATE_RAMP                         1
#define LOCKIN_STATE_ACCELERATE                   2
#define LOCKIN_STATE_CONST_VEL                    3

// ODrive.Motor.Error
#define MOTOR_ERROR_NONE                          0x00000000
#define MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE  0x00000001
#define MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE  0x00000002
#define MOTOR_ERROR_ADC_FAILED                    0x00000004
#define MOTOR_ERROR_DRV_FAULT                     0x00000008
#define MOTOR_ERROR_CONTROL_DEADLINE_MISSED       0x00000010
#define MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE    0x00000020
#define MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE    0x00000040
#define MOTOR_ERROR_MODULATION_MAGNITUDE          0x00000080
#define MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION      0x00000100
#define MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK     0x00000200
#define MOTOR_ERROR_CURRENT_SENSE_SATURATION      0x00000400
#define MOTOR_ERROR_CURRENT_LIMIT_VIOLATION       0x00001000
#define MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN          0x00002000
#define MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT     0x00004000
#define MOTOR_ERROR_DC_BUS_OVER_CURRENT           0x00008000

// ODrive.Motor.ArmedState
#define ARMED_STATE_DISARMED                      0
#define ARMED_STATE_WAITING_FOR_TIMINGS           1
#define ARMED_STATE_WAITING_FOR_UPDATE            2
#define ARMED_STATE_ARMED                         3

// ODrive.Motor.GateDriver.DrvFault
#define DRV_FAULT_NO_FAULT                        0x00000000
#define DRV_FAULT_FET_LOW_C_OVERCURRENT           0x00000001
#define DRV_FAULT_FET_HIGH_C_OVERCURRENT          0x00000002
#define DRV_FAULT_FET_LOW_B_OVERCURRENT           0x00000004
#define DRV_FAULT_FET_HIGH_B_OVERCURRENT          0x00000008
#define DRV_FAULT_FET_LOW_A_OVERCURRENT           0x00000010
#define DRV_FAULT_FET_HIGH_A_OVERCURRENT          0x00000020
#define DRV_FAULT_OVERTEMPERATURE_WARNING         0x00000040
#define DRV_FAULT_OVERTEMPERATURE_SHUTDOWN        0x00000080
#define DRV_FAULT_P_VDD_UNDERVOLTAGE              0x00000100
#define DRV_FAULT_G_VDD_UNDERVOLTAGE              0x00000200
#define DRV_FAULT_G_VDD_OVERVOLTAGE               0x00000400

// ODrive.Controller.Error
#define CONTROLLER_ERROR_NONE                     0x00000000
#define CONTROLLER_ERROR_OVERSPEED                0x00000001
#define CONTROLLER_ERROR_INVALID_INPUT_MODE       0x00000002
#define CONTROLLER_ERROR_UNSTABLE_GAIN            0x00000004
#define CONTROLLER_ERROR_INVALID_MIRROR_AXIS      0x00000008
#define CONTROLLER_ERROR_INVALID_LOAD_ENCODER     0x00000010
#define CONTROLLER_ERROR_INVALID_ESTIMATE         0x00000020

// ODrive.Encoder.Error
#define ENCODER_ERROR_NONE                        0x00000000
#define ENCODER_ERROR_UNSTABLE_GAIN               0x00000001
#define ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH      0x00000002
#define ENCODER_ERROR_NO_RESPONSE                 0x00000004
#define ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE    0x00000008
#define ENCODER_ERROR_ILLEGAL_HALL_STATE          0x00000010
#define ENCODER_ERROR_INDEX_NOT_FOUND_YET         0x00000020
#define ENCODER_ERROR_ABS_SPI_TIMEOUT             0x00000040
#define ENCODER_ERROR_ABS_SPI_COM_FAIL            0x00000080
#define ENCODER_ERROR_ABS_SPI_NOT_READY           0x00000100

// ODrive.SensorlessEstimator.Error
#define SENSORLESS_ESTIMATOR_ERROR_NONE           0x00000000
#define SENSORLESS_ESTIMATOR_ERROR_UNSTABLE_GAIN  0x00000001

#endif

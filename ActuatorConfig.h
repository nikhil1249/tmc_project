#ifndef ACTUATORCONFIG_H
#define ACTUATORCONFIG_H

/*
 * Single actuator configuration file.
 * Keep all project-level enable/disable macros here to avoid duplicated values.
 */

#define TMC6460_ENABLE_TORQUE_MODE                         0
#define TMC6460_ENABLE_POSITION_MODE                       0

#define DEFAULT_VELOCITY_TORQUE_LIMIT_RAW                  1800
#define MIN_VELOCITY_TORQUE_LIMIT_RAW                      300
#define MAX_VELOCITY_TORQUE_LIMIT_RAW                      2000

#define DEFAULT_AUTO_TORQUE_LEARN_VELOCITY_RAW             4000000
#define TMC6460_ENABLE_AUTO_TORQUE_LEARN_ON_INIT           0
#define TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION         1
#define TMC6460_CALIBRATION_FILE_NAME                      "tmc6460_calibration.ini"

/*
 * Hold torque estimate:
 * learned hold limit = abs(measured hold torque) + margin,
 * clamped to MIN/MAX_VELOCITY_TORQUE_LIMIT_RAW.
 */
#define AUTO_TORQUE_HOLD_MARGIN_RAW                        300

/*
 * End detection stays velocity-only and uses POSITION_ACTUAL only as feedback.
 * It does not use position target / position mode.
 */
#define TMC6460_USE_VELOCITY_ZERO_HOLD_AFTER_END           1
#define TMC6460_ENABLE_ACTUATOR_STOP_DETECTION             1
#define TMC6460_ENABLE_FINAL_END_CREEP                     1
#define TMC6460_CALIBRATION_LOG_EVERY_SAMPLE               1

#endif // ACTUATORCONFIG_H

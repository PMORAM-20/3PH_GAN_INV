/*
 * control_defines.h
 *
 * @file    control_defines.h
 * @brief   Header file defining constant values for control schemes, modulation
 *          types, control modes, speed profiles, and data logging modes for a
 *          three-phase GaN inverter using the F28379D DSP.
 * @author  [Your Name]
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Defines constant values (e.g., FOC = 0x1) used in configuration selections
 *          in control_config.h. No configuration parameters or selection macros
 *          are defined here.
 *
 * @note    Selection macros (e.g., CONTROL_SEL) and parameters are defined in
 *          control_config.h and system_config.h.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_COMMON_CONTROL_DEFINES_H_
#define INCLUDE_COMMON_CONTROL_DEFINES_H_

/* System States */

/** @brief System off state */
#define SYSTEM_OFF 0x0

/** @brief System on state */
#define SYSTEM_ON  0x1

/* Control Modes */

/** @brief Speed control mode */
#define SPEED_CTRL   0x0

/** @brief Position control mode */
#define POS_CTRL     0x1

/** @brief Torque control mode */
#define TORQUE_CTRL  0x2

/* Control Schemes */

/** @brief Model-Free Deadbeat Control */
#define MFDC         0x0

/** @brief Field-Oriented Control */
#define FOC          0x1

/** @brief Model Predictive Control */
#define MPC          0x2

/* Modulation Types */

/** @brief Finite Control Set modulation */
#define FCS          0x0

/** @brief Space Vector Modulation */
#define SVM          0x1

/** @brief Sinusoidal PWM (Carrier-Based) */
#define SPWM         0x2

/* Speed Profiles */

/** @brief Trapezoidal speed profile */
#define TRAPEZOIDAL  0x0

/** @brief Dataset-driven speed profile */
#define DATASET      0x1

/* Data Logging Modes */

/** @brief Default data logging mode */
#define DATALOG_DEFAULT  0x0

/** @brief Extended data logging mode */
#define DATALOG_EXTENDED 0x1

#endif /* INCLUDE_COMMON_CONTROL_DEFINES_H_ */
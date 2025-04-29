/*
 * system_config.h
 *
 * @file    system_config.h
 * @brief   Header file defining system and hardware parameters for a three-phase
 *          GaN inverter using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Defines parameters for the motor, PWM, and system (e.g., DC bus voltage).
 *          Configurations are independent of control selections defined in
 *          control_config.h.
 *
 * @note    Include this file in modules requiring system parameters.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_CONFIG_SYSTEM_CONFIG_H_
#define INCLUDE_CONFIG_SYSTEM_CONFIG_H_

#include <stdint.h>

/*====================== SYSTEM PARAMETERS ======================*/

/**
 * @brief DC bus voltage (V).
 */
#define DC_BUS_VOLTAGE 60.0f

/**
 * @brief Control loop frequency (Hz).
 */
#define CONTROL_FREQUENCY_HZ 5000.0f

/**
 * @brief PWM frequency for GaN semiconductors (Hz).
 */
#define PWM_FREQUENCY_HZ 50000

/*====================== PWM CONFIGURATION ======================*/

/**
 * @brief Dead time for PWM signals (cycles, based on 200 MHz CPU clock).
 * @details Calculated for 100 ns dead time at 50 kHz PWM for GaN compatibility.
 */
#define EPWM_DEADBAND_CYCLES (uint16_t)(200000000.0f / PWM_FREQUENCY_HZ * 0.0000001f)

/*====================== ELECTRICAL MACHINE PARAMETERS ======================*/

/**
 * @brief Stator resistance (Ohm).
 */
#define RS 1.0f

/**
 * @brief D-axis inductance (H).
 */
#define LD 0.0711f

/**
 * @brief Q-axis inductance (H).
 */
#define LQ 0.0711f

/**
 * @brief Rotor flux linkage (Wb).
 */
#define LAMBDA 0.85f

/**
 * @brief Number of pole pairs.
 */
#define P 3.0f

#endif /* INCLUDE_CONFIG_SYSTEM_CONFIG_H_ */

/*
 * foc_control.h
 *
 * @file    foc_control.h
 * @brief   Header file defining data structures and global variables specific
 *          to Field-Oriented Control (FOC) for a three-phase GaN inverter.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Defines structures and extern declarations for FOC-specific data,
 *          including voltage references and controller states.
 *
 * @note    Include this file only in FOC-related modules.
 *          Requires control_config.h for parameters.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_CONTROL_FOC_CONTROL_H_
#define INCLUDE_CONTROL_FOC_CONTROL_H_

#include <stdint.h>
#include "common/data_types.h"
#include "config/control_config.h"

/* Extern Variable Declarations */

/**
 * @brief Global PI controller for d-axis current.
 */
extern PiController g_d_current_controller;

/**
 * @brief Global PI controller for q-axis current.
 */
extern PiController g_q_current_controller;

/**
 * @brief Global reference voltages in d-q frame.
 */
extern ParkData g_dqxy_volt_ref;

/**
 * @brief Global per-unit phase voltages.
 */
extern PhaseData g_phase_volt_pu;

/**
 * @brief Global saturated voltages in d-q frame.
 */
extern ParkData g_dqxy_volt_sat;

/**
 * @brief Global reference voltages in alpha-beta frame.
 */
extern ClarkeData g_abxy_volt_ref;

/**
 * @brief Global reference phase voltages.
 */
extern PhaseData g_phase_volt_ref;

/**
 * @brief Global maximum phase voltage.
 */
extern float g_max_phase_voltage;

/* Function Prototypes */

/**
 * @brief Initializes FOC-specific data structures.
 */
void foc_types_init(void);

#endif /* INCLUDE_CONTROL_FOC_CONTROL_H_ */

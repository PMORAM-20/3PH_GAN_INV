/*
 * control_config.h
 *
 * @file    control_config.h
 * @brief   Header file defining control selections and parameters for a three-phase
 *          GaN inverter using the F28379D DSP.
 * @author  [Your Name]
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Defines selection macros (e.g., control scheme, modulation) and control
 *          parameters (limits, gains, speed profiles, logging). Configurations are
 *          conditionally defined based on selections.
 *
 * @note    Include this file in modules requiring control configurations.
 *          Requires control_defines.h for constant values and system_config.h for
 *          system parameters.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_CONFIG_CONTROL_CONFIG_H_
#define INCLUDE_CONFIG_CONTROL_CONFIG_H_

#include <math.h>
#include "common/control_defines.h"
#include "config/system_config.h"

/*====================== CORE CONTROL SELECTION ======================*/

/**
 * @brief Selects the control scheme for the inverter.
 * @details Valid options: MFDC (Model-Free Deadbeat Control), FOC (Field-Oriented Control),
 *          MPC (Model Predictive Control).
 */
#define CONTROL_SEL FOC

/**
 * @brief Selects the PWM modulation type.
 * @details Valid options: FCS (Finite Control Set), SVM (Space Vector Modulation),
 *          SPWM (Sinusoidal PWM).
 */
#define MODULATION SVM

/**
 * @brief Selects the control mode for the inverter.
 * @details Valid options: SPEED_CTRL (speed control), POS_CTRL (position control),
 *          TORQUE_CTRL (torque control).
 */
#define CONTROL_MODE SPEED_CTRL

/**
 * @brief Selects the speed profile for motor operation.
 * @details Valid options: TRAPEZOIDAL (trapezoidal profile), DATASET (dataset-driven profile).
 */
#define SPEED_PROFILE TRAPEZOIDAL

/**
 * @brief Selects the data logging mode.
 * @details Valid options: DATALOG_DEFAULT (default logging), DATALOG_EXTENDED (extended variables).
 */
#define DATA_LOG_SEL DATALOG_DEFAULT

/**
 * @brief Enables or disables sensorless operation.
 * @details Set to 1 to enable sensorless algorithms (SMO, PLL, etc.), 0 to use encoder (eQEP).
 */
#define SENSORLESS_EN 1

/*====================== COMPENSATION FLAGS ======================*/

/**
 * @brief Enable/disable sensorless compensation.
 */
#define SENSORLESS_COMP FALSE

/*====================== CONTROL LIMITS ======================*/

/**
 * @brief Maximum phase current (A).
 */
#define IS_MAX 14.0f

/*====================== SPEED PROFILE CONFIG ======================*/

#if SPEED_PROFILE == TRAPEZOIDAL
    /**
     * @brief Target speed for trapezoidal profile (rad/s).
     */
    #define TARGET_SPEED 50.0f

    /**
     * @brief Speed ramp slope (rad/s^2).
     */
    #define SPEED_SLOPE 0.05f

    /**
     * @brief Load torque (Nm).
     */
    #define LOAD_TORQUE 1.0f
#elif SPEED_PROFILE == DATASET
    /**
     * @brief Dataset buffer size for speed profile.
     */
    #define DATASET_BUFFER_SIZE 1000U
#endif

/*====================== DATA LOGGING ======================*/

/**
 * @brief Decimation factor for data logging.
 */
#define DECIMATION_FACTOR 1U

/**
 * @brief Data logging buffer length.
 */
#define DAT_LOG_BUFFER_LENGTH 2000U

#if DATA_LOG_SEL == DATALOG_DEFAULT
    /**
     * @brief Number of logged variables (default mode).
     */
    #define LOGGED_VARIABLES 6U
#elif DATA_LOG_SEL == DATALOG_EXTENDED
    /**
     * @brief Number of logged variables (extended mode).
     */
    #define LOGGED_VARIABLES 9U
#endif

/*====================== CONTROL PARAMETERS ======================*/

#if CONTROL_SEL == FOC
    /**
     * @brief Proportional gain for d-axis current controller.
     */
    #define KP_ID 0.5f

    /**
     * @brief Integral gain for d-axis current controller.
     */
    #define KI_ID 10.0f

    /**
     * @brief Proportional gain for q-axis current controller.
     */
    #define KP_IQ 0.5f

    /**
     * @brief Integral gain for q-axis current controller.
     */
    #define KI_IQ 10.0f

    /**
     * @brief Proportional gain for speed controller.
     */
    #define KP_SPEED 0.1f

    /**
     * @brief Integral gain for speed controller.
     */
    #define KI_SPEED 2.0f

    /**
     * @brief Maximum d-axis voltage (V).
     */
    #define VD_MAX 20.0f

    /**
     * @brief Maximum q-axis voltage (V).
     * @details Calculated to respect DC bus voltage limit.
     */
    #define VQ_MAX (sqrtf(DC_BUS_VOLTAGE * DC_BUS_VOLTAGE - VD_MAX * VD_MAX))
#elif CONTROL_SEL == MPC
    /**
     * @brief Weight for cost function component 1.
     */
    #define COST_WEIGHT_K1 0.05f

    /**
     * @brief Weight for cost function component 2.
     */
    #define COST_WEIGHT_K2 0.05f
#elif CONTROL_SEL == MFDC
    /**
     * @brief Modulation rate for MFDC.
     */
    #define MODULATION_RATE 0.25f
#endif

/*====================== VALIDATION ======================*/

/**
 * @brief Validates the control scheme selection.
 */
#if !defined(CONTROL_SEL) || (CONTROL_SEL != FOC && CONTROL_SEL != MPC && CONTROL_SEL != MFDC)
#error "CONTROL_SEL must be defined as FOC, MPC, or MFDC"
#endif

/**
 * @brief Validates the modulation type selection.
 */
#if !defined(MODULATION) || (MODULATION != FCS && MODULATION != SVM && MODULATION != SPWM)
#error "MODULATION must be defined as FCS, SVM, or SPWM"
#endif

/**
 * @brief Validates the control mode selection.
 */
#if !defined(CONTROL_MODE) || (CONTROL_MODE != SPEED_CTRL && CONTROL_MODE != POS_CTRL && CONTROL_MODE != TORQUE_CTRL)
#error "CONTROL_MODE must be defined as SPEED_CTRL, POS_CTRL, or TORQUE_CTRL"
#endif

/**
 * @brief Validates the speed profile selection.
 */
#if !defined(SPEED_PROFILE) || (SPEED_PROFILE != TRAPEZOIDAL && SPEED_PROFILE != DATASET)
#error "SPEED_PROFILE must be defined as TRAPEZOIDAL or DATASET"
#endif

/**
 * @brief Validates the data logging mode selection.
 */
#if !defined(DATA_LOG_SEL) || (DATA_LOG_SEL != DATALOG_DEFAULT && DATA_LOG_SEL != DATALOG_EXTENDED)
#error "DATA_LOG_SEL must be defined as DATALOG_DEFAULT or DATALOG_EXTENDED"
#endif

#endif /* INCLUDE_CONFIG_CONTROL_CONFIG_H_ */
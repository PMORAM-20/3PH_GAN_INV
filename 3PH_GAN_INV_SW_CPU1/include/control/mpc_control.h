/*
 * mpc_control.h
 *
 * @file    mpc_control.h
 * @brief   Header file defining data structures, global variables, and function
 *          prototypes specific to Model Predictive Control (MPC) for a three-phase
 *          GaN inverter using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    May 1, 2025
 * @version 1.0.0
 *
 * @details Defines structures for MPC-specific data, such as cost function states
 *          and voltage gain arrays, along with macros for MPC calculations.
 *          Includes extern declarations for global variables and function prototypes
 *          for MPC initialization and control operations. Optimized for memory and
 *          performance on the F28379D.
 *
 * @note    Requires data_types.h, system_config.h, and control_config.h for data
 *          structures and parameters. Include this file only in MPC-related modules.
 *          Use IQmathLib for fixed-point arithmetic where applicable.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_CONTROL_MPC_CONTROL_H_
#define INCLUDE_CONTROL_MPC_CONTROL_H_

#define CONTROL_SCHEME "MPC"

/* Standard Includes */
#include <stdint.h>

/* Project Includes */
#include "common/data_types.h"
#include "config/system_config.h"
#include "config/control_config.h"

/* Macros */

/** @brief Computes mean squared error for current error in alpha-beta frame */
#define MEAN_SQUARED_ERROR(data) ( \
    (data).alpha * (data).alpha + (data).beta * (data).beta \
)

/** @brief Computes alpha component for OSDC-MPC */
#define OSDCMPC_ALPHA(currMeas, gammaV, fiSSMod, a12SSMod, delta21SSMod, sinT) ( \
    (fiSSMod)->c11 * (currMeas)->alpha + (a12SSMod) * rotor.current_electrical_speed * (currMeas)->beta \
    + (gammaV)->alpha - (delta21SSMod) * rotor.current_electrical_speed * (sinT) \
)

/** @brief Computes beta component for OSDC-MPC */
#define OSDCMPC_BETA(currMeas, gammaV, fiSSMod, a21SSMod, delta21SSMod, cosT) ( \
    (a21SSMod) * rotor.current_electrical_speed * (currMeas)->alpha + (fiSSMod)->c22 * (currMeas)->beta \
    + (gammaV)->beta + (delta21SSMod) * rotor.current_electrical_speed * (cosT) \
)

/** @brief Computes c11 element of phi matrix */
static inline float fiC11(float tm) { return 1.0f - tm * (RS / LD); }

/** @brief Computes c22 element of phi matrix */
static inline float fiC22(float tm) { return 1.0f - tm * (RS / LQ); }

/** @brief Computes a12 element for MPC model */
static inline float a12(float tm) { return tm * (LQ / LD); }

/** @brief Computes a21 element for MPC model */
static inline float a21(float tm) { return -tm * (LQ / LD); }

/** @brief Computes gamma11 element for voltage gain */
static inline float gamma11(float tm) { return tm / LD; }

/** @brief Computes gamma22 element for voltage gain */
static inline float gamma22(float tm) { return tm / LQ; }

/** @brief Computes delta21 element for MPC model */
static inline float delta21(float tm) { return -tm * (LAMBDA / LQ); }

/* Types */

/**
 * @brief Stores MPC switching state table.
 */
typedef struct {
    uint16_t states[8][3]; /**< Switching states for 8 possible inverter configurations */
} MpcSwitchingTable;

/* Extern Variable Declarations */

/**
 * @brief Global MPC switching state table.
 */
extern const MpcSwitchingTable g_mpc_switching_table;

/**
 * @brief Global cost function value for MPC optimization.
 */
extern volatile float g_cost_function;

/**
 * @brief Global optimal switching state from previous iteration.
 */
extern volatile uint16_t g_x_opt_km1;

/**
 * @brief Global iteration counter for MPC control loop.
 */
extern volatile uint16_t g_k;

/**
 * @brief Global phi matrix for MPC state-space model.
 */
extern volatile DiagonalMatrix g_phiSSMod;

/**
 * @brief Global gamma matrix for MPC voltage gain.
 */
extern volatile DiagonalMatrix g_gammaSSMod;

/**
 * @brief Global voltage gain array for 8 switching states.
 */
extern volatile ClarkeData g_voltageGain[8];

/**
 * @brief Global a12 element for MPC state-space model.
 */
extern float g_a12SSMod;

/**
 * @brief Global a21 element for MPC state-space model.
 */
extern float g_a21SSMod;

/**
 * @brief Global delta21 element for MPC state-space model.
 */
extern float g_delta21SSMod;

/**
 * @brief Global predicted currents in alpha-beta frame (k+2).
 */
extern volatile ClarkeData g_abCurrkp2;

/**
 * @brief Global current error in alpha-beta frame.
 */
extern volatile ClarkeData g_abCurrError;

/* Function Prototypes */

/**
 * @brief Initializes MPC-specific data structures and variables.
 */
void mpc_control_init(void);

#endif /* INCLUDE_CONTROL_MPC_CONTROL_H_ */

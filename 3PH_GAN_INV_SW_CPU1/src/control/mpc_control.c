/*
 * mpc_control.c
 *
 * @file    mpc_control.c
 * @brief   Implementation of Model Predictive Control (MPC) functionality for
 *          a three-phase GaN inverter control system using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    May 1, 2025
 * @version 1.0.0
 *
 * @details Implements initialization of MPC-specific data structures and the
 *          core MPC control algorithm. Optimized for performance on the F28379D
 *          with variables placed in fast RAM where applicable.
 *
 * @note    Requires mpc_control.h for declarations and system_config.h for
 *          parameters. Uses data_types.h for shared structures.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#include "control/mpc_control.h"
#include "common/math_utils.h"

/* Global Variable Definitions */

/** @brief Global MPC switching state table, stored in flash */
#pragma DATA_SECTION(g_Sout, ".const")
const uint16_t g_Sout[8];

/** @brief Global cost function, stored in fast RAM */
#pragma DATA_SECTION(g_cost_function, ".ebss")
volatile float g_cost_function;

/** @brief Global optimal switching state, stored in fast RAM */
#pragma DATA_SECTION(g_x_opt_km1, ".ebss")
volatile uint16_t g_x_opt_km1;

/** @brief Global iteration counter, stored in fast RAM */
#pragma DATA_SECTION(g_k, ".ebss")
volatile uint16_t g_k;

/** @brief Global phi matrix, stored in fast RAM */
#pragma DATA_SECTION(g_phiSSMod, ".ebss")
volatile DiagonalMatrix g_phiSSMod;

/** @brief Global gamma matrix, stored in fast RAM */
#pragma DATA_SECTION(g_gammaSSMod, ".ebss")
volatile DiagonalMatrix g_gammaSSMod;

/** @brief Global voltage gain array, stored in fast RAM */
#pragma DATA_SECTION(g_voltageGain, ".ebss")
volatile ClarkeData g_voltageGain[8];

/** @brief Global a12 element, stored in flash */
#pragma DATA_SECTION(g_a12SSMod, ".const")
float g_a12SSMod;

/** @brief Global a21 element, stored in flash */
#pragma DATA_SECTION(g_a21SSMod, ".const")
float g_a21SSMod;

/** @brief Global delta21 element, stored in flash */
#pragma DATA_SECTION(g_delta21SSMod, ".const")
float g_delta21SSMod;

/** @brief Global predicted currents, stored in fast RAM */
#pragma DATA_SECTION(g_abCurrkp2, ".ebss")
volatile ClarkeData g_abCurrkp2;

/** @brief Global current error, stored in fast RAM */
#pragma DATA_SECTION(g_abCurrError, ".ebss")
volatile ClarkeData g_abCurrError;

/* Function Implementations */

void mpc_control_init(void)
{
    uint32_t k;

    /* Initialize cost function */
    g_cost_function = 9999999999.0f;

    /* Initialize optimal switching state */
    g_x_opt_km1 = 7U;

    /* Initialize iteration counter */
    g_k = 0;

    /* Initialize phi matrix */
    g_phiSSMod.c11 = fiC11(1.0f / CONTROL_FREQUENCY_HZ);
    g_phiSSMod.c22 = fiC22(1.0f / CONTROL_FREQUENCY_HZ);

    /* Initialize gamma matrix */
    g_gammaSSMod.c11 = gamma11(1.0f / CONTROL_FREQUENCY_HZ);
    g_gammaSSMod.c22 = gamma22(1.0f / CONTROL_FREQUENCY_HZ);

    /* Initialize voltage gain array */
    for (k = 0; k < 8; k++) {
        g_voltageGain[k].alpha = 0.0f;
        g_voltageGain[k].beta  = 0.0f;
        /* Scale voltage gain based on DC bus voltage */
        g_voltageGain[k].alpha *= g_gammaSSMod.c11 * DC_BUS_VOLTAGE;
        g_voltageGain[k].beta  *= g_gammaSSMod.c22 * DC_BUS_VOLTAGE;
    }

    /* Initialize state-space model coefficients */
    g_a12SSMod = a12(1.0f / CONTROL_FREQUENCY_HZ);
    g_a21SSMod = a21(1.0f / CONTROL_FREQUENCY_HZ);
    g_delta21SSMod = delta21(1.0f / CONTROL_FREQUENCY_HZ);

    /* Initialize predicted currents */
    g_abCurrkp2.alpha = 0.0f;
    g_abCurrkp2.beta = 0.0f;
    g_abCurrkp2.zero = 0.0f;

    /* Initialize current error */
    g_abCurrError.alpha = 0.0f;
    g_abCurrError.beta = 0.0f;
    g_abCurrError.zero = 0.0f;
}

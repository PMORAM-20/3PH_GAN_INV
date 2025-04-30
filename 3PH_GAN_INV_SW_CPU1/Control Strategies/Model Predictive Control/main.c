/*
 * mpc_control.c
 *
 * @file    mpc_control.c
 * @brief   Finite Control Set Model Predictive Control (FCS-MPC) for a three-phase
 *          machine using the F28379D DSP.
 * @author  Pablo Mora Moreno (adapted from 9-phase example)
 * @date    April 29, 2025
 * @version 1.0.0
 *
 * @details Implements FCS-MPC for a three-phase machine, predicting currents one step
 *          ahead and selecting the optimal switching state from 8 possible states.
 *          Uses Clarke and Park transformations for current control in the d-q frame.
 *          Offloads cost function evaluation to the CLA for performance. Integrates
 *          with existing project peripherals (ADC, ePWM, eQEP) and data structures.
 *
 * @note    Requires data_types.h for data structures, math_utils.h for mathematical
 *          functions, cla_driver.h for CLA configuration, and peripheral drivers
 *          (adc_driver.h, pwm_driver.h, peripheral_setup.h).
 */

#include "common/data_types.h"
#include "common/math_utils.h"
#include "peripherals/CLA_driver.h"
#include "peripherals/adc_driver.h"
#include "peripherals/pwm_driver.h"
#include "peripherals/peripheral_setup.h"
#include "control/mpc_control.h"
#include "driverlib.h"
#include <string.h>

int main(void)
{
    pwm_ctrl_period = PWM_FREQUENCY_CLK;

    


}

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
// #include "peripherals/CLA_driver.h"
#include "peripherals/adc_driver.h"
#include "peripherals/pwm_driver.h"
#include "peripherals/peripheral_setup.h"
#include "control/mpc_control.h"
#include "driverlib.h"
#include "device.h"
#include <string.h>

int main(void)
{
   
    Device_init();

    Device_initGPIO();

    DINT;

    Interrupt_initModule();

    IER = 0x0000;
    IFR = 0x0000;

    Interrupt_initVectorTable();

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EALLOW;

    //ConfigureGPIO(); /* Falta por hacer esta función en alguna parte*/
    peripheral_setup_init();

    // Definir las interrupciones en cada módulo
    // Interrupt_register(INT_ADCA1, &handleAdcA1Interrupt);
    // Interrupt_register(INT_ADCB1, &handleAdcB1Interrupt);
    // Interrupt_register(INT_EPWM1, &handleSynchronousInterrupt);

    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_ADCB1);
    Interrupt_enable(INT_EPWM1);

    SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
    EDIS;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EINT;
    ERTM;

}

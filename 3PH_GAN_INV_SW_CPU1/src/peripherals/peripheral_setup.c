/*
 * peripheralsetup.c
 *
 * @file    peripheralsetup.c
 * @brief   Implementation of the peripheral setup module for a three-phase GaN
 *          inverter using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Implements the centralized initialization function for all peripherals
 *          by calling the initialization functions of the PWM, ADC, CMPSS, and CLA
 *          drivers. Ensures all peripherals are properly configured for the
 *          three-phase inverter system.
 *
 * @note    Requires peripheralsetup.h for function prototypes and driver includes
 *          for PWM_driver.h, ADC_driver.h, CMPSS_driver.h, and CLA_driver.h.
 */

#include "peripherals/peripheral_setup.h"

/**
 * @brief Initializes all peripherals for the three-phase inverter system.
 *
 * @details Sequentially calls PWM_init(), ADC_init(), CMPSS_init(), and CLA_init()
 *          to configure the ePWM, ADC, CMPSS, and CLA modules. Ensures all peripherals
 *          are ready for operation in the control loop of the inverter.
 */
void peripheral_setup_init(void) {
    PWM_init();
    ADC_init();
    CMPSS_init();
    CLA_init();
}

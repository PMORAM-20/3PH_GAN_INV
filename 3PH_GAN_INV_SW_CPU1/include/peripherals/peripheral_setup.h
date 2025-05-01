/*
 * peripheralsetup.h
 *
 * @file    peripheralsetup.h
 * @brief   Header file for the peripheral setup module, providing centralized
 *          initialization for all peripherals in a three-phase GaN inverter using
 *          the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details This module centralizes the initialization of PWM, ADC, CMPSS, and CLA
 *          peripherals by calling their respective driver initialization functions.
 *          It ensures all peripherals are configured correctly before the control
 *          loop begins.
 *
 * @note    Requires PWM_driver.h, ADC_driver.h, CMPSS_driver.h, and CLA_driver.h
 *          for driver definitions. Assumes all drivers are configured in their
 *          respective modules.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_DRIVERS_PERIPHERALSETUP_H_
#define INCLUDE_DRIVERS_PERIPHERALSETUP_H_

/* Driver Includes */
#include "peripherals/PWM_driver.h"
#include "peripherals/ADC_driver.h"
#include "peripherals/CMPSS_driver.h"
// #include "peripherals/CLA_driver.h"

/**
 * @brief Initializes all peripherals for the three-phase inverter system.
 *
 * @details Calls the initialization functions for PWM, ADC, CMPSS, and CLA modules
 *          to configure all necessary peripherals. Ensures proper setup before
 *          entering the control loop.
 */
void peripheral_setup_init(void);

#endif /* INCLUDE_DRIVERS_PERIPHERALSETUP_H_ */

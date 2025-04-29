/*
 * ADC_driver.h
 *
 * @file    ADC_driver.h
 * @brief   Header file for the ADC driver module, providing configuration and
 *          initialization for ADC modules in a three-phase GaN inverter using
 *          the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details This module configures one or more ADC modules to sample phase currents
 *          or voltages in a three-phase inverter. It defines an array of ADCConfig
 *          structures to specify parameters such as resolution, signal mode,
 *          trigger source, and channels to convert.
 *
 * @note    Requires driverlib.h for peripheral access. Configurations are stored
 *          in an array for scalability. Assumes system_config.h defines ADC-related
 *          parameters (e.g., ADC_SAMPLING_FREQ).
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_DRIVERS_ADC_DRIVER_H_
#define INCLUDE_DRIVERS_ADC_DRIVER_H_

/* Standard Includes */
#include "driverlib.h"
#include "config/system_config.h"

/**
 * @brief Structure to hold configuration parameters for a single ADC module.
 */
typedef struct {
    uint32_t base;              /**< Base address of the ADC module (e.g., ADC_A_BASE) */
    ADC_Resolution resolution;  /**< ADC resolution: 12-bit or 16-bit */
    ADC_SignalMode signal_mode; /**< Signal mode: Single-ended or Differential */
    ADC_Trigger trigger_source; /**< Trigger source (e.g., EPWM1_SOCA) */
    uint16_t channels;          /**< Bitmask of channels to convert (e.g., 0x0007 for channels 0, 1, 2) */
} ADCConfig;

/**
 * @brief Number of ADC modules used in the system.
 */
#define NUM_ADC_MODULES 1       // One ADC module for three-phase current sampling

/**
 * @brief Array holding configurations for all ADC modules.
 */
extern ADCConfig adc_configs[NUM_ADC_MODULES];

/**
 * @brief Initializes all ADC modules based on the configurations in adc_configs.
 *
 * @details Configures the ADC module with the specified resolution, signal mode,
 *          trigger source, and channels. Sets up sample-and-hold (SOC) parameters
 *          for synchronous sampling of three-phase signals.
 */
void ADC_init(void);

#endif /* INCLUDE_DRIVERS_ADC_DRIVER_H_ */

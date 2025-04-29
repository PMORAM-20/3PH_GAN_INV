/*
 * CMPSS_driver.h
 *
 * @file    CMPSS_driver.h
 * @brief   Header file for the CMPSS driver module, providing configuration and
 *          initialization for CMPSS modules in a three-phase GaN inverter using
 *          the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details This module configures one or more CMPSS (Comparator Subsystem)
 *          modules for analog signal comparison, typically used for overcurrent
 *          or overvoltage protection in a three-phase inverter. It defines an
 *          array of CMPSSConfig structures to specify parameters like comparator
 *          selection, reference voltage, and hysteresis.
 *
 * @note    Requires driverlib.h for peripheral access. Configurations are stored
 *          in an array for scalability. Assumes system_config.h defines CMPSS-related
 *          parameters (e.g., VREF).
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_DRIVERS_CMPSS_DRIVER_H_
#define INCLUDE_DRIVERS_CMPSS_DRIVER_H_

/* Standard Includes */
#include "driverlib.h"
#include "config/system_config.h"

/**
 * @brief Structure to hold configuration parameters for a single CMPSS module.
 */
typedef struct {
    uint32_t base;              /**< Base address of the CMPSS module (e.g., CMPSS1_BASE) */
    uint16_t comparator;        /**< Comparator to use (e.g., CMPSS_COMP1) */
    float reference;            /**< Reference voltage for the DAC (V) */
    uint16_t hysteresis;        /**< Hysteresis value for the comparator */
} CMPSSConfig;

/**
 * @brief Number of CMPSS modules used in the system.
 */
#define NUM_CMPSS_MODULES 1     // One CMPSS module for protection

/**
 * @brief Voltage logic level of the DSP F28379D
 */
#define VREF 3.3f

/**
 * @brief Array holding configurations for all CMPSS modules.
 */
extern CMPSSConfig cmpss_configs[NUM_CMPSS_MODULES];

/**
 * @brief Initializes all CMPSS modules based on the configurations in cmpss_configs.
 *
 * @details Configures each CMPSS module with the specified comparator, reference
 *          voltage, and hysteresis. Sets up the high comparator for protection
 *          purposes (e.g., overcurrent detection).
 */
void CMPSS_init(void);

#endif /* INCLUDE_DRIVERS_CMPSS_DRIVER_H_ */

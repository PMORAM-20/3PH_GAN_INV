/*
 * CLA_driver.h
 *
 * @file    CLA_driver.h
 * @brief   Header file for the CLA driver module, providing configuration and
 *          initialization for the Control Law Accelerator (CLA) in a three-phase
 *          GaN inverter using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details This module configures the CLA to execute mathematical tasks such as
 *          Clarke and Park transformations, PI control, and sliding mode observer
 *          (SMO) for a three-phase inverter. It defines an array of CLAConfig
 *          structures to specify task parameters, including task selection, input/output
 *          data pointers, and execution triggers. The CLA offloads computations
 *          from the main CPU, improving real-time performance.
 *
 * @note    Requires driverlib.h for CLA and peripheral access, data_types.h for
 *          data structures, and system_config.h for system parameters (e.g.,
 *          CONTROL_FREQUENCY_HZ). Assumes CLA tasks are defined in CLA_tasks.cla.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_DRIVERS_CLA_DRIVER_H_
#define INCLUDE_DRIVERS_CLA_DRIVER_H_

/* Standard Includes */
#include "driverlib.h"
#include "common/data_types.h"
#include "config/system_config.h"

/**
 * @brief Enumeration of available CLA tasks.
 */
typedef enum {
    CLA_TASK_CLARKE,       /**< Clarke transformation task */
    CLA_TASK_PARK,         /**< Park transformation task */
    CLA_TASK_PI,           /**< PI controller task */
    CLA_TASK_SMO,          /**< Sliding mode observer task */
    CLA_TASK_COUNT         /**< Number of available tasks */
} CLATaskType;

/**
 * @brief Structure to hold configuration parameters for a single CLA task.
 */
typedef struct {
    uint32_t task_id;          /**< CLA task ID (1 to 8) */
    CLATaskType task_type;     /**< Type of task to execute (e.g., CLA_TASK_CLARKE) */
    void *input_data;          /**< Pointer to input data (e.g., PhaseData, ClarkeData) */
    void *output_data;         /**< Pointer to output data (e.g., ClarkeData, ParkData) */
    uint32_t trigger_source;   /**< Trigger source (e.g., ADC_INT1, EPWM1_INT) */
    float dt;                  /**< Time step for the task (s) */
} CLAConfig;

/**
 * @brief Number of CLA tasks configured in the system.
 */
#define NUM_CLA_TASKS 3        // Clarke, Park, and PI tasks for three-phase control

/**
 * @brief Array holding configurations for all CLA tasks.
 */
extern CLAConfig cla_configs[NUM_CLA_TASKS];

/**
 * @brief Initializes the CLA module and configures all tasks.
 *
 * @details Sets up the CLA clock, memory, and task vectors based on the
 *          configurations in cla_configs. Configures triggers and data pointers
 *          for each task, enabling parallel execution of mathematical computations.
 */
void CLA_init(void);

#endif /* INCLUDE_DRIVERS_CLA_DRIVER_H_ */
/*
 * PWM_driver.h
 *
 * @file    PWM_driver.h
 * @brief   Header file for the PWM driver module, providing configuration and
 *          initialization for ePWM modules in a three-phase GaN inverter using
 *          the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.1
 *
 * @details This module configures three ePWM modules (one per phase) for a
 *          three-phase inverter. It defines an array of PWMConfig structures to
 *          hold configuration parameters such as period, compare values, dead
 *          time, clock prescalers, and interrupt settings, enabling flexible
 *          setup for each phase. The initialization function applies these
 *          configurations to the hardware, supporting complementary outputs,
 *          dead-band, and synchronized operation.
 *
 * @note    Requires driverlib.h for peripheral access. Configurations are stored
 *          in an array for scalability and ease of modification.
 *          Assumes system_config.h defines PWM-related parameters (e.g., PWM_FREQUENCY_HZ).
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_DRIVERS_PWM_DRIVER_H_
#define INCLUDE_DRIVERS_PWM_DRIVER_H_

/* Standard Includes */
#include "driverlib.h"
#include "config/system_config.h"

/**
 * @brief Structure to hold configuration parameters for a single ePWM module.
 */
typedef struct {
    uint32_t base;                       /**< Base address of the ePWM module (e.g., EPWM1_BASE) */
    EPWM_TimeBaseCountMode counter_mode; /**< Counter mode: Up, Down, or Up-Down */
    uint16_t period;                     /**< PWM period in timer ticks */
    uint16_t cmp_a;                      /**< Compare value for PWMxA output (duty cycle control) */
    uint16_t cmp_b;                      /**< Compare value for PWMxB output (complementary signal) */
    uint16_t red_delay;                  /**< Rising edge delay (RED) for dead-band in cycles */
    uint16_t fed_delay;                  /**< Falling edge delay (FED) for dead-band in cycles */
    EPWM_ClockDivider clock_div;         /**< Clock prescaler for ePWM module */
    EPWM_HSClockDivider hsclock_div;     /**< High-speed clock prescaler */
    uint32_t int_source;     /**< Interrupt source (e.g., TBCTR_ZERO) */
    uint16_t int_event_count;            /**< Number of events to generate an interrupt */
} PWMConfig;

/**
 * @brief Number of PWM modules used in the three-phase system.
 */
#define NUM_PWM_MODULES 3       // One module per phase: A, B, C

/**
 * @brief Array holding configurations for all PWM modules.
 */
extern PWMConfig pwm_configs[NUM_PWM_MODULES];

/**
 * @brief Initializes all PWM modules based on the configurations in pwm_configs.
 *
 * @details Configures the period, counter mode, compare values, dead-band,
 *          clock prescalers, and interrupts for each ePWM module. Ensures
 *          synchronous operation with complementary outputs for the three-phase
 *          inverter.
 */
void PWM_init(void);

#endif /* INCLUDE_DRIVERS_PWM_DRIVER_H_ */

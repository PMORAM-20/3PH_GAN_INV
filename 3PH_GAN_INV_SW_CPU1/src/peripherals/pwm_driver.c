/*
 * PWM_driver.c
 *
 * @file    PWM_driver.c
 * @brief   Implementation of the PWM driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Implements the initialization function for ePWM modules, applying
 *          configurations from the pwm_configs array. Each module is set up with
 *          the specified period, counter mode, compare values, and dead time to
 *          generate PWM signals for the three-phase inverter.
 *
 * @note    Requires PWM_driver.h for configuration definitions and driverlib.h
 *          for peripheral access. Assumes system_config.h provides parameters
 *          like PWM_FREQUENCY_HZ and CPU frequency.
 */

#include "drivers/PWM_driver.h"

/**
 * @brief Array holding configurations for all PWM modules.
 *
 * @details Default configurations for three ePWM modules (EPWM1, EPWM2, EPWM3)
 *          corresponding to phases A, B, and C. The period is calculated based
 *          on PWM_FREQUENCY_HZ, and initial compare values are set to 50% duty
 *          cycle. Dead time is set to ensure safe switching in GaN transistors.
 */
PWMConfig pwm_configs[NUM_PWM_MODULES] = {
    {EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN, (uint16_t)(SYSCLK_HZ / (2 * PWM_FREQUENCY_HZ)), 500, 500, 50}, // Phase A
    {EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN, (uint16_t)(SYSCLK_HZ / (2 * PWM_FREQUENCY_HZ)), 500, 500, 50}, // Phase B
    {EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN, (uint16_t)(SYSCLK_HZ / (2 * PWM_FREQUENCY_HZ)), 500, 500, 50}  // Phase C
};

/**
 * @brief Initializes all PWM modules based on the configurations in pwm_configs.
 *
 * @details Configures each ePWM module with the specified period, counter mode,
 *          compare values, and dead time. Enables complementary outputs with
 *          dead time for safe operation of the GaN inverter. Synchronizes the
 *          modules for consistent phase alignment.
 */
void PWM_init(void) {
    for (int i = 0; i < NUM_PWM_MODULES; i++) {
        // Disable the time-base clock to configure
        EALLOW;
        SysCtl_enablePeripheral((SysCtl_PeripheralPCLOCKCR)(SYSCTL_PERIPH_CLK_EPWM1 + i));
        EDIS;

        // Set time-base period and counter mode
        EPWM_setTimeBasePeriod(pwm_configs[i].base, pwm_configs[i].period);
        EPWM_setTimeBaseCounterMode(pwm_configs[i].base, pwm_configs[i].counter_mode);

        // Configure compare values for PWMxA and PWMxB
        EPWM_setCounterCompareValue(pwm_configs[i].base, EPWM_COUNTER_COMPARE_A, pwm_configs[i].cmp_a);
        EPWM_setCounterCompareValue(pwm_configs[i].base, EPWM_COUNTER_COMPARE_B, pwm_configs[i].cmp_b);

        // Configure dead time
        EPWM_setDeadBandDelay(pwm_configs[i].base, pwm_configs[i].dead_time);
        EPWM_enableDeadBand(pwm_configs[i].base, EPWM_DB_RED | EPWM_DB_FED);

        // Configure action-qualifier for complementary outputs
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        // Enable PWM output
        EPWM_enableOutput(pwm_configs[i].base);
    }

    // Synchronize all PWM modules
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
    EPWM_enablePhaseShift(EPWM2_BASE, true);
    EPWM_enablePhaseShift(EPWM3_BASE, true);
}
/*
 * PWM_driver.c
 *
 * @file    PWM_driver.c
 * @brief   Implementation of the PWM driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.1
 *
 * @details Implements the initialization function for ePWM modules, applying
 *          configurations from the pwm_configs array. Each module is set up with
 *          the specified period, counter mode, compare values, dead-band delays,
 *          clock prescalers, and interrupt settings to generate PWM signals for
 *          the three-phase inverter. Supports complementary outputs with dead-band
 *          and synchronized operation.
 *
 * @note    Requires PWM_driver.h for configuration definitions and driverlib.h
 *          for peripheral access. Assumes system_config.h provides parameters
 *          like PWM_FREQUENCY_HZ, PWM_PERIOD_CTRL, EPWM_DEADBAND_MIN, and
 *          EPWM_DEADBAND_MAX.
 */

#include "peripherals/pwm_driver.h"

/**
 * @brief Array holding configurations for all PWM modules.
 *
 * @details Default configurations for three ePWM modules (EPWM1, EPWM2, EPWM3)
 *          corresponding to phases A, B, and C. The period is set to
 *          PWM_PERIOD_CTRL, with initial compare values at 50% duty cycle.
 *          Dead-band delays are set for safe switching in GaN transistors.
 *          Clock prescalers are set to 1, and interrupts are enabled on TBCTR_ZERO.
 */
PWMConfig pwm_configs[NUM_PWM_MODULES] = {
    {EPWM1_BASE, EPWM_COUNTER_MODE_UP, PWM_FREQUENCY_HZ, 500, 500, EPWM_DEADBAND_CYCLES, EPWM_DEADBAND_CYCLES, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, EPWM_INT_TBCTR_ZERO, 1}, // Phase A
    {EPWM2_BASE, EPWM_COUNTER_MODE_UP, PWM_FREQUENCY_HZ, 500, 500, EPWM_DEADBAND_CYCLES, EPWM_DEADBAND_CYCLES, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, EPWM_INT_TBCTR_ZERO, 1}, // Phase B
    {EPWM3_BASE, EPWM_COUNTER_MODE_UP, PWM_FREQUENCY_HZ, 500, 500, EPWM_DEADBAND_CYCLES, EPWM_DEADBAND_CYCLES, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1, EPWM_INT_TBCTR_ZERO, 1}  // Phase C
};

/**
 * @brief Initializes all PWM modules based on the configurations in pwm_configs.
 *
 * @details Configures each ePWM module with the specified period, counter mode,
 *          compare values, dead-band delays, clock prescalers, and interrupt
 *          settings. Enables complementary outputs with dead-band for safe
 *          operation of the GaN inverter. Synchronizes the modules for consistent
 *          phase alignment, with phase shift disabled for direct control.
 */
void PWM_init(void) {
    int i;
    for (i = 0; i < NUM_PWM_MODULES; i++) {
        // Enable the time-base clock to configure
        EALLOW;
        SysCtl_enablePeripheral((SysCtl_PeripheralPCLOCKCR)(SYSCTL_PERIPH_CLK_EPWM1 + i));
        EDIS;

        // Set clock prescalers
        EPWM_setClockPrescaler(pwm_configs[i].base, pwm_configs[i].clock_div, pwm_configs[i].hsclock_div);

        // Set time-base period, counter mode, and reset counter
        EPWM_setTimeBasePeriod(pwm_configs[i].base, pwm_configs[i].period);
        EPWM_setTimeBaseCounter(pwm_configs[i].base, 0);
        EPWM_setTimeBaseCounterMode(pwm_configs[i].base, pwm_configs[i].counter_mode);

        // Disable phase shift
        EPWM_disablePhaseShiftLoad(pwm_configs[i].base);
        EPWM_setPhaseShift(pwm_configs[i].base, 0);

        // Configure compare values and shadow load mode
        EPWM_setCounterCompareValue(pwm_configs[i].base, EPWM_COUNTER_COMPARE_A, pwm_configs[i].cmp_a);
        EPWM_setCounterCompareValue(pwm_configs[i].base, EPWM_COUNTER_COMPARE_B, pwm_configs[i].cmp_b);
        EPWM_setCounterCompareShadowLoadMode(pwm_configs[i].base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
        EPWM_setCounterCompareShadowLoadMode(pwm_configs[i].base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

        // Configure action-qualifier for complementary outputs
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
        EPWM_setActionQualifierAction(pwm_configs[i].base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        // Configure dead-band
        EPWM_setDeadBandDelayMode(pwm_configs[i].base, EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(pwm_configs[i].base, EPWM_DB_FED, true);
        EPWM_setDeadBandDelayPolarity(pwm_configs[i].base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(pwm_configs[i].base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setRisingEdgeDelayCount(pwm_configs[i].base, pwm_configs[i].red_delay);
        EPWM_setFallingEdgeDelayCount(pwm_configs[i].base, pwm_configs[i].fed_delay);
        EPWM_setRisingEdgeDelayCountShadowLoadMode(pwm_configs[i].base, EPWM_RED_LOAD_ON_CNTR_ZERO);
        EPWM_setFallingEdgeDelayCountShadowLoadMode(pwm_configs[i].base, EPWM_FED_LOAD_ON_CNTR_ZERO);

        // Configure interrupts
        EPWM_setInterruptSource(pwm_configs[i].base, pwm_configs[i].int_source);
        EPWM_enableInterrupt(pwm_configs[i].base);
        EPWM_setInterruptEventCount(pwm_configs[i].base, pwm_configs[i].int_event_count);
    }

    // Synchronize all PWM modules
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
}

/*
 * ADC_driver.c
 *
 * @file    ADC_driver.c
 * @brief   Implementation of the ADC driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Implements the initialization function for ADC modules, applying
 *          configurations from the adc_configs array. The module is set up to
 *          sample three-phase currents or voltages with specified resolution,
 *          signal mode, and trigger source, ensuring synchronized conversions.
 *
 * @note    Requires ADC_driver.h for configuration definitions and driverlib.h
 *          for peripheral access. Assumes system_config.h provides parameters
 *          like ADC_SAMPLING_FREQ.
 */

#include "drivers/ADC_driver.h"

/**
 * @brief Array holding configurations for all ADC modules.
 *
 * @details Default configuration for one ADC module (ADCA) to sample three
 *          channels (ADCIN0, ADCIN1, ADCIN2) corresponding to phase currents.
 *          Uses 12-bit resolution, single-ended mode, and EPWM1_SOCA as the
 *          trigger source for synchronized sampling.
 */
ADCConfig adc_configs[NUM_ADC_MODULES] = {
    {ADC_A_BASE, ADC_RESOLUTION_12BIT, ADC_SIGNAL_MODE_SINGLE_ENDED, ADC_TRIGGER_EPWM1_SOCA, 0x0007} // Channels 0, 1, 2
};

/**
 * @brief Initializes all ADC modules based on the configurations in adc_configs.
 *
 * @details Configures each ADC module with the specified resolution, signal mode,
 *          and trigger source. Sets up three SOCs (Start of Conversion) for
 *          sampling phase currents on channels ADCIN0, ADCIN1, and ADCIN2.
 *          Configures interrupt pulse mode for end-of-conversion signaling.
 */
void ADC_init(void) {
    for (int i = 0; i < NUM_ADC_MODULES; i++) {
        // Enable ADC clock
        EALLOW;
        SysCtl_enablePeripheral((SysCtl_PeripheralPCLOCKCR)(SYSCTL_PERIPH_CLK_ADCA + i));
        EDIS;

        // Configure ADC mode
        ADC_setMode(adc_configs[i].base, adc_configs[i].resolution, adc_configs[i].signal_mode);

        // Set interrupt pulse mode to end of conversion
        ADC_setInterruptPulseMode(adc_configs[i].base, ADC_PULSE_END_OF_CONV);

        // Configure SOCs for three-phase sampling
        if (adc_configs[i].channels & 0x0001) {
            ADC_setupSOC(adc_configs[i].base, ADC_SOC_NUMBER0, adc_configs[i].trigger_source,
                         ADC_CH_ADCIN0, 64); // Sample window: 64 cycles
        }
        if (adc_configs[i].channels & 0x0002) {
            ADC_setupSOC(adc_configs[i].base, ADC_SOC_NUMBER1, adc_configs[i].trigger_source,
                         ADC_CH_ADCIN1, 64);
        }
        if (adc_configs[i].channels & 0x0004) {
            ADC_setupSOC(adc_configs[i].base, ADC_SOC_NUMBER2, adc_configs[i].trigger_source,
                         ADC_CH_ADCIN2, 64);
        }

        // Enable ADC module
        ADC_enableConverter(adc_configs[i].base);
    }
}
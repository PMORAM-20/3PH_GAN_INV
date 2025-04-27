/*
 * CMPSS_driver.c
 *
 * @file    CMPSS_driver.c
 * @brief   Implementation of the CMPSS driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Implements the initialization function for CMPSS modules, applying
 *          configurations from the cmpss_configs array. Each module is set up
 *          with the specified comparator, reference voltage, and hysteresis for
 *          protection tasks such as overcurrent or overvoltage detection.
 *
 * @note    Requires CMPSS_driver.h for configuration definitions and driverlib.h
 *          for peripheral access. Assumes system_config.h provides parameters
 *          like VREF (reference voltage, e.g., 3.3V).
 */

#include "drivers/CMPSS_driver.h"

/**
 * @brief Array holding configurations for all CMPSS modules.
 *
 * @details Default configuration for one CMPSS module (CMPSS1) using comparator 1.
 *          The reference voltage is set to 1.65V (midpoint of 3.3V rail), and
 *          hysteresis is set to 10 units for noise immunity in overcurrent
 *          detection.
 */
CMPSSConfig cmpss_configs[NUM_CMPSS_MODULES] = {
    {CMPSS1_BASE, CMPSS_COMP1, 1.65f, 10} // Comparator 1, reference 1.65V, hysteresis 10
};

/**
 * @brief Initializes all CMPSS modules based on the configurations in cmpss_configs.
 *
 * @details Configures each CMPSS module with the specified comparator, reference
 *          voltage (converted to DAC counts), and hysteresis. Enables the high
 *          comparator for protection tasks, ensuring reliable operation in the
 *          three-phase inverter.
 */
void CMPSS_init(void) {
    for (int i = 0; i < NUM_CMPSS_MODULES; i++) {
        // Enable CMPSS clock
        EALLOW;
        SysCtl_enablePeripheral((SysCtl_PeripheralPCLOCKCR)(SYSCTL_PERIPH_CLK_CMPSS1 + i));
        EDIS;

        // Configure high comparator
        CMPSS_configHighComparator(cmpss_configs[i].base, cmpss_configs[i].comparator);

        // Set DAC reference voltage (assuming 3.3V VREF)
        uint16_t dac_value = (uint16_t)(cmpss_configs[i].reference * 4096 / VREF);
        CMPSS_setDACValue(cmpss_configs[i].base, dac_value);

        // Set hysteresis
        CMPSS_setHysteresis(cmpss_configs[i].base, cmpss_configs[i].hysteresis);

        // Enable CMPSS module
        CMPSS_enableModule(cmpss_configs[i].base);
    }
}
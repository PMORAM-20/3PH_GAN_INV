/*
 * CMPSS_driver.c
 *
 * @file    CMPSS_driver.c
 * @brief   Implementation of the CMPSS driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.1
 *
 * @details Implements the initialization function for the CMPSS module, configuring
 *          it for protection tasks such as overcurrent detection. Uses an array of
 *          CMPSSConfig structures to specify parameters like base address, comparator
 *          selection, and reference voltage.
 *
 * @note    Requires CMPSS_driver.h for configuration definitions and driverlib.h
 *          for peripheral access. Assumes system_config.h provides parameters
 *          like VREF.
 */

#include "peripherals/cmpss_driver.h"
#include "driverlib.h" // Incluye todas las definiciones de DriverLib

/**
 * @brief Array holding configurations for all CMPSS modules.
 *
 * @details Default configuration for one CMPSS module (CMPSS1) for overcurrent
 *          protection. Uses a DAC reference voltage (VREF) at 1.65V (midpoint
 *          of 3.3V rail) and enables hysteresis for noise immunity.
 */
CMPSSConfig cmpss_configs[NUM_CMPSS_MODULES] = {
    {CMPSS1_BASE, CMPSS_INSRC_DAC, 1.65f, 1} // CMPSS1, DAC source, 1.65V, hysteresis
};

/**
 * @brief Initializes all CMPSS modules based on the configurations in cmpss_configs.
 *
 * @details Configures each CMPSS module with the specified base address, comparator
 *          selection, reference voltage, and hysteresis. Enables the high comparator
 *          for protection tasks (e.g., overcurrent detection).
 */
void CMPSS_init(void) {
    int i;
    for (i = 0; i < NUM_CMPSS_MODULES; i++) {
        // Enable CMPSS clock
        EALLOW;
        SysCtl_enablePeripheral((SysCtl_PeripheralPCLOCKCR)(SYSCTL_PERIPH_CLK_CMPSS1 + i));
        EDIS;

        // Configure comparator
        CMPSS_enableModule(cmpss_configs[i].base);
        CMPSS_configHighComparator(cmpss_configs[i].base, cmpss_configs[i].comparator);
        CMPSS_configOutputsHigh(cmpss_configs[i].base, CMPSS_TRIP_LATCH | CMPSS_TRIPOUT_LATCH);

        // Set DAC reference voltage
        CMPSS_setDACValueHigh(cmpss_configs[i].base, (uint16_t)((cmpss_configs[i].reference / VREF) * 4095));

        // Enable hysteresis (2 levels for noise immunity)
        CMPSS_setHysteresis(cmpss_configs[i].base, cmpss_configs[i].hysteresis);
    }
}

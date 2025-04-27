/*
 * CLA_driver.c
 *
 * @file    CLA_driver.c
 * @brief   Implementation of the CLA driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Implements the initialization function for the CLA module, configuring
 *          the clock, memory, and task vectors based on the cla_configs array.
 *          Each task is set up with its trigger source, input/output data pointers,
 *          and execution parameters. The CLA executes mathematical tasks such as
 *          Clarke, Park, and PI control in parallel with the main CPU.
 *
 * @note    Requires CLA_driver.h for configuration definitions, driverlib.h for
 *          CLA and peripheral access, and data_types.h for data structures.
 *          Assumes CLA tasks are implemented in CLA_tasks.cla.
 */

#include "drivers/CLA_driver.h"

/* External data structures for CLA tasks */
extern PhaseData phase_data;
extern ClarkeData clarke_data;
extern ParkData park_data;
extern PiController pi_controller;
extern RotorData rotor_data;
extern RotorData rotor_est;

/**
 * @brief Array holding configurations for all CLA tasks.
 *
 * @details Default configurations for three CLA tasks:
 *          - Task 1: Clarke transformation (phase currents to alpha-beta).
 *          - Task 2: Park transformation (alpha-beta to d-q).
 *          - Task 3: PI controller update (current control).
 *          Tasks are triggered by ADC_INT1 (end of conversion) and use a time
 *          step based on CONTROL_FREQUENCY_HZ.
 */
CLAConfig cla_configs[NUM_CLA_TASKS] = {
    {1, CLA_TASK_CLARKE, &phase_data, &clarke_data, ADC_INT1, 1.0f / CONTROL_FREQUENCY_HZ}, // Clarke
    {2, CLA_TASK_PARK, &clarke_data, &park_data, ADC_INT1, 1.0f / CONTROL_FREQUENCY_HZ},   // Park
    {3, CLA_TASK_PI, &park_data, &pi_controller, ADC_INT1, 1.0f / CONTROL_FREQUENCY_HZ}   // PI
};

/**
 * @brief Initializes the CLA module and configures all tasks.
 *
 * @details Configures the CLA clock, assigns program and data memory, and sets up
 *          task vectors for each configured task. Maps input/output data pointers
 *          and configures triggers (e.g., ADC interrupts). Enables CLA tasks for
 *          parallel execution of mathematical computations.
 */
void CLA_init(void) {
    // Enable CLA clock
    EALLOW;
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    EDIS;

    // Configure CLA memory
    MemCfg_setLSRAMMaster(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CLA);
    MemCfg_setLSRAMMaster(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CLA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);

    // Initialize CLA task vectors
    for (int i = 0; i < NUM_CLA_TASKS; i++) {
        // Map task vector
        CLA_mapTaskVector(CLA1_BASE, (CLA_TaskNumber)cla_configs[i].task_id,
                          cla_configs[i].task_type == CLA_TASK_CLARKE ? (uint16_t)&Cla1Task1 :
                          cla_configs[i].task_type == CLA_TASK_PARK ? (uint16_t)&Cla1Task2 :
                          cla_configs[i].task_type == CLA_TASK_PI ? (uint16_t)&Cla1Task3 :
                          (uint16_t)&Cla1Task4);

        // Configure trigger
        CLA_setTriggerSource((CLA_TaskNumber)cla_configs[i].task_id, cla_configs[i].trigger_source);

        // Enable task
        CLA_enableTasks(CLA1_BASE, 1 << (cla_configs[i].task_id - 1));
    }

    // Enable CLA interrupts
    CLA_enableSoftwareInterrupt(CLA1_BASE);
    CLA_enableHardwareInterrupt(CLA1_BASE);

    // Start CLA
    CLA_enable(CLA1_BASE);
}
/*
 * CLA_driver.c
 *
 * @file    CLA_driver.c
 * @brief   Implementation of the CLA driver module for a three-phase GaN inverter
 *          using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.2
 *
 * @details Implements the initialization function for the CLA module, configuring
 *          the clock, memory (LSRAM), and task vectors based on the cla_configs
 *          array. Configures tasks for Clarke transformation, Park transformation,
 *          and a generic task, with specific triggers (e.g., EPWM1 interrupt,
 *          software). The CLA executes tasks in parallel with the main CPU,
 *          offloading mathematical computations.
 *
 * @note    Requires CLA_driver.h for configuration definitions, driverlib.h for
 *          CLA and peripheral access, data_types.h for data structures, and
 *          CLA_tasks.cla for task implementations.
 */

#include "peripherals/CLA_driver.h"
#include "driverlib/cla.h" // Incluye definiciones para CLA

/* External data structures for CLA tasks */
extern PhaseData phase_data;
extern ClarkeData clarke_data;
extern ParkData park_data;
extern PiController pi_controller;

/* External CLA task declarations */
extern __interrupt void Cla1Task1(void);
extern __interrupt void Cla1Task2(void);
extern __interrupt void Cla1Task8(void);

/**
 * @brief Array holding configurations for all CLA tasks.
 *
 * @details Default configurations for three CLA tasks:
 *          - Task 1: Clarke transformation (phase currents to alpha-beta), triggered by EPWM1_INT.
 *          - Task 2: Park transformation (alpha-beta to d-q), triggered by software.
 *          - Task 8: Generic task for additional computations, triggered by software.
 *          Tasks use a time step based on CONTROL_FREQUENCY_HZ.
 */
CLAConfig cla_configs[NUM_CLA_TASKS] = {
    {CLA_MVECT_1, CLA_TASK_CLARKE, &phase_data, &clarke_data, CLA_TRIGGER_EPWM1INT, 1.0f / CONTROL_FREQUENCY_HZ}, // Clarke
    {CLA_MVECT_2, CLA_TASK_PARK, &clarke_data, &park_data, CLA_TRIGGER_SOFTWARE, 1.0f / CONTROL_FREQUENCY_HZ},   // Park
    {CLA_MVECT_3, CLA_TASK_GENERIC, &park_data, &pi_controller, CLA_TRIGGER_SOFTWARE, 1.0f / CONTROL_FREQUENCY_HZ} // Generic (PI)
};

/**
 * @brief Initializes the CLA module and configures all tasks.
 *
 * @details Configures the CLA clock, assigns program and data memory (LSRAM),
 *          and sets up task vectors for each configured task. Maps input/output
 *          data pointers and configures triggers (e.g., EPWM1 interrupt, software).
 *          Enables CLA tasks for parallel execution, disables background tasks,
 *          and enables interrupt acknowledgment.
 */
void CLA_init(void) {
#pragma diag_suppress=770
    int i;

    // Enable CLA clock
    EALLOW;
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    EDIS;

    // Configure LSRAM
    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);
    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);
    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5, MEMCFG_CLA_MEM_PROGRAM);

    // Initialize CLA task vectors
    for (i = 0; i < NUM_CLA_TASKS; i++) {
        // Map task vector
        CLA_mapTaskVector(CLA1_BASE, cla_configs[i].task_id,
                          cla_configs[i].task_type == CLA_TASK_CLARKE ? (uint16_t)&Cla1Task1 :
                          cla_configs[i].task_type == CLA_TASK_PARK ? (uint16_t)&Cla1Task2 :
                          cla_configs[i].task_type == CLA_TASK_GENERIC ? (uint16_t)&Cla1Task8 :
                          (uint16_t)&Cla1Task1); // Default to Task1 if invalid

        // Configure trigger
        CLA_setTriggerSource((CLA_TaskNumber)cla_configs[i].task_id, cla_configs[i].trigger_source);

        // Enable task
        CLA_enableTasks(CLA1_BASE, 1 << (cla_configs[i].task_id - 1));
    }

    // Enable CLA interrupt acknowledgment
    CLA_enableIACK(CLA1_BASE);

    // Enable all CLA tasks
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_ALL);

#pragma diag_warning=770
}

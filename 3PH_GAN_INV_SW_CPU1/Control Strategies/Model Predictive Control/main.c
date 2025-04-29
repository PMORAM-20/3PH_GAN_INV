/*
 * mpc_control.c
 *
 * @file    mpc_control.c
 * @brief   Finite Control Set Model Predictive Control (FCS-MPC) for a three-phase
 *          machine using the F28379D DSP.
 * @author  Pablo Mora Moreno (adapted from 9-phase example)
 * @date    April 29, 2025
 * @version 1.0.0
 *
 * @details Implements FCS-MPC for a three-phase machine, predicting currents one step
 *          ahead and selecting the optimal switching state from 8 possible states.
 *          Uses Clarke and Park transformations for current control in the d-q frame.
 *          Offloads cost function evaluation to the CLA for performance. Integrates
 *          with existing project peripherals (ADC, ePWM, eQEP) and data structures.
 *
 * @note    Requires data_types.h for data structures, math_utils.h for mathematical
 *          functions, cla_driver.h for CLA configuration, and peripheral drivers
 *          (adc_driver.h, pwm_driver.h, peripheral_setup.h).
 */

#include "common/data_types.h"
#include "common/math_utils.h"
#include "peripherals/CLA_driver.h"
#include "peripherals/adc_driver.h"
#include "peripherals/pwm_driver.h"
#include "peripherals/peripheral_setup.h"
#include "driverlib.h"
#include <string.h>

// Control configuration (assumed defined in a header like ControlDefines.h)
#ifndef CONTROL_FREQUENCY_HZ
#define CONTROL_FREQUENCY_HZ 10000.0f // 10 kHz control frequency
#endif
#ifndef DC_BUS_VOLTAGE
#define DC_BUS_VOLTAGE 400.0f // DC bus voltage in volts
#endif
#ifndef TARGET_SPEED_RPM
#define TARGET_SPEED_RPM 1500.0f // Target speed in RPM
#endif
#define RPM_TO_RADS (2.0f * 3.1415926535f / 60.0f) // RPM to rad/s conversion
#define PWM_PERIOD_CTRL 1000 // PWM period (adjust based on your pwm_driver.c)

// Data logging configuration
#define DAT_LOG_BUFFER_LENGTH 1000
#define LOGGED_VARIABLES 4 // d, q currents (measured and reference)
#define DECIMATION_FACTOR 10

// Switching states for 3-phase VSI (8 states: 000, 001, 010, 011, 100, 101, 110, 111)
const uint8_t Sout[8][3] = {
    {0, 0, 0}, // V0
    {0, 0, 1}, // V1
    {0, 1, 0}, // V2
    {0, 1, 1}, // V3
    {1, 0, 0}, // V4
    {1, 0, 1}, // V5
    {1, 1, 0}, // V6
    {1, 1, 1}  // V7
};

// Voltage vectors in α-β frame (normalized to Vdc)
const ClarkeData gamma_V[8] = {
    { 0.0f,  0.0f},           // V0: (0,0,0)
    {-2.0f/3.0f, -1.0f/3.0f}, // V1: (0,0,1)
    { 0.0f,      2.0f/3.0f},  // V2: (0,1,0)
    {-2.0f/3.0f,  1.0f/3.0f}, // V3: (0,1,1)
    { 2.0f/3.0f, -1.0f/3.0f}, // V4: (1,0,0)
    { 0.0f,     -2.0f/3.0f},  // V5: (1,0,1)
    { 2.0f/3.0f,  1.0f/3.0f}, // V6: (1,1,0)
    { 0.0f,      0.0f}        // V7: (1,1,1)
};

// Global variables
static PhaseData PhaseCurrMeas; // Measured phase currents (Ia, Ib, Ic)
static ClarkeData abxyCurrMeas; // Measured currents in α-β frame
static ClarkeData abxyCurrkp1;  // Predicted currents (k+1) in α-β frame
static ClarkeData abxyCurrkp2;  // Predicted currents (k+2) in α-β frame
static ClarkeData abxyCurrRef;  // Reference currents in α-β frame
static ClarkeData abxyerror;    // Current error in α-β frame
static ParkData dqxyCurrMeas;   // Measured currents in d-q frame
static ParkData dqxyCurrRef;    // Reference currents in d-q frame
static float sintheta, costheta;// Sine and cosine of rotor angle
static uint8_t x_opt_k = 7;     // Current optimal switching state (init to V7)
static uint8_t x_opt_km1 = 7;   // Previous optimal switching state
static float Cf = 999999999.0f; // Cost function value
static uint32_t cont = 0;       // Control loop counter
static uint16_t dataLogIndex = 0;// Data log index
static uint8_t endDataLog = 0;  // Data log end flag
static uint16_t decPassCounter = 0; // Decimation counter

// Data logging structure
typedef struct {
    float dataLogMat[DAT_LOG_BUFFER_LENGTH][LOGGED_VARIABLES];
    uint16_t structureSize;
    float loadTorque;
    float controlFrequency;
    float targetSpeed;
    uint16_t dataLength;
    uint16_t numberOfVariables;
    uint16_t decimationFactor;
    float startTime;
    float endAngle;
    char controlStrategy[20];
} ExperimentSettings;

static ExperimentSettings g_experiment;

// Rotor data
typedef struct {
    float electricalAngle;
    float mechanicalSpeed;
    float targetMechanicalSpeed;
} RotorData;

static RotorData rotor;

// CLA communication structure
typedef struct {
    float J_opt; // Optimal cost function value
    uint8_t x_opt; // Optimal switching state
} CostFunOpt;

static volatile CostFunOpt CpuToClaCostFunOpt;
static volatile CostFunOpt ClaToCpuCostFunOpt;

// Machine parameters (assumed defined in math_utils.h)
extern float Rs; // Stator resistance
extern float Ls; // Stator inductance
extern float Ts; // Sampling period (1/CONTROL_FREQUENCY_HZ)

// Interrupt handlers (assumed defined in peripheral_setup.c)
extern void handleAdcA1Interrupt(void);
extern void handleAdcB1Interrupt(void);
extern void handleEqepInterrupt(void);
extern void handleSynchronousInterrupt(void);

// Interrupt status flags (assumed defined in peripheral_setup.c)
extern volatile struct {
    uint16_t synchronous : 1;
    uint16_t eqep1 : 1;
} interruptStatus;

// CLA flags (defined in CLA_tasks.cla)
extern volatile uint16_t CLA1_flag1;
extern volatile uint16_t CLA1_flag2;
extern volatile uint16_t CLA1_flag3;

// Function prototypes
static float meanSquaredError(ClarkeData *error);
static void logData(ParkData *meas, ParkData *ref, uint8_t x_opt);
static void computeRotorSpeed(void);
static void setControlActions(const uint8_t *state);

// Main function
void MPC_init(void)
{
    // Initialize rotor
    rotor.targetMechanicalSpeed = RPM_TO_RADS * TARGET_SPEED_RPM;
    rotor.electricalAngle = 0.0f;

    // Initialize experiment settings
    memset(g_experiment.dataLogMat, 0, sizeof(g_experiment.dataLogMat));
    g_experiment.structureSize = sizeof(ExperimentSettings);
    g_experiment.loadTorque = 0.0f; // Adjust as needed
    g_experiment.controlFrequency = CONTROL_FREQUENCY_HZ;
    g_experiment.targetSpeed = TARGET_SPEED_RPM;
    g_experiment.dataLength = DAT_LOG_BUFFER_LENGTH;
    g_experiment.numberOfVariables = LOGGED_VARIABLES;
    g_experiment.decimationFactor = DECIMATION_FACTOR;
    g_experiment.startTime = 0.1f; // Start logging after 0.1s
    g_experiment.endAngle = 0.0f;
    strncpy(g_experiment.controlStrategy, "FCS-MPC", sizeof(g_experiment.controlStrategy));

    // Initialize peripherals
    Device_init();
    Device_initGPIO();
    DINT;
    Interrupt_initModule();
    IER = 0x0000;
    IFR = 0x0000;
    Interrupt_initVectorTable();
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EALLOW;

    // Configure peripherals (from peripheral_setup.c)
    ConfigureGPIO();
    InitEPWM();
    ADC_init();
    eQEP_init();
    CLA_init(); // From cla_driver.c
    MEMCFG_init();

    // Register interrupts
    Interrupt_register(INT_ADCA1, &handleAdcA1Interrupt);
    Interrupt_register(INT_ADCB1, &handleAdcB1Interrupt);
    Interrupt_register(INT_EQEP1, &handleEqepInterrupt);
    Interrupt_register(INT_EPWM1, &handleSynchronousInterrupt);

    // Enable interrupts
    Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_ADCB1);
    Interrupt_enable(INT_EQEP1);
    Interrupt_enable(INT_EPWM1);

    // Configure sync
    SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
    EDIS;

    // Enable PWM clock
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Enable interrupts
    EINT;
    ERTM;

    // Initialize PWM to zero
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0.0f);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, 0.0f);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 0.0f);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, 0.0f);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, 0.0f);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, 0.0f);

    // Reset eQEP
    EQEP_setPosition(EQEP1_BASE, 0U);
}

void MPC_run(void)
{
    float Ts = 1.0f / CONTROL_FREQUENCY_HZ;
    uint32_t seg = (uint32_t)CONTROL_FREQUENCY_HZ;
    float tsim = 5.0f; // Simulation time (adjust as needed)

    // Wait for initial stabilization
    while (cont < 2U * seg) {
        while (!interruptStatus.synchronous);
        interruptStatus.synchronous = 0;
        cont++;
        setControlActions(Sout[x_opt_k]); // Apply initial state (V7)
    }

    cont = 0;
    EQEP_setPosition(EQEP1_BASE, 0U);
    rotor.electricalAngle = 0.0f;

    // Main control loop
    while (cont < ((uint32_t)(tsim * seg))) {
        // Update rotor angle
        sintheta = sinf(rotor.electricalAngle);
        costheta = cosf(rotor.electricalAngle);

        // Wait for ADC and CLA Task 1 (current measurement)
        while (!CLA1_flag1);
        CLA1_flag1 = 0U;

        // Clarke transformation
        abxyCurrMeas.alpha = clarkeAlpha(&PhaseCurrMeas);
        abxyCurrMeas.beta = clarkeBeta(&PhaseCurrMeas);

        // Park transformation
        dqxyCurrMeas.d = parkD(&abxyCurrMeas, costheta, sintheta);
        dqxyCurrMeas.q = parkQ(&abxyCurrMeas, costheta, sintheta);

        // Reference currents (example: constant d-q references)
        dqxyCurrRef.d = 0.0f; // Adjust based on control requirements
        dqxyCurrRef.q = 5.0f; // Example torque-producing current
        abxyCurrRef.alpha = parkAlpha(&dqxyCurrRef, costheta, sintheta);
        abxyCurrRef.beta = parkBeta(&dqxyCurrRef, costheta, sintheta);

        // Predict currents (k+1)
        abxyCurrkp1.alpha = abxyCurrMeas.alpha + (Ts / Ls) * (gamma_V[x_opt_k].alpha * DC_BUS_VOLTAGE - Rs * abxyCurrMeas.alpha);
        abxyCurrkp1.beta = abxyCurrMeas.beta + (Ts / Ls) * (gamma_V[x_opt_k].beta * DC_BUS_VOLTAGE - Rs * abxyCurrMeas.beta);

        // Trigger CLA Task 2 for cost function evaluation
        CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_2);

        // CPU evaluates cost function for states 0-3
        CpuToClaCostFunOpt.J_opt = 999999999.0f;
        for (int k = 0; k < 4; k++) {
            abxyCurrkp2.alpha = abxyCurrkp1.alpha + (Ts / Ls) * (gamma_V[k].alpha * DC_BUS_VOLTAGE - Rs * abxyCurrkp1.alpha);
            abxyCurrkp2.beta = abxyCurrkp1.beta + (Ts / Ls) * (gamma_V[k].beta * DC_BUS_VOLTAGE - Rs * abxyCurrkp1.beta);

            abxyerror.alpha = abxyCurrRef.alpha - abxyCurrkp2.alpha;
            abxyerror.beta = abxyCurrRef.beta - abxyCurrkp2.beta;

            Cf = meanSquaredError(&abxyerror);

            if (CpuToClaCostFunOpt.J_opt > Cf) {
                CpuToClaCostFunOpt.x_opt = k;
                CpuToClaCostFunOpt.J_opt = Cf;
            }
        }

        // Wait for CLA Task 2 (states 4-7)
        while (!CLA1_flag2);
        CLA1_flag2 = 0U;

        // Select optimal state
        x_opt_k = CpuToClaCostFunOpt.J_opt < ClaToCpuCostFunOpt.J_opt ? CpuToClaCostFunOpt.x_opt : ClaToCpuCostFunOpt.x_opt;
        x_opt_km1 = x_opt_k;

        // Apply control actions
        setControlActions(Sout[x_opt_k]);

        // Data logging
        if (cont > (uint32_t)(g_experiment.startTime * seg) && !endDataLog) {
            decPassCounter++;
            if (decPassCounter == DECIMATION_FACTOR) {
                if (dataLogIndex < DAT_LOG_BUFFER_LENGTH) {
                    logData(&dqxyCurrMeas, &dqxyCurrRef, x_opt_k);
                    dataLogIndex++;
                    decPassCounter = 0;
                    if (dataLogIndex == DAT_LOG_BUFFER_LENGTH) {
                        endDataLog = 1;
                        g_experiment.endAngle = rotor.electricalAngle;
                    }
                }
            }
        }

        // Update rotor speed
        if (interruptStatus.eqep1) {
            computeRotorSpeed();
            interruptStatus.eqep1 = 0;
        }

        // Wait for synchronous interrupt
        while (!interruptStatus.synchronous);
        interruptStatus.synchronous = 0;
        cont++;
    }

    // Stop PWM
    setControlActions(Sout[0]); // Apply V0 (all off)
    for (;;);
}

// Cost function: Mean squared error
static float meanSquaredError(ClarkeData *error)
{
    return error->alpha * error->alpha + error->beta * error->beta;
}

// Data logging
static void logData(ParkData *meas, ParkData *ref, uint8_t x_opt)
{
    g_experiment.dataLogMat[dataLogIndex][0] = meas->d;
    g_experiment.dataLogMat[dataLogIndex][1] = meas->q;
    g_experiment.dataLogMat[dataLogIndex][2] = ref->d;
    g_experiment.dataLogMat[dataLogIndex][3] = ref->q;
}

// Compute rotor speed
static void computeRotorSpeed(void)
{
    // Example implementation (adjust based on eQEP setup)
    uint32_t pos = EQEP_getPosition(EQEP1_BASE);
    rotor.electricalAngle = (float)pos * (2.0f * 3.1415926535f / 4096.0f); // Adjust for encoder resolution
    rotor.mechanicalSpeed = rotor.electricalAngle * CONTROL_FREQUENCY_HZ; // Simplified
}

// Set PWM control actions
static void setControlActions(const uint8_t *state)
{
    float duty = (float)PWM_PERIOD_CTRL * 0.5f;
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, state[0] ? duty : 0.0f);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, state[1] ? duty : 0.0f);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, state[2] ? duty : 0.0f);
}
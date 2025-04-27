Three-Phase GaN Inverter Control System for F28379D DSP
Overview
This project implements a control system for a three-phase GaN-based inverter using the Texas Instruments F28379D DSP. The system supports advanced control strategies including Field-Oriented Control (FOC), Model Predictive Control (MPC), Model-Free Deadbeat Control (MFDC), and sensorless operation via a Sliding Mode Observer (SMO). The codebase is designed to be modular, reusable, and optimized for real-time performance, leveraging the DSP's FPU32 and Control Law Accelerator (CLA) for efficient computation.
Recent updates have added mathematical utilities, peripheral drivers (PWM, ADC, CMPSS), a CLA module for parallel task execution, and a centralized peripheral setup module, all tailored for a three-phase inverter system. The codebase is fully documented using Doxygen, ensuring clarity for developers, and adheres to best practices for embedded systems development.
Project Structure
The project is organized into the following directories, reflecting a modular and scalable architecture:
File Architecture
.
├── include/
│   ├── common/
│   │   ├── data_types.h          # Data structures for phase, Clarke, Park, rotor, PI, and ADC data
│   │   └── math_utils.h          # Mathematical utilities for control (Clarke, Park, PI, SMO)
│   └── drivers/
│       ├── PWM_driver.h          # PWM driver for three-phase ePWM modules
│       ├── ADC_driver.h          # ADC driver for phase current/voltage sampling
│       ├── CMPSS_driver.h        # CMPSS driver for protection (e.g., overcurrent)
│       ├── CLA_driver.h          # CLA driver for parallel mathematical tasks
│       └── peripheralsetup.h     # Centralized peripheral initialization
├── src/
│   ├── common/
│   │   └── math_utils.c          # Implementation of mathematical utilities
│   └── drivers/
│       ├── PWM_driver.c          # Implementation of PWM driver
│       ├── ADC_driver.c          # Implementation of ADC driver
│       ├── CMPSS_driver.c        # Implementation of CMPSS driver
│       ├── CLA_driver.c          # Implementation of CLA driver
│       └── CLA_tasks.cla         # CLA task implementations (Clarke, Park, PI)
├── config/
│   ├── system_config.h           # System parameters (SYSCLK_HZ, PWM_FREQUENCY_HZ, etc.)
│   └── control_config.h          # Control parameters (CONTROL_SEL, SENSORLESS_EN, etc.)
├── LICENSE                       # MIT License file
└── README.md                     # Project documentation

Key Files
Common Modules

include/common/data_types.h:

Defines data structures used across the project:
PhaseData: Three-phase currents or voltages (abc).
ClarkeData: Alpha-beta-zero coordinates.
ParkData: D-q-zero coordinates.
RotorData: Rotor angle, speed, and trigonometric values (sintheta, costheta).
PiController: PI controller state (error, integral, output) and parameters (kp, ki).
AdcResult: ADC conversion results for phase measurements.


Ensures consistent data handling for control algorithms and peripheral drivers.


include/common/math_utils.h & src/common/math_utils.c:

Provides mathematical utilities inspired by LabLib.h/.c for inverter control.
Macros: Constants (MATH_PI, MATH_SQRT_2, MATH_SQRT_3), conversions (MATH_DEG_TO_RAD, MATH_PU_TO_PHYS), and utilities (MATH_CLAMP, MATH_MAX, MATH_MIN).
Inline Functions: Fast operations for:
Clarke transformation (math_clarke_transform): abc to alpha-beta-zero.
Park transformation (math_park_transform): alpha-beta to d-q.
Inverse transformations (math_inv_clarke_transform, math_inv_park_transform).
Vector magnitude (math_vector_magnitude): Alpha-beta vector magnitude.
Clamping (math_clamp_float): Limits values within bounds.
PI controller update (math_pi_controller_update): Updates error and integral terms.


Non-Inline Functions: Complex tasks including:
Sine/cosine computation (math_sincos): Uses FPU or optional lookup table.
Arctangent (math_atan2): For angle estimation.
Low-pass filter (math_low_pass_filter): Signal smoothing.
Sliding Mode Observer (math_smo_estimator): Sensorless rotor angle/speed estimation.


Optimized for FPU32, with optional lookup tables for trigonometric functions and fast RAM allocation.



Driver Modules

include/drivers/PWM_driver.h & src/drivers/PWM_driver.c:

Configures three ePWM modules (EPWM1, EPWM2, EPWM3) for the three-phase inverter.
Uses an array of PWMConfig structures to parametrize:
Base address (EPWM1_BASE, etc.).
Counter mode (up-down for symmetric PWM).
Period (based on SYSCLK_HZ and PWM_FREQUENCY_HZ).
Compare values (cmp_a, cmp_b) for duty cycle control.
Dead time for GaN transistor safety.


PWM_init: Initializes PWM modules with synchronized operation, complementary outputs, and dead time.
Optimized for high-frequency PWM (e.g., 20 kHz) using system clock parameters.


include/drivers/ADC_driver.h & src/drivers/ADC_driver.c:

Configures one ADC module (ADCA) to sample three-phase currents or voltages.
Uses an array of ADCConfig structures to specify:
Base address (ADC_A_BASE).
Resolution (12-bit).
Signal mode (single-ended).
Trigger source (EPWM1_SOCA for PWM synchronization).
Channels (ADCIN0-2 for three phases).


ADC_init: Sets up three SOCs (Start of Conversion) for synchronous sampling with a 64-cycle sample window.
Integrates with AdcResult in data_types.h for storing conversion results.


include/drivers/CMPSS_driver.h & src/drivers/CMPSS_driver.c:

Configures one CMPSS module (CMPSS1) for protection tasks (e.g., overcurrent detection).
Uses an array of CMPSSConfig structures to specify:
Base address (CMPSS1_BASE).
Comparator selection (CMPSS_COMP1).
Reference voltage (e.g., 1.65V, midpoint of 3.3V rail).
Hysteresis for noise immunity.


CMPSS_init: Configures the high comparator with a DAC reference and hysteresis.
Extensible for additional protection features (e.g., overvoltage).


include/drivers/CLA_driver.h & src/drivers/CLA_driver.c:

Configures the CLA for parallel execution of mathematical tasks, offloading the main CPU.
Uses an array of CLAConfig structures to parametrize:
Task ID (1 to 8).
Task type (CLA_TASK_CLARKE, CLA_TASK_PARK, CLA_TASK_PI).
Input/output data pointers (e.g., PhaseData, ClarkeData).
Trigger source (ADC_INT1 for ADC completion).
Time step (dt based on CONTROL_FREQUENCY_HZ).


CLA_init: Configures CLA clock, memory (LS0 for program, LS1 for data), task vectors, and triggers.
Integrates with CLA_tasks.cla for task implementations.


src/drivers/CLA_tasks.cla:

Implements CLA tasks adapted from math_utils.h for parallel execution:
Cla1Task1: Clarke transformation (abc to alpha-beta-zero).
Cla1Task2: Park transformation (alpha-beta to d-q).
Cla1Task3: PI controller update (e.g., d-axis current control).


Tasks are triggered by ADC interrupts (ADC_INT1) and use shared data structures (PhaseData, ClarkeData, ParkData, PiController).
Optimized for CLA's FPU and pipeline, with minimal overhead and flag clearing for task completion.



Peripheral Setup

include/drivers/peripheralsetup.h & src/drivers/peripheralsetup.c:
Centralizes initialization of all peripherals: PWM, ADC, CMPSS, and CLA.
peripheral_setup_init: Calls PWM_init, ADC_init, CMPSS_init, and CLA_init to ensure all peripherals are configured before the control loop.
Updated to include CLA initialization for parallel computation support.



Configuration

config/system_config.h (assumed):
Defines system parameters:
SYSCLK_HZ: System clock (e.g., 200 MHz).
PWM_FREQUENCY_HZ: PWM frequency (e.g., 20 kHz).
CONTROL_FREQUENCY_HZ: Control loop frequency (e.g., 10 kHz).
VREF: Reference voltage (e.g., 3.3V).
Motor parameters: RS (stator resistance), LD/LQ (inductances), P (pole pairs).




config/control_config.h (assumed):
Defines control parameters:
CONTROL_SEL: Selects control strategy (FOC, MPC, MFDC).
SENSORLESS_EN: Enables sensorless operation.
IS_MAX: Maximum current limit.
KP_ID/KI_ID: PI controller gains for d-axis current.





Features

Three-Phase Support: Handles three-phase currents/voltages with dedicated PWM and ADC configurations for phases A, B, and C.
Control Strategies:
FOC: Supported via Clarke/Park transformations and PI controllers in math_utils and CLA.
MPC: Extensible with cost function evaluation (can be added to math_utils or CLA).
MFDC: Extensible for deadbeat control (can be added to math_utils or CLA).
Sensorless: SMO implemented in math_smo_estimator for rotor angle/speed estimation.


CLA Offloading: Critical tasks (Clarke, Park, PI) run on the CLA, freeing the main CPU for higher-level tasks or additional control logic.
Modular Design:
Common utilities in math_utils for reusability across FOC, MPC, and MFDC.
Driver modules (PWM_driver, ADC_driver, CMPSS_driver, CLA_driver) use array-based configurations for scalability.


Optimization:
Leverages FPU32 for floating-point calculations in math_utils and CLA.
Optional sine/cosine lookup table in math_sincos for reduced computation time.
CLA tasks use dedicated LS0/LS1 RAM for low-latency execution.
Fast RAM (e.g., .ebss, RAMGS0) used for critical data and lookup tables.


Protection: CMPSS module configured for overcurrent detection, with hysteresis for noise immunity.
Documentation: Comprehensive Doxygen comments for all files, structures, functions, and macros, ensuring maintainability.

Dependencies

Texas Instruments DriverLib: For peripheral access (ePWM, ADC, CMPSS, CLA).
Code Composer Studio (CCS): For building, debugging, and flashing the F28379D.
C2000Ware: Provides DriverLib and device support files for the F28379D.
F28379D DSP: Target hardware with FPU32 and CLA support.

Building the Project

Setup Environment:

Install CCS and C2000Ware.
Import the project into CCS.
Add include and src directories to the build path.


Configure Parameters:

Update system_config.h with system-specific values (e.g., SYSCLK_HZ, PWM_FREQUENCY_HZ, VREF).
Update control_config.h with control-specific values (e.g., CONTROL_SEL, SENSORLESS_EN, KP_ID).


Build:

Compile the project in CCS (use Release mode for optimized performance).
Link against DriverLib and C2000Ware libraries.


Flash:

Flash the compiled binary to the F28379D using a JTAG debugger (e.g., XDS110).



Usage
Initialization
In main.c, initialize the system as follows:
#include "drivers/peripheralsetup.h"
#include "common/data_types.h"
#include "config/control_config.h"

void main(void) {
    // Initialize device and interrupts
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // Initialize peripherals (PWM, ADC, CMPSS, CLA)
    peripheral_setup_init();

    // Initialize common data
    data_types_init();

    // Main control loop
    while (1) {
        // Access ADC results
        const AdcResult *adc = get_adc_result();
        // Use CLA results (e.g., park_data, pi_controller)
        const ParkData *park = &park_data;
        const PiController *pi = &pi_controller;
        // Update PWM or control logic
        set_pwm_period_ctrl(pwm_configs[0].period);
    }
}

Control Loop

ADC: Samples phase currents on ADCIN0-2, triggered by EPWM1_SOCA, and stores results in AdcResult.
CLA: Executes Clarke (Cla1Task1), Park (Cla1Task2), and PI (Cla1Task3) tasks in parallel, updating clarke_data, park_data, and pi_controller.
CPU: Uses CLA results for higher-level control (e.g., FOC, MPC) or runs math_smo_estimator for sensorless operation.
PWM: Updates duty cycles via pwm_configs[i].cmp_a/b for SVPWM or other modulation schemes.
CMPSS: Monitors for faults (e.g., overcurrent) and can trigger interrupts for protection.

Example Workflow (FOC with Sensorless)

ADC samples phase currents (phase_data.current_a/b/c).
CLA Task 1 computes Clarke transformation (clarke_data.alpha/beta/zero).
CLA Task 2 computes Park transformation (park_data.d/q/zero).
CLA Task 3 updates PI controller (pi_controller.output) for d-axis current control.
CPU runs math_smo_estimator (or a future CLA SMO task) to estimate rotor angle/speed (rotor_est).
CPU updates PWM compare values (pwm_configs[i].cmp_a/b) for SVPWM modulation.

Future Improvements

CLA Enhancements:
Add a SMO task (CLA_TASK_SMO) for sensorless control, adapting math_smo_estimator.
Implement SVPWM generation in a CLA task for direct PWM updates.
Utilize additional CLA tasks (up to 8) for other computations (e.g., PLL, deadbeat control).


Control Strategies:
Extend math_utils with MPC cost functions or MFDC deadbeat algorithms.
Add a Phase-Locked Loop (PLL) for enhanced sensorless estimation.


Protection:
Configure CMPSS interrupts for real-time fault handling (e.g., overcurrent, overvoltage).
Add support for multiple CMPSS modules for comprehensive protection.


Optimization:
Implement IQmath for fixed-point arithmetic if floating-point is not preferred.
Profile CLA task latency using CCS debugger to optimize execution time.
Optimize memory usage by fine-tuning LS0/LS1 RAM allocation.


Testing:
Develop unit tests for math_utils functions (e.g., Clarke, Park).
Perform hardware-in-the-loop (HIL) testing to validate control algorithms.
Verify ADC sampling and PWM synchronization with oscilloscope measurements.



Contributing
Contributions are welcome! To contribute:

Fork the repository.
Create a feature branch (git checkout -b feature/my-feature).
Commit changes with descriptive messages (git commit -m "Add feature X").
Push to the branch (git push origin feature/my-feature).
Open a pull request with a detailed description of changes.

Please ensure all code includes Doxygen comments and is tested on the F28379D DSP.
License
This project is licensed under the MIT License. See the LICENSE file for details.
Acknowledgments

Texas Instruments: For the F28379D DSP, C2000Ware, and DriverLib.
Pablo Mora Moreno: Project lead and primary contributor.

Last Updated: April 27, 2025

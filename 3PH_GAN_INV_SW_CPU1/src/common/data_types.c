/*
 * data_types.c
 *
 * @file    data_types.c
 * @brief   Implementation of common data structures and access functions for
 *          a three-phase GaN inverter control system.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 */

#include "common/data_types.h"
#include <string.h>

/* Global Variable Definitions */

/** @brief Global interrupt status, stored in fast RAM */
#pragma DATA_SECTION(g_interrupt_status, ".ebss")
volatile InterruptStatus g_interrupt_status;

/** @brief Global rotor data, stored in fast RAM */
#pragma DATA_SECTION(g_rotor, ".ebss")
volatile RotorData g_rotor;

/** @brief Global PWM period control, stored in fast RAM */
#pragma DATA_SECTION(g_pwm_period_ctrl, ".ebss")
volatile uint32_t g_pwm_period_ctrl;

/** @brief Global ADC results, stored in fast RAM */
#pragma DATA_SECTION(g_adc_result, ".ebss")
volatile AdcResult g_adc_result;

/** @brief Global experimental settings, stored in general RAM */
#pragma DATA_SECTION(g_experiment, "DATA_LOG_SECTION")
ExperimentSettings g_experiment;

/** @brief Global main clock, stored in fast RAM */
#pragma DATA_SECTION(g_main_clock, ".ebss")
volatile float g_main_clock;

/** @brief Total test duration, stored in fast RAM */
#pragma DATA_SECTION(g_simultaion_time, ".ebss")
volatile float g_simultaion_time;

#if SPEED_PROFILE == TRAPEZOIDAL
/** @brief Global trapezoidal speed profile, stored in fast RAM */
#pragma DATA_SECTION(g_speed_profile, ".ebss")
TrapezoidalData g_speed_profile;
#endif

#if CONTROL_MODE == SPEED_CTRL
/** @brief Global speed controller, stored in fast RAM */
#pragma DATA_SECTION(g_speed_controller, ".ebss")
PiController g_speed_controller;
#elif CONTROL_MODE == POS_CTRL
/** @brief Global angle controller, stored in fast RAM */
#pragma DATA_SECTION(g_angle_controller, ".ebss")
PiController g_angle_controller;
#elif CONTROL_MODE == TORQUE_CTRL
/** @brief Global torque controller, stored in fast RAM */
#pragma DATA_SECTION(g_torque_controller, ".ebss")
PiController g_torque_controller;
#endif

/** @brief Global integration gain, stored in fast RAM */
#pragma DATA_SECTION(g_integration_gain, ".ebss")
float g_integration_gain;

/** @brief Global reference currents in d-q frame, stored in fast RAM */
#pragma DATA_SECTION(g_dq_curr_ref, ".ebss")
ParkData g_dq_curr_ref;

/** @brief Global saturated currents in d-q frame, stored in fast RAM */
#pragma DATA_SECTION(g_dq_curr_sat, ".ebss")
ParkData g_dq_curr_sat;

/** @brief Global measured phase currents, stored in fast RAM */
#pragma DATA_SECTION(g_phase_curr_meas, ".ebss")
PhaseData g_phase_curr_meas;

/** @brief Global measured currents in alpha-beta frame, stored in fast RAM */
#pragma DATA_SECTION(g_ab_curr_meas, ".ebss")
ClarkeData g_ab_curr_meas;

/** @brief Global reference currents in alpha-beta frame, stored in fast RAM */
#pragma DATA_SECTION(g_ab_curr_ref, ".ebss")
ClarkeData g_ab_curr_ref;

/** @brief Global measured currents in d-q frame, stored in fast RAM */
#pragma DATA_SECTION(g_dq_curr_meas, ".ebss")
ParkData g_dq_curr_meas;

/** @brief Global profiling data, stored in fast RAM */
#pragma DATA_SECTION(g_profiling, ".ebss")
DelayMeasurement g_profiling[NUM_PROFILING_POINTS];

/* Function Implementations */

void data_types_init(void) {
    // Initialize interrupt status
    g_interrupt_status.adca1 = 0;
    g_interrupt_status.adcb1 = 0;
    g_interrupt_status.adcc1 = 0;
    g_interrupt_status.adca1_mid = 0;
    g_interrupt_status.adcb1_mid = 0;
    g_interrupt_status.adcc1_mid = 0;
    g_interrupt_status.synchronous = false;

    // Initialize rotor data
    g_rotor.electrical_angle = 0.0f;
    g_rotor.ref_mechanical_speed = 0.0f;
    g_rotor.target_mechanical_speed = RPM_TO_RADS(TARGET_SPEED_RPM);
    g_rotor.current_mechanical_speed = 0.0f;
    g_rotor.prev_mechanical_speed = 0.0f;
    g_rotor.current_electrical_speed = 0.0f;
    g_rotor.filtered_mechanical_speed = 0.0f;
    g_rotor.sintheta = 0.0f;
    g_rotor.costheta = 0.0f;

    // Initialize PWM period
    g_pwm_period_ctrl = (uint32_t)(200000000.0f / CONTROL_FREQUENCY_HZ);

    // Initialize ADC results
    g_adc_result.current_a = 0;
    g_adc_result.current_b = 0;
    g_adc_result.current_c = 0;

    // Initialize experimental settings
    memset(g_experiment.data_log, 0.0f, sizeof(g_experiment.data_log));
    g_experiment.structure_size = sizeof(ExperimentSettings);
    g_experiment.switching_frequency = PWM_FREQUENCY_HZ;
    g_experiment.load_torque = LOAD_TORQUE;
    g_experiment.control_frequency = CONTROL_FREQUENCY_HZ;
    g_experiment.target_speed = TARGET_SPEED;
    g_experiment.data_length = DAT_LOG_BUFFER_LENGTH;
    g_experiment.num_variables = LOGGED_VARIABLES;
    g_experiment.decimation_factor = DECIMATION_FACTOR;
    g_experiment.start_time = 0.0f;
    g_experiment.end_angle = 0.0f;
    strncpy(g_experiment.control_strategy, CONTROL_SCHEME, CONTROL_STR_MAX_LEN);

    // Initialize main clock
    g_main_clock = 0;

    // Initialize conditional variables
    #if SPEED_PROFILE == TRAPEZOIDAL
    g_speed_profile.tUp = 0;
    g_speed_profile.tDown = 0;
    g_speed_profile.tConst = 0;
    g_speed_profile.ReferenceValue = RPM_TO_RADS(TARGET_SPEED_RPM);
    #endif

    #if CONTROL_MODE == SPEED_CTRL
    g_speed_controller.kp = KP_SPEED;
    g_speed_controller.ki = KI_SPEED;
    g_speed_controller.integral = 0.0f;
    g_speed_controller.prev_integral = 0.0f;
    g_speed_controller.error = 0.0f;
    g_speed_controller.prev_error = 0.0f;
    #elif CONTROL_MODE == POS_CTRL
    g_angle_controller.kp = KP_POS;
    g_angle_controller.ki = KI_POS;
    g_angle_controller.integral = 0.0f;
    g_angle_controller.prev_integral = 0.0f;
    g_angle_controller.error = 0.0f;
    g_angle_controller.prev_error = 0.0f;
    #elif CONTROL_MODE == TORQUE_CTRL
    g_torque_controller.kp = KP_TORQUE;
    g_torque_controller.ki = KI_TORQUE;
    g_torque_controller.integral = 0.0f;
    g_torque_controller.prev_integral = 0.0f;
    g_torque_controller.error = 0.0f;
    g_torque_controller.prev_error = 0.0f;
    #endif

    // Initialize currents and gains
    g_integration_gain = 0.5/CONTROL_FREQUENCY_HZ;
    g_dq_curr_ref.d = 0.0f;
    g_dq_curr_ref.q = 0.0f;
    g_dq_curr_ref.zero = 0.0f;
    g_dq_curr_sat.d = 0.0f;
    g_dq_curr_sat.q = 0.0f;
    g_dq_curr_sat.zero = 0.0f;
    g_phase_curr_meas.current_a = 0.0f;
    g_phase_curr_meas.current_b = 0.0f;
    g_phase_curr_meas.current_c = 0.0f;
    g_ab_curr_meas.alpha = 0.0f;
    g_ab_curr_meas.beta = 0.0f;
    g_ab_curr_meas.zero = 0.0f;
    g_ab_curr_ref.alpha = 0.0f;
    g_ab_curr_ref.beta = 0.0f;
    g_ab_curr_ref.zero = 0.0f;
    g_dq_curr_meas.d = 0.0f;
    g_dq_curr_meas.q = 0.0f;
    g_dq_curr_meas.zero = 0.0f;

    // Initialize profiling
    uint32_t i = 0;
    for (i = 0; i < NUM_PROFILING_POINTS; i++) {
        g_profiling[i].start = 0.0f;
        g_profiling[i].end = 0.0f;
        g_profiling[i].elapsed = 0.0f;
    }
}

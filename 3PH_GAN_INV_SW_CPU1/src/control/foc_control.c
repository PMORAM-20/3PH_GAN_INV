/*
 * foc_control.c
 *
 * @file    foc_control.c
 * @brief   Implementation of FOC-specific data structures and access functions.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 */

#include "control/foc_types.h"

/* Global Variable Definitions */

/** @brief Global d-axis current controller, stored in fast RAM */
#pragma DATA_SECTION(g_d_current_controller, ".ebss")
static PiController g_d_current_controller = {KP_ID, KI_ID, 0.0f, 0.0f, 0.0f, 0.0f};

/** @brief Global q-axis current controller, stored in fast RAM */
#pragma DATA_SECTION(g_q_current_controller, ".ebss")
static PiController g_q_current_controller = {KP_IQ, KI_IQ, 0.0f, 0.0f, 0.0f, 0.0f};

/** @brief Global reference voltages in d-q frame, stored in fast RAM */
#pragma DATA_SECTION(g_dqxy_volt_ref, ".ebss")
static ParkData g_dqxy_volt_ref = {0.0f, 0.0f, 0.0f};

/** @brief Global per-unit phase voltages, stored in fast RAM */
#pragma DATA_SECTION(g_phase_volt_pu, ".ebss")
static PhaseData g_phase_volt_pu = {0.0f, 0.0f, 0.0f};

/** @brief Global saturated voltages in d-q frame, stored in fast RAM */
#pragma DATA_SECTION(g_dqxy_volt_sat, ".ebss")
static ParkData g_dqxy_volt_sat = {0.0f, 0.0f, 0.0f};

/** @brief Global reference voltages in alpha-beta frame, stored in fast RAM */
#pragma DATA_SECTION(g_abxy_volt_ref, ".ebss")
static ClarkeData g_abxy_volt_ref = {0.0f, 0.0f, 0.0f};

/** @brief Global reference phase voltages, stored in fast RAM */
#pragma DATA_SECTION(g_phase_volt_ref, ".ebss")
static PhaseData g_phase_volt_ref = {0.0f, 0.0f, 0.0f};

/** @brief Global maximum phase voltage, stored in fast RAM */
#pragma DATA_SECTION(g_max_phase_voltage, ".ebss")
static float g_max_phase_voltage = 0.0f;

/* Function Implementations */

void foc_types_init(void) {
    g_d_current_controller.kp = KP_ID;
    g_d_current_controller.ki = KI_ID;
    g_d_current_controller.integral = 0.0f;
    g_d_current_controller.prev_integral = 0.0f;
    g_d_current_controller.error = 0.0f;
    g_d_current_controller.prev_error = 0.0f;

    g_q_current_controller.kp = KP_IQ;
    g_q_current_controller.ki = KI_IQ;
    g_q_current_controller.integral = 0.0f;
    g_q_current_controller.prev_integral = 0.0f;
    g_q_current_controller.error = 0.0f;
    g_q_current_controller.prev_error = 0.0f;

    g_dqxy_volt_ref.d = 0.0f;
    g_dqxy_volt_ref.q = 0.0f;
    g_dqxy_volt_ref.zero = 0.0f;

    g_phase_volt_pu.current_a = 0.0f;
    g_phase_volt_pu.current_b = 0.0f;
    g_phase_volt_pu.current_c = 0.0f;

    g_dqxy_volt_sat.d = 0.0f;
    g_dqxy_volt_sat.q = 0.0f;
    g_dqxy_volt_sat.zero = 0.0f;

    g_abxy_volt_ref.alpha = 0.0f;
    g_abxy_volt_ref.beta = 0.0f;
    g_abxy_volt_ref.zero = 0.0f;

    g_phase_volt_ref.current_a = 0.0f;
    g_phase_volt_ref.current_b = 0.0f;
    g_phase_volt_ref.current_c = 0.0f;

    g_max_phase_voltage = VQ_MAX;
}
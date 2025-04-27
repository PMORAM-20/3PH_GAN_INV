/*
 * math_utils.c
 *
 * @file    math_utils.c
 * @brief   Implementation of mathematical utilities for a three-phase GaN
 *          inverter control system using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Implements complex mathematical functions such as sine/cosine lookup,
 *          arctangent, low-pass filter, and sliding mode observer. Optimized for
 *          FPU32 and the F28379D.
 */

#include "common/math_utils.h"
#include <string.h>

/* Lookup Table for Sine/Cosine (optional, can use FPU) */
#define SINCOS_TABLE_SIZE 1024
static float sincos_table[SINCOS_TABLE_SIZE][2]; /* [sin, cos] */

/* Function Implementations */

void math_utils_init(void)
{
    /* Initialize sine/cosine lookup table (optional) */
    for (uint32_t i = 0; i < SINCOS_TABLE_SIZE; i++) {
        float angle = (2.0f * MATH_PI * i) / SINCOS_TABLE_SIZE;
        sincos_table[i][0] = sinf(angle); /* sine */
        sincos_table[i][1] = cosf(angle); /* cosine */
    }
}

void math_sincos(float angle, float *sin_val, float *cos_val)
{
    /* Normalize angle to [0, 2*PI) */
    while (angle >= 2.0f * MATH_PI) angle -= 2.0f * MATH_PI;
    while (angle < 0.0f) angle += 2.0f * MATH_PI;

    /* Use lookup table or FPU */
    #ifdef USE_SINCOS_TABLE
    uint32_t index = (uint32_t)((angle / (2.0f * MATH_PI)) * SINCOS_TABLE_SIZE);
    index = index % SINCOS_TABLE_SIZE;
    *sin_val = sincos_table[index][0];
    *cos_val = sincos_table[index][1];
    #else
    *sin_val = sinf(angle);
    *cos_val = cosf(angle);
    #endif
}

float math_atan2(float y, float x)
{
    return atan2f(y, x);
}

float math_low_pass_filter(float input, float prev_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * prev_output;
}

void math_smo_estimator(const ClarkeData *clarke_meas, const ClarkeData *clarke_volt,
                        const RotorData *rotor, RotorData *rotor_est)
{
    /* Sliding Mode Observer for sensorless control */
    /* Parameters (tunable, could be in control_config.h) */
    const float k_smo = 0.1f; /* SMO gain */
    const float dt = 1.0f / CONTROL_FREQUENCY_HZ;

    /* Observer states */
    static float i_alpha_est = 0.0f, i_beta_est = 0.0f;
    static float e_alpha = 0.0f, e_beta = 0.0f;

    /* Current errors */
    float error_alpha = clarke_meas->alpha - i_alpha_est;
    float error_beta = clarke_meas->beta - i_beta_est;

    /* Sliding mode terms */
    float z_alpha = k_smo * (error_alpha > 0.0f ? 1.0f : -1.0f);
    float z_beta = k_smo * (error_beta > 0.0f ? 1.0f : -1.0f);

    /* Update observer */
    i_alpha_est += (clarke_volt->alpha - z_alpha - RS * i_alpha_est) * dt / LD;
    i_beta_est += (clarke_volt->beta - z_beta - RS * i_beta_est) * dt / LQ;

    /* Estimate back-EMF */
    e_alpha = z_alpha;
    e_beta = z_beta;

    /* Estimate rotor angle using atan2 */
    rotor_est->electrical_angle = math_atan2(e_beta, e_alpha);

    /* Estimate speed (simple differentiation) */
    rotor_est->current_electrical_speed = (rotor_est->electrical_angle - rotor->electrical_angle) / dt;
    rotor_est->current_mechanical_speed = rotor_est->current_electrical_speed / P;
    rotor_est->filtered_mechanical_speed = math_low_pass_filter(
        rotor_est->current_mechanical_speed,
        rotor->filtered_mechanical_speed,
        0.1f
    );

    /* Update sin/cos for Park transformation */
    math_sincos(rotor_est->electrical_angle, &rotor_est->sintheta, &rotor_est->costheta);
}

/*
 * math_utils.h
 *
 * @file    math_utils.h
 * @brief   Header file providing mathematical utilities for a three-phase GaN
 *          inverter control system using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Provides macros, inline functions, and function prototypes for common
 *          mathematical operations, including coordinate transformations, PI control,
 *          saturation, and sensorless algorithms. Inspired by LabLib.h, optimized
 *          for FPU32 and the F28379D.
 *
 * @note    Requires data_types.h for data structures and system_config.h for
 *          system parameters. Use IQmathLib for fixed-point arithmetic if needed.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_COMMON_MATH_UTILS_H_
#define INCLUDE_COMMON_MATH_UTILS_H_

/* Standard Includes */
#include <stdint.h>
#include <math.h>

/* Project Includes */
#include "common/data_types.h"
#include "config/system_config.h"
#include "config/control_config.h"

/* Macros */

/** @brief Mathematical constant PI */
#define MATH_PI 3.141592653589793f

/** @brief Square root of 2 */
#define MATH_SQRT_2 1.414213562373095f

/** @brief Square root of 3 */
#define MATH_SQRT_3 1.732050807568877f

/** @brief Converts degrees to radians */
#define MATH_DEG_TO_RAD(deg) ((deg) * MATH_PI / 180.0f)

/** @brief Converts radians to degrees */
#define MATH_RAD_TO_DEG(rad) ((rad) * 180.0f / MATH_PI)

/** @brief Converts per-unit to physical value */
#define MATH_PU_TO_PHYS(pu, base) ((pu) * (base))

/** @brief Converts physical to per-unit value */
#define MATH_PHYS_TO_PU(phys, base) ((phys) / (base))

/** @brief Returns the maximum of two values */
#define MATH_MAX(a, b) ((a) > (b) ? (a) : (b))

/** @brief Returns the minimum of two values */
#define MATH_MIN(a, b) ((a) < (b) ? (a) : (b))

/** @brief Clamps a value between min and max */
#define MATH_CLAMP(val, min, max) MATH_MIN(MATH_MAX((val), (min)), (max))

/* Inline Functions */

/**
 * @brief Performs Clarke transformation (abc to alpha-beta-zero).
 * @param[in] phase Pointer to the PhaseData structure with phase currents.
 * @param[out] clarke Pointer to the ClarkeData structure for results.
 */
static inline void math_clarke_transform(const PhaseData *phase, ClarkeData *clarke)
{
    clarke->alpha = (2.0f / 3.0f) * (phase->current_a - 0.5f * (phase->current_b + phase->current_c));
    clarke->beta = (MATH_SQRT_3 / 3.0f) * (phase->current_b - phase->current_c);
    clarke->zero = (1.0f / 3.0f) * (phase->current_a + phase->current_b + phase->current_c);
}

/**
 * @brief Performs inverse Clarke transformation (alpha-beta-zero to abc).
 * @param[in] clarke Pointer to the ClarkeData structure with alpha-beta currents.
 * @param[out] phase Pointer to the PhaseData structure for results.
 */
static inline void math_inv_clarke_transform(const ClarkeData *clarke, PhaseData *phase)
{
    phase->current_a = clarke->alpha + 0.5f * clarke->zero;
    phase->current_b = (-0.5f * clarke->alpha + (MATH_SQRT_3 / 2.0f) * clarke->beta) + 0.5f * clarke->zero;
    phase->current_c = (-0.5f * clarke->alpha - (MATH_SQRT_3 / 2.0f) * clarke->beta) + 0.5f * clarke->zero;
}

/**
 * @brief Performs Park transformation (alpha-beta to d-q).
 * @param[in] clarke Pointer to the ClarkeData structure with alpha-beta currents.
 * @param[in] rotor Pointer to the RotorData structure with electrical angle.
 * @param[out] park Pointer to the ParkData structure for results.
 */
static inline void math_park_transform(const ClarkeData *clarke, const RotorData *rotor, ParkData *park)
{
    park->d = clarke->alpha * rotor->costheta + clarke->beta * rotor->sintheta;
    park->q = -clarke->alpha * rotor->sintheta + clarke->beta * rotor->costheta;
    park->zero = clarke->zero;
}

/**
 * @brief Performs inverse Park transformation (d-q to alpha-beta).
 * @param[in] park Pointer to the ParkData structure with d-q currents.
 * @param[in] rotor Pointer to the RotorData structure with electrical angle.
 * @param[out] clarke Pointer to the ClarkeData structure for results.
 */
static inline void math_inv_park_transform(const ParkData *park, const RotorData *rotor, ClarkeData *clarke)
{
    clarke->alpha = park->d * rotor->costheta - park->q * rotor->sintheta;
    clarke->beta = park->d * rotor->sintheta + park->q * rotor->costheta;
    clarke->zero = park->zero;
}

/**
 * @brief Computes the magnitude of a vector in alpha-beta frame.
 * @param[in] clarke Pointer to the ClarkeData structure.
 * @return Magnitude of the vector.
 */
static inline float math_vector_magnitude(const ClarkeData *clarke)
{
    return sqrtf(clarke->alpha * clarke->alpha + clarke->beta * clarke->beta);
}

/**
 * @brief Clamps a value between a minimum and maximum.
 * @param[in] value Value to clamp.
 * @param[in] min Minimum allowed value.
 * @param[in] max Maximum allowed value.
 * @return Clamped value.
 */
static inline float math_clamp_float(float value, float min, float max)
{
    return MATH_MIN(MATH_MAX(value, min), max);
}

/**
 * @brief Updates a PI controller.
 * @param[in,out] pi Pointer to the PiController structure.
 * @param[in] error Current error.
 * @param[in] dt Time step (s).
 */
static inline void math_pi_controller_update(PiController *pi, float error, float dt)
{
    pi->prev_error = pi->error;
    pi->error = error;
    pi->prev_integral = pi->integral;
    pi->integral += pi->ki * error * dt;
    pi->integral = math_clamp_float(pi->integral, -IS_MAX, IS_MAX);
}

/* Function Prototypes */

/**
 * @brief Initializes the math utilities module.
 */
void math_utils_init(void);

/**
 * @brief Computes sine and cosine of an angle using a lookup table or FPU.
 * @param[in] angle Angle in radians.
 * @param[out] sin_val Pointer to store sine value.
 * @param[out] cos_val Pointer to store cosine value.
 */
void math_sincos(float angle, float *sin_val, float *cos_val);

/**
 * @brief Computes arctangent (atan2) for angle calculation.
 * @param[in] y Y-component (e.g., beta).
 * @param[in] x X-component (e.g., alpha).
 * @return Angle in radians.
 */
float math_atan2(float y, float x);

/**
 * @brief Implements a first-order low-pass filter.
 * @param[in] input Current input value.
 * @param[in] prev_output Previous output value.
 * @param[in] alpha Filter coefficient (0 to 1).
 * @return Filtered output.
 */
float math_low_pass_filter(float input, float prev_output, float alpha);

/**
 * @brief Implements a sliding mode observer (SMO) for sensorless control.
 * @param[in] clarke_meas Measured currents in alpha-beta frame.
 * @param[in] clarke_volt Applied voltages in alpha-beta frame.
 * @param[in] rotor Current rotor data.
 * @param[out] rotor_est Estimated rotor data (angle, speed).
 */
void math_smo_estimator(const ClarkeData *clarke_meas, const ClarkeData *clarke_volt,
                        const RotorData *rotor, RotorData *rotor_est);

#endif /* INCLUDE_COMMON_MATH_UTILS_H_ */

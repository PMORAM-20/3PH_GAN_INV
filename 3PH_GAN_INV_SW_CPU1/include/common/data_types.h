/*
 * data_types.h
 *
 * @file    data_types.h
 * @brief   Header file defining common data structures and global variables for
 *          a three-phase GaN inverter control system using the F28379D DSP.
 * @author  Pablo Mora Moreno
 * @date    April 27, 2025
 * @version 1.0.0
 *
 * @details Defines structures for ADC results, phase currents, transformations,
 *          rotor data, PI controllers, experimental settings, and time management.
 *          Includes extern declarations for global variables and access functions
 *          shared across control schemes (FOC, MPC, MFDC). Optimized for memory
 *          and performance on the F28379D.
 *
 * @note    Requires system_config.h and control_config.h for parameters.
 *          Specific data types are defined in control/<scheme>_types.h.
 *          Use IQmathLib for fixed-point arithmetic where applicable.
 *
 * @license MIT License (see LICENSE file in project root)
 */

#ifndef INCLUDE_COMMON_DATA_TYPES_H_
#define INCLUDE_COMMON_DATA_TYPES_H_

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "config/system_config.h"
#include "config/control_config.h"

/* Macros */

/** @brief Maximum length of control strategy string */
#define CONTROL_STR_MAX_LEN 32

/* Types */

/**
 * @brief Stores raw ADC results for three-phase currents.
 */
typedef struct {
    uint16_t current_a; /**< ADC result for phase A current (0-4095) */
    uint16_t current_b; /**< ADC result for phase B current (0-4095) */
    uint16_t current_c; /**< ADC result for phase C current (0-4095) */
} AdcResult;

/**
 * @brief Stores processed phase currents (A).
 */
typedef struct {
    float current_a; /**< Phase A current (A) */
    float current_b; /**< Phase B current (A) */
    float current_c; /**< Phase C current (A) */
} PhaseData;

/**
 * @brief Stores Clarke transformation results (alpha-beta-zero).
 */
typedef struct {
    float alpha; /**< Alpha-axis component (A) */
    float beta;  /**< Beta-axis component (A) */
    float zero;  /**< Zero-sequence component (A) */
} ClarkeData;

/**
 * @brief Stores Park transformation results (d-q-zero).
 */
typedef struct {
    float d;    /**< D-axis component (A) */
    float q;    /**< Q-axis component (A) */
    float zero; /**< Zero-sequence component (A) */
} ParkData;

/**
 * @brief Stores rotor position and speed data.
 */
typedef struct {
    float electrical_angle;         /**< Rotor electrical angle (rad) */
    float ref_mechanical_speed;     /**< Reference mechanical speed (rad/s) */
    float target_mechanical_speed;  /**< Target mechanical speed (rad/s) */
    float current_mechanical_speed; /**< Current mechanical speed (rad/s) */
    float prev_mechanical_speed;    /**< Previous mechanical speed (rad/s) */
    float current_electrical_speed; /**< Current electrical speed (rad/s) */
    float filtered_mechanical_speed; /**< Filtered mechanical speed (rad/s) */
    float sintheta;                 /**< Alpha-axis projection of rotor position */
    float costheta;                 /**< Beta-axis projection of rotor position */
} RotorData;

/**
 * @brief Stores PI controller parameters and states.
 */
typedef struct {
    float kp;            /**< Proportional gain */
    float ki;            /**< Integral gain */
    float integral;      /**< Current integral term */
    float prev_integral; /**< Previous integral term */
    float error;         /**< Current error */
    float prev_error;    /**< Previous error */
} PiController;

/**
 * @brief Stores experimental settings and data logging buffer.
 */
typedef struct {
    float structure_size;         /**< Size of control structure (bytes) */
    float switching_frequency;    /**< Switching frequency (Hz) */
    float load_torque;            /**< Load torque (Nm) */
    float control_frequency;      /**< Control frequency (Hz) */
    float target_speed;           /**< Target speed (rad/s) */
    float data_length;            /**< Length of data logging buffer */
    float num_variables;          /**< Number of logged variables */
    float decimation_factor;      /**< Decimation factor for logging */
    float start_time;             /**< Experiment start time (s) */
    float end_angle;              /**< Final rotor angle (rad) */
    char control_strategy[CONTROL_STR_MAX_LEN]; /**< Control strategy name */
    float data_log[DAT_LOG_BUFFER_LENGTH][LOGGED_VARIABLES]; /**< Data logging buffer */
} ExperimentSettings;

/**
 * @brief Stores interrupt status for ADC sampling.
 */
typedef struct {
    volatile uint16_t adca1;      /**< ADC A1 result */
    volatile uint16_t adcb1;      /**< ADC B1 result */
    volatile uint16_t adcc1;      /**< ADC C1 result */
    volatile uint16_t adca1_mid;  /**< ADC A1 mid-point result */
    volatile uint16_t adcb1_mid;  /**< ADC B1 mid-point result */
    volatile uint16_t adcc1_mid;  /**< ADC C1 mid-point result */
    volatile bool synchronous;    /**< Synchronous sampling flag */
} InterruptStatus;

/**
 * @brief Stores delay measurement data.
 */
typedef struct {
    volatile float start;   /**< Start time (s) */
    volatile float end;     /**< End time (s) */
    volatile float elapsed; /**< Elapsed time (s) */
} DelayMeasurement;

/**
 * @brief Stores a time structure composed of seconds and microseconds.
 */
typedef struct {
    uint32_t seconds;  /**< Seconds component */
    uint32_t useconds; /**< Microseconds component */
} Time;

/**
 * @brief Stores parameters for a trapezoidal time waveform.
 */
typedef struct {
    Time tUp;            /**< Time for rising edge */
    Time tDown;          /**< Time for falling edge */
    Time tConst;         /**< Time for constant phase */
    float ReferenceValue; /**< Reference value for waveform */
} TrapezoidalData;

/* Extern Variable Declarations */

/**
 * @brief Global interrupt status for ADC sampling.
 */
extern volatile InterruptStatus g_interrupt_status;

/**
 * @brief Global rotor position and speed data.
 */
extern volatile RotorData g_rotor;

/**
 * @brief Global PWM period control value.
 */
extern volatile uint32_t g_pwm_period_ctrl;

/**
 * @brief Global ADC results for phase currents.
 */
extern volatile AdcResult g_adc_result;

/**
 * @brief Global experimental settings and logging buffer.
 */
extern ExperimentSettings g_experiment;

/**
 * @brief Global main clock for timekeeping.
 */
extern volatile Time g_main_clock;

#if SPEED_PROFILE == TRAPEZOIDAL
/**
 * @brief Global trapezoidal speed profile data.
 */
extern TrapezoidalData g_speed_profile;
#endif

#if CONTROL_MODE == SPEED_CTRL
/**
 * @brief Global PI controller for speed control.
 */
extern PiController g_speed_controller;
#elif CONTROL_MODE == POS_CTRL
/**
 * @brief Global PI controller for position control.
 */
extern PiController g_angle_controller;
#elif CONTROL_MODE == TORQUE_CTRL
/**
 * @brief Global PI controller for torque control.
 */
extern PiController g_torque_controller;
#endif

/**
 * @brief Global integration gain for control loops.
 */
extern float g_integration_gain;

/**
 * @brief Global reference currents in d-q frame.
 */
extern ParkData g_dq_curr_ref;

/**
 * @brief Global saturated currents in d-q frame.
 */
extern ParkData g_dq_curr_sat;

/**
 * @brief Global measured phase currents.
 */
extern PhaseData g_phase_curr_meas;

/**
 * @brief Global measured currents in alpha-beta frame.
 */
extern ClarkeData g_ab_curr_meas;

/**
 * @brief Global reference currents in alpha-beta frame.
 */
extern ClarkeData g_ab_curr_ref;

/**
 * @brief Global measured currents in d-q frame.
 */
extern ParkData g_dq_curr_meas;

/**
 * @brief Global profiling data for delay measurements.
 */
extern DelayMeasurement g_profiling[NUM_PROFILING_POINTS];

/* Function Prototypes */

/**
 * @brief Initializes global data structures with default values.
 */
void data_types_init(void);

#endif /* INCLUDE_COMMON_DATA_TYPES_H_ */
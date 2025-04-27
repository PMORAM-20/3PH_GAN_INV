/*
 * ControlSetup.h
 *
 * Created on: 4 mar. 2025
 *
 * Author: Pablo Mora Moreno
 *
 * Description: This header file contains all needed definitions and
 * configurations for running and testing each motor control setup for
 * the studied test bench.
 *
 * Version: v0.1
 */

#ifndef LIBRARIES_CONTROLSETUP_H_
#define LIBRARIES_CONTROLSETUP_H_

#include <ControlDefines.h>

/*====================== CORE CONTROL SELECTION ======================*/
#define CONTROL_SCHEME         FOC

#define COSTF_SEL              MEANSQUARE
#define SPEED_PROF_SEL         TRAPEZOIDAL
#define MODULATION             CARRIER_BASED

/*====================== COMPENSATION FLAGS ======================*/
#define MUS2_COMP              FALSE
#define NULL_COMP              FALSE
#define MOD_RAD                FALSE
#define GRAD_NORM              FALSE
#define EQEP_SAME_DIR          TRUE
#define CLA_SUPPORT            FALSE

/*====================== SYSTEM PARAMETERS ======================*/
#define CONTROL_FREQUENCY_HZ   5000.0f
#define DC_BUS_VOLTAGE         500.0f

/*====================== VELOCITY CONTROL ======================*/
#define SPEED_LOOP_KP          0.8f
#define SPEED_LOOP_KI          3.0f

/*====================== SPEED PROFILE CONFIG ======================*/
#if (SPEED_PROF_SEL == TRAPEZOIDAL) || (SPEED_PROF_SEL == TRANSIENT)
    #define TARGET_SPEED_RPM   500.0f
    #define SPEED_SLOPE        0.05f
    #define TIME_CONSTANT      5.0f
    #define LOAD_TORQUE        80.0f
    #define TORQUE_SLOPE       0.01f
#endif

/*====================== DATA LOGGING ======================*/
#define DECIMATION_FACTOR      1U
#define DAT_LOG_BUFFER_LENGTH  4000U

#if DATA_LOG_SEL == DATLOGDEF
    #define LOGGED_VARIABLES   6U
#elif DATA_LOG_SEL == DATLOGMPC
    #define LOGGED_VARIABLES   9U
#elif DATA_LOG_SEL == DATLOGMID
    #define LOGGED_VARIABLES   4U
#elif DATA_LOG_SEL == DATALOGTRANS
    #define LOGGED_VARIABLES   5U
#endif

/*====================== PROFILING ======================*/
#define NUM_PROFILING_POINTS 10

/*====================== ADVANCED CONTROL ======================*/
#if MUS2_COMP == TRUE
    #define MUS_COMPENSATION_FACTOR  0.835f
#endif

#if CONTROL_SEL == MFDC
    #define MODULATION_RATE    0.25f

    #if NULL_COMP
        #define NULL_COMP_BUFFER_SIZE  100U
    #endif

    #if COSTF_SEL == MEANSQUARE
        #define COST_WEIGHT_KXY1  0.6f
        #define COST_WEIGHT_KXY2  1.0f
    #elif COSTF_SEL == SCALARPROD
        #define SCALAR_WEIGHT_KXY1  0.1f
        #define SCALAR_WEIGHT_KXY2  0.1f
    #endif
#elif CONTROL_SEL == FOC

    #define VD_MAX  150.0f
    #define VXY_MAX 10.0f
    #define VQ_MAX  (sqrtf(DC_BUS_VOLTAGE*DC_BUS_VOLTAGE - VD_MAX*VD_MAX -4.0f*VXY_MAX*VXY_MAX))

    #define KP_ID    15.0f
    #define KI_ID    5.0f

    #define KP_IQ    15.0f
    #define KI_IQ    5.0f

    #define KP_IX1   7.0f
    #define KI_IX1   5.0f

    #define KP_IY1   7.0f
    #define KI_IY1   5.0f

    #define KP_IX2   7.0f
    #define KI_IX2   5.0f

    #define KP_IY2   7.0f
    #define KI_IY2   5.0f

#elif CONTROL_SEL == MPC
    #if COSTF_SEL == MEANSQUARE
        #define COST_WEIGHT_KXY1  0.05f
        #define COST_WEIGHT_KXY2  0.05f
    #endif
#endif

#endif /* LIBRARIES_CONTROLSETUP_H_ */

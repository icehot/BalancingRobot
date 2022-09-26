/*
 * PID_Cfg.c
 *
 *  Created on: Sep 26, 2022
 *      Author: balaz
 */

#include "PID_Cfg.h"

/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN 0.0f
#define PID_LIM_MAX  3600.0f

#define PID_LIM_MIN_INT 0.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.05f

PIDController pid = { PID_KP, PID_KI, PID_KD,
                      PID_TAU,
                      PID_LIM_MIN, PID_LIM_MAX,
			          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                      SAMPLE_TIME_S };

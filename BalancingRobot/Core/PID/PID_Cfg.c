/*
 * PID_Cfg.c
 *
 *  Created on: Sep 26, 2022
 *      Author: balaz
 */

#include "PID_Cfg.h"

/* Controller parameters */
#define PID_KP  100.0f
#define PID_KI  80.0f
#define PID_KD  0.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN -3600.0f
#define PID_LIM_MAX  3600.0f

#define PID_LIM_MIN_INT   0.0f
#define PID_LIM_MAX_INT   0.0f

#define SAMPLE_TIME_S 0.01f

PIDController PID_Speed_L = { PID_KP, PID_KI, PID_KD,
                      PID_TAU,
                      PID_LIM_MIN, PID_LIM_MAX,
			          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                      SAMPLE_TIME_S,
					  0,
					  0,
					  0,
					  0,
					  0
};

PIDController PID_Speed_R = { PID_KP, PID_KI, PID_KD,
                      PID_TAU,
                      PID_LIM_MIN, PID_LIM_MAX,
			          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                      SAMPLE_TIME_S,
					  0,
					  0,
					  0,
					  0,
					  0
};

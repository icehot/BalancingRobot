/*
 * Encoder.c
 *
 *  Created on: Sep 18, 2022
 *      Author: balazs
 */

#include "Encoder.h"
#include "cmsis_os.h"
#include "../FilterFIR/FilterFIR.h"
#include <stdio.h>

Encoder_T Encoder[NR_OF_ENCODERS] = {

		{/*ENCODER RIGHT MOTOR*/
				&htim1, /* Timer Handle   */
				0,      /* Counter value  */
				0,      /* Overflow value */
				0,      /* Position value */
				0,      /* Previous Position value */
				0       /* Speed value */
		},
		{/*ENCODER LEFT MOTOR*/
				&htim8, /* Timer Handle   */
				0,      /* Counter value  */
				0,      /* Overflow value */
				0,      /* Position value */
				0,      /* Previous Position value */
				0       /* Speed value */
		}
};

/* Start of Filter related definitions */
/* Filter Order: 8 Sampling Frequency (Hz): 50.000000 Cut-Off Frequency Lo (Hz): 20.000000 Cut-Off Frequency Hi (Hz): 20.000000
 *
0.0000000,0.044148226,-0.113524009,0.175404241,0.800000000,0.175404241,-0.113524009,0.044148226
*/
FIRFilter FiltEncLeft;
FIRFilter FiltEncRight;

#define FIR_ORDER 8
float FIR_coeff[FIR_ORDER] = {0.0000000f,0.0441482f,-0.1135240f,0.1754042f,0.8000000f,0.1754042f,-0.1135240f,0.0441482f};
float FIR_buf_Right[FIR_ORDER];
float FIR_buf_Left[FIR_ORDER];
/* Enf of Filter related definitions */

void Encoder_Init(void)
{
	  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	  FilterFIR_Init(&FiltEncRight,&FIR_coeff[0],&FIR_buf_Right[0], FIR_ORDER);
	  FilterFIR_Init(&FiltEncLeft, &FIR_coeff[0],&FIR_buf_Left[0], FIR_ORDER);
}

uint32_t OsTickCount = 0;

void Encoder_Cyclic(void)
{
	int i;
	static uint32_t OsTickCount_Prev = 0;

	OsTickCount = osKernelGetTickCount();

	for (i = 0; i < NR_OF_ENCODERS; i++)
	{
		Encoder[i].counter = __HAL_TIM_GET_COUNTER(Encoder[i].htim);

		if((Encoder[i].counter & UPDATE_FLAG_MASK ) == UPDATE_FLAG_MASK )
		{/* overflow or underflow occurred */

		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(Encoder[i].htim))
		  {/* Underflow */
			  Encoder[i].overflow--;
		  }
		  else
		  {/* Overflow */
			  Encoder[i].overflow++;
		  }

		  /* Clear the flag */
		  Encoder[i].counter &= ~UPDATE_FLAG_MASK;
		  __HAL_TIM_CLEAR_IT(Encoder[i].htim, TIM_IT_UPDATE);
		}

		Encoder[i].position = (int64_t)Encoder[i].counter + (int64_t)Encoder[i].overflow*(int64_t)PERIOD;

		/*Output RPM = ((Pulses Recieved in 1 sec * 60) / PPR) / Gear Ratio*/
		Encoder[i].speed = (((float)(Encoder[i].position - Encoder[i].prev_position) * (1000/(float)(OsTickCount - OsTickCount_Prev)) * 60)/(float)RESOLUTION)/(float)GEAR_RATIO;

		if (OsTickCount > OsTickCount_Prev)
		{
			if (i == ENCODER_RIGHT)
			{
				Encoder[i].speed = FIRFilter_Update(&FiltEncRight, (((float)(Encoder[i].position - Encoder[i].prev_position) * (1000/(float)(OsTickCount - OsTickCount_Prev)) * 60)/(float)RESOLUTION)/(float)GEAR_RATIO);

			}
			else if (i == ENCODER_LEFT)
			{
				Encoder[i].speed = FIRFilter_Update(&FiltEncLeft, (((float)(Encoder[i].position - Encoder[i].prev_position) * (1000/(float)(OsTickCount - OsTickCount_Prev)) * 60)/(float)RESOLUTION)/(float)GEAR_RATIO);
			}
			else
			{
				/* Invalid encoder instance, should never get here */
			}
		}
		else
		{
              /* Do not execute speed calculation in case of OS Tick counter stuck or overflow,
               * The previous value is acceptable in that case */
		}

		Encoder[i].prev_position = Encoder[i].position;
	}

	OsTickCount_Prev = OsTickCount;
}

int64_t Encoder_GetPosition(EncoderInstance_T instance)
{
	return Encoder[instance].position;
}

float Encoder_GetSpeed(EncoderInstance_T instance)
{
	return Encoder[instance].speed;
}

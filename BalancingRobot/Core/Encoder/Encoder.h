/*
 * Encoder.h
 *
 *  Created on: Sep 18, 2022
 *      Author: balaz
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f3xx_hal.h"
#include "main.h"

#define PERIOD 0xFFFFu
#define UPDATE_FLAG_MASK 0x80000000u
#define GEAR_RATIO 30u //1:30 reduction ratio
#define RESOLUTION 44u //11x4 pulses for the full turn of the motor

typedef struct{
	TIM_HandleTypeDef* htim;
	uint32_t counter;
	int32_t overflow;
	int64_t position;
	int64_t prev_position;
	float speed;
}Encoder_T;

typedef enum {
	ENCODER_RIGHT = 0,
	ENCODER_LEFT = 1,
	NR_OF_ENCODERS
}EncoderInstance_T;

extern uint32_t OsTickCount;

void Encoder_Init(void);
void Encoder_Cyclic(void);
int64_t Encoder_GetPosition(EncoderInstance_T instance);
float Encoder_GetSpeed(EncoderInstance_T instance);

#endif /* INC_ENCODER_H_ */

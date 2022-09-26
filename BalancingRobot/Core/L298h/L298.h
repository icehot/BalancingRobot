/*
 * L298.h
 *
 *  Created on: Sep 21, 2022
 *      Author: balaz
 */

#ifndef INC_L298_H_
#define INC_L298_H_

#include "stm32f3xx_hal.h"
#include "main.h"

typedef enum{
	L298_MOTOR_RIGHT = TIM_CHANNEL_1,
	L298_MOTOR_LEFT  = TIM_CHANNEL_2,
}L298_Motor_Instance;

typedef struct
{
	TIM_HandleTypeDef* timer;
	uint32_t frequency;
	uint32_t period;
	uint32_t dutyRight;
	uint32_t dutyLeft;
}PWM_Config_T;

void L298_Init(void);
void L298_PWM_Config(uint32_t channel, uint32_t period, uint32_t pulse);
void L298_SetDutyInPerc( L298_Motor_Instance channel, float duty);
void L298_SetDutyInTick( L298_Motor_Instance channel, uint32_t duty);
uint32_t L298_GetPeriod();

#endif /* INC_L298_H_ */

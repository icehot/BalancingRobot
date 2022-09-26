/*
 * L298.c
 *
 *  Created on: Sep 21, 2022
 *      Author: balaz
 */

#include "L298.h"

PWM_Config_T PWM_Config = {
		&htim2, /* timer*/
		20000, /* frequency in HZ*/
		0,     /* Period calculated in runtime */
		0, /* Duty Right */
		0  /* Duty Left */
};

/* Init L298 */
void L298_Init(void)
{
	uint32_t PCLK1Freq;

	/* Configure In1 In2 pins */
	HAL_GPIO_WritePin(MOT_1_IN1_GPIO_Port, MOT_1_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOT_1_IN2_GPIO_Port, MOT_1_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_2_IN3_GPIO_Port, MOT_2_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_2_IN4_GPIO_Port, MOT_2_IN4_Pin, GPIO_PIN_SET);

	/* Calculate Timer period based on Timer Frequency */
	PCLK1Freq = HAL_RCC_GetPCLK1Freq();
	/* Timer Clock = 2 * PCLK1Freq */
	PWM_Config.period = (uint32_t)((PCLK1Freq * 2) / (float)PWM_Config.frequency); /* 2 * 36 MHz / 20 KHz = 3600 */

	/*Configure and Start PWM channels*/
	L298_PWM_Config(L298_MOTOR_RIGHT, PWM_Config.period, PWM_Config.dutyRight);
	L298_PWM_Config(L298_MOTOR_LEFT , PWM_Config.period, PWM_Config.dutyLeft);
}

/* Configure PWM channel*/
void L298_PWM_Config(uint32_t channel, uint32_t period, uint32_t pulse)
{
	HAL_TIM_PWM_Stop(PWM_Config.timer, channel); // stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	PWM_Config.timer->Init.Period = period; // set the period duration
	HAL_TIM_PWM_Init(PWM_Config.timer); // reinititialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(PWM_Config.timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(PWM_Config.timer, channel); // start pwm generation
}


/* Set Duty in Percentage [0 ... 1] */
void L298_SetDutyInPerc( L298_Motor_Instance channel, float duty)
{
	__HAL_TIM_SET_COMPARE(PWM_Config.timer, channel, (uint32_t)((float)L298_GetPeriod() * duty));
}

/* Set Duty in Ticks [0 ... Period], use L298_GetPeriod() to retrieve max value */
void L298_SetDutyInTick( L298_Motor_Instance channel, uint32_t duty)
{
	__HAL_TIM_SET_COMPARE(PWM_Config.timer, channel, duty);
}

/* Return period in ticks, for maximal resolution, use after Init only */
uint32_t L298_GetPeriod()
{
	return PWM_Config.period;
}


#include "motor.h"

#define PWM_MAX 100
#define PWM_MIN -100
extern TIM_HandleTypeDef htim1;
int abs(int p){
	return p>0?p:-p;
}

void Load(int motor1, int motor2)
{
	if(motor1 < 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 ,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 ,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4 ,abs(motor1));
	
	if(motor2 < 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15 ,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15 ,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1 ,abs(motor2));
}

void Limit(int *MOTO1 , int *MOTO2){
	if(*MOTO1 > PWM_MAX) 
		*MOTO1 = PWM_MAX;
	if(*MOTO1 < PWM_MIN)
		*MOTO1 = PWM_MIN;
	
	if(*MOTO2 > PWM_MAX) 
		*MOTO2 = PWM_MAX;
	if(*MOTO2 < PWM_MIN)
		*MOTO2 = PWM_MIN;
}
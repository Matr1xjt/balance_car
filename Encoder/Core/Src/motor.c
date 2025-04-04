#include "motor.h"

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

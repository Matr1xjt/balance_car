#include "encode.h"


int read_speed(TIM_HandleTypeDef *htim){
	int tmp;
	tmp = (short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SET_COUNTER(htim, 0);
	return tmp;
}
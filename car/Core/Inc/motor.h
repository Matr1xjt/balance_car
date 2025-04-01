#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"
void Load(int motor1, int motor2);
void Limit(int *MOTO1 , int *MOTO2);
#endif

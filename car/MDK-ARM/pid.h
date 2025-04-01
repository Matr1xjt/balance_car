#ifndef __PID_H_
#define __PID_H_
#include "stm32f1xx_hal.h"
int Turn(float gyro_Z, int target_turn);
void control(void);
int Velocity(int Target, int Encoder_left, int Encoder_right);
int Vertical(float Med, float Angle, float gyro_Y);
#endif
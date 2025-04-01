#include "pid.h"
#include "encode.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "stm32f1xx_it.h"
#include "math.h"
#include "stdio.h"
//����������
int Encoder_left,Encoder_right;
float pitch, roll, yaw;
float real_roll;
short gyrox, gyroy, gyroz;
short accx, accy, accz;
//PID����
float Med_Angle = 2;//�Ƕ�ƫ��
int Vertical_out, Velocity_out, Turn_out,Target_Speed, Target_Turn;
int MOTO1,MOTO2;

float Vertical_Kp=8.5*0.6	,Vertical_Kd = 0.05*0.6;//kp:1. - 15 kd:0 - 0.1
float Verticity_Kp= 0.50 ,Verticity_Ki;//kp:0 - 1
float turn_Kp = 10,turn_Kd = 0.006;

extern TIM_HandleTypeDef htim2,htim4;
extern uint8_t Fore,Back,Left,Right;


float  last_real_roll;
double acc_roll; 
double Gyro_roll;
#define SPEED_Y 20
#define SPEED_Z 10
uint8_t stop;
//ֱ��PD������
/*
@parameter
�����Ƕ�--�⻷
ʵ�ʽǶ�--mpu
���ٶ�--mpu
*/
int Vertical(float Med, float Angle, float gyro_Y){
	int temp;
	temp = Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y ;

	return temp;
}

//�ٶȻ�PI������
/*
@parameter
�����ٶ�
�������
�ұ�����
*/
int Velocity(int Target, int Encoder_left, int Encoder_right){
	Verticity_Ki = Verticity_Kp /200;
	static int error_LowOut_last, Encoder_S;
	static float a = 0.7;
	int error, error_LowOut, temp;
	//1.����ƫ��ֵ
	
	error = (Encoder_left + Encoder_right) - Target;
	//2.��ͨ�˲�
	error_LowOut = (1 - a) * error + a * error_LowOut_last;
	error_LowOut_last = error_LowOut;
	//3.����
	Encoder_S +=  error_LowOut;
	//4.�����޷�
	Encoder_S = Encoder_left > 20000?20000:(Encoder_S < -20000?-20000:Encoder_S);
	if(stop == 1) Encoder_S = 0,stop =0;
	//5.�ٶȻ�

	temp = Verticity_Kp * error_LowOut + Verticity_Ki * Encoder_S;
	
	return temp;
}

//ת��PD������
/*
@parameter
���ٶ�
�Ƕ�ֵ
*/
int Turn(float gyro_Z, int target_turn){
	int temp;
	temp = turn_Kp *  target_turn + turn_Kd * gyro_Z;
	return temp;
	
}

void control(void){
	int PWM_out;
	//1.��ȡ���������������������ǣ�
	Encoder_left = read_speed(&htim2);
	Encoder_right = -read_speed(&htim4);
	mpu_dmp_get_data(&pitch, &roll, &yaw);
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
	MPU_Get_Accelerometer(&accx, &accy, &accz);
	
	//ң��
	if(Fore == 0 && Back == 0)
		Target_Speed=0;
	if(Fore == 1)
		Target_Speed++;
	if(Back == 1)
		Target_Speed--;
	Target_Speed = Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?-SPEED_Y:Target_Speed);
	//����
	if(Right == 0 && Left == 0)
		Target_Turn = 0;
	if(Right == 1)
		Target_Turn =1;
	if(Left == 1)
		Target_Turn =-1;
	Target_Turn = Target_Turn>SPEED_Z?SPEED_Z:(Target_Turn<-SPEED_Z?-SPEED_Z:Target_Turn);
	
	if(Right == 1||Left == 1)
		turn_Kd = 0;
	if(Right == 0&& Left == 0)
		turn_Kd = 0.006;
	
	//2.���ݴ���,���������������������ת�٣�

	acc_roll= atan((float)accy/(float)accz) *57.32 	;
	Gyro_roll =  (Gyro_roll+gyrox/16.4)*0.01 ;
	float a = 0.02;
//	real_roll =Gyro_roll;
	real_roll = (1 - a)*(last_real_roll + Gyro_roll ) + a * acc_roll;
	last_real_roll = real_roll;
	
  Velocity_out = Velocity(Target_Speed, Encoder_left, Encoder_right);
	Vertical_out = Vertical(Velocity_out + Med_Angle, real_roll, gyrox);
	Turn_out = Turn(gyroz, Target_Turn);
	PWM_out = Vertical_out;
	PWM_out = PWM_out>40?40:(PWM_out<-40?-40:PWM_out);
	MOTO1 = PWM_out - Turn_out;
	MOTO2 = PWM_out + Turn_out;
	Limit(&MOTO1, &MOTO2);
	Load(MOTO1, MOTO2);
//	Load(0, 0);
}

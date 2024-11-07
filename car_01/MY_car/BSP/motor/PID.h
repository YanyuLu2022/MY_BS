#ifndef __PID__H
#define __PID__H

typedef struct
{
	float Kp;
	float Ki;
	float Kd;

	float last_error;  //上一次偏差
	float prev_error;  //上上次偏差

	float limit;  //限制输出幅值
	float Velocity_return; //输出的PWM值
}PID;

void PID_Init(void * P,float Velocity_Kp,float Velocity_Ki,float Velocity_Kd,float limit_value);

void PID_Formula(float targetSpeed,float currentSpeed,void * P);
#endif // __PID__H

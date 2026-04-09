/**
  ******************************************************************************
  * @file    pid.c
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   对每一个pid结构体都要先进行函数的连接，再进行初始化
  ******************************************************************************
  * @attention 应该是用二阶差分(d)云台会更加稳定
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stm32f4xx.h"
#include "motor_control.h"

#define ABS(x)		((x>0)? x: -x) 

PID_TypeDef MotorPositionPid1,MotorSpeedPid1,MotorPositionPid2,MotorSpeedPid2;
PID_TypeDef MotorPositionPid3,MotorSpeedPid3,MotorPositionPid4,MotorSpeedPid4;

PID_TypeDef MotorPositionPid5,MotorSpeedPid5,MotorPositionPid6,MotorSpeedPid6;
PID_TypeDef MotorPositionPid7,MotorSpeedPid7,MotorPositionPid8,MotorSpeedPid8;

void Motor_set_pos_with_speed(MotorTypeDef *Motor_x, float exp_pos, float exp_speed, float pos_range)
{
	float delta = exp_pos - Motor_x->PositionMeasure;
	if(ABS(delta) > pos_range){
		Motor_x->State = PIDSPEED;
		if(delta > 0)	Motor_x->SpeedExpected = exp_speed;
		if(delta < 0)	Motor_x->SpeedExpected = -exp_speed;
	}else{
		Motor_x->State = PIDPOSITION;
		Motor_x->PositionExpected = exp_pos;
	}
}

void MotorSpeedPidInit(void)
{
  MotorSpeedPid1.Kp = 40.0f;//
	MotorSpeedPid1.Ki = 0.006f;//    
	MotorSpeedPid1.Kd = 0.0f;
	MotorSpeedPid1.LimitOutput = 8000;//
	MotorSpeedPid1.LimitIntegral = 2000;//
	MotorSpeedPid1.iout = 0;
	MotorSpeedPid1.last_err = 0;
	MotorSpeedPid1.KpMax = 400;
	
	MotorSpeedPid2.Kp = 40.0f;// 
	MotorSpeedPid2.Ki =0.006f;//     
	MotorSpeedPid2.Kd = 0.0f;
	MotorSpeedPid2.LimitOutput = 8000;//
	MotorSpeedPid2.LimitIntegral = 2000;//
	MotorSpeedPid2.iout = 0;
	MotorSpeedPid2.last_err = 0;
	MotorSpeedPid2.KpMax = 400;
	
	MotorSpeedPid3.Kp = 40.0f;//  
	MotorSpeedPid3.Ki =0.006f;//  //0.006   
	MotorSpeedPid3.Kd = 0.0f;
	MotorSpeedPid3.LimitOutput = 8000;//
	MotorSpeedPid3.LimitIntegral = 2000;//  //2000
	MotorSpeedPid3.iout = 0;
	MotorSpeedPid3.last_err = 0;
	MotorSpeedPid3.KpMax = 400;
	
	MotorSpeedPid4.Kp = 40.0f;//
	MotorSpeedPid4.Ki =0.006f;//    
	MotorSpeedPid4.Kd = 0.0f;
	MotorSpeedPid4.LimitOutput = 8000;//
	MotorSpeedPid4.LimitIntegral = 2000;//
	MotorSpeedPid4.iout = 0;
	MotorSpeedPid4.last_err = 0;
	MotorSpeedPid4.KpMax = 400;
	
	MotorSpeedPid5.Kp = 40.0f;//  
	MotorSpeedPid5.Ki =0.1f;//     
	MotorSpeedPid5.Kd = 0.0f;
	MotorSpeedPid5.LimitOutput = 8000;//
	MotorSpeedPid5.LimitIntegral = 2000;//
	MotorSpeedPid5.iout = 0;
	MotorSpeedPid5.last_err = 0;
	MotorSpeedPid5.KpMax = 400;
	
	MotorSpeedPid6.Kp = 40.0f;//
	MotorSpeedPid6.Ki =0.006f;//   
	MotorSpeedPid6.Kd = 0.0f;
	MotorSpeedPid6.LimitOutput = 8000;//
	MotorSpeedPid6.LimitIntegral = 2000;//
	MotorSpeedPid6.iout = 0;
	MotorSpeedPid6.last_err = 0;
	MotorSpeedPid6.KpMax = 400;
		
}

void MotorPositionPidInit(void)
{
  MotorPositionPid1.Kp = 5.0f;//
	MotorPositionPid1.Ki = 0.001f;
	MotorPositionPid1.Kd = 0.0f;//
	MotorPositionPid1.LimitOutput = 250;
	MotorPositionPid1.LimitIntegral = 500;
	MotorPositionPid1.iout = 0;
	MotorPositionPid1.last_err = 0;
	MotorPositionPid1.KpMax = 50;

	MotorPositionPid2.Kp = 5.0f;//4.5;   //3.5
	MotorPositionPid2.Ki = 0.001f;
	MotorPositionPid2.Kd = 0;//0.01;
	MotorPositionPid2.LimitOutput = 250;
	MotorPositionPid2.LimitIntegral = 500;
	MotorPositionPid2.iout = 0;
	MotorPositionPid2.last_err = 0;
	MotorPositionPid2.KpMax = 50;
	
	MotorPositionPid3.Kp = 5.0f;   //5
	MotorPositionPid3.Ki = 0.001f;  //0.001
	MotorPositionPid3.Kd = 0.0f;
	MotorPositionPid3.LimitOutput = 250;
	MotorPositionPid3.LimitIntegral = 500;//500
	MotorPositionPid3.iout = 0;
	MotorPositionPid3.last_err = 0;  
	MotorPositionPid3.KpMax = 50;
	
	MotorPositionPid4.Kp =  5.0f;
	MotorPositionPid4.Ki = 0.001f;
	MotorPositionPid4.Kd = 0.0f;
	MotorPositionPid4.LimitOutput = 250;//200;
	MotorPositionPid4.LimitIntegral = 500;
	MotorPositionPid4.iout = 0;
	MotorPositionPid4.last_err = 0;
	MotorPositionPid4.KpMax = 50;
	
	MotorPositionPid5.Kp =  5.0f;   //2.2
	MotorPositionPid5.Ki = 0.001f;
	MotorPositionPid5.Kd = 0.0f;
	MotorPositionPid5.LimitOutput = 250;//16000
	MotorPositionPid5.LimitIntegral = 500;
	MotorPositionPid5.iout = 0;
	MotorPositionPid5.last_err = 0;
	MotorPositionPid5.KpMax = 50;
	
	MotorPositionPid6.Kp =  5.0f;
	MotorPositionPid6.Ki = 0.001f;
	MotorPositionPid6.Kd = 0.0f;
	MotorPositionPid6.LimitOutput = 250;//200;
	MotorPositionPid6.LimitIntegral = 500;
	MotorPositionPid6.iout = 0;
	MotorPositionPid6.last_err = 0;
	MotorPositionPid6.KpMax = 50;
	
}

/*参数初始化--------------------------------------------------------------*/
void PidInit(void)
{
	  MotorSpeedPidInit();	  
    MotorPositionPidInit();
}


/*中途更改参数设定--------------------------------------------------------------*/
void pid_reset(PID_TypeDef * pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

/*pid计算-----------------------------------------------------------------------*/
static float utils_angle_difference(float angle1, float angle2) 
{
	float difference = angle1 - angle2;
	while (difference < -180.0f) difference += 2.0f * 180.0f;
	while (difference > 180.0f) difference -= 2.0f * 180.0f;
	return difference;
}

float VESCPidRegulate(float Reference, float PresentFeedback,PID_TypeDef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = utils_angle_difference(Reference, PresentFeedback);	//走劣弧
	
	/*proportional term computation*/
	pTerm = error * PID_Struct->Kp;
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->iout + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->iout = PID_Struct->LimitIntegral;
	} else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->iout = -1*PID_Struct->LimitIntegral;
	} else
	{
	  PID_Struct->iout = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->last_err;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->last_err = error;

	output = pTerm + PID_Struct->iout + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} 
	else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} 
	else
	{
		return output;
	}
}

float ClassicPidRegulate(float Reference, float PresentFeedback,PID_TypeDef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = Reference - PresentFeedback;
	
	/*proportional term computation*/
	pTerm = error * PID_Struct->Kp;
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->iout + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->iout = PID_Struct->LimitIntegral;
	} else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->iout = -1*PID_Struct->LimitIntegral;
	} else
	{
	  PID_Struct->iout = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->last_err;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->last_err = error;

	output = pTerm + PID_Struct->iout + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} else
	{
		return output;
	}
}

float MotorClassicPidRegulate(float Reference, float PresentFeedback,PID_TypeDef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = utils_angle_difference(Reference, PresentFeedback);	//走劣弧
	
	/*proportional term computation*/
	pTerm = error * PID_Struct->Kp;
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->iout + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->iout = PID_Struct->LimitIntegral;
	} 
	else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->iout = -1*PID_Struct->LimitIntegral;
	}
	else
	{
	  PID_Struct->iout = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->last_err;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->last_err = error;

	output = pTerm + PID_Struct->iout + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} else
	{
		return output;
	}
}





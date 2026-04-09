/**
  ******************************************************************************
  * @file		 pid.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"
typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	
//	float target;							//目标值
//	float lastNoneZeroTarget;
	float Kp;
	float Ki;
	float Kd;
	
//	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	
	float pout;
	float iout;
	float dout;
	
	float output;						//本次输出
	float last_output;			//上次输出
	
	float LimitOutput;				//输出限幅
	float LimitIntegral;		//积分限幅

//	float DeadBand;			  //死区（绝对值）
//	float ControlPeriod;		//控制周期
//	float Max_Err;					//最大误差
  float KpMax;
	
//	uint32_t thistime;
//	uint32_t lasttime;
//	uint8_t  dtime;
	
}PID_TypeDef;


void  PidInit(void);
void pid_reset(PID_TypeDef * pid, float Kp, float Ki, float Kd);
float VESCPidRegulate(float Reference, float PresentFeedback,PID_TypeDef *PID_Struct);
float ClassicPidRegulate(float Reference, float PresentFeedback,PID_TypeDef *PID_Struct);
float MotorClassicPidRegulate(float Reference, float PresentFeedback,PID_TypeDef *PID_Struct);

extern PID_TypeDef MotorPositionPid1,MotorSpeedPid1,MotorCurrentPid1,MotorPositionPid2,MotorSpeedPid2,MotorCurrentPid2;
extern PID_TypeDef MotorPositionPid3,MotorSpeedPid3,MotorCurrentPid3,MotorPositionPid4,MotorSpeedPid4,MotorCurrentPid4;

extern PID_TypeDef MotorPositionPid5,MotorSpeedPid5,MotorCurrentPid5,MotorPositionPid6,MotorSpeedPid6,MotorCurrentPid6;
extern PID_TypeDef MotorPositionPid7,MotorSpeedPid7,MotorCurrentPid7,MotorPositionPid8,MotorSpeedPid8,MotorCurrentPid8;


#endif

//extern PID_TypeDef pid_pitch;    

//extern PID_TypeDef motor_pid[4];





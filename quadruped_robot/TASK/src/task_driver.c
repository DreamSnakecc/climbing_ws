//#include "motor_control.h"
//#include "main.h"
//#include "cmsis_os.h"
//#include "pid.h"

////int count_mad=0;
////int count_num = 0;
////bool sendFlag = false;


//void StartTask02(void const * argument)
//{
//	RMMotor_Init();
//	
// while(1)
// {
//	  led3_off;
////		switch(Motor_1.State)
////		{
////			case PIDPOSITION:				
////				Motor_1.SpeedExpected =  ClassicPidRegulate(Motor_1.PositionExpected,Motor_1.PositionMeasure,&MotorPositionPid1);
////			case PIDSPEED:						
////				Motor_1.CurrentExpected = ClassicPidRegulate(Motor_1.SpeedExpected,Motor_1.SpeedMeasure,&MotorSpeedPid1);		
////      case MOTOR_CURRENT: 					 
////				Motor_1.PWM = Motor_1.CurrentExpected;//ClassicPidRegulate(Motor_1.CurrentExpected,Motor_1.CurrentMeasure,&MotorCurrentPid1);
////				break; 
////			case MOTOR_PWM:			
////				break;
////			default:
////				Motor_1.PWM = 0;
////				break;	
////		}
////		switch(Motor_2.State)
////		{
////			case PIDPOSITION:				
////				Motor_2.SpeedExpected =  ClassicPidRegulate(Motor_2.PositionExpected,Motor_2.PositionMeasure,&MotorPositionPid2);
////			case PIDSPEED:						
////				Motor_2.CurrentExpected = ClassicPidRegulate(Motor_2.SpeedExpected,Motor_2.SpeedMeasure,&MotorSpeedPid2);		
////          	case MOTOR_CURRENT: 					 
////				Motor_2.PWM = Motor_2.CurrentExpected;//ClassicPidRegulate(Motor_2.CurrentExpected,Motor_2.CurrentMeasure,&MotorCurrentPid1);
////				break; 
////			case MOTOR_PWM:			
////				break;
////			default:
////				Motor_2.PWM = 0;
////				break;	
////		}
////		switch(Motor_3.State)
////		{
////			case PIDPOSITION:				
////				Motor_3.SpeedExpected =  MotorClassicPidRegulate(Motor_3.PositionExpected,Motor_3.PositionMeasure,&MotorPositionPid3);
////			case PIDSPEED:						
////				Motor_3.CurrentExpected = ClassicPidRegulate(Motor_3.SpeedExpected,Motor_3.SpeedMeasure,&MotorSpeedPid3);		
////          	case MOTOR_CURRENT: 					 
////				Motor_3.PWM = Motor_3.CurrentExpected;//ClassicPidRegulate(Motor_3.CurrentExpected,Motor_3.CurrentMeasure,&MotorCurrentPid1);
////				break; 
////			case MOTOR_PWM:			
////				break;
////			default:
////				Motor_3.PWM = 0;
////				break;	
////		}
////	switch(Motor_4.State)
////		{
////			case PIDPOSITION:				
////				Motor_4.SpeedExpected =  MotorClassicPidRegulate(Motor_4.PositionExpected,Motor_4.PositionMeasure,&MotorPositionPid4);
////			case PIDSPEED:						
////				Motor_4.CurrentExpected = ClassicPidRegulate(Motor_4.SpeedExpected,Motor_4.SpeedMeasure,&MotorSpeedPid4);		
////          	case MOTOR_CURRENT: 					 
////				Motor_4.PWM = Motor_4.CurrentExpected;//ClassicPidRegulate(Motor_3.CurrentExpected,Motor_3.CurrentMeasure,&MotorCurrentPid1);
////				break; 
////			case MOTOR_PWM:			
////				break;
////			default:
////				Motor_4.PWM = 0;
////				break;	
////		}
////		switch(Motor_5.State)
////		{
////			case PIDPOSITION:				
////				Motor_5.SpeedExpected =  ClassicPidRegulate(Motor_5.PositionExpected,Motor_5.PositionMeasure,&MotorPositionPid5);
////			case PIDSPEED:						
////				Motor_5.CurrentExpected = ClassicPidRegulate(Motor_5.SpeedExpected,Motor_5.SpeedMeasure,&MotorSpeedPid5);		
////          	case MOTOR_CURRENT: 					 
////				Motor_5.PWM = Motor_5.CurrentExpected;//ClassicPidRegulate(Motor_5.CurrentExpected,Motor_5.CurrentMeasure,&MotorCurrentPid1);
////				break; 
////			case MOTOR_PWM:			
////				break;
////			default:
////				Motor_5.PWM = 0;
////				break;	
////		}
////		switch(Motor_6.State)
////		{
////			case PIDPOSITION:				
////				Motor_6.SpeedExpected =  MotorClassicPidRegulate(Motor_6.PositionExpected,Motor_6.PositionMeasure,&MotorPositionPid6);
////			case PIDSPEED:						
////				Motor_6.CurrentExpected = ClassicPidRegulate(Motor_6.SpeedExpected,Motor_6.SpeedMeasure,&MotorSpeedPid6);		
////      case MOTOR_CURRENT: 					 
////				Motor_6.PWM = Motor_6.CurrentExpected;//ClassicPidRegulate(Motor_6.CurrentExpected,Motor_6.CurrentMeasure,&MotorCurrentPid1);
////				break; 
////			case MOTOR_PWM:			
////				break;
////			default:
////				Motor_6.PWM = 0;
////				break;	
////		}
////		CAN1_cmd_chassis(Motor_1.PWM,Motor_2.PWM,Motor_3.PWM,Motor_4.PWM);
////  	CAN2_cmd_chassis(Motor_5.PWM,Motor_6.PWM,0,0);
////		
//	  vTaskDelay(300);
// }
//// 
////  /* 安全起见，如果程序由于异常执行到这，则删除当前任务 */
////    vTaskDelete( NULL );

//}


//		


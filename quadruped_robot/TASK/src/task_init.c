//#include "system_config.h"
//#include "cmsis_os.h"
//#include "main.h"

//bool disable_allmotor = true;
//bool corrEnable = true;
//float corr_finished = true;

//MotorTypeDef* motors_2006[3] = {&Motor_3,&Motor_4,&Motor_6};
//MotorTypeDef* motors_3508[3] = {&Motor_1,&Motor_2,&Motor_5};

//float velocity[3]={0};//rpm

//float angle[3] = {0};

//void StartTask03(void const * argument)
//{
//	 
//    while(1)
//    {  
//			led3_on;
////			if(disable_allmotor)//停止所有电机（保护措施）
////			{
////				for(int i =0;i<3;i++)
////				{
////					motors_2006[i]->State = IDLE;
////					motors_3508[i]->State = IDLE;
////				}
////				disable_allmotor = false;
////			}
////			else if(corr_finished)
////			{
////				for(int i =0;i<3;i++)
////					{
////						motors_2006[i]->State = PIDPOSITION;//PIDPOSITION;
////					
////						motors_3508[i]->State = PIDSPEED;//IDLE;

////					}
////				for(int i =0;i<3;i++)
////					{
////					  motors_2006[i]->PositionExpected = angle[i];
////						motors_3508[i]->SpeedExpected = velocity[i];
////						
////					}
//				vTaskDelay(300);
//			
////			}
//		}
//}

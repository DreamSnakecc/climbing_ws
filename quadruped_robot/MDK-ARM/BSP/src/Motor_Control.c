#include "Motor_Control.h"
#include "can.h"
#include "pid.h"
#include "vesc_can.h"

//#include "stm32f4xx_hal_can.h"

 MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
 MotorTypeDef Motor_5, Motor_6,Motor_7, Motor_8;

#define ABS(x)	( (x>0) ? (x) : (-x) )

void get_total_angle(MotorTypeDef *p);
void get_moto_offset(MotorTypeDef *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func		my_can_filter_init
	* @Brief    CAN1 and CAN2 filter initialization
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/

// Internal test gear ratio for RM2006
const float ratio_2006 = 36.0f;//36.0f*7;// Gear reduction ratio
// Internal test gear ratio for RM3508
const float ratio_3508 = 19.0f;//71.0f;// Gear reduction ratio

static float Get_RM2006_Distance(MotorTypeDef Motor)
{
	int Distance = Motor.PosNow - Motor.PosPre;
	if(ABS(Distance) > 4000)
		Distance = Distance - Distance/abs(Distance) * 8192;
//	return ((float)Distance*360.0f/36.0f/8192.0f);		
	return ((float)Distance*360.0f/ratio_2006/8192.0f);	
}


static float Get_RM3508_Distance(MotorTypeDef Motor)
{
		int Distance = Motor.PosNow - Motor.PosPre;
		if(ABS(Distance) > 4000)
			Distance = Distance - Distance/abs(Distance) * 8192;
    /*
      if(Distance > 4000)
        Distance = Distance - 8192;
      else(Distance < -4000)
        Distance = Distance + 8192;
    */
		return ((float)Distance*360.0f/ratio_3508/8192.0f);	// Single-turn angle increment
//		return ((float)Distance*360.0f/19.0f/8192.0f);		
}

//motor data read
static void  get_motor_measure(MotorTypeDef *ptr, uint8_t *Data)                                    
{                                                                   
	ptr->CurrentMeasure=(float)(short)(Data[4]<<8 | Data[5]);//*5.f/16384.f;
	ptr->SpeedMeasure =((float)(short)(Data[2]<<8 | Data[3]));
	if(ptr->PosPre == 0 && ptr->PosNow == 0 )
			ptr->PosPre = ptr->PosNow = (short)(Data[0]<<8 | Data[1]);
	else
		{
			ptr->PosPre = ptr->PosNow;
			ptr->PosNow =  (short)(Data[0]<<8 | Data[1]);
		}
//	
//		ptr->PositionMeasure += Get_RM3508_Distance(Motor_1);

		//	ptr->hall = Data[6];
//	if(ptr->PosNow - ptr->PosPre > 4096)
//	  ptr->round_cnt --;
//	else if (ptr->PosNow - ptr->PosPre < -4096)
//		ptr->round_cnt ++;
//	ptr->total_angle = ptr->round_cnt * 8192 + ptr->PosNow - ptr->offset_angle;

}

/*this function should be called after system+can init */
//void get_moto_offset(MotorTypeDef *ptr, CAN_HandleTypeDef* hcan)
//{
//	CAN_RxHeaderTypeDef rx_header;
//	uint8_t rx_data[8];
//	if(hcan == &hcan1)
//	{
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//		ptr->PosNow = (uint16_t)(rx_data[0]<<8 |rx_data[1]) ;
//		ptr->offset_angle = ptr->PosNow;
//	}
//	else 
//	{
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
//		ptr->PosNow = (uint16_t)(rx_data[0]<<8 |rx_data[1]) ;
//		ptr->offset_angle = ptr->PosNow;
//	}
//}

/**
*@bref Calculate cumulative angle with encoder wrap-around handling.
	*/
//void get_total_angle(MotorTypeDef *p)
//{
//	
//	int res1, res2, delta;
//	if(p->PosNow < p->PosPre){			// Possible encoder wrap-around
//		res1 = p->PosNow + 8192 - p->PosPre;	// Forward wrap, delta = +
//		res2 = p->PosNow - p->PosPre;				// Reverse rotation, delta = -
//	}else{	//angle > last
//		res1 = p->PosNow - 8192 - p->PosPre ;// Reverse wrap, delta = -
//		res2 = p->PosNow - p->PosPre;				// Forward rotation, delta = +
//	}
//	// Choose the smaller absolute angle change as the valid delta
//	if(ABS(res1)<ABS(res2))
//		delta = res1;
//	else
//		delta = res2;

//	p->total_angle += delta;
//	p->PosNow = p->PosNow;
//}





/**
  * @brief          hal CAN fifo call back, receive motor data 
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header1;
	  uint8_t rx1_data[8];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
   if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, rx1_data) == HAL_OK)
    {
		if (rx_header1.IDE == CAN_ID_EXT)
		{
			vesc_can_handle_rx_frame(&rx_header1, rx1_data);
			return;
		}
    switch (rx_header1.StdId)
    {
        case Motor_1_ID:
				{
					get_motor_measure(&Motor_1, rx1_data);
					Motor_1.SpeedMeasure = Motor_1.SpeedMeasure/ratio_3508;
					Motor_1.PositionMeasure += Get_RM3508_Distance(Motor_1);
					break;
				 }	
        case Motor_2_ID:
				{
					get_motor_measure(&Motor_2, rx1_data);
					Motor_2.SpeedMeasure = Motor_2.SpeedMeasure/ratio_3508;
					Motor_2.PositionMeasure += Get_RM3508_Distance(Motor_2);
					break;
				 
				 }	
        case Motor_3_ID:
				{
					get_motor_measure(&Motor_3, rx1_data);
					Motor_3.SpeedMeasure = Motor_3.SpeedMeasure/ratio_2006;
					Motor_3.PositionMeasure += Get_RM2006_Distance(Motor_3);
					break;
				}
        case Motor_4_ID:
				{
					get_motor_measure(&Motor_4, rx1_data);
					Motor_4.SpeedMeasure = Motor_4.SpeedMeasure/ratio_2006;
					Motor_4.PositionMeasure += Get_RM2006_Distance(Motor_4);
					break;
				}
			  default:
        {
            break;
        }
			}
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx2_header;
	  uint8_t rx2_data[8];

//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx2_header, rx2_data) == HAL_OK)
  {
		if (rx2_header.IDE == CAN_ID_EXT)
		{
			vesc_can_handle_rx_frame(&rx2_header, rx2_data);
			return;
		}
    switch (rx2_header.StdId)
    {
        case Motor_5_ID:
				{
					get_motor_measure(&Motor_5, rx2_data);
					Motor_5.SpeedMeasure = Motor_5.SpeedMeasure/ratio_3508;
					Motor_5.PositionMeasure += Get_RM3508_Distance(Motor_5);
					break;
				 }	 
				case Motor_6_ID:
				{
					get_motor_measure(&Motor_6, rx2_data);
					Motor_6.SpeedMeasure = Motor_6.SpeedMeasure/ratio_2006;
					Motor_6.PositionMeasure += Get_RM2006_Distance(Motor_6);
					break;
				}

        default:
        {
            break;
        }
    }
	}
}

/**
  * @brief send control current of motor (0x201, 0x202, 0x203, 0x204)
*/
void CAN1_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box1;
    static CAN_TxHeaderTypeDef  chassis_tx_message1;
		static uint8_t              chassis_can1_send_data[8];
    chassis_tx_message1.StdId = Can1_Send_ID;
    chassis_tx_message1.IDE = CAN_ID_STD;
    chassis_tx_message1.RTR = CAN_RTR_DATA;
    chassis_tx_message1.DLC = 0x08;
    chassis_can1_send_data[0] = motor1 >> 8;
    chassis_can1_send_data[1] = motor1;
    chassis_can1_send_data[2] = motor2 >> 8;
    chassis_can1_send_data[3] = motor2;
    chassis_can1_send_data[4] = motor3 >> 8;
    chassis_can1_send_data[5] = motor3;
    chassis_can1_send_data[6] = motor4 >> 8;
    chassis_can1_send_data[7] = motor4;

//    HAL_CAN_AddTxMessage(& CHASSIS_CAN1, &chassis_tx_message1, chassis_can1_send_data, &send_mail_box1);
	// Send CAN message
  if(HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message1, chassis_can1_send_data, &send_mail_box1) != HAL_OK) 
  {
   
  }
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) 
  {

  }
}
/**
  * @brief send control current of motor (0x205, 0x206, 0x207, 0x208)
*/
void CAN2_cmd_chassis(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box2;
    static CAN_TxHeaderTypeDef  chassis_tx_message2;
		static uint8_t              chassis_can2_send_data[8];
    chassis_tx_message2.StdId = Can2_Send_ID;
    chassis_tx_message2.IDE = CAN_ID_STD;
    chassis_tx_message2.RTR = CAN_RTR_DATA;
    chassis_tx_message2.DLC = 0x08;
    chassis_can2_send_data[0] = motor5 >> 8;
    chassis_can2_send_data[1] = motor5;
    chassis_can2_send_data[2] = motor6 >> 8;
    chassis_can2_send_data[3] = motor6;
    chassis_can2_send_data[4] = motor7 >> 8;
    chassis_can2_send_data[5] = motor7;
    chassis_can2_send_data[6] = motor8 >> 8;
    chassis_can2_send_data[7] = motor8;

	// Send CAN message
  if(HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message2, chassis_can2_send_data, &send_mail_box2) != HAL_OK) 
  {
   
  }
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) 
  {

  }
}

static void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) 
{
	
	uint32_t send_mail_box3;	
	uint8_t  Mad_can2_send_data[8];

  if (len > 8) 
		len = 8;
	CAN_TxHeaderTypeDef txmsg;
	txmsg.IDE = CAN_ID_EXT;
	txmsg.ExtId = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;

	// 发送CAN消息
//  if(HAL_CAN_AddTxMessage(&hcan2, &txmsg, data, &send_mail_box3) != HAL_OK) 
//  {
//   
//  }
//  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) 
//  {

//  }	
	
	 if(HAL_CAN_AddTxMessage(&hcan1, &txmsg, data, &send_mail_box3) != HAL_OK) 
  {
   
  }
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) 
  {

  }	
	
}
static void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}


void RMMotor_Init()
{
	Motor_1.State = IDLE;
	Motor_2.State = IDLE;
	Motor_3.State = IDLE;
	Motor_4.State = IDLE;
	Motor_5.State = IDLE;
	Motor_6.State = IDLE;
}






#include "system_config.h"

void System_Init(void)
{
#ifdef _USE_CAN1
  MX_CAN1_Init();
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

#ifdef _USE_CAN2
	MX_CAN2_Init();
	my_can_filter_init_recv_all(&hcan2);
  HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif
	
				
#ifdef _USE_PC_USART
	MX_DMA_Init();
  MX_UART7_Init();
  MX_UART8_Init();
	MX_USART6_UART_Init();
#endif

//#ifdef _USE_SPI
//	// SPI2_Init();
//#endif


//	#ifdef _USE_ENCODER
//	ENC_Init();
//	ENCTIM5_Init();
//	#endif

#ifdef _USE_PID
	PidInit();
#endif
	vesc_can_init();
	HAL_Delay(200);		
}

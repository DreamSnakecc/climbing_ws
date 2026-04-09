#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H
#include "stm32f4xx.h"
#include "stdbool.h"
#include "Motor_Control.h"
#include "vesc_can.h"
#include "can.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "stdio.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "main.h"
#include "cmsis_os.h"



// User-configurable feature switches
#define SYSTEM_SUPPORT_FREERTOS      1

#define _USE_MOTOR
#define _USE_CAN1
#define _USE_CAN2
#define _USE_EXT_IO
#define _USE_LOCATE_USART
#define _USE_PC_USART
#define _USE_EXT_USART
#define _USE_BLUETEETH_USART
#define _USE_DMA
#define _USE_ENCODER
#define _USE_PID
#define _USE_SPI




//#ifdef _USE_CAN1
//#include "Can.h"
//#endif

//#ifdef _USE_PC_USART
//#include "usart.h"
//#endif



//#ifdef _USE_MOTOR
//#include "motor_control.h"
//#endif

//#ifdef _USE_ENCODER
//#include "Encoder.h"
//#endif

//#ifdef _USE_PID
//#include "motor_control.h"
//#include "pid.h"
//#endif

//#ifdef _USE_SPI
//#include "spi.h"
//#endif

//#ifdef _USE_Cylinder
//#include "IIC.h"
//#endif


void System_Init(void);

#endif

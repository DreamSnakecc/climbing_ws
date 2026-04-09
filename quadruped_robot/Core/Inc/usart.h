/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define BUFFERSIZE                  100



/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART6_UART_Init(void);
void Jetson_Decode();
void Jetson_Send_Vesc_Feedback(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t ReceiveBuffer1[BUFFERSIZE];    // UART8 DMA receive buffer
extern uint8_t ReceiveBuffer2[BUFFERSIZE];    // UART7 receive buffer
extern uint8_t ReceiveBuffer3[BUFFERSIZE];    // USART6 receive buffer

extern  uint8_t rx_len ;             // Length of one received frame
extern  uint8_t recv_end_flag;        // One frame reception complete flag
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


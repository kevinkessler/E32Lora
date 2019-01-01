/*
 * E32Lora.cpp
 *
 *  Created on: Jan 1, 2019
 *      Author: kevin
 */

#include "main.h"
#include "E32Lora.h"
#include <stdio.h>

static UART_HandleTypeDef *huart;

void E32_Init(UART_HandleTypeDef *h)
{
	huart = h;
}

void E32_Poll()
{
	  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  printf("Loop\r\n");
	  uint8_t send[]={0xc1, 0xc1, 0xc1 };
	  HAL_UART_Transmit(huart, send, 3, HAL_MAX_DELAY);
	  printf("Sent\r\n");
	  uint8_t recv[6];
//	  for (int n=0;n<7;n++)
//	  {
		  HAL_StatusTypeDef status = HAL_UART_Receive(huart, recv, 6, 2000);
//		  printf("%x %x\r\n",status,recv[0]);
//	  }
	  printf("%x %x %x %x %x %x %x\r\n",status,recv[0],recv[1],recv[2],recv[3],recv[4],recv[5]);

}

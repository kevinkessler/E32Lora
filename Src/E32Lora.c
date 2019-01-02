/*
 * E32Lora.cpp
 *
 *  Created on: Jan 1, 2019
 *      Author: kevin
 */

#include "main.h"
#include "E32Lora.h"
#include <stdio.h>
#include "stm32l432xx.h"

static UART_HandleTypeDef *huart;
static GPIO_TypeDef* m0Port;
static uint16_t m0Pin;
static GPIO_TypeDef* m1Port;
static uint16_t m1Pin;
static GPIO_TypeDef* auxPort;
static uint16_t auxPin;

static E32_STATUS E32_WaitForAux(uint8_t state)
{
	printf("WaitForAux\r\n");
	uint16_t count = 0;
	while(HAL_GPIO_ReadPin(auxPort, auxPin) != state)
	{
		if (count++ > 2500)
			return E32_TIMEOUT;

		HAL_Delay(1);
	}
	printf("WaitForAux OK\r\n");
	return E32_OK;
}

static E32_STATUS E32_ConfigResponse(uint8_t *response, uint8_t responseLength)
{
	printf("ConfigResponse\r\n");
	E32_STATUS error;
	if ((error = E32_WaitForAux(1)) != E32_OK)
		return error;

	HAL_Delay(500);
	HAL_StatusTypeDef status = HAL_UART_Receive(huart, response, responseLength, 2000);
	if (status == HAL_TIMEOUT)
		return E32_TIMEOUT;
	else if (status != HAL_OK)
		return E32_ERROR;

	printf("ConfigResponse OK \r\n");
	return E32_OK;
}

static E32_STATUS E32_ConfigRequest(uint8_t *request, uint8_t requestLength,
		uint8_t *response, uint8_t responseLength)
{
	printf("ConfigRequest\r\n");
	E32_STATUS error;

	uint8_t origMode = E32_GetMode();
	if ((error = E32_SetMode(SLEEP_MODE)) != E32_OK)
		return error;

	uint8_t status = HAL_UART_Transmit(huart, request, requestLength, 2000);
	if (status == HAL_TIMEOUT)
		return E32_TIMEOUT;
	else if (status != HAL_OK)
		return E32_ERROR;

	if ((error = E32_ConfigResponse(response, responseLength)) != E32_OK)
		return error;

	if ((error = E32_SetMode(origMode)) != E32_OK)
		return error;

	printf("ConfigRequest OK\r\n");
	return E32_OK;
}

E32_STATUS E32_Init(GPIO_TypeDef* portM0, uint16_t pinM0, GPIO_TypeDef* portM1, uint16_t pinM1,
		GPIO_TypeDef* portAux, uint16_t pinAux, UART_HandleTypeDef *h)
{
	huart = h;

	m0Port = portM0;
	m0Pin = pinM0;

	m1Port = portM1;
	m1Pin = pinM1;

	auxPort = portAux;
	auxPin = pinAux;

	return E32_SetMode(NORMAL_MODE);
}

E32_STATUS E32_SetMode(uint8_t mode)
{
	printf("SetMode %x \r\n",mode);
	if (mode == E32_GetMode())
			return E32_OK;

	printf("SettingMode\r\n");
	if (E32_WaitForAux(1) != E32_OK)
		return E32_ERROR;

	HAL_GPIO_WritePin(m0Port, m0Pin, (mode & 1));
	HAL_GPIO_WritePin(m1Port, m1Pin, (mode & 2));

	if (E32_WaitForAux(1) != E32_OK)
		return E32_ERROR;

	HAL_Delay(50);

	printf("SetMode OK\r\n");
	return E32_OK;
}

uint8_t E32_GetMode()
{
	return (HAL_GPIO_ReadPin(m1Port, m1Pin) << 1) |
			(HAL_GPIO_ReadPin(m0Port, m0Pin));
}

uint8_t E32_GetConfig(uint8_t *configBuffer)
{
	uint8_t message[]={0xc1, 0xc1, 0xc1 };

	return E32_ConfigRequest(message, 3, configBuffer, 6);
}


void E32_Poll()
{

	  uint8_t recv[] = {0x01,0x02,0x03,0x04,0x05,0x06};
	  printf("----------\r\n");
	  E32_STATUS status= E32_GetConfig(recv);
	  printf("============\r\n");
	  printf("%x - %x %x %x %x %x %x\r\n",status,recv[0],recv[1],recv[2],recv[3],recv[4],recv[5]);

}

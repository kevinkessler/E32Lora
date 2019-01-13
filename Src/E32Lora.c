/*
 * E32Lora.cpp
 *
 *  Created on: Jan 1, 2019
 *      Author: kevin
 */

#include "main.h"
#include "E32Lora.h"
#include <stdio.h>
#include <string.h>
#include "stm32l432xx.h"

static UART_HandleTypeDef *huart;
static GPIO_TypeDef* m0Port;
static uint16_t m0Pin;
static GPIO_TypeDef* m1Port;
static uint16_t m1Pin;
static GPIO_TypeDef* auxPort;
static uint16_t auxPin;
static uint32_t baudRateList[] = {1200,2400,4800,9600,19200,38400,57600,115200};
static uint8_t targetChannel;
static uint16_t targetAddress;
static uint8_t currentConfig[6];

static E32_STATUS E32_WaitForAux(uint8_t state)
{

	uint16_t count = 0;
	while(HAL_GPIO_ReadPin(auxPort, auxPin) != state)
	{
		if (count++ > 2500)
			return E32_TIMEOUT;

		HAL_Delay(1);
	}

	return E32_OK;
}

static E32_STATUS E32_ConfigResponse(uint8_t *response, uint8_t responseLength)
{
	E32_STATUS error;
	if ((error = E32_WaitForAux(0)) != E32_OK)
		return error;

	HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
	HAL_StatusTypeDef status = HAL_UART_Receive(huart, response, responseLength, 2000);
	if (status == HAL_TIMEOUT)
		return E32_TIMEOUT;
	else if (status != HAL_OK)
		return status;

	return E32_OK;
}

static E32_STATUS E32_ConfigRequest(uint8_t *request, uint8_t requestLength,
		uint8_t *response, uint8_t responseLength)
{
	E32_STATUS error;

	uint8_t origMode = E32_GetMode();
	if ((error = E32_SetMode(SLEEP_MODE)) != E32_OK)
		return error;

	uint8_t status = HAL_UART_Transmit(huart, request, requestLength, 2000);

	if (status == HAL_TIMEOUT)
		return E32_TIMEOUT;
	else if (status != HAL_OK)
		return status;

	if(response != NULL)
		if ((error = E32_ConfigResponse(response, responseLength)) != E32_OK)
			return error;


	if ((error = E32_SetMode(origMode)) != E32_OK)
		return error;

	return E32_OK;
}

static uint32_t E32_GetBaud() {
	return baudRateList[(currentConfig[3] & 0xC7) >> 3];
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

	E32_STATUS error;
	uint8_t dummy[6];
	if((error = E32_GetConfig(dummy)) != E32_OK)
		return error;

	return E32_SetMode(NORMAL_MODE);
}

E32_STATUS E32_SetMode(uint8_t mode)
{
	if (mode == E32_GetMode())
			return E32_OK;

	if (E32_WaitForAux(1) != E32_OK)
		return E32_ERROR;

	HAL_GPIO_WritePin(m0Port, m0Pin, (mode & 1));
	HAL_GPIO_WritePin(m1Port, m1Pin, (mode & 2));

	if (E32_WaitForAux(1) != E32_OK)
		return E32_ERROR;

	if (mode == SLEEP_MODE)
		huart->Init.BaudRate = 9600;
	else
		huart->Init.BaudRate = E32_GetBaud();

	HAL_UART_Init(huart);
	HAL_Delay(50);

	return E32_OK;
}

uint8_t E32_GetMode()
{
	return (HAL_GPIO_ReadPin(m1Port, m1Pin) << 1) |
			(HAL_GPIO_ReadPin(m0Port, m0Pin));
}

E32_STATUS E32_GetConfig(uint8_t *configBuffer)
{
	uint8_t message[]={0xc1, 0xc1, 0xc1 };

	E32_STATUS retval = E32_ConfigRequest(message, 3, configBuffer, 6);
	if (retval == E32_OK)
		memcpy(currentConfig, configBuffer, 6);

	return retval;
}

E32_STATUS E32_GetVersion(uint8_t *configBuffer)
{
	uint8_t message[]={0xc3, 0xc3, 0xc3 };

	return E32_ConfigRequest(message, 3, configBuffer, 6);
}

E32_STATUS E32_Reset()
{
	uint8_t message[] = {0xc4, 0xc4, 0xc4};
	E32_STATUS error;

	if((error = E32_ConfigRequest(message,3,NULL,0)) != E32_OK)
		return error;

	if ((error=E32_WaitForAux(0)) != E32_OK)
		return error;

	if ((error=E32_WaitForAux(1)) != E32_OK)
		return error;

	if((error = E32_SetMode(NORMAL_MODE)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SaveParams()
{
	uint8_t config[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	if((error=E32_ConfigRequest(config,6,NULL,0))!=E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetAddress(uint16_t addr) {
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[1] = (addr & 0xff00) >> 8;
	config[2] = addr & 0xff;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetParity(enum uartParity parity) {
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[3] = (config[3] & 0x3f) | parity << 6;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;

}

E32_STATUS E32_SetUartBaud(enum uartBaud baud)
{
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[3] = (config[3] & 0xc7) | baud << 3;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetAirRate(enum airRate rate)
{
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[3] = (config[3] & 0xF8) | rate;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetChannel(uint8_t channel)
{
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[4] = channel & 0x1F;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetTransmissionMode(enum txMode mode)
{
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[5] = (config[5] & 0x7F) | mode << 7;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetIOMode(enum ioMode mode)
{
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[5] = (config[5] & 0xBF) | mode << 6;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetWakeTime(enum wakeupTime wake) {
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[5] = (config[5] & 0xC7) | wake << 3;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetFECSwitch(enum fecSwitch fec) {
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[5] = (config[5] & 0xFB) | fec << 2;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetTXPower(enum txPower power) {
	uint8_t config[6], resp[6];
	E32_STATUS error;

	if((error=E32_GetConfig(config)) != E32_OK)
		return error;

	config[0] = 0xc2;
	config[5] = (config[5] & 0xFC) | power;

	if ((error=E32_ConfigRequest(config, 6 , resp, 6)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetTargetChannel(uint8_t channel) {
	targetChannel = channel & 0x1F;

	return E32_OK;
}

E32_STATUS E32_SetTargetAddress(uint8_t addr) {
	targetAddress = addr;

	return E32_OK;
}

E32_STATUS E32_Transmit(uint8_t *message, uint16_t length) {
	if (length > 512)
		return E32_MESSAGE_TOO_LONG;

	if (currentConfig[5] & 0x7F) {
		if(length > 509)
			return E32_MESSAGE_TOO_LONG;
		uint8_t header[3 + length];
		header[0] = (targetAddress & 0xFF) >> 8;
		header[1] = targetAddress &0xff;
		header[2] = targetChannel;
		memcpy(&header[3],message,length);
		if(HAL_UART_Transmit(huart, header, length+3, 2000) != HAL_OK)
			return E32_ERROR;
	}
	else
		if(HAL_UART_Transmit(huart, message, length, 2000) != HAL_OK)
			return E32_ERROR;

	return E32_OK;
}

void E32_Poll()
{


	E32_SetUartBaud(Baud_9600);
	  uint8_t recv[] = {0x01,0x02,0x03,0x04,0x05,0x06};
	  printf("----------\r\n");
	  E32_STATUS status= E32_GetConfig(recv);
	  printf("============\r\n");
	  printf("%x - %x %x %x %x %x %x\r\n",status,recv[0],recv[1],recv[2],recv[3],recv[4],recv[5]);

	  status = E32_Reset();
	  printf("Reset %x\r\n",status);
	  //E32_SetMode(3);
	  //uint8_t send[] = {0xc1,0xc1,0xc1};
	  //HAL_UART_Transmit(huart, send, 3, 2000);
	  //HAL_StatusTypeDef status2 = HAL_UART_Receive(huart, recv, 6, 2000);
	  //E32_STATUS status2 = E32_ConfigResponse(recv,6);
	  //printf(">>>%x - %x %x %x %x %x %x\r\n",status2,recv[0],recv[1],recv[2],recv[3],recv[4],recv[5]);

}

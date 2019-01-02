/*
 * E32Lora.h
 *
 *  Created on: Jan 1, 2019
 *      Author: kevin
 */

#ifndef INC_E32LORA_H_
#define INC_E32LORA_H_
#include "main.h"

#define NORMAL_MODE 0
#define WAKEUP_MODE 2
#define POWERSAVING_MODE 1
#define SLEEP_MODE 3

#define E32_OK 0
#define E32_TIMEOUT 1
#define E32_ERROR 100

typedef uint8_t E32_STATUS;

E32_STATUS E32_Init(GPIO_TypeDef* portM0, uint16_t pinM0, GPIO_TypeDef* portM1, uint16_t pinM1,
		GPIO_TypeDef* portAux, uint16_t pinAux, UART_HandleTypeDef *h);
E32_STATUS E32_SetMode(uint8_t mode);
uint8_t E32_GetMode(void);
E32_STATUS E32_GetConfig(uint8_t *configBuffer);
E32_STATUS E32_GetVersion(uint8_t *configBuffer);
E32_STATUS E32_Reset(void);
E32_STATUS E32_SaveParams(void);



void E32_Poll(void);


#endif /* INC_E32LORA_H_ */

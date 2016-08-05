/**
  ******************************************************************************
  * @file    Relay_Control.h
  * @author  Thai Pham
  * @version V1.0
  * @date    
  * @brief   relay routine and process
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * 
  */ 

#ifndef _RELAY_CONTROL_H__
#define _RELAY_CONTROL_H__

#define RELAY_1 21
#define RELAY_2 22
/* external functions ------------------------------------------------------- */
extern void Relay_Init( void );
extern void SmartRelay_CMD_Process(uint8_t *data, uint16_t len);
extern void Horn_Lamp_CMD_Process(uint8_t *data, uint16_t len);
extern void Relay_Turn_ON (uint8_t relay);
extern void Relay_Turn_OFF (uint8_t relay);

#endif


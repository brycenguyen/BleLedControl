/**
  ******************************************************************************
  * @file    UART_Host.h
  * @author  Thai Pham
  * @version V1.0
  * @date    
  * @brief   Uart routine and process
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
#ifndef _UART_HOST_H__
#define _UART_HOST_H__


/* external functions ------------------------------------------------------- */
void uart_init(void);
void Uart_data_write(uint8_t * p_data, uint16_t length);

#endif

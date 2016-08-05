/**
  ******************************************************************************
  * @file    UART_Host.c
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
/* Includes ------------------------------------------------------------------*/
#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "rbc_mesh.h"
#include "app_uart.h"

/* Internal Struct ---------------------------------------------------T--------*/
/* definition ---------------------------------------------------T--------*/
#define Host_START_CHAR        '#'
#define Host_END_CHAR           '*'

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
#define UART_MAX_DATA_LEN               20

typedef enum{
    START_MSG,
    GET_CMD,
    END_CHAR
}_Host_RX_STATUS;

/* variable ---------------------------------------------------T--------*/
static _Host_RX_STATUS RF_Rx_Status;

#define MAX_RX_BUF_SIZE 50
uint8_t BLE_RxBuf_index = 0;
uint8_t BLE_Rx_buf[MAX_RX_BUF_SIZE];

/**
* @brief  Call in ISR.
* @param  None
* @retval None
*/
void Host_RX_process_char( unsigned char data)
{
    if(RF_Rx_Status == START_MSG){
        if(data == Host_START_CHAR){
            RF_Rx_Status = GET_CMD;
            BLE_RxBuf_index = 0;
            BLE_Rx_buf[BLE_RxBuf_index++] = data;
        }
    }else if(RF_Rx_Status == GET_CMD){
        BLE_Rx_buf[BLE_RxBuf_index++] = data;
        if(BLE_RxBuf_index >= MAX_RX_BUF_SIZE)
        {
            RF_Rx_Status = START_MSG;
            BLE_RxBuf_index = 0;
        }
        if(data == Host_END_CHAR)
        {
            rbc_mesh_value_set(1, BLE_Rx_buf, BLE_RxBuf_index);
            nrf_gpio_pin_toggle(LED_2);
            BLE_RxBuf_index = 0;
            RF_Rx_Status = START_MSG;
        }
    }else{
        RF_Rx_Status = START_MSG;
    }
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
static void uart_event_handle(app_uart_evt_t * p_event)
{
uint8_t tmp_data;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&tmp_data));
            Host_RX_process_char(tmp_data);
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */

void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void Uart_data_write(uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
}





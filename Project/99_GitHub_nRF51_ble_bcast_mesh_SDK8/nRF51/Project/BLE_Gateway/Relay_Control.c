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

#include "Relay_Control.h"

/* defined ------------------------------------------------------- */
#define CMD_RELAY_START_CHAR        '#'
#define CMD_RELAY_END_CHAR           '*'

#define APP_OPCODE_RELAY_CMD        'R'
#define APP_OPCODE_WARNING_CMD        'W'

typedef enum{
    CMD_RELAY_ON = 0x30,    // '0'
    CMD_RELAY_OFF = 0x31,    // '1'
}_App_Relay_Command;

/**
* @brief  Call when want to init relay. Here can define desired GPIO for Relay
* @param  None
* @retval None
*/
void Relay_Init(void){
    // Init GPIO like output
    nrf_gpio_cfg_output(RELAY_1);
    nrf_gpio_cfg_output(RELAY_2);
    //set output high level - relay active Low
    nrf_gpio_pin_set(RELAY_1);
    nrf_gpio_pin_set(RELAY_2);
}

/**
* @brief  turn ON relay
* @param  None
* @retval None
*/
void Relay_Turn_ON (uint8_t relay){
    nrf_gpio_pin_clear(relay);
}

/**
* @brief  turn OFF relay
* @param  None
* @retval None
*/
void Relay_Turn_OFF(uint8_t relay){
    nrf_gpio_pin_set(relay);
}

/**
* @brief  get all msg from BLE mesh, parser and process.
*         cmd format: #R01 - relay 1 ON
*                     #R02 - relay 2 ON
*                     #R03 - relay 1 & 2 ON
*                     #R11 - relay 1 OFF
*                     #R12 - relay 2 OFF
* @param  None
* @retval None
*/
void SmartRelay_CMD_Process(uint8_t *data, uint16_t len){
uint8_t i;
uint8_t state = 0;

    //check correct len cmd
    if(len < 4)
        return;
    
    for(i=0; i<len; i++){
        switch(state){
            case 0:
                //check start char of cmd
                if(data[0] == CMD_RELAY_START_CHAR){
                    state = 1;
                }
                break;
            case 1:
                //check correct relay command
                if(data[1] == APP_OPCODE_RELAY_CMD){
                    state = 2;
                }
                break;
            case 2:
                //process command
                if(data[2] == CMD_RELAY_OFF){
                    if(data[3] == '1'){
                        Relay_Turn_OFF(RELAY_1);
                    }else if(data[3] == '2'){
                        Relay_Turn_OFF(RELAY_2);
                    }else if(data[3] == '3'){
                        Relay_Turn_OFF(RELAY_1);
                        Relay_Turn_OFF(RELAY_2);
                    }
                }else if(data[2] == CMD_RELAY_ON){
                    if(data[3] == '1'){
                        Relay_Turn_ON(RELAY_1);
                    }else if(data[3] == '2'){
                        Relay_Turn_ON(RELAY_2);
                    }else if(data[3] == '3'){
                        Relay_Turn_ON(RELAY_1);
                        Relay_Turn_ON(RELAY_2);
                    }
                }else{
                    // do nothing
                }
                state = 0;
                break;
            default:
                break;
        }
        if(0 == state)  // cmd wrong, then return
            break;
    }
}

/**
* @brief  get all msg from BLE mesh, parser and process.
*         cmd format: #Wx1 - warning occur - x: source sensor 1-F
*                     #Wx0 - NO warning - x: source sensor 1-F
*                     #WFF - enable warning
*                     #W00 - disable warning
*                     
* @param  None
* @retval None
*/
uint8_t warning_status = 0; //0-warning_statusdiable; 1-enable
void Horn_Lamp_CMD_Process(uint8_t *data, uint16_t len){
uint8_t i;
uint8_t state = 0;

    //check correct len cmd
    if(len < 4)
        return;
    
    for(i=0; i<len; i++){
        switch(state){
            case 0:
                //check start char of cmd
                if(data[0] == CMD_RELAY_START_CHAR){
                    state = 1;
                }
                break;
            case 1:
                //check correct relay command
                if(data[1] == APP_OPCODE_WARNING_CMD){
                    state = 2;
                }
                break;
            case 2:
                //process command
                if((data[2] == 'F')&&(data[3] == 'F')){
									warning_status = 1;
                }else if((data[2] == '0')&&(data[3] == '0')){
									warning_status = 0;
                }else if(data[3] == '1'){
									if(warning_status == 1)
											Relay_Turn_ON(RELAY_1);
                }else if(data[3] == '0'){
									Relay_Turn_OFF(RELAY_1);
								}
                state = 0;
                break;
            default:
                break;
        }
        if(0 == state)  // cmd wrong, then return
            break;
    }
}

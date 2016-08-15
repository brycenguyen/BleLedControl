/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/
//#define HAS_HORN_LAMP
//#define HAS_TOUCH_SENSOR
#define HAS_UART
#define HAS_RGB

#include "rbc_mesh.h"
#include "nrf_adv_conn.h"
#include "led_config.h"
#include "timeslot_handler.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#ifdef HAS_UART
#include "UART_Host.h"
#endif
#include "Relay_Control.h"

#ifdef HAS_RGB

#include "hal_rgb.h"
#include "rgb_config.h"

#endif

//#include "nrf_delay.h"

/* definition ---------------------------------------------------T--------*/

#define START_STRING                    "Start..."                                /**< The string that will be sent over the UART when the application starts. */

/* struct ---------------------------------------------------T--------*/

/* Static funcion ---------------------------------------------------T--------*/
//huy
uint8_t RxComplete = 0;
uint8_t LedBuffer[20];

void LedRGB_CMD_Process(uint8_t* data, uint8_t len);
void Input_Cyclic_Process( void );
#ifdef HAS_TOUCH_SENSOR
static void timer_init(void);
static void nrf_timer_delay_ms(uint_fast16_t volatile number_of_ms);
#endif
/* variable ---------------------------------------------------T--------*/

/* Debug macros for debugging with logic analyzer */
#define SET_PIN(x) /*NRF_GPIO->OUTSET = (1 << (x))*/
#define CLEAR_PIN(x) /*NRF_GPIO->OUTCLR = (1 << (x))*/
#define TICK_PIN(x) /*do { SET_PIN((x)); CLEAR_PIN((x)); }while(0)*/

/**
* @brief General error handler.
*/
static void error_loop(void)
{
    //led_config(3, 1);
    
    NVIC_SystemReset(); /* reset the system.  */
    
    /*while (true)
    {
    }*/
}    

/**
* @brief Softdevice crash handler, never returns
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    error_loop();
}

void HardFault_Handler(void)
{
    error_loop();
}

/**
* @brief Softdevice event handler 
*/
uint32_t sd_evt_handler(void)
{
    rbc_mesh_sd_irq_handler();
    
    ble_evt_t ble_evt;
    uint16_t len = sizeof(ble_evt);
    while (sd_ble_evt_get((uint8_t*) &ble_evt, &len) == NRF_SUCCESS)
    {
        nrf_adv_conn_evt_handler(&ble_evt);
    }
    return NRF_SUCCESS;
}
/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    TICK_PIN(28);
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:   
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            if (evt->value_handle > 2)
                break;
            //SmartRelay_CMD_Process(&evt->data[0], evt->data_len);
            #ifdef HAS_UART
						Uart_data_write(&evt->data[0], evt->data_len);
						#endif
						#ifdef HAS_HORN_LAMP
						Horn_Lamp_CMD_Process(&evt->data[0], evt->data_len);
						#endif
						
						//huy:process RGB data received
						LedRGB_CMD_Process(&evt->data[0], evt->data_len);
						
            //nrf_gpio_pin_toggle(LED_3);
            break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            /* init BLE gateway softdevice application: */
            nrf_adv_conn_init();
            break;  
    }
		
		/*--khai-- :Put code to setup new RGB Pattern*/
//		unsigned char* pPattern;
//		HAL_RGB_Setup_Pattern(pPattern);
			
}	


/**
* @brief Initialize GPIO pins, for LEDs and debugging
*/
void gpio_init(void)
{
    //nrf_gpio_range_cfg_output(LED_START, LED_STOP);
#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028) || defined(BOARD_RHC)
    nrf_gpio_range_cfg_output(0, 32);
#endif

    nrf_gpio_pin_set(LED_START); //turn on LED START
    for (uint32_t i = 1; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_clear(LED_START + i);    //turn off the rest LED
    }
    //init button or input signal
		#ifndef HAS_HORN_LAMP
		nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
		#endif
#ifdef BUTTONS
    #ifdef defined(BOARD_PCA10028)
        
        nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_4, NRF_GPIO_PIN_PULLUP);

    #elif defined(BOARD_RHC)
        nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
    #endif
#endif
    
} 
/** @brief main function */

int main(void){
		unsigned char m_default_pattern[] = 
		{
		'S', //Start
		'1','F',  //len
		RGB_SPEED_FAST,   //Speed
		RGB_MODE_FLASH,   //mode   
		'F','F','0','0','0','0',
		'0','0','F','F','0','0',
		'0','0','F','F','F','F',
		'0','0','0','0','F','F',
		};
		
		HAL_RGB_Init();
		HAL_RGB_Setup_Pattern(m_default_pattern);
		RGB_NODE node = {255,0,0,1000,2000};
		//HAL_RGB_Run_Node(node);
		while(true)
		{
			HAL_RGB_Run_Pattern();
		}
}
int main_1(void)
{   
    //uint8_t  start_string[] = START_STRING;
    
    NRF_POWER->RESET = 1;
    // Init relay, must put here because of initilize of PIN out - active Low
    Relay_Init();
    /* init leds and pins */
    gpio_init();
    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_evt_handler);
	
		#ifdef HAS_RGB
		HAL_RGB_Init();
		#endif
	
    #ifdef HAS_UART
		uart_init();
		#endif
	
		#ifdef HAS_TOUCH_SENSOR
		timer_init();
		#endif
#ifdef RBC_MESH_SERIAL
    
    /* only want to enable serial interface, and let external host setup the framework */
    mesh_aci_init();

#else    
    /* Enable mesh framework on channel 37, min adv interval at 100ms, 
        2 characteristics */
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = 0xA541A68F;
    init_params.adv_int_ms = 100;
    init_params.channel = 38;
    init_params.handle_count = 2;
    init_params.packet_format = RBC_MESH_PACKET_FORMAT_ORIGINAL;
    init_params.radio_mode = RBC_MESH_RADIO_MODE_BLE_1MBIT;
    
    uint32_t error_code;
    error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    /* request values for both LEDs on the mesh */
    error_code = rbc_mesh_value_enable(1);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(2);
    APP_ERROR_CHECK(error_code);
    
    
    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();
    
#endif
    NRF_GPIO->OUTCLR = (1 << 4);
    
#ifndef BUTTONS
    /* sleep */
    while (true)
    {
        //sd_app_evt_wait();
			#ifndef HAS_HORN_LAMP
				Input_Cyclic_Process();
			#endif
    }
    
#else
    uint8_t mesh_data[16] = {0,0};
    while (true)
    {
        // red off
        if(nrf_gpio_pin_read(BUTTON_1) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_1) == 0);
            mesh_data[0] = 0;
            rbc_mesh_value_set(1, mesh_data, 1);
            led_config(1, 0);
        }
        // red on
        if(nrf_gpio_pin_read(BUTTON_2) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_2) == 0);
            mesh_data[0] = 1;
            rbc_mesh_value_set(1, mesh_data, 1);
            led_config(1, 1);
        }
        // green off 
        if(nrf_gpio_pin_read(BUTTON_3) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_3) == 0);
            mesh_data[0] = 0;
            rbc_mesh_value_set(2, mesh_data, 1);
            led_config(2, 0);
        }
        // green on
         if(nrf_gpio_pin_read(BUTTON_4) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_4) == 0);
            mesh_data[0] = 1;
            rbc_mesh_value_set(2, mesh_data, 1);
            led_config(2, 1);
        }
				
				if(RxComplete)
				{
						HAL_RGB_Setup_Pattern(LedBuffer);
						RxComplete = 0;
				}
				HAL_RGB_Run_Pattern();
    }   
#endif 
		
		

}

#ifdef HAS_TOUCH_SENSOR
/**
 * @brief Function for timer initialization.
 */
static void timer_init( void)
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
    NRF_TIMER0->MODE        = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
    NRF_TIMER0->PRESCALER   = 9;                           // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    NRF_TIMER0->BITMODE     = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
}


/** @brief Function for using the peripheral hardware timers to generate an event after requested number of milliseconds.
 *
 * @param[in] timer Timer to be used for delay, values from @ref p_timer
 * @param[in] number_of_ms Number of milliseconds the timer will count.
 * @note This function will power ON the requested timer, wait until the delay, and then power OFF that timer.
 */
static void nrf_timer_delay_ms(uint_fast16_t volatile number_of_ms)
{
    NRF_TIMER0->TASKS_CLEAR = 1;                           // clear the task first to be usable for later.

    // With 32 us ticks, we need to multiply by 31.25 to get milliseconds.
    NRF_TIMER0->CC[0]       = number_of_ms * 31;
    NRF_TIMER0->CC[0]      += number_of_ms / 4;
    NRF_TIMER0->TASKS_START = 1; // Start timer.

    while (NRF_TIMER0->EVENTS_COMPARE[0] == 0)
    {
        // Do nothing.
    }

    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->TASKS_STOP        = 1; // Stop timer.
}
#endif
/**
* @brief  Call in while loop of main()
* @param  None
* @retval None
*/
#define SENSOR_NUMBER	'2'
uint8_t warning_flag = 0;
void Input_Cyclic_Process( void )
{
	static uint8_t Old_status = 0;
	uint8_t Loc_var;
	uint8_t RFTx_buf[4] = {'#','W',SENSOR_NUMBER, 0};
	
	Loc_var = (uint8_t)nrf_gpio_pin_read(BUTTON_1);
	if(Old_status != Loc_var){
		RFTx_buf[3] = Loc_var + 0x30;
		rbc_mesh_value_set(1, RFTx_buf, 4);
	}
	#ifdef HAS_TOUCH_SENSOR
	else if(warning_flag == 1){
		RFTx_buf[3] = 0x30;
		rbc_mesh_value_set(1, RFTx_buf, 4);
		warning_flag = 0;
	}
	#endif
	if(Loc_var == 0)
	{
		nrf_gpio_pin_set(LED_3);
	}else{
		nrf_gpio_pin_clear(LED_3);
		#ifdef HAS_TOUCH_SENSOR
			nrf_timer_delay_ms(20000);
			warning_flag = 1;
		#endif
	}
	Old_status = Loc_var;
}

void LedRGB_CMD_Process(uint8_t* data, uint8_t len)
{
	uint8_t i=0;
	static uint8_t RxLen=0;
	for(i=0;i<len;i++)
	{
		LedBuffer[RxLen+i]=*(data++);		
	}
	RxLen+=len;
	if(RxLen>=LedBuffer[1]) //receive all the bytes
	{
			RxLen=0;//reset counting bytes received
			RxComplete=1; //set flag to indicate pattern received
	}		
	else
	{
			
	}
}

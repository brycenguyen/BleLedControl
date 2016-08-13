#include "rgb_config.h"
#include "nrf_gpio.h"
#include "app_pwm.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"

#define PIN_R	6
#define PIN_G	7
#define PIN_B 8
/*
Note :
		1. app_timer runs with RTC tick of 32768 (32KHz)
    2. pwm duty cycle is updated after 10ms

*/


unsigned int s_TimePeriod = 0;
app_timer_id_t rgb_app_timer = 1;

APP_PWM_INSTANCE(PWM1,1);
APP_PWM_INSTANCE(PWM2,2);

void pwm_ready_callback_1()
{
	
}

void pwm_ready_callback_2()
{
	
}

void BSP_Init_Timer(void * timer_callback)
{
    nrf_drv_clock_init(NULL);
    nrf_drv_clock_lfclk_request();
		APP_TIMER_INIT(32,1,15,false);
		app_timer_create(&rgb_app_timer,APP_TIMER_MODE_REPEATED,timer_callback);
		app_timer_start(rgb_app_timer,10,NULL);
}
void BSP_Init_GPIO()
{
		nrf_gpio_cfg_output(PIN_R);
		nrf_gpio_pin_clear(PIN_R);
		nrf_gpio_cfg_output(PIN_G);
		nrf_gpio_pin_clear(PIN_G);
		nrf_gpio_cfg_output(PIN_B);
		nrf_gpio_pin_clear(PIN_B);
}
void BSP_Init_PWM(unsigned int ms)
{
		//Init PWM
		app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH_RGB(ms*1000,PIN_R,PIN_G);
		app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback_1); 
	
		app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH_RGB(ms*1000,PIN_B);	
		app_pwm_init(&PWM2,&pwm2_cfg,pwm_ready_callback_2); 
	
		app_pwm_enable(&PWM1);
		app_pwm_enable(&PWM2);
}


/*Set duty cycle for individual led.*/
void BSP_Set_Led_Brightness(RGB_LED led, unsigned char brightness)
{
    
    switch(led)
    {
        case RGB_LED_RED :   
						app_pwm_channel_duty_set(&PWM1,0,(brightness*100/255));
        break;
        case RGB_LED_GREEN :    
						app_pwm_channel_duty_set(&PWM1,1,(brightness*100/255));					
        break;
        case RGB_LED_BLUE : 
						app_pwm_channel_duty_set(&PWM2,0,(brightness*100/255));					
        break;

    }

}

/*This is just temporary to adapt with test system.
This is going to be replaced when deployed in a specified system
*/
unsigned int  g_rgbTime = 0;
unsigned int  g_rgbRunning = 0;

unsigned char BSP_Is_RGB_Running()
{
    return (g_rgbRunning > 0);
}

void BSP_Set_Time_ToUpdate(unsigned int ms)
{
    /*Currenly , systick timer of 10ms*/
    g_rgbTime = (ms / 10);
    g_rgbRunning = 1;
}
unsigned char  BSP_RGB_Is_Updated()
{
    if(g_rgbRunning)
    {
        if(g_rgbTime)
        {
            g_rgbTime--;
            return 0;
        }
        else
        {
            g_rgbRunning = 0;
            return 1;
        }
    }
    else return 0;
        
} 
typedef enum{
    RGB_LED_RED     =   0,
    RGB_LED_GREEN   =   1,
    RGB_LED_BLUE    =   2,

}RGB_LED;

void BSP_Init_GPIO();
void BSP_Init_PWM(unsigned int ms);
void BSP_Set_Led_Brightness(RGB_LED led, int brightness); 
void BSP_Set_Time_ToUpdate(unsigned int ms);
void BSP_Init_Timer();
unsigned char BSP_RGB_Is_Updated();


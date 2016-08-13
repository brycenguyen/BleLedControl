typedef struct {
    unsigned char red;                      //range from 0-255
    unsigned char green;                    //range from 0-255
    unsigned char blue;                     //range from 0-255
    unsigned int time_to_hold;              //in milisecond
    unsigned int time_to_transition;        //in milisecond

}RGB_NODE;

#define RGB_SPEED_SLOW      0
#define RGB_SPEED_MEDIUM    1
#define RGB_SPEED_FAST      2

#define RGB_MODE_SINGLE     0
#define RGB_MODE_FLASH      1
#define RGB_MODE_CONTINUOUS 2


void HAL_RGB_Run_Node(RGB_NODE node);
unsigned char HAL_RGB_Running_Pattern(void* pPattern);
void HAL_RGB_Init();
unsigned char HAL_RGB_Setup_Pattern(void* pPattern);
void HAL_RGB_Run_Pattern();

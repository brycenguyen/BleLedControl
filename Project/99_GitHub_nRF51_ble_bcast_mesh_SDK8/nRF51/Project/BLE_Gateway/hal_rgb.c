#include "hal_rgb.h"
#include "rgb_config.h"
#include "string.h"
#include "stdlib.h"

typedef enum{
    TRUE  =  1,
    FALSE = 0,
}BOOL;

#define PWM_FREQ_MS 		            5
#define MIN_TIME_MS 		            10
#define TIME_BASE_MS                    10

#define PATTERN_START_INDX              0
#define PATTERN_START_INDX_LEN          1
#define PATTERN_LEN_INDX                (PATTERN_START_INDX  + PATTERN_START_INDX_LEN)
#define PATTERN_LEN_INDX_LEN            1
#define PATTERN_SPEED_INDX              (PATTERN_LEN_INDX+PATTERN_LEN_INDX_LEN)
#define PATTERN_SPEED_INDX_LEN          1
#define PATTERN_MODE_INDX               (PATTERN_SPEED_INDX + PATTERN_SPEED_INDX_LEN)
#define PATTERN_MODE_INDX_LEN           1
#define PATTERN_DATA_INDX               (PATTERN_MODE_INDX + PATTERN_MODE_INDX_LEN)
#define PATTERN_DATA_EACH_LEN                1

#define PATTERN_RAW_START_INDX          0
#define PATTERN_RAW_START_INDX_LEN      1
#define PATTERN_RAW_LEN_INDX            (PATTERN_RAW_START_INDX + PATTERN_RAW_START_INDX_LEN)
#define PATTERN_RAW_LEN_INDX_LEN        2
#define PATTERN_RAW_SPEED_INDX          (PATTERN_RAW_LEN_INDX + PATTERN_RAW_LEN_INDX_LEN)
#define PATTERN_RAW_SPEED_INDX_LEN      2
#define PATTERN_RAW_MODE_INDX           (PATTERN_RAW_SPEED_INDX + PATTERN_RAW_SPEED_INDX_LEN)
#define PATTERN_RAW_MODE_INDX_LEN       (2)
#define PATTERN_RAW_DATA_INDX           (PATTERN_RAW_MODE_INDX + PATTERN_RAW_MODE_INDX_LEN)
#define PATTERN_RAW_DATA_INDX_EACH_LEN  2



#define PATTERN_START_VAL   'S'

static RGB_NODE s_curRGBNode    = {0,0,0,0,0};
static volatile RGB_NODE s_nextRGBNode   = {0,0,0,0,0};
static volatile int s_redStepVal         = 0;
static volatile int s_greenStepVal       = 0;
static volatile int s_blueStepVal        = 0;
static volatile int s_curStep            = 0;
static volatile int s_timeEachTransitionStep       = 0;
static volatile int s_timeToHold         = 0;
static unsigned char s_NumStepForTransition     = 10;
static volatile unsigned char s_finish_node = 1;

RGB_NODE pattern_single_arr[]=
{
    {0,0,0,0,0},//fill color here
};

//5  node for maximum, be always multiple of 2
RGB_NODE pattern_dimming_arr[] = 
{
    {0,0,0,0,0},//fill color here
    {0,0,0,0,0},
    {0,0,0,0,0},//fill color here  
    {0,0,0,0,0},
    {0,0,0,0,0},//fill color here
    {0,0,0,0,0},
    {0,0,0,0,0},//fill color here
    {0,0,0,0,0},
    {0,0,0,0,0},//fill color here   
    {0,0,0,0,0},
};

//5  node for maximum
RGB_NODE pattern_continuous_arr[] = 
{
    {0,0,0,0,0},//fill color here
    {0,0,0,0,0},//fill color here
    {0,0,0,0,0},//fill color here  
    {0,0,0,0,0},//fill color here
    {0,0,0,0,0},//fill color here
};

RGB_NODE* rgb_mode_arr[] = 
{
    pattern_single_arr,
    pattern_dimming_arr,
    pattern_continuous_arr,
};

unsigned int rgb_single_speed_arr[][3] = 
{
    //time_to_hold, time_to_transition
    {5000,          5000,}, 
    {2000,          2000,},
    {1000,          1000,},
};

unsigned int rgb_dimming_speed_arr[][3] = 
{
    //time_to_hold, time_to_transition
    {5000,          200,},
    {3000,          200,},
    {200,           100,},

};
unsigned int rgb_continuous_speed_arr[][3] = 
{
    //time_to_hold, time_to_transition
    {1000,           6000,},
    {1000,           4000,},
    {1000,           2000,},
};


unsigned char current_mode;
void fill_single_pattern(unsigned char speed, unsigned char* pValue, unsigned char lenVal)
{
    unsigned char i = 0;
    unsigned char len = 0;
    len = (sizeof(pattern_single_arr)/sizeof(RGB_NODE));
    len = (len > lenVal)? lenVal : len;
    for(i = 0; i< len; i++)
    {
        pattern_single_arr[i].red   = *(pValue + i*3 + 0);
        pattern_single_arr[i].green = *(pValue + i*3 + 1);
        pattern_single_arr[i].blue  = *(pValue + i*3 + 2);       
        pattern_single_arr[i].time_to_hold = rgb_single_speed_arr[speed][0];
        pattern_single_arr[i].time_to_transition = rgb_single_speed_arr[speed][1];     
    }
}

void fill_dimming_pattern(unsigned char speed, unsigned char* pValue, unsigned char lenVal)
{
    unsigned char i = 0, j =0;
    unsigned char len = 0;
    
    //Reset pattern
    memset(pattern_dimming_arr, 0x00, sizeof(pattern_dimming_arr));

    //fill with new value 
    len = (sizeof(pattern_dimming_arr)/sizeof(RGB_NODE));
    len = len / 2;
    len = (len > lenVal)? lenVal : len;
    for(i = 0; i< len; i++)
    {
        j = i*2;
        //Set time on
        pattern_dimming_arr[j].red   = *(pValue + i*3 + 0);
        pattern_dimming_arr[j].green = *(pValue + i*3 + 1);
        pattern_dimming_arr[j].blue  = *(pValue + i*3 + 2);
        
        pattern_dimming_arr[j].time_to_hold = rgb_dimming_speed_arr[speed][0];
        pattern_dimming_arr[j].time_to_transition = rgb_dimming_speed_arr[speed][1];     
        
        //Set time off
        pattern_dimming_arr[j+1].time_to_hold = rgb_dimming_speed_arr[speed][0];
        pattern_dimming_arr[j+1].time_to_transition = rgb_dimming_speed_arr[speed][1];
    }
}


void fill_continuous_pattern(unsigned char speed, unsigned char* pValue, unsigned char lenVal)
{
    unsigned char i = 0;
    unsigned char len = 0;
    
    //Reset pattern
    memset(pattern_continuous_arr, 0x00, sizeof(pattern_continuous_arr));

    //fill with new value 
    len = (sizeof(pattern_continuous_arr)/sizeof(RGB_NODE));
    len = (len > lenVal)? lenVal : len;
    for(i = 0; i< len; i++)
    {
        pattern_continuous_arr[i].red   = *(pValue + i*3 + 0);
        pattern_continuous_arr[i].green = *(pValue + i*3 + 1);
        pattern_continuous_arr[i].blue  = *(pValue + i*3 + 2);
        
        pattern_continuous_arr[i].time_to_hold = rgb_continuous_speed_arr[speed][0];
        pattern_continuous_arr[i].time_to_transition = rgb_continuous_speed_arr[speed][1];     
    }
}

BOOL hal_is_the_same_node(RGB_NODE node);
void hal_set_current_node(RGB_NODE node);
void hal_set_next_node(RGB_NODE node);

void HAL_RGB_Callback()
{
		/*Finish time_to_transition, turn to time_to_hold*/
    if(s_curStep == 0)
		{
				if(s_timeToHold >= TIME_BASE_MS)
				{
					
 					BSP_Set_Time_ToUpdate(TIME_BASE_MS);				
					BSP_Set_Led_Brightness(RGB_LED_RED,s_curRGBNode.red);
					BSP_Set_Led_Brightness(RGB_LED_GREEN,s_curRGBNode.green);
					BSP_Set_Led_Brightness(RGB_LED_BLUE,s_curRGBNode.blue);
					s_timeToHold = s_timeToHold - TIME_BASE_MS;
					//s_timeToHold = 0;
				}
				else
					s_finish_node = 1;
					return;				
		}	
		
		/*Running the last step of time_to_transition*/
    else if((s_curStep == (s_NumStepForTransition -1)))
    {
        
        BSP_Set_Time_ToUpdate(s_timeEachTransitionStep);
        BSP_Set_Led_Brightness(RGB_LED_RED,s_nextRGBNode.red);
        BSP_Set_Led_Brightness(RGB_LED_GREEN,s_nextRGBNode.green);
        BSP_Set_Led_Brightness(RGB_LED_BLUE,s_nextRGBNode.blue);

        hal_set_current_node(s_nextRGBNode);

        s_curStep = 0;
        s_timeEachTransitionStep = 0;
    }
		
		/*Running the step of timer_to_transition*/
    else
    {  
        s_curStep++; 
        
        BSP_Set_Time_ToUpdate(s_timeEachTransitionStep);    
        BSP_Set_Led_Brightness(RGB_LED_RED,s_curRGBNode.red + s_curStep * s_redStepVal);
        BSP_Set_Led_Brightness(RGB_LED_GREEN,s_curRGBNode.green + s_curStep * s_greenStepVal);
        BSP_Set_Led_Brightness(RGB_LED_BLUE,s_curRGBNode.blue + s_curStep * s_blueStepVal);    
    }
}

void rgb_app_timer_event_callback(void* p_context)
{
  if(BSP_RGB_Is_Updated())
  {
    HAL_RGB_Callback();
  }
}
void HAL_RGB_Init()
{
    BSP_Init_GPIO();
    BSP_Init_PWM(PWM_FREQ_MS);
		BSP_Init_Timer(rgb_app_timer_event_callback);
    
}
BOOL hal_null_node(RGB_NODE* pNode){
		if((pNode->blue == 0) &&
			 (pNode->green == 0) &&
		   (pNode->red == 0) &&
			 (pNode->time_to_hold == 0) &&
			 (pNode->time_to_transition == 0))
			return TRUE;
		else
			return FALSE;
}
void HAL_RGB_Run_Node(RGB_NODE node)
{
    int minTime = 0;

		if(hal_null_node(&node)) return;
    if(hal_is_the_same_node(node)== TRUE) return;
    
    //set next node
    hal_set_next_node(node);
    minTime = (node.time_to_hold < node.time_to_transition)? node.time_to_hold : node.time_to_transition;
    minTime = (minTime < MIN_TIME_MS)? MIN_TIME_MS : minTime;
    s_NumStepForTransition = minTime / TIME_BASE_MS;

		if(s_NumStepForTransition == 0) return;
	
    //set RED brightness
    s_redStepVal =  (int)node.red - (int)s_curRGBNode.red;
    s_redStepVal /=  s_NumStepForTransition; 
    
    //set GREEN brightness
    s_greenStepVal =  (int)node.green - (int)s_curRGBNode.green;
    s_greenStepVal /=  s_NumStepForTransition; 

    //set BLUE brightness
    s_blueStepVal =  (int)node.blue - (int)s_curRGBNode.blue;
    s_blueStepVal /=  s_NumStepForTransition; 
    
    //time to change for each step
    s_timeEachTransitionStep = node.time_to_transition/ s_NumStepForTransition;

    //time to hold to this node
    s_timeToHold = node.time_to_hold;

    s_curStep   =   1;
    BSP_Set_Time_ToUpdate(s_timeEachTransitionStep);
    BSP_Set_Led_Brightness(RGB_LED_RED,s_curRGBNode.red + s_curStep * s_redStepVal);
    BSP_Set_Led_Brightness(RGB_LED_GREEN,s_curRGBNode.green + s_curStep * s_greenStepVal);
    BSP_Set_Led_Brightness(RGB_LED_BLUE,s_curRGBNode.blue + s_curStep * s_blueStepVal);

    //while(s_curStep);
		//while(s_timeToHold);  
		s_finish_node = 0;
}

BOOL hal_is_the_same_node(RGB_NODE node)
{
    if( node.red                    ==  s_curRGBNode.red        &&
        node.green                  ==  s_curRGBNode.green      &&
        node.blue                   ==  s_curRGBNode.blue       &&
        node.time_to_transition     ==  s_curRGBNode.time_to_transition)
            return TRUE;
    else    
            return FALSE;
}
void hal_set_current_node(RGB_NODE node)
{
    s_curRGBNode.red    =   node.red;
    s_curRGBNode.green  =   node.green;
    s_curRGBNode.blue   =   node.blue;
    s_curRGBNode.time_to_transition   =   node.time_to_transition;
		s_curRGBNode.time_to_hold   			=   node.time_to_hold;
}
void hal_set_next_node(RGB_NODE node)
{    
    s_nextRGBNode.red    =   node.red;
    s_nextRGBNode.green  =   node.green;
    s_nextRGBNode.blue   =   node.blue;
    s_nextRGBNode.time_to_transition   =   node.time_to_transition;
    s_nextRGBNode.time_to_hold   			 =   node.time_to_hold;
}
void hal_reset_all_nodes()
{
    s_curStep = 0;
		s_timeToHold = 0;
		memset(&s_curRGBNode,0x00,sizeof(RGB_NODE));
}
BOOL hal_previous_node_finished(){
	return (s_finish_node > 0);
}
/*****************************************************************
    Pattern format : S/xx/yy/zz/rrggbb/rrggbb.....
    where : 
    - S : start pattern = 'S'
    - xx: pattern len , 1 byte.
    - yy: speed, 1 byte. Value : {0,1,2} = {Low , Medium , Fast}
    - zz: mode, 1 byte. Value : {0,1,2} = {Single, Dimming, Continuous}
    -rrggbb : color value, red green blue respectively.
*****************************************************************/
    
/*****************************************************************
        Raw data received : S/xx/yy/zz/rrggbb/rrggbb.....
        where : 
        - S : start pattern = 'S'
        - xx: pattern len , 2 byte. In acii format
        - yy: speed, 2 byte in ascii format. Value : {'00','01','02'} = {Low , Medium , Fast}
        - zz: mode, 2 byte in ascii format . Value : {'00','01','02'} = {Single, Dimming, Continuous}
        -rrggbb : color value, red green blue respectively. 6 byte for every color. In ascii format
*****************************************************************/


static unsigned char s_numOfNode = 0;
static unsigned char s_curNode = 0;
volatile RGB_NODE* s_pRunningPattern;
volatile unsigned char* m_pPatternSetting = 0;

unsigned int hal_convert_ascii_to_value(unsigned char* pAscii, unsigned char len)
{
    unsigned int value = 0;
    unsigned char i = 0;
    unsigned char tmp = 0;

    for(i = 0; i < len; i++)
    {
        value = value << 4;
        tmp = *(pAscii+i);
        if(tmp >='0' && tmp <='9')
        {
            value += (tmp - '0');
        }
        else if((tmp >= 'A') && (tmp <= 'F'))
        {
            value += (10 + tmp - 'A');
        }
        else if((tmp >= 'a') && (tmp <= 'f'))
        {
            value += (10 + tmp - 'a');
        }    
    }

    return value;
    
}
BOOL hal_rgb_retrieve_data(unsigned char* pRawValue)
{
    unsigned int tmp;
    unsigned char* pTmp;
    unsigned int len = 0;
    
    /*calculate the len of pattern*/
    pTmp  = &pRawValue[PATTERN_RAW_LEN_INDX];
    tmp = hal_convert_ascii_to_value(pTmp,PATTERN_RAW_LEN_INDX_LEN);

    if(tmp == 0) return FALSE;
    tmp -= ((PATTERN_RAW_START_INDX_LEN) + PATTERN_RAW_LEN_INDX_LEN +
            (PATTERN_RAW_MODE_INDX_LEN + PATTERN_RAW_SPEED_INDX_LEN));
    tmp /= (PATTERN_RAW_DATA_INDX_EACH_LEN);

    tmp *= PATTERN_DATA_EACH_LEN;
    tmp += ((PATTERN_START_INDX_LEN) + PATTERN_LEN_INDX_LEN +
            (PATTERN_MODE_INDX_LEN + PATTERN_SPEED_INDX_LEN));
    len = tmp;
             
    if(m_pPatternSetting != 0) free(m_pPatternSetting);
    m_pPatternSetting = (unsigned char*)malloc(len);

    /*Retrieve start pattern*/
    m_pPatternSetting[PATTERN_START_INDX] = pRawValue[PATTERN_START_INDX];

    /*Retrive len*/
    m_pPatternSetting[PATTERN_LEN_INDX] = len;

    /*Retrieve speed*/
    m_pPatternSetting[PATTERN_SPEED_INDX] = 
        hal_convert_ascii_to_value(&pRawValue[PATTERN_RAW_SPEED_INDX],PATTERN_RAW_SPEED_INDX_LEN);
    /*Retrive mode*/
    m_pPatternSetting[PATTERN_MODE_INDX] = 
        hal_convert_ascii_to_value(&pRawValue[PATTERN_RAW_MODE_INDX],PATTERN_RAW_MODE_INDX_LEN);
    /*Retrieve data*/
    len = len - (PATTERN_START_INDX_LEN + PATTERN_LEN_INDX_LEN + 
                 PATTERN_MODE_INDX_LEN + PATTERN_SPEED_INDX_LEN);
    for(tmp = 0 ; tmp < len ; tmp++)
    {
        m_pPatternSetting[PATTERN_DATA_INDX + tmp] = hal_convert_ascii_to_value(&pRawValue[PATTERN_RAW_DATA_INDX + tmp*PATTERN_RAW_DATA_INDX_EACH_LEN],
																																								PATTERN_RAW_DATA_INDX_EACH_LEN);
    }

    return TRUE;

}
unsigned char HAL_RGB_Setup_Pattern(void* pRawValue)
{
    //unsigned char* pPattern;
    unsigned char len;
    unsigned char speed = 0;
    unsigned char mode = 0;

    if(hal_rgb_retrieve_data((unsigned char*)pRawValue) == FALSE)
        return FALSE;  
    
    /*Is this start of RGB Pattern*/
    if(m_pPatternSetting[PATTERN_START_INDX] == PATTERN_START_VAL)
    {   
        len = m_pPatternSetting[PATTERN_LEN_INDX] - 4;
        len = len / 3;
        speed = m_pPatternSetting[PATTERN_SPEED_INDX];
        mode = m_pPatternSetting[PATTERN_MODE_INDX];
        switch(mode)
        {
            case 0:
                fill_single_pattern(speed,&m_pPatternSetting[PATTERN_DATA_INDX],len);
                s_numOfNode = sizeof(pattern_single_arr)/sizeof(RGB_NODE);
                s_pRunningPattern = pattern_single_arr;
                break;
            case 1:
                fill_dimming_pattern(speed,&m_pPatternSetting[PATTERN_DATA_INDX],len);
                s_numOfNode = sizeof(pattern_dimming_arr)/sizeof(RGB_NODE);
                s_pRunningPattern = pattern_dimming_arr;
                break;
            case 2 :
            default :
                fill_continuous_pattern(speed,&m_pPatternSetting[PATTERN_DATA_INDX],len);
                s_numOfNode = sizeof(pattern_continuous_arr)/sizeof(RGB_NODE);
                s_pRunningPattern = pattern_continuous_arr;
                break;

        }
				return TRUE;       
    }
    else 
        return FALSE;
}

/*Must keep this function inside a loop*/
void HAL_RGB_Run_Pattern()
{
		if(s_numOfNode == 0) return;
	
		if(s_curNode == (s_numOfNode - 1))
				s_curNode = 0;
	
		if(hal_previous_node_finished())
			HAL_RGB_Run_Node(s_pRunningPattern[s_curNode++]);
}

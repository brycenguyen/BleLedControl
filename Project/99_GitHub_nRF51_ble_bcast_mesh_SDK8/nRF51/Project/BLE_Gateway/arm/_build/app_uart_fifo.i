#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"










 

#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart.h"










 








 




#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 







 




  
 







#line 37 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 208 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 272 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 26 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 27 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"










 








 




#line 26 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\compiler_abstraction.h"




























 



 


    



    



    



    



    

  
#line 123 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\compiler_abstraction.h"

 

#line 27 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"

 








































 





 



 









 

typedef enum {
 
  Reset_IRQn                    = -15,               
  NonMaskableInt_IRQn           = -14,               
  HardFault_IRQn                = -13,               
  SVCall_IRQn                   =  -5,               
  DebugMonitor_IRQn             =  -4,               
  PendSV_IRQn                   =  -2,               
  SysTick_IRQn                  =  -1,               
 
  POWER_CLOCK_IRQn              =   0,               
  RADIO_IRQn                    =   1,               
  UART0_IRQn                    =   2,               
  SPI0_TWI0_IRQn                =   3,               
  SPI1_TWI1_IRQn                =   4,               
  GPIOTE_IRQn                   =   6,               
  ADC_IRQn                      =   7,               
  TIMER0_IRQn                   =   8,               
  TIMER1_IRQn                   =   9,               
  TIMER2_IRQn                   =  10,               
  RTC0_IRQn                     =  11,               
  TEMP_IRQn                     =  12,               
  RNG_IRQn                      =  13,               
  ECB_IRQn                      =  14,               
  CCM_AAR_IRQn                  =  15,               
  WDT_IRQn                      =  16,               
  RTC1_IRQn                     =  17,               
  QDEC_IRQn                     =  18,               
  LPCOMP_IRQn                   =  19,               
  SWI0_IRQn                     =  20,               
  SWI1_IRQn                     =  21,               
  SWI2_IRQn                     =  22,               
  SWI3_IRQn                     =  23,               
  SWI4_IRQn                     =  24,               
  SWI5_IRQn                     =  25                
} IRQn_Type;




 


 
 
 

 




   

#line 1 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"
 







 

























 
























 




 


 

 













#line 100 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"


 







#line 125 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"

#line 127 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 



#line 292 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmInstr.h"



#line 685 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmInstr.h"

   

#line 128 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 271 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"


#line 307 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"


#line 632 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cmFunc.h"

 


#line 129 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"








 
#line 154 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"

 






 
#line 170 "..\\..\\..\\..\\..\\..\\components\\toolchain\\gcc\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








#line 120 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\system_nrf51.h"




























 








#line 39 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\system_nrf51.h"


extern uint32_t SystemCoreClock;     









 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 121 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


 
 
 




 


 

  #pragma push
  #pragma anon_unions
#line 148 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


typedef struct {
  volatile uint32_t  CPU0;                               
  volatile uint32_t  SPIS1;                              
  volatile uint32_t  RADIO;                              
  volatile uint32_t  ECB;                                
  volatile uint32_t  CCM;                                
  volatile uint32_t  AAR;                                
} AMLI_RAMPRI_Type;

typedef struct {
  volatile uint32_t  SCK;                                
  volatile uint32_t  MOSI;                               
  volatile uint32_t  MISO;                               
} SPIM_PSEL_Type;

typedef struct {
  volatile uint32_t  PTR;                                
  volatile uint32_t  MAXCNT;                             
  volatile const  uint32_t  AMOUNT;                             
} SPIM_RXD_Type;

typedef struct {
  volatile uint32_t  PTR;                                
  volatile uint32_t  MAXCNT;                             
  volatile const  uint32_t  AMOUNT;                             
} SPIM_TXD_Type;

typedef struct {
  volatile  uint32_t  EN;                                 
  volatile  uint32_t  DIS;                                
} PPI_TASKS_CHG_Type;

typedef struct {
  volatile uint32_t  EEP;                                
  volatile uint32_t  TEP;                                
} PPI_CH_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[30];
  volatile  uint32_t  TASKS_CONSTLAT;                     
  volatile  uint32_t  TASKS_LOWPWR;                       
  volatile const  uint32_t  RESERVED1[34];
  volatile uint32_t  EVENTS_POFWARN;                     
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile uint32_t  RESETREAS;                          
  volatile const  uint32_t  RESERVED4[9];
  volatile const  uint32_t  RAMSTATUS;                          
  volatile const  uint32_t  RESERVED5[53];
  volatile  uint32_t  SYSTEMOFF;                          
  volatile const  uint32_t  RESERVED6[3];
  volatile uint32_t  POFCON;                             
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  GPREGRET;                          
 
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  RAMON;                              
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RESET;                             
 
  volatile const  uint32_t  RESERVED10[3];
  volatile uint32_t  RAMONB;                             
  volatile const  uint32_t  RESERVED11[8];
  volatile uint32_t  DCDCEN;                             
  volatile const  uint32_t  RESERVED12[291];
  volatile uint32_t  DCDCFORCE;                          
} NRF_POWER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_HFCLKSTART;                   
  volatile  uint32_t  TASKS_HFCLKSTOP;                    
  volatile  uint32_t  TASKS_LFCLKSTART;                   
  volatile  uint32_t  TASKS_LFCLKSTOP;                    
  volatile  uint32_t  TASKS_CAL;                          
  volatile  uint32_t  TASKS_CTSTART;                      
  volatile  uint32_t  TASKS_CTSTOP;                       
  volatile const  uint32_t  RESERVED0[57];
  volatile uint32_t  EVENTS_HFCLKSTARTED;                
  volatile uint32_t  EVENTS_LFCLKSTARTED;                
  volatile const  uint32_t  RESERVED1;
  volatile uint32_t  EVENTS_DONE;                        
  volatile uint32_t  EVENTS_CTTO;                        
  volatile const  uint32_t  RESERVED2[124];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[63];
  volatile const  uint32_t  HFCLKRUN;                           
  volatile const  uint32_t  HFCLKSTAT;                          
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  LFCLKRUN;                           
  volatile const  uint32_t  LFCLKSTAT;                          
  volatile const  uint32_t  LFCLKSRCCOPY;                      
 
  volatile const  uint32_t  RESERVED5[62];
  volatile uint32_t  LFCLKSRC;                           
  volatile const  uint32_t  RESERVED6[7];
  volatile uint32_t  CTIV;                               
  volatile const  uint32_t  RESERVED7[5];
  volatile uint32_t  XTALFREQ;                           
} NRF_CLOCK_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[330];
  volatile uint32_t  PERR0;                              
  volatile uint32_t  RLENR0;                             
  volatile const  uint32_t  RESERVED1[52];
  volatile uint32_t  PROTENSET0;                         
  volatile uint32_t  PROTENSET1;                         
  volatile uint32_t  DISABLEINDEBUG;                     
  volatile uint32_t  PROTBLOCKSIZE;                      
} NRF_MPU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[448];
  volatile uint32_t  REPLACEADDR[8];                     
  volatile const  uint32_t  RESERVED1[24];
  volatile uint32_t  PATCHADDR[8];                       
  volatile const  uint32_t  RESERVED2[24];
  volatile uint32_t  PATCHEN;                            
  volatile uint32_t  PATCHENSET;                         
  volatile uint32_t  PATCHENCLR;                         
} NRF_PU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[896];
  AMLI_RAMPRI_Type RAMPRI;                           
} NRF_AMLI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_TXEN;                         
  volatile  uint32_t  TASKS_RXEN;                         
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_DISABLE;                      
  volatile  uint32_t  TASKS_RSSISTART;                    
  volatile  uint32_t  TASKS_RSSISTOP;                     
  volatile  uint32_t  TASKS_BCSTART;                      
  volatile  uint32_t  TASKS_BCSTOP;                       
  volatile const  uint32_t  RESERVED0[55];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_ADDRESS;                     
  volatile uint32_t  EVENTS_PAYLOAD;                     
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_DISABLED;                    
  volatile uint32_t  EVENTS_DEVMATCH;                    
  volatile uint32_t  EVENTS_DEVMISS;                     
  volatile uint32_t  EVENTS_RSSIEND;                    
 
  volatile const  uint32_t  RESERVED1[2];
  volatile uint32_t  EVENTS_BCMATCH;                     
  volatile const  uint32_t  RESERVED2[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[61];
  volatile const  uint32_t  CRCSTATUS;                          
  volatile const  uint32_t  CD;                                 
  volatile const  uint32_t  RXMATCH;                            
  volatile const  uint32_t  RXCRC;                              
  volatile const  uint32_t  DAI;                                
  volatile const  uint32_t  RESERVED5[60];
  volatile uint32_t  PACKETPTR;                          
  volatile uint32_t  FREQUENCY;                          
  volatile uint32_t  TXPOWER;                            
  volatile uint32_t  MODE;                               
  volatile uint32_t  PCNF0;                              
  volatile uint32_t  PCNF1;                              
  volatile uint32_t  BASE0;                              
  volatile uint32_t  BASE1;                              
  volatile uint32_t  PREFIX0;                            
  volatile uint32_t  PREFIX1;                            
  volatile uint32_t  TXADDRESS;                          
  volatile uint32_t  RXADDRESSES;                        
  volatile uint32_t  CRCCNF;                             
  volatile uint32_t  CRCPOLY;                            
  volatile uint32_t  CRCINIT;                            
  volatile uint32_t  TEST;                               
  volatile uint32_t  TIFS;                               
  volatile const  uint32_t  RSSISAMPLE;                         
  volatile const  uint32_t  RESERVED6;
  volatile const  uint32_t  STATE;                              
  volatile uint32_t  DATAWHITEIV;                        
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  BCC;                                
  volatile const  uint32_t  RESERVED8[39];
  volatile uint32_t  DAB[8];                             
  volatile uint32_t  DAP[8];                             
  volatile uint32_t  DACNF;                              
  volatile const  uint32_t  RESERVED9[56];
  volatile uint32_t  OVERRIDE0;                          
  volatile uint32_t  OVERRIDE1;                          
  volatile uint32_t  OVERRIDE2;                          
  volatile uint32_t  OVERRIDE3;                          
  volatile uint32_t  OVERRIDE4;                          
  volatile const  uint32_t  RESERVED10[561];
  volatile uint32_t  POWER;                              
} NRF_RADIO_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile  uint32_t  TASKS_STOPRX;                       
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile  uint32_t  TASKS_STOPTX;                       
  volatile const  uint32_t  RESERVED0[3];
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile const  uint32_t  RESERVED1[56];
  volatile uint32_t  EVENTS_CTS;                         
  volatile uint32_t  EVENTS_NCTS;                        
  volatile uint32_t  EVENTS_RXDRDY;                      
  volatile const  uint32_t  RESERVED2[4];
  volatile uint32_t  EVENTS_TXDRDY;                      
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED4[7];
  volatile uint32_t  EVENTS_RXTO;                        
  volatile const  uint32_t  RESERVED5[111];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED6[93];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED7[31];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELRTS;                            
  volatile uint32_t  PSELTXD;                            
  volatile uint32_t  PSELCTS;                            
  volatile uint32_t  PSELRXD;                            
  volatile const  uint32_t  RXD;                               

 
  volatile  uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED9;
  volatile uint32_t  BAUDRATE;                           
  volatile const  uint32_t  RESERVED10[17];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED11[675];
  volatile uint32_t  POWER;                              
} NRF_UART_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[66];
  volatile uint32_t  EVENTS_READY;                       
  volatile const  uint32_t  RESERVED1[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELMISO;                           
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED7[681];
  volatile uint32_t  POWER;                              
} NRF_SPI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile const  uint32_t  RESERVED1[2];
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED2;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED3[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile uint32_t  EVENTS_RXDREADY;                    
  volatile const  uint32_t  RESERVED4[4];
  volatile uint32_t  EVENTS_TXDSENT;                     
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED6[4];
  volatile uint32_t  EVENTS_BB;                          
  volatile const  uint32_t  RESERVED7[3];
  volatile uint32_t  EVENTS_SUSPENDED;                   
  volatile const  uint32_t  RESERVED8[45];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED9[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED10[110];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED11[14];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  PSELSCL;                            
  volatile uint32_t  PSELSDA;                            
  volatile const  uint32_t  RESERVED13[2];
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED15[24];
  volatile uint32_t  ADDRESS;                            
  volatile const  uint32_t  RESERVED16[668];
  volatile uint32_t  POWER;                              
} NRF_TWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[9];
  volatile  uint32_t  TASKS_ACQUIRE;                      
  volatile  uint32_t  TASKS_RELEASE;                      
  volatile const  uint32_t  RESERVED1[54];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED2[8];
  volatile uint32_t  EVENTS_ACQUIRED;                    
  volatile const  uint32_t  RESERVED3[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED4[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED5[61];
  volatile const  uint32_t  SEMSTAT;                            
  volatile const  uint32_t  RESERVED6[15];
  volatile uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED7[47];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMISO;                           
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELCSN;                            
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RXDPTR;                             
  volatile uint32_t  MAXRX;                              
  volatile const  uint32_t  AMOUNTRX;                           
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  TXDPTR;                             
  volatile uint32_t  MAXTX;                              
  volatile const  uint32_t  AMOUNTTX;                           
  volatile const  uint32_t  RESERVED11;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  DEF;                                
  volatile const  uint32_t  RESERVED13[24];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED14[654];
  volatile uint32_t  POWER;                              
} NRF_SPIS_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED2[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile const  uint32_t  RESERVED3[2];
  volatile uint32_t  EVENTS_ENDRX;                       
  volatile const  uint32_t  RESERVED4;
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ENDTX;                       
  volatile const  uint32_t  RESERVED6[10];
  volatile uint32_t  EVENTS_STARTED;                     
  volatile const  uint32_t  RESERVED7[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED8[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED9[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED10;
  SPIM_PSEL_Type PSEL;                               
  volatile const  uint32_t  RESERVED11[4];
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED12[3];
  SPIM_RXD_Type RXD;                                 
  volatile const  uint32_t  RESERVED13;
  SPIM_TXD_Type TXD;                                 
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED15[26];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED16[654];
  volatile uint32_t  POWER;                              
} NRF_SPIM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_OUT[4];                       
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_IN[4];                       
  volatile const  uint32_t  RESERVED1[27];
  volatile uint32_t  EVENTS_PORT;                        
  volatile const  uint32_t  RESERVED2[97];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[129];
  volatile uint32_t  CONFIG[4];                          
  volatile const  uint32_t  RESERVED4[695];
  volatile uint32_t  POWER;                              
} NRF_GPIOTE_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  BUSY;                               
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_ADC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_COUNT;                        
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_SHUTDOWN;                     
  volatile const  uint32_t  RESERVED0[11];
  volatile  uint32_t  TASKS_CAPTURE[4];                   
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[126];
  volatile uint32_t  MODE;                               
  volatile uint32_t  BITMODE;                            
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED7[683];
  volatile uint32_t  POWER;                              
} NRF_TIMER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_TRIGOVRFLW;                   
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_TICK;                        
  volatile uint32_t  EVENTS_OVRFLW;                      
  volatile const  uint32_t  RESERVED1[14];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[109];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[13];
  volatile uint32_t  EVTEN;                              
  volatile uint32_t  EVTENSET;                          
 
  volatile uint32_t  EVTENCLR;                          
 
  volatile const  uint32_t  RESERVED4[110];
  volatile const  uint32_t  COUNTER;                            
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED5[13];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED6[683];
  volatile uint32_t  POWER;                              
} NRF_RTC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_DATARDY;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[127];
  volatile const  int32_t   TEMP;                               
  volatile const  uint32_t  RESERVED3[700];
  volatile uint32_t  POWER;                              
} NRF_TEMP_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_VALRDY;                      
  volatile const  uint32_t  RESERVED1[63];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[126];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  VALUE;                              
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_RNG_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTECB;                    

 
  volatile  uint32_t  TASKS_STOPECB;                     
 
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_ENDECB;                      
  volatile uint32_t  EVENTS_ERRORECB;                   
 
  volatile const  uint32_t  RESERVED1[127];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  ECBDATAPTR;                         
  volatile const  uint32_t  RESERVED3[701];
  volatile uint32_t  POWER;                              
} NRF_ECB_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                       
 
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_RESOLVED;                    
  volatile uint32_t  EVENTS_NOTRESOLVED;                 
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  NIRK;                               
  volatile uint32_t  IRKPTR;                             
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  ADDRPTR;                            
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED6[697];
  volatile uint32_t  POWER;                              
} NRF_AAR_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_KSGEN;                       
 
  volatile  uint32_t  TASKS_CRYPT;                       
 
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_ENDKSGEN;                    
  volatile uint32_t  EVENTS_ENDCRYPT;                    
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  MICSTATUS;                          
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  MODE;                               
  volatile uint32_t  CNFPTR;                             
  volatile uint32_t  INPTR;                              
  volatile uint32_t  OUTPTR;                             
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED5[697];
  volatile uint32_t  POWER;                              
} NRF_CCM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile const  uint32_t  RESERVED0[63];
  volatile uint32_t  EVENTS_TIMEOUT;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  RUNSTATUS;                          
  volatile const  uint32_t  REQSTATUS;                          
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  CRV;                                
  volatile uint32_t  RREN;                               
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED4[60];
  volatile  uint32_t  RR[8];                              
  volatile const  uint32_t  RESERVED5[631];
  volatile uint32_t  POWER;                              
} NRF_WDT_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_READCLRACC;                  
 
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_SAMPLERDY;                   
  volatile uint32_t  EVENTS_REPORTRDY;                  
 
  volatile uint32_t  EVENTS_ACCOF;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[125];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  LEDPOL;                             
  volatile uint32_t  SAMPLEPER;                          
  volatile const  int32_t   SAMPLE;                             
  volatile uint32_t  REPORTPER;                          
  volatile const  int32_t   ACC;                                
  volatile const  int32_t   ACCREAD;                           
 
  volatile uint32_t  PSELLED;                            
  volatile uint32_t  PSELA;                              
  volatile uint32_t  PSELB;                              
  volatile uint32_t  DBFEN;                              
  volatile const  uint32_t  RESERVED4[5];
  volatile uint32_t  LEDPRE;                             
  volatile const  uint32_t  ACCDBL;                             
  volatile const  uint32_t  ACCDBLREAD;                        
 
  volatile const  uint32_t  RESERVED5[684];
  volatile uint32_t  POWER;                              
} NRF_QDEC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_SAMPLE;                       
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_DOWN;                        
  volatile uint32_t  EVENTS_UP;                          
  volatile uint32_t  EVENTS_CROSS;                       
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  PSEL;                               
  volatile uint32_t  REFSEL;                             
  volatile uint32_t  EXTREFSEL;                          
  volatile const  uint32_t  RESERVED5[4];
  volatile uint32_t  ANADETECT;                          
  volatile const  uint32_t  RESERVED6[694];
  volatile uint32_t  POWER;                              
} NRF_LPCOMP_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  UNUSED;                             
} NRF_SWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[256];
  volatile const  uint32_t  READY;                              
  volatile const  uint32_t  RESERVED1[64];
  volatile uint32_t  CONFIG;                             
  
  union {
    volatile uint32_t  ERASEPCR1;                        
    volatile uint32_t  ERASEPAGE;                        
  };
  volatile uint32_t  ERASEALL;                           
  volatile uint32_t  ERASEPCR0;                          
  volatile uint32_t  ERASEUICR;                          
} NRF_NVMC_Type;


 
 
 




 

typedef struct {                                     
  PPI_TASKS_CHG_Type TASKS_CHG[4];                   
  volatile const  uint32_t  RESERVED0[312];
  volatile uint32_t  CHEN;                               
  volatile uint32_t  CHENSET;                            
  volatile uint32_t  CHENCLR;                            
  volatile const  uint32_t  RESERVED1;
  PPI_CH_Type CH[16];                                
  volatile const  uint32_t  RESERVED2[156];
  volatile uint32_t  CHG[4];                             
} NRF_PPI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile const  uint32_t  CODEPAGESIZE;                       
  volatile const  uint32_t  CODESIZE;                           
  volatile const  uint32_t  RESERVED1[4];
  volatile const  uint32_t  CLENR0;                             
  volatile const  uint32_t  PPFC;                               
  volatile const  uint32_t  RESERVED2;
  volatile const  uint32_t  NUMRAMBLOCK;                        
  
  union {
    volatile const  uint32_t  SIZERAMBLOCK[4];                 

 
    volatile const  uint32_t  SIZERAMBLOCKS;                    
  };
  volatile const  uint32_t  RESERVED3[5];
  volatile const  uint32_t  CONFIGID;                           
  volatile const  uint32_t  DEVICEID[2];                        
  volatile const  uint32_t  RESERVED4[6];
  volatile const  uint32_t  ER[4];                              
  volatile const  uint32_t  IR[4];                              
  volatile const  uint32_t  DEVICEADDRTYPE;                     
  volatile const  uint32_t  DEVICEADDR[2];                      
  volatile const  uint32_t  OVERRIDEEN;                         
  volatile const  uint32_t  NRF_1MBIT[5];                      
 
  volatile const  uint32_t  RESERVED5[10];
  volatile const  uint32_t  BLE_1MBIT[5];                      
 
} NRF_FICR_Type;


 
 
 




 

typedef struct {                                     
  volatile uint32_t  CLENR0;                             
  volatile uint32_t  RBPCONF;                            
  volatile uint32_t  XTALFREQ;                           
  volatile const  uint32_t  RESERVED0;
  volatile const  uint32_t  FWID;                               
  
  union {
    volatile uint32_t  NRFFW[15];                        
    volatile uint32_t  BOOTLOADERADDR;                   
  };
  volatile uint32_t  NRFHW[12];                          
  volatile uint32_t  CUSTOMER[32];                       
} NRF_UICR_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[321];
  volatile uint32_t  OUT;                                
  volatile uint32_t  OUTSET;                             
  volatile uint32_t  OUTCLR;                             
  volatile const  uint32_t  IN;                                 
  volatile uint32_t  DIR;                                
  volatile uint32_t  DIRSET;                             
  volatile uint32_t  DIRCLR;                             
  volatile const  uint32_t  RESERVED1[120];
  volatile uint32_t  PIN_CNF[32];                        
} NRF_GPIO_Type;


 

  #pragma pop
#line 1214 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"




 
 
 

#line 1256 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


 
 
 

#line 1296 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51.h"


   
   
   








#line 28 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"


































  






 




#line 48 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"
#line 49 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_svc.h"


































 
 







#line 69 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_svc.h"

#line 50 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"
#line 51 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"




























 



 

 
 

 
 

 






 






 






 
 

 






 






 






 
 

 



 
 

 





 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 





 
 

 






 
#line 182 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 190 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 199 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 






 
 

 



 
 

 






 
 

 
 

 
#line 241 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 253 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 265 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 277 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 289 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 301 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 313 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 325 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 340 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 352 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 364 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 376 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 388 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 400 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 412 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 424 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 439 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 451 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 463 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 475 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 487 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 499 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 511 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 523 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 538 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 550 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 562 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 574 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 586 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 598 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 610 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 622 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 637 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 649 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 661 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 673 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 685 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 697 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 709 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 721 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 736 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 748 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 760 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 772 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 784 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 796 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 808 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
#line 820 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"


 
 

 
 

 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 





 
 

 






 
 

 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 





 
 

 





 
 

 





 






 
 

 






 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 
 

 






 






 
 

 






 
 

 
 

 





 
 

 



 



 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 
#line 2682 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 






 





 






 
 

 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 
#line 2798 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 



 






 
 

 






 
 

 
 

 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 
 

 
#line 2950 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 2965 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 
 

 





 
 

 
 

 





 
 

 






 
 

 





 
 

 






 
 

 
 

 






 
 

 






 
 

 





 





 





 





 





 





 





 
 

 





 





 





 





 
 

 




 
 

 
#line 3753 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 





 
 

 



 
 

 





 





 





 





 
 

 





 
 

 





 





 





 





 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 
 

 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 







 
 

 
 

 





 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 
#line 4877 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 
#line 4899 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 
#line 5184 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 
#line 5195 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 



 



 
 

 





 





 



 



 



 
 

 



 



 



 



 
 

 



 



 



 



 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 





 
#line 5350 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 





 





 
 

 



 
 

 



 
 

 
#line 5409 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 



 



 



 



 



 



 



 





 





 





 





 





 





 





 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 



 
 

 






 
 

 
 

 





 
 

 






 
 

 






 
 

 





 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 





 





 





 





 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 



 
 

 



 
 

 
#line 5928 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 





 





 
 

 






 
 

 
 

 





 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 
 

 
#line 6070 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 






 
 

 
 

 





 
 

 






 






 
 

 






 






 
 

 
#line 6207 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 
#line 6475 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 
 

 





 
 

 



 
 

 



 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 
#line 6844 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_bitfields.h"

 
 

 





 





 
 

 






 
 

 
 

 





 





 
 

 





 
 

 




 
 

 
 

 






 
 

 






 
 

 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 
 

 




 
 

 






 
#line 52 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error_soc.h"


































 








 

 



#line 1 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error.h"


































  
 




 

 




 




 

#line 73 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error.h"





 
#line 51 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_error_soc.h"

 


 




 




 


 







 
#line 53 "..\\..\\..\\..\\..\\..\\components\\softdevice\\s110\\headers\\nrf_soc.h"


 

 



 


 




















 


 

 
enum NRF_SOC_SVCS
{
  SD_PPI_CHANNEL_ENABLE_GET = (0x20),
  SD_PPI_CHANNEL_ENABLE_SET,
  SD_PPI_CHANNEL_ENABLE_CLR,
  SD_PPI_CHANNEL_ASSIGN,
  SD_PPI_GROUP_TASK_ENABLE,
  SD_PPI_GROUP_TASK_DISABLE,
  SD_PPI_GROUP_ASSIGN,
  SD_PPI_GROUP_GET,
  SD_FLASH_PAGE_ERASE,
  SD_FLASH_WRITE,
  SD_FLASH_PROTECT,
  SD_MUTEX_NEW = (0x2B),
  SD_MUTEX_ACQUIRE,
  SD_MUTEX_RELEASE,
  SD_NVIC_ENABLEIRQ,
  SD_NVIC_DISABLEIRQ,
  SD_NVIC_GETPENDINGIRQ,
  SD_NVIC_SETPENDINGIRQ,
  SD_NVIC_CLEARPENDINGIRQ,
  SD_NVIC_SETPRIORITY,
  SD_NVIC_GETPRIORITY,
  SD_NVIC_SYSTEMRESET,
  SD_NVIC_CRITICAL_REGION_ENTER,
  SD_NVIC_CRITICAL_REGION_EXIT,
  SD_RAND_APPLICATION_POOL_CAPACITY,
  SD_RAND_APPLICATION_BYTES_AVAILABLE,
  SD_RAND_APPLICATION_GET_VECTOR,
  SD_POWER_MODE_SET,
  SD_POWER_SYSTEM_OFF,
  SD_POWER_RESET_REASON_GET,
  SD_POWER_RESET_REASON_CLR,
  SD_POWER_POF_ENABLE,
  SD_POWER_POF_THRESHOLD_SET,
  SD_POWER_RAMON_SET,
  SD_POWER_RAMON_CLR,
  SD_POWER_RAMON_GET,
  SD_POWER_GPREGRET_SET,
  SD_POWER_GPREGRET_CLR,
  SD_POWER_GPREGRET_GET,
  SD_POWER_DCDC_MODE_SET,
  SD_APP_EVT_WAIT,
  SD_CLOCK_HFCLK_REQUEST,
  SD_CLOCK_HFCLK_RELEASE,
  SD_CLOCK_HFCLK_IS_RUNNING,
  SD_RADIO_NOTIFICATION_CFG_SET,
  SD_ECB_BLOCK_ENCRYPT,
  SD_RADIO_SESSION_OPEN,
  SD_RADIO_SESSION_CLOSE,
  SD_RADIO_REQUEST,
  SD_EVT_GET,
  SD_TEMP_GET,
  SVC_SOC_LAST
};

 
enum NRF_MUTEX_VALUES
{
  NRF_MUTEX_FREE,
  NRF_MUTEX_TAKEN
};

 
enum NRF_APP_PRIORITIES
{
  NRF_APP_PRIORITY_HIGH = 1,
  NRF_APP_PRIORITY_LOW = 3
};

 
enum NRF_POWER_MODES
{
  NRF_POWER_MODE_CONSTLAT,   
  NRF_POWER_MODE_LOWPWR      
};


 
enum NRF_POWER_THRESHOLDS
{
  NRF_POWER_THRESHOLD_V21,   
  NRF_POWER_THRESHOLD_V23,   
  NRF_POWER_THRESHOLD_V25,    
  NRF_POWER_THRESHOLD_V27    
};


 
enum NRF_POWER_DCDC_MODES
{
  NRF_POWER_DCDC_DISABLE,           
  NRF_POWER_DCDC_ENABLE             
};

 
enum NRF_RADIO_NOTIFICATION_DISTANCES
{
  NRF_RADIO_NOTIFICATION_DISTANCE_NONE = 0,  
  NRF_RADIO_NOTIFICATION_DISTANCE_800US,     
  NRF_RADIO_NOTIFICATION_DISTANCE_1740US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_2680US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_3620US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_4560US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_5500US     
};


 
enum NRF_RADIO_NOTIFICATION_TYPES
{
  NRF_RADIO_NOTIFICATION_TYPE_NONE = 0,         
  NRF_RADIO_NOTIFICATION_TYPE_INT_ON_ACTIVE,    
  NRF_RADIO_NOTIFICATION_TYPE_INT_ON_INACTIVE,  
  NRF_RADIO_NOTIFICATION_TYPE_INT_ON_BOTH,      
};

 
enum NRF_SOC_EVTS
{
  NRF_EVT_HFCLKSTARTED,                          
  NRF_EVT_POWER_FAILURE_WARNING,                 
  NRF_EVT_FLASH_OPERATION_SUCCESS,               
  NRF_EVT_FLASH_OPERATION_ERROR,                 
  NRF_EVT_RADIO_BLOCKED,                         
  NRF_EVT_RADIO_CANCELED,                        
  NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN,  
  NRF_EVT_RADIO_SESSION_IDLE,                    
  NRF_EVT_RADIO_SESSION_CLOSED,                  
  NRF_EVT_NUMBER_OF_EVTS
};

 


 



 
typedef volatile uint8_t nrf_mutex_t;

 
typedef uint8_t nrf_app_irq_priority_t;

 
typedef uint8_t nrf_power_mode_t;

 
typedef uint8_t nrf_power_failure_threshold_t;

 
typedef uint32_t nrf_power_dcdc_mode_t;

 
typedef uint8_t nrf_radio_notification_distance_t;

 
typedef uint8_t nrf_radio_notification_type_t;

 
enum NRF_RADIO_CALLBACK_SIGNAL_TYPE
{
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_START,              
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0,             
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO,              
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED,      
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED    
};





 
enum NRF_RADIO_SIGNAL_CALLBACK_ACTION
{
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,             
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,           
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_END,              
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END   
};

 
enum NRF_RADIO_HFCLK_CFG
{
  NRF_RADIO_HFCLK_CFG_DEFAULT,                       
  NRF_RADIO_HFCLK_CFG_FORCE_XTAL,                    
};

 
enum NRF_RADIO_PRIORITY
{
  NRF_RADIO_PRIORITY_HIGH,                           
  NRF_RADIO_PRIORITY_NORMAL,                         
};

 
enum NRF_RADIO_REQUEST_TYPE
{
  NRF_RADIO_REQ_TYPE_EARLIEST,                       
  NRF_RADIO_REQ_TYPE_NORMAL                          
};

 
typedef struct
{
  uint8_t       hfclk;                               
  uint8_t       priority;                            
  uint32_t      length_us;                           
  uint32_t      timeout_us;                          
} nrf_radio_request_earliest_t;

 
typedef struct
{
  uint8_t       hfclk;                               
  uint8_t       priority;                            
  uint32_t      distance_us;                         
  uint32_t      length_us;                           
} nrf_radio_request_normal_t;

 
typedef struct
{
  uint8_t                         request_type;      
  union
  {
    nrf_radio_request_earliest_t  earliest;          
    nrf_radio_request_normal_t    normal;            
  } params;
} nrf_radio_request_t;

 
typedef struct
{
  uint8_t               callback_action;             
  union
  {
    struct
    {
      nrf_radio_request_t * p_next;                  
    } request;                                       
    struct
    {
      uint32_t              length_us;               
    } extend;                                        
  } params;
} nrf_radio_signal_callback_return_param_t;












 
typedef nrf_radio_signal_callback_return_param_t * (*nrf_radio_signal_callback_t) (uint8_t signal_type);

 
typedef struct
{
  uint8_t key[(16)];                   
  uint8_t cleartext[(16)];       
  uint8_t ciphertext[((16))];     
} nrf_ecb_hal_data_t;

 


 






 
uint32_t __svc(SD_MUTEX_NEW) sd_mutex_new(nrf_mutex_t * p_mutex);







 
uint32_t __svc(SD_MUTEX_ACQUIRE) sd_mutex_acquire(nrf_mutex_t * p_mutex);






 
uint32_t __svc(SD_MUTEX_RELEASE) sd_mutex_release(nrf_mutex_t * p_mutex);











 
uint32_t __svc(SD_NVIC_ENABLEIRQ) sd_nvic_EnableIRQ(IRQn_Type IRQn);










 
uint32_t __svc(SD_NVIC_DISABLEIRQ) sd_nvic_DisableIRQ(IRQn_Type IRQn);











 
uint32_t __svc(SD_NVIC_GETPENDINGIRQ) sd_nvic_GetPendingIRQ(IRQn_Type IRQn, uint32_t * p_pending_irq);










 
uint32_t __svc(SD_NVIC_SETPENDINGIRQ) sd_nvic_SetPendingIRQ(IRQn_Type IRQn);










 
uint32_t __svc(SD_NVIC_CLEARPENDINGIRQ) sd_nvic_ClearPendingIRQ(IRQn_Type IRQn);













 
uint32_t __svc(SD_NVIC_SETPRIORITY) sd_nvic_SetPriority(IRQn_Type IRQn, nrf_app_irq_priority_t priority);











 
uint32_t __svc(SD_NVIC_GETPRIORITY) sd_nvic_GetPriority(IRQn_Type IRQn, nrf_app_irq_priority_t * p_priority);





 
uint32_t __svc(SD_NVIC_SYSTEMRESET) sd_nvic_SystemReset(void);










 
uint32_t __svc(SD_NVIC_CRITICAL_REGION_ENTER) sd_nvic_critical_region_enter(uint8_t * p_is_nested_critical_region);









 
uint32_t __svc(SD_NVIC_CRITICAL_REGION_EXIT) sd_nvic_critical_region_exit(uint8_t is_nested_critical_region);






 
uint32_t __svc(SD_RAND_APPLICATION_POOL_CAPACITY) sd_rand_application_pool_capacity_get(uint8_t * p_pool_capacity);






 
uint32_t __svc(SD_RAND_APPLICATION_BYTES_AVAILABLE) sd_rand_application_bytes_available_get(uint8_t * p_bytes_available);








 
uint32_t __svc(SD_RAND_APPLICATION_GET_VECTOR) sd_rand_application_vector_get(uint8_t * p_buff, uint8_t length);






 
uint32_t __svc(SD_POWER_RESET_REASON_GET) sd_power_reset_reason_get(uint32_t * p_reset_reason);






 
uint32_t __svc(SD_POWER_RESET_REASON_CLR) sd_power_reset_reason_clr(uint32_t reset_reason_clr_msk);







 
uint32_t __svc(SD_POWER_MODE_SET) sd_power_mode_set(nrf_power_mode_t power_mode);




 
uint32_t __svc(SD_POWER_SYSTEM_OFF) sd_power_system_off(void);









 
uint32_t __svc(SD_POWER_POF_ENABLE) sd_power_pof_enable(uint8_t pof_enable);







 
uint32_t __svc(SD_POWER_POF_THRESHOLD_SET) sd_power_pof_threshold_set(nrf_power_failure_threshold_t threshold);






 
uint32_t __svc(SD_POWER_RAMON_SET) sd_power_ramon_set(uint32_t ramon);






 
uint32_t __svc(SD_POWER_RAMON_CLR) sd_power_ramon_clr(uint32_t ramon);






 
uint32_t __svc(SD_POWER_RAMON_GET) sd_power_ramon_get(uint32_t * p_ramon);






 
uint32_t __svc(SD_POWER_GPREGRET_SET) sd_power_gpregret_set(uint32_t gpregret_msk);






 
uint32_t __svc(SD_POWER_GPREGRET_CLR) sd_power_gpregret_clr(uint32_t gpregret_msk);






 
uint32_t __svc(SD_POWER_GPREGRET_GET) sd_power_gpregret_get(uint32_t *p_gpregret);









 
uint32_t __svc(SD_POWER_DCDC_MODE_SET) sd_power_dcdc_mode_set(nrf_power_dcdc_mode_t dcdc_mode);










 
uint32_t __svc(SD_CLOCK_HFCLK_REQUEST) sd_clock_hfclk_request(void);









 
uint32_t __svc(SD_CLOCK_HFCLK_RELEASE) sd_clock_hfclk_release(void);









 
uint32_t __svc(SD_CLOCK_HFCLK_IS_RUNNING) sd_clock_hfclk_is_running(uint32_t * p_is_running);























 
uint32_t __svc(SD_APP_EVT_WAIT) sd_app_evt_wait(void);






 
uint32_t __svc(SD_PPI_CHANNEL_ENABLE_GET) sd_ppi_channel_enable_get(uint32_t * p_channel_enable);






 
uint32_t __svc(SD_PPI_CHANNEL_ENABLE_SET) sd_ppi_channel_enable_set(uint32_t channel_enable_set_msk);






 
uint32_t __svc(SD_PPI_CHANNEL_ENABLE_CLR) sd_ppi_channel_enable_clr(uint32_t channel_enable_clr_msk);









 
uint32_t __svc(SD_PPI_CHANNEL_ASSIGN) sd_ppi_channel_assign(uint8_t channel_num, const volatile void * evt_endpoint, const volatile void * task_endpoint);







 
uint32_t __svc(SD_PPI_GROUP_TASK_ENABLE) sd_ppi_group_task_enable(uint8_t group_num);







 
uint32_t __svc(SD_PPI_GROUP_TASK_DISABLE) sd_ppi_group_task_disable(uint8_t group_num);








 
uint32_t __svc(SD_PPI_GROUP_ASSIGN) sd_ppi_group_assign(uint8_t group_num, uint32_t channel_msk);








 
uint32_t __svc(SD_PPI_GROUP_GET) sd_ppi_group_get(uint8_t group_num, uint32_t * p_channel_msk);



























 
uint32_t __svc(SD_RADIO_NOTIFICATION_CFG_SET) sd_radio_notification_cfg_set(nrf_radio_notification_type_t type, nrf_radio_notification_distance_t distance);









 
uint32_t __svc(SD_ECB_BLOCK_ENCRYPT) sd_ecb_block_encrypt(nrf_ecb_hal_data_t * p_ecb_data);









 
uint32_t __svc(SD_EVT_GET) sd_evt_get(uint32_t * p_evt_id);











 
uint32_t __svc(SD_TEMP_GET) sd_temp_get(int32_t * p_temp);






























 
uint32_t __svc(SD_FLASH_WRITE) sd_flash_write(uint32_t * const p_dst, uint32_t const * const p_src, uint32_t size);



























 
uint32_t __svc(SD_FLASH_PAGE_ERASE) sd_flash_page_erase(uint32_t page_number);













 
uint32_t __svc(SD_FLASH_PROTECT) sd_flash_protect(uint32_t protenset0, uint32_t protenset1);




















 
 uint32_t __svc(SD_RADIO_SESSION_OPEN) sd_radio_session_open(nrf_radio_signal_callback_t p_radio_signal_callback);











 
 uint32_t __svc(SD_RADIO_SESSION_CLOSE) sd_radio_session_close(void);































 
 uint32_t __svc(SD_RADIO_REQUEST) sd_radio_request(nrf_radio_request_t * p_request );

 



 
#line 30 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"










 
 







 




#line 26 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"
#line 27 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"
#line 28 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"






 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);




 
#line 54 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"



     
#line 67 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"
    



 
#line 81 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_error.h"



 
#line 31 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"

 
typedef enum
{



    APP_IRQ_PRIORITY_HIGH    = 1,



    APP_IRQ_PRIORITY_LOW     = 3
} app_irq_priority_t;



 

 



void critical_region_enter (void);
void critical_region_exit (void);






 
#line 82 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"






 
#line 105 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util_platform.h"
       






 
static __inline uint8_t current_int_priority_get(void)
{
    uint32_t isr_vector_num = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICSR & (0x1FFUL << 0));
    if (isr_vector_num > 0)
    {
        int32_t irq_type = ((int32_t)isr_vector_num - 16);
        return (NVIC_GetPriority((IRQn_Type)irq_type) & 0xFF);
    }
    else
    {
        return 4;
    }
}



 
#line 28 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart.h"




 
typedef enum
{
    APP_UART_FLOW_CONTROL_DISABLED,  
    APP_UART_FLOW_CONTROL_ENABLED,   
    APP_UART_FLOW_CONTROL_LOW_POWER  
} app_uart_flow_control_t;


 
typedef struct
{
    uint8_t                 rx_pin_no;     
    uint8_t                 tx_pin_no;     
    uint8_t                 rts_pin_no;    
    uint8_t                 cts_pin_no;    
    app_uart_flow_control_t flow_control;  
    _Bool                    use_parity;    
    uint32_t                baud_rate;     
} app_uart_comm_params_t;


 
typedef struct
{
    uint8_t * rx_buf;       
    uint32_t  rx_buf_size;  
    uint8_t * tx_buf;       
    uint32_t  tx_buf_size;  
} app_uart_buffers_t;











 
typedef enum
{
    APP_UART_DISCONNECTED,  
    APP_UART_CONNECTED      
} app_uart_connection_state_t;





 
typedef enum
{
    APP_UART_DATA_READY,           
    APP_UART_FIFO_ERROR,           
    APP_UART_COMMUNICATION_ERROR,  
    APP_UART_TX_EMPTY,             
    APP_UART_DATA,                 
} app_uart_evt_type_t;





 
typedef struct
{
    app_uart_evt_type_t evt_type;  
    union
    {
        uint32_t error_communication;  
        uint32_t error_code;           
        uint8_t  value;                
    } data;
} app_uart_evt_t;







 
typedef void (* app_uart_event_handler_t) (app_uart_evt_t * p_app_uart_event);















 
#line 148 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart.h"












 
#line 167 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart.h"




































 
uint32_t app_uart_init(const app_uart_comm_params_t * p_comm_params,
                       app_uart_buffers_t *           p_buffers,
                       app_uart_event_handler_t       error_handler,
                       app_irq_priority_t             irq_priority,
                       uint16_t *                     p_uart_uid);











 
uint32_t app_uart_get(uint8_t * p_byte);











 
uint32_t app_uart_put(uint8_t byte);






















 
uint32_t app_uart_get_connection_state(app_uart_connection_state_t * p_connection_state);





 
uint32_t app_uart_flush(void);












 
uint32_t app_uart_close(uint16_t app_uart_id);




 
#line 14 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\fifo\\app_fifo.h"










 








 




#line 26 "..\\..\\..\\..\\..\\..\\components\\libraries\\fifo\\app_fifo.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
 
 
 




 
 



 







 




  
 










  


 








#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 86 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 107 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   



 

   




 
#line 126 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 429 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 517 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 546 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(__int64  , __int64  );
   











 
#line 627 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 885 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


 

#line 27 "..\\..\\..\\..\\..\\..\\components\\libraries\\fifo\\app_fifo.h"




 
typedef struct
{
    uint8_t *          p_buf;            
    uint16_t           buf_size_mask;    
    volatile uint32_t  read_pos;         
    volatile uint32_t  write_pos;        
} app_fifo_t;










 
uint32_t app_fifo_init(app_fifo_t * p_fifo, uint8_t * p_buf, uint16_t buf_size);








 
uint32_t app_fifo_put(app_fifo_t * p_fifo, uint8_t byte);








 
uint32_t app_fifo_get(app_fifo_t * p_fifo, uint8_t * p_byte);






 
uint32_t app_fifo_flush(app_fifo_t * p_fifo);



 
#line 15 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf.h"




























 




#line 41 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf.h"

     
#line 45 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf.h"
#line 46 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_deprecated.h"




























 




 




 

 
 


 
 
 


 
 
 
 




 
 
 
 




 




 




 





 
 
 

 




 




 






 
 




 


 




 




 




 
 
#line 137 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_deprecated.h"
 
#line 170 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_deprecated.h"
 




 
#line 432 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf51_deprecated.h"



 



#line 47 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf.h"




#line 52 "C:\\Keil_v5\\ARM\\PACK\\NordicSemiconductor\\nRF_DeviceFamilyPack\\7.2.1\\Device\\Include\\nrf.h"





#line 16 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"










 












 

#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpiote.h"










 



#line 16 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpiote.h"
#line 17 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpiote.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stddef.h"

typedef signed int ptrdiff_t;


  
  typedef unsigned int size_t;








   



      typedef unsigned short wchar_t;  
#line 70 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stddef.h"



    

   









#line 91 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stddef.h"



 

#line 18 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpiote.h"
#line 19 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpiote.h"






 



 


 
typedef enum
{
  NRF_GPIOTE_POLARITY_LOTOHI = (0x01UL),       
  NRF_GPIOTE_POLARITY_HITOLO = (0x02UL),       
  NRF_GPIOTE_POLARITY_TOGGLE = (0x03UL)        
} nrf_gpiote_polarity_t;


 


 
typedef enum
{
  NRF_GPIOTE_INITIAL_VALUE_LOW  = (0UL),       
  NRF_GPIOTE_INITIAL_VALUE_HIGH = (1UL)       
} nrf_gpiote_outinit_t;



 
typedef enum  
{
    NRF_GPIOTE_TASKS_OUT_0     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->TASKS_OUT[0]))),  
    NRF_GPIOTE_TASKS_OUT_1     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->TASKS_OUT[1]))),  
    NRF_GPIOTE_TASKS_OUT_2     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->TASKS_OUT[2]))),  
    NRF_GPIOTE_TASKS_OUT_3     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->TASKS_OUT[3]))),  
     
} nrf_gpiote_tasks_t;



 
typedef enum  
{
    NRF_GPIOTE_EVENTS_IN_0     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->EVENTS_IN[0]))),  
    NRF_GPIOTE_EVENTS_IN_1     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->EVENTS_IN[1]))),  
    NRF_GPIOTE_EVENTS_IN_2     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->EVENTS_IN[2]))),  
    NRF_GPIOTE_EVENTS_IN_3     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->EVENTS_IN[3]))),  
    NRF_GPIOTE_EVENTS_PORT     = (( size_t)__INTADDR__(&(((NRF_GPIOTE_Type *)0)->EVENTS_PORT))),  
     
} nrf_gpiote_events_t;




 
typedef enum
{
    NRF_GPIOTE_INT_IN0_MASK  = (0x1UL << (0UL)),   
    NRF_GPIOTE_INT_IN1_MASK  = (0x1UL << (1UL)),   
    NRF_GPIOTE_INT_IN2_MASK  = (0x1UL << (2UL)),   
    NRF_GPIOTE_INT_IN3_MASK  = (0x1UL << (3UL)),   
    NRF_GPIOTE_INT_PORT_MASK = (int)(0x1UL << (31UL)),  
} nrf_gpiote_int_t;







 
static __inline void nrf_gpiote_task_set(nrf_gpiote_tasks_t task);







 
static __inline uint32_t nrf_gpiote_task_addr_get(nrf_gpiote_tasks_t task);





 
static __inline _Bool nrf_gpiote_event_is_set(nrf_gpiote_events_t event);





 
static __inline void nrf_gpiote_event_clear(nrf_gpiote_events_t event);







 
static __inline uint32_t nrf_gpiote_event_addr_get(nrf_gpiote_events_t event);




 
static __inline void nrf_gpiote_int_enable(uint32_t mask);




 
static __inline void nrf_gpiote_int_disable(uint32_t mask);






 
static __inline uint32_t nrf_gpiote_int_is_enabled(uint32_t mask);




 
static __inline void nrf_gpiote_event_enable(uint32_t idx);




 
static __inline void nrf_gpiote_event_disable(uint32_t idx);






 
static __inline void nrf_gpiote_event_configure(uint32_t idx, uint32_t pin,
                                                nrf_gpiote_polarity_t polarity);






 
static __inline uint32_t nrf_gpiote_event_pin_get(uint32_t idx);






 
static __inline nrf_gpiote_polarity_t nrf_gpiote_event_polarity_get(uint32_t idx);




 
static __inline void nrf_gpiote_task_enable(uint32_t idx);




 
static __inline void nrf_gpiote_task_disable(uint32_t idx);







 
static __inline void nrf_gpiote_task_configure(uint32_t idx, uint32_t pin,
                                               nrf_gpiote_polarity_t polarity,
                                               nrf_gpiote_outinit_t  init_val);





 
static __inline void nrf_gpiote_task_force(uint32_t idx, nrf_gpiote_outinit_t init_val);




 
static __inline void nrf_gpiote_te_default(uint32_t idx);


static __inline void nrf_gpiote_task_set(nrf_gpiote_tasks_t task)
{
    *(volatile uint32_t *)((uint32_t)((NRF_GPIOTE_Type *) 0x40006000UL) + task) = 0x1UL;
}

static __inline uint32_t nrf_gpiote_task_addr_get(nrf_gpiote_tasks_t task)
{
    return ((uint32_t)((NRF_GPIOTE_Type *) 0x40006000UL) + task);
}

static __inline _Bool nrf_gpiote_event_is_set(nrf_gpiote_events_t event)
{
    return (*(uint32_t *)nrf_gpiote_event_addr_get(event) == 0x1UL) ? 1 : 0;
}

static __inline void nrf_gpiote_event_clear(nrf_gpiote_events_t event)
{
    *(uint32_t *)nrf_gpiote_event_addr_get(event) = 0;
}

static __inline uint32_t nrf_gpiote_event_addr_get(nrf_gpiote_events_t event)
{
    return ((uint32_t)((NRF_GPIOTE_Type *) 0x40006000UL) + event);
}

static __inline void nrf_gpiote_int_enable(uint32_t mask)
{
    ((NRF_GPIOTE_Type *) 0x40006000UL)->INTENSET = mask;
}

static __inline void nrf_gpiote_int_disable(uint32_t mask)
{
    ((NRF_GPIOTE_Type *) 0x40006000UL)->INTENCLR = mask;
}

static __inline uint32_t nrf_gpiote_int_is_enabled(uint32_t mask)
{
    return (((NRF_GPIOTE_Type *) 0x40006000UL)->INTENSET & mask);
}

static __inline void nrf_gpiote_event_enable(uint32_t idx)
{
   ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] |= (0x01UL);
}

static __inline void nrf_gpiote_event_disable(uint32_t idx)
{
   ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] &= ~(0x01UL);
}

static __inline void nrf_gpiote_event_configure(uint32_t idx, uint32_t pin, nrf_gpiote_polarity_t polarity)
{
  ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] &= ~((0x1FUL << (8UL)) | (0x3UL << (16UL)));
  ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] |= ((pin << (8UL)) & (0x1FUL << (8UL))) |
                              ((polarity << (16UL)) & (0x3UL << (16UL)));
}

static __inline uint32_t nrf_gpiote_event_pin_get(uint32_t idx)
{
    return ((((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] & (0x1FUL << (8UL))) >> (8UL));
}

static __inline nrf_gpiote_polarity_t nrf_gpiote_event_polarity_get(uint32_t idx)
{
    return (nrf_gpiote_polarity_t)((((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] & (0x3UL << (16UL))) >> (16UL));
}

static __inline void nrf_gpiote_task_enable(uint32_t idx)
{
    uint32_t final_config = ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] | (0x03UL);
    

 
     
    ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] = final_config | ((31 << (8UL)) & (0x1FUL << (8UL)));
    __nop();
    __nop();
    __nop();
    ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] = final_config;
}

static __inline void nrf_gpiote_task_disable(uint32_t idx)
{
    ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] &= ~(0x03UL);
}

static __inline void nrf_gpiote_task_configure(uint32_t idx, uint32_t pin,
                                                nrf_gpiote_polarity_t polarity,
                                                nrf_gpiote_outinit_t  init_val)
{
  ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] &= ~((0x1FUL << (8UL)) |
                               (0x3UL << (16UL)) |
                               (0x1UL << (20UL)));

  ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] |= ((pin << (8UL)) & (0x1FUL << (8UL))) |
                             ((polarity << (16UL)) & (0x3UL << (16UL))) |
                             ((init_val << (20UL)) & (0x1UL << (20UL)));
}

static __inline void nrf_gpiote_task_force(uint32_t idx, nrf_gpiote_outinit_t init_val)
{
  ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] &= ~(0x1UL << (20UL));
  ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] |= (init_val << (20UL)) & (0x1UL << (20UL));
}

static __inline void nrf_gpiote_te_default(uint32_t idx)
{
    ((NRF_GPIOTE_Type *) 0x40006000UL)->CONFIG[idx] = 0;
}

 

#line 27 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"



#line 5 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"
#line 6 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"
#line 7 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\hal\\nrf_gpio.h"












 






 
typedef enum
{
    NRF_GPIO_PORT_DIR_OUTPUT,       
    NRF_GPIO_PORT_DIR_INPUT         
} nrf_gpio_port_dir_t;




 
typedef enum
{
    NRF_GPIO_PIN_DIR_INPUT,   
    NRF_GPIO_PIN_DIR_OUTPUT   
} nrf_gpio_pin_dir_t;




 
typedef enum
{
    NRF_GPIO_PORT_SELECT_PORT0 = 0,           
    NRF_GPIO_PORT_SELECT_PORT1,               
    NRF_GPIO_PORT_SELECT_PORT2,               
    NRF_GPIO_PORT_SELECT_PORT3,               
} nrf_gpio_port_select_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOPULL   = (0x00UL),                 
    NRF_GPIO_PIN_PULLDOWN = (0x01UL),                 
    NRF_GPIO_PIN_PULLUP   = (0x03UL),                   
} nrf_gpio_pin_pull_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOSENSE    = (0x00UL),              
    NRF_GPIO_PIN_SENSE_LOW  = (0x03UL),                   
    NRF_GPIO_PIN_SENSE_HIGH = (0x02UL),                  
} nrf_gpio_pin_sense_t;











 
static __inline void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end);













 
static __inline void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config);








 
static __inline void nrf_gpio_cfg_output(uint32_t pin_number);









 
static __inline void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config);





 
static __inline void nrf_gpio_cfg_default(uint32_t pin_number);






 
static __inline void nrf_gpio_cfg_watcher(uint32_t pin_number);






 
static __inline void nrf_gpio_input_disconnect(uint32_t pin_number);









 
static __inline void nrf_gpio_cfg_sense_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config, nrf_gpio_pin_sense_t sense_config);







 
static __inline void nrf_gpio_cfg_sense_set(uint32_t pin_number, nrf_gpio_pin_sense_t sense_config);








 
static __inline void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction);









 
static __inline void nrf_gpio_pin_set(uint32_t pin_number);









 
static __inline void nrf_gpio_pin_clear(uint32_t pin_number);









 
static __inline void nrf_gpio_pin_toggle(uint32_t pin_number);













 
static __inline void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value);














 
static __inline uint32_t nrf_gpio_pin_read(uint32_t pin_number);








 
static __inline uint32_t nrf_gpio_pins_read(void);








 
static __inline nrf_gpio_pin_sense_t nrf_gpio_pin_sense_get(uint32_t pin_number);














 
static __inline void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value);













 
static __inline uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no);







 
static __inline void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir);







 
static __inline uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port);









 
static __inline void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value);











 
static __inline void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask);











 
static __inline void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask);


static __inline void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | ((0x00UL) << (2UL))
                                        | ((1UL) << (1UL))
                                        | ((1UL) << (0UL));
    }
}

static __inline void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
    }
}

static __inline void nrf_gpio_cfg_output(uint32_t pin_number)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                            | ((0x00UL) << (8UL))
                                            | ((0x00UL) << (2UL))
                                            | ((1UL) << (1UL))
                                            | ((1UL) << (0UL));
}

static __inline void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}

static __inline void nrf_gpio_cfg_default(uint32_t pin_number)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (NRF_GPIO_PIN_NOPULL << (2UL))
                                        | ((1UL) << (1UL))
                                        | ((0UL) << (0UL));
}

static __inline void nrf_gpio_cfg_watcher(uint32_t pin_number)
{
     
    uint32_t cnf = ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] & ~(0x1UL << (1UL));
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = cnf | ((0UL) << (1UL));
}

static __inline void nrf_gpio_input_disconnect(uint32_t pin_number)
{
     
    uint32_t cnf = ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] & ~(0x1UL << (1UL));
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = cnf | ((1UL) << (1UL));
}

static __inline void nrf_gpio_cfg_sense_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config, nrf_gpio_pin_sense_t sense_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = (sense_config << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}

static __inline void nrf_gpio_cfg_sense_set(uint32_t pin_number, nrf_gpio_pin_sense_t sense_config)
{
     
    
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] &= ~(0x3UL << (16UL));
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] |= (sense_config << (16UL));
}

static __inline void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction)
{
    if(direction == NRF_GPIO_PIN_DIR_INPUT)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] =
          ((0x00UL) << (16UL))
        | ((0x00UL) << (8UL))
        | ((0x00UL) << (2UL))
        | ((0UL) << (1UL))
        | ((0UL) << (0UL));
    }
    else
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->DIRSET = (1UL << pin_number);
    }
}

static __inline void nrf_gpio_pin_set(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = (1UL << pin_number);
}

static __inline void nrf_gpio_pin_clear(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = (1UL << pin_number);
}

static __inline void nrf_gpio_pin_toggle(uint32_t pin_number)
{
    const uint32_t pin_bit   = 1UL << pin_number;
    const uint32_t pin_state = ((((NRF_GPIO_Type *) 0x50000000UL)->OUT >> pin_number) & 1UL);

    if (pin_state == 0)
    {
        
        ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = pin_bit;
    }
    else
    {
        
        ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = pin_bit;
    }
}

static __inline void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value)
{
    if (value == 0)
    {
        nrf_gpio_pin_clear(pin_number);
    }
    else
    {
        nrf_gpio_pin_set(pin_number);
    }
}

static __inline uint32_t nrf_gpio_pin_read(uint32_t pin_number)
{
    return  ((((NRF_GPIO_Type *) 0x50000000UL)->IN >> pin_number) & 1UL);
}

static __inline uint32_t nrf_gpio_pins_read(void)
{
    return ((NRF_GPIO_Type *) 0x50000000UL)->IN;
}

static __inline nrf_gpio_pin_sense_t nrf_gpio_pin_sense_get(uint32_t pin_number)
{
    return (nrf_gpio_pin_sense_t)((((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] & (0x3UL << (16UL))) >> (16UL));
}

static __inline void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value)
{
    *((volatile uint8_t*)(word_address) + byte_no) = value;
}

static __inline uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no)
{
    return (*((const volatile uint8_t*)(word_address) + byte_no));
}

static __inline void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir)
{
    if (dir == NRF_GPIO_PORT_DIR_OUTPUT)
    {
        nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->DIRSET, port, 0xFF);
    }
    else
    {
        nrf_gpio_range_cfg_input(port*8, (port+1)*8-1, NRF_GPIO_PIN_NOPULL);
    }
}

static __inline uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port)
{
    return nrf_gpio_word_byte_read(&((NRF_GPIO_Type *) 0x50000000UL)->IN, port);
}

static __inline void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUT, port, value);
}

static __inline void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTSET, port, set_mask);
}

static __inline void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR, port, clr_mask);
}

 

#line 28 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"
#line 1 "..\\include\\nrf_drv_config.h"










 




 





 








 


#line 42 "..\\include\\nrf_drv_config.h"



#line 53 "..\\include\\nrf_drv_config.h"
 


#line 64 "..\\include\\nrf_drv_config.h"



 


#line 77 "..\\include\\nrf_drv_config.h"



#line 87 "..\\include\\nrf_drv_config.h"





 









 


#line 117 "..\\include\\nrf_drv_config.h"

 


#line 127 "..\\include\\nrf_drv_config.h"

 








#line 29 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\sdk_errors.h"










 




























 




#line 46 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\sdk_errors.h"




 


 




 


 





 


 
 




 
#line 85 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\sdk_errors.h"
 





 




 











 
typedef uint32_t ret_code_t;
 
 



#line 30 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"
#line 31 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"
#line 32 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"

 
typedef struct
{
    nrf_gpiote_polarity_t sense;       
    nrf_gpio_pin_pull_t   pull;        
    _Bool                  is_watcher;  
    _Bool                  hi_accuracy; 
} nrf_drv_gpiote_in_config_t;


 
#line 51 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"


 
#line 61 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"


 
#line 71 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"

 
typedef struct
{
    nrf_gpiote_polarity_t action;     
    nrf_gpiote_outinit_t  init_state;  
    _Bool                  task_pin;   
} nrf_drv_gpiote_out_config_t;

 







 
#line 95 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"


 
#line 104 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"


 
#line 113 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\gpiote\\nrf_drv_gpiote.h"

 
typedef uint32_t nrf_drv_gpiote_pin_t;





 
typedef void (*nrf_drv_gpiote_evt_handler_t)(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);









 
ret_code_t nrf_drv_gpiote_init(void);









 
_Bool nrf_drv_gpiote_is_init(void);



 
void nrf_drv_gpiote_uninit(void);














 
ret_code_t nrf_drv_gpiote_out_init(nrf_drv_gpiote_pin_t pin, nrf_drv_gpiote_out_config_t * p_config);






 
void nrf_drv_gpiote_out_uninit(nrf_drv_gpiote_pin_t pin);





 
void nrf_drv_gpiote_out_set(nrf_drv_gpiote_pin_t pin);





 
void nrf_drv_gpiote_out_clear(nrf_drv_gpiote_pin_t pin);





 
void nrf_drv_gpiote_out_toggle(nrf_drv_gpiote_pin_t pin);





 
void nrf_drv_gpiote_out_task_enable(nrf_drv_gpiote_pin_t pin);





 
void nrf_drv_gpiote_out_task_disable(nrf_drv_gpiote_pin_t pin);





 
uint32_t nrf_drv_gpiote_out_task_addr_get(nrf_drv_gpiote_pin_t pin);



















 
ret_code_t nrf_drv_gpiote_in_init(nrf_drv_gpiote_pin_t pin, nrf_drv_gpiote_in_config_t * p_config, nrf_drv_gpiote_evt_handler_t evt_handler);






 
void nrf_drv_gpiote_in_uninit(nrf_drv_gpiote_pin_t pin);











 
void nrf_drv_gpiote_in_event_enable(nrf_drv_gpiote_pin_t pin, _Bool int_enable);





 
void nrf_drv_gpiote_in_event_disable(nrf_drv_gpiote_pin_t pin);







 
_Bool nrf_drv_gpiote_in_is_set(nrf_drv_gpiote_pin_t pin);






 
uint32_t nrf_drv_gpiote_in_event_addr_get(nrf_drv_gpiote_pin_t pin);






 
void nrf_drv_gpiote_out_task_force(nrf_drv_gpiote_pin_t pin, uint8_t state);





 
void nrf_drv_gpiote_out_task_trigger(nrf_drv_gpiote_pin_t pin);



 

#line 17 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"
#line 18 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"
#line 19 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"
#line 1 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"










 








 




#line 26 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"
#line 27 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"
#line 28 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"

enum
{
    UNIT_0_625_MS = 625,                                 
    UNIT_1_25_MS  = 1250,                                
    UNIT_10_MS    = 10000                                
};














 

#line 58 "..\\..\\..\\..\\..\\..\\components\\libraries\\util\\app_util.h"


 
typedef uint8_t uint16_le_t[2];

 
typedef uint8_t uint32_le_t[4];

 
typedef struct
{
    uint16_t  size;                  
    uint8_t * p_data;                
} uint8_array_t;
    






 








 





 












 











 
static __inline uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}
    






 
static __inline uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}






 
static __inline uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) | 
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}






 
static __inline uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}
    



















 
static __inline uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}






 
static __inline _Bool is_word_aligned(void * p)
{
    return (((uintptr_t)p & 0x03) == 0);
}



 
#line 20 "..\\..\\..\\..\\..\\..\\components\\drivers_nrf\\uart\\app_uart_fifo.c"

static __inline uint32_t fifo_length(app_fifo_t * const fifo)
{
  uint32_t tmp = fifo->read_pos;
  return fifo->write_pos - tmp;
}







 
typedef enum
{
    UART_OFF,                                                                
    UART_READY,                                                              
    UART_ON,                                                                 
    UART_WAIT_CLOSE,                                                         
} app_uart_states_t;

 
typedef enum
{
    ON_CTS_HIGH,                                                             
    ON_CTS_LOW,                                                              
    ON_UART_PUT,                                                             
    ON_TX_READY,                                                             
    ON_UART_CLOSE,                                                           
} app_uart_state_event_t;

static app_fifo_t                  m_rx_fifo;                                
static app_fifo_t                  m_tx_fifo;                                

static uint8_t                     m_instance_counter = 1;                         
static app_uart_event_handler_t    m_event_handler;                          
static volatile app_uart_states_t  m_current_state = UART_OFF;               


 
static void action_uart_deactivate(void)
{
    m_current_state         = UART_OFF;
    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STOPTX = 1;
    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STOPRX = 1;
    ((NRF_UART_Type *) 0x40002000UL)->ENABLE       = ((0x00UL) << (0UL));
}


void action_tx_stop()
{
    app_uart_evt_t app_uart_event;
    
    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STOPTX = 1;
    m_current_state         = UART_READY;
    
    
    app_uart_event.evt_type = APP_UART_TX_EMPTY;
    m_event_handler(&app_uart_event);
}




 
static void action_tx_send()
{
    uint8_t tx_byte;

    if (m_current_state != UART_ON)
    {
        
        ((NRF_UART_Type *) 0x40002000UL)->TASKS_STARTTX = 1;
    }

    if (app_fifo_get(&m_tx_fifo, &tx_byte) != ((0x0) + 0))
    {
        action_tx_stop();
        return;
    }

    ((NRF_UART_Type *) 0x40002000UL)->INTENCLR = ((1UL) << (7UL));
    ((NRF_UART_Type *) 0x40002000UL)->TXD      = tx_byte;
    m_current_state     = UART_ON;
    ((NRF_UART_Type *) 0x40002000UL)->INTENSET = ((1UL) << (7UL));
}


static void action_tx_ready()
{
    
    if (fifo_length(&m_tx_fifo) != 0)
    {
        action_tx_send();
    }
    else
    {
        action_tx_stop();
    }
}



 
static void on_cts_high(void)
{
    switch (m_current_state)
    {
        case UART_READY:
            action_uart_deactivate();
            break;

        case UART_ON:
            m_current_state = UART_WAIT_CLOSE;
            break;

        default:
            
            break;
    }
}



 
static void on_cts_low(void)
{
    switch (m_current_state)
    {
        case UART_OFF:
            ((NRF_UART_Type *) 0x40002000UL)->ENABLE        = ((0x04UL) << (0UL));
            ((NRF_UART_Type *) 0x40002000UL)->TASKS_STARTRX = 1;

            if (fifo_length(&m_tx_fifo) != 0)
            {
                action_tx_send();
            }
            else
            {
                m_current_state = UART_READY;
            }
            break;

        case UART_WAIT_CLOSE:
            m_current_state = UART_ON;
            break;

        default:
            
            break;
    }
}



 
static void on_tx_ready(void)
{
    switch (m_current_state)
    {
        case UART_WAIT_CLOSE:
            action_uart_deactivate();
            break;

        case UART_ON:
        case UART_READY:
            action_tx_ready();
            break;

        default:
            
            break;
    }
}



 
static void on_uart_put(void)
{
    if (m_current_state == UART_READY)
    {
        action_tx_send();
    }
}



 
static void on_uart_close(void)
{
    action_uart_deactivate();
}





 
static void on_uart_event(app_uart_state_event_t event)
{
    switch (event)
    {
        case ON_CTS_HIGH:
            on_cts_high();
            break;

        case ON_CTS_LOW:
            on_cts_low();
            break;

        case ON_TX_READY:
            on_tx_ready();
            break;

        case ON_UART_PUT:
            on_uart_put();
            break;

        case ON_UART_CLOSE:
            on_uart_close();
            break;

        default:
            
            break;
    }
}



 
static void gpiote_uart_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (nrf_drv_gpiote_in_is_set(pin))
    {
        on_uart_event(ON_CTS_HIGH);
    }
    else
    {
        on_uart_event(ON_CTS_LOW);
    }
}






 
void UART0_IRQHandler(void)
{
    
    if (((NRF_UART_Type *) 0x40002000UL)->EVENTS_RXDRDY != 0)
    {
        uint32_t err_code;

        
        ((NRF_UART_Type *) 0x40002000UL)->EVENTS_RXDRDY = 0;

        
        err_code = app_fifo_put(&m_rx_fifo, (uint8_t)((NRF_UART_Type *) 0x40002000UL)->RXD);
        if (err_code != ((0x0) + 0))
        {
            app_uart_evt_t app_uart_event;
            app_uart_event.evt_type          = APP_UART_FIFO_ERROR;
            app_uart_event.data.error_code   = err_code;
            m_event_handler(&app_uart_event);
        }
        
        else if (fifo_length(&m_rx_fifo) == 1)
        {
            app_uart_evt_t app_uart_event;
            app_uart_event.evt_type = APP_UART_DATA_READY;
            m_event_handler(&app_uart_event);
        }
        else
        {
            
        }
    }

    
    if (((NRF_UART_Type *) 0x40002000UL)->EVENTS_TXDRDY != 0)
    {
        
        ((NRF_UART_Type *) 0x40002000UL)->EVENTS_TXDRDY = 0;
        on_uart_event(ON_TX_READY);
    }

    
    if (((NRF_UART_Type *) 0x40002000UL)->EVENTS_ERROR != 0)
    {
        uint32_t       error_source;
        app_uart_evt_t app_uart_event;

        
        ((NRF_UART_Type *) 0x40002000UL)->EVENTS_ERROR = 0;

        
        error_source        = ((NRF_UART_Type *) 0x40002000UL)->ERRORSRC;
        ((NRF_UART_Type *) 0x40002000UL)->ERRORSRC = error_source;

        app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
        app_uart_event.data.error_communication = error_source;

        m_event_handler(&app_uart_event);
    }
}



 
static void uart_no_flow_control_init(void)
{
    ((NRF_UART_Type *) 0x40002000UL)->ENABLE        = ((0x04UL) << (0UL));
    ((NRF_UART_Type *) 0x40002000UL)->EVENTS_RXDRDY = 0;
    ((NRF_UART_Type *) 0x40002000UL)->EVENTS_TXDRDY = 0;

    ((NRF_UART_Type *) 0x40002000UL)->CONFIG       &= ~((1UL) << (0UL));

    ((NRF_UART_Type *) 0x40002000UL)->PSELRTS       = 0xFFFFFFFF;
    ((NRF_UART_Type *) 0x40002000UL)->PSELCTS       = 0xFFFFFFFF;

    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STARTTX = 1;
    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STARTRX = 1;
}



 
static void uart_standard_flow_control_init(const app_uart_comm_params_t * p_comm_params)
{
    ((NRF_UART_Type *) 0x40002000UL)->ENABLE        = ((0x04UL) << (0UL));
    ((NRF_UART_Type *) 0x40002000UL)->EVENTS_RXDRDY = 0;
    ((NRF_UART_Type *) 0x40002000UL)->EVENTS_TXDRDY = 0;

    ((NRF_UART_Type *) 0x40002000UL)->CONFIG       |= ((1UL) << (0UL));

    ((NRF_UART_Type *) 0x40002000UL)->PSELCTS       = p_comm_params->cts_pin_no;
    ((NRF_UART_Type *) 0x40002000UL)->PSELRTS       = p_comm_params->rts_pin_no;

    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STARTTX = 1;
    ((NRF_UART_Type *) 0x40002000UL)->TASKS_STARTRX = 1;
}


uint32_t app_uart_init(const app_uart_comm_params_t * p_comm_params,
                             app_uart_buffers_t *     p_buffers,
                             app_uart_event_handler_t event_handler,
                             app_irq_priority_t       irq_priority,
                             uint16_t *               p_app_uart_uid)
{
    uint32_t err_code;

    m_current_state = UART_OFF;
    m_event_handler = event_handler;

    if (p_buffers == 0)
    {
        return ((0x0) + 7);
    }

    
    err_code = app_fifo_init(&m_rx_fifo, p_buffers->rx_buf, p_buffers->rx_buf_size);
    if (err_code != ((0x0) + 0))
    {
        
        return err_code;
    }

    
    err_code = app_fifo_init(&m_tx_fifo, p_buffers->tx_buf, p_buffers->tx_buf_size);
    if (err_code != ((0x0) + 0))
    {
        
        return err_code;
    }

    
    nrf_gpio_cfg_output(p_comm_params->tx_pin_no);
    nrf_gpio_cfg_input(p_comm_params->rx_pin_no, NRF_GPIO_PIN_NOPULL);

    ((NRF_UART_Type *) 0x40002000UL)->PSELTXD = p_comm_params->tx_pin_no;
    ((NRF_UART_Type *) 0x40002000UL)->PSELRXD = p_comm_params->rx_pin_no;

    
    ((NRF_UART_Type *) 0x40002000UL)->BAUDRATE = (p_comm_params->baud_rate << (0UL));
    if (p_comm_params->use_parity)
    {
        ((NRF_UART_Type *) 0x40002000UL)->CONFIG = ((7UL) << (1UL));
    }
    else
    {
        ((NRF_UART_Type *) 0x40002000UL)->CONFIG = ((0UL) << (1UL));
    }

    if (p_comm_params->flow_control == APP_UART_FLOW_CONTROL_LOW_POWER)
    {
        if (!nrf_drv_gpiote_is_init())
        {
            err_code = nrf_drv_gpiote_init();
            if (err_code != ((0x0) + 0))
            {
                return err_code;
            }
        }

        
        nrf_drv_gpiote_out_config_t rts_config = { . init_state = 1 ? NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW, . task_pin = 0, };
        err_code = nrf_drv_gpiote_out_init(p_comm_params->rts_pin_no, &rts_config);
        if (err_code != ((0x0) + 0))
        {
            return err_code;
        }

        ((NRF_UART_Type *) 0x40002000UL)->PSELCTS = 0xFFFFFFFF;
        ((NRF_UART_Type *) 0x40002000UL)->PSELRTS = p_comm_params->rts_pin_no;
        ((NRF_UART_Type *) 0x40002000UL)->CONFIG |= ((1UL) << (0UL));

        
        
        
        
        nrf_drv_gpiote_in_config_t cts_config = { . is_watcher = 0, . hi_accuracy = 0, . pull = NRF_GPIO_PIN_NOPULL, . sense = NRF_GPIOTE_POLARITY_TOGGLE, };
        err_code = nrf_drv_gpiote_in_init(p_comm_params->cts_pin_no, &cts_config, gpiote_uart_event_handler);
        if (err_code != ((0x0) + 0))
        {
            return err_code;
        }

        nrf_drv_gpiote_in_event_enable(p_comm_params->cts_pin_no, 1);

        
        if (nrf_drv_gpiote_in_is_set(p_comm_params->cts_pin_no))
        {
            on_uart_event(ON_CTS_HIGH);
        }
        else
        {
            on_uart_event(ON_CTS_LOW);
        }

    }
    else if (p_comm_params->flow_control == APP_UART_FLOW_CONTROL_ENABLED)
    {
        uart_standard_flow_control_init(p_comm_params);
        m_current_state = UART_READY;
    }
    else
    {
        uart_no_flow_control_init();
        m_current_state = UART_READY;
    }

    if (*p_app_uart_uid == 0x0000)
    {
        *p_app_uart_uid = m_instance_counter++;
    }

    
    ((NRF_UART_Type *) 0x40002000UL)->INTENCLR = 0xffffffffUL;
    ((NRF_UART_Type *) 0x40002000UL)->INTENSET = ((1UL) << (2UL)) |
                          ((1UL) << (7UL)) |
                          ((1UL) << (9UL));

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, irq_priority);
    NVIC_EnableIRQ(UART0_IRQn);

    return ((0x0) + 0);
}


uint32_t app_uart_get(uint8_t * p_byte)
{
    return app_fifo_get(&m_rx_fifo, p_byte);
}


uint32_t app_uart_put(uint8_t byte)
{
    uint32_t err_code;

    err_code = app_fifo_put(&m_tx_fifo, byte);

    on_uart_event(ON_UART_PUT);

    return err_code;
}


uint32_t app_uart_flush(void)
{
    uint32_t err_code;

    err_code = app_fifo_flush(&m_rx_fifo);
    if (err_code != ((0x0) + 0))
    {
        return err_code;
    }

    err_code = app_fifo_flush(&m_tx_fifo);
    if (err_code != ((0x0) + 0))
    {
        return err_code;
    }

    return ((0x0) + 0);
}


uint32_t app_uart_get_connection_state(app_uart_connection_state_t * p_conn_state)
{
    *p_conn_state = ((m_current_state == UART_OFF) ? APP_UART_DISCONNECTED : APP_UART_CONNECTED);

    return ((0x0) + 0);
}

uint32_t app_uart_close(uint16_t app_uart_uid)
{
    if (app_uart_uid < 0x00FF)
    {
        on_uart_event(ON_UART_CLOSE);
        return ((0x0) + 0);
    }

    on_uart_event(ON_UART_CLOSE);

    nrf_drv_gpiote_in_uninit(((NRF_UART_Type *) 0x40002000UL)->PSELCTS);
    return ((0x0) + 0);
}


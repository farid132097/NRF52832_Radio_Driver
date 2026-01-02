

#ifndef  _CLOCKS_H_
#define  _CLOCKS_H_


#include "nrf.h"


void     Clocks_Struct_Init(void);

//HFXTALCLK Related Functions
void     Clocks_HFCLK_Xtal_Start(void);
uint8_t  Clocks_HFCLK_Xtal_Started(void);
void     Clocks_HFCLK_Xtal_Wait_Until_Ready(void);
void     Clocks_HFCLK_Xtal_Stop(void);
uint8_t  Clocks_HFCLK_Xtal_Stopped(void);
void     Clocks_HFCLK_Xtal_Wait_Until_Stopped(void);
void     Clocks_HFCLK_Xtal_Start_Request(void);
void     Clocks_HFCLK_Xtal_Stop_Request(void);
uint8_t  Clocks_HFCLK_Xtal_Request_Count_Get(void);


//LFRCCLK Related Functions
void     Clocks_LFCLK_RC_Start(void);
void     Clocks_LFCLK_RC_Stop(void);
void     Clocks_LFCLK_RC_Start_Request(void);
void     Clocks_LFCLK_RC_Stop_Request(void);
uint8_t  Clocks_LFCLK_RC_Request_Count_Get(void);


//LFXTALCLK Related Functions
void     Clocks_LFCLK_Xtal_Start(void);
void     Clocks_LFCLK_Xtal_Stop(void);
void     Clocks_LFCLK_Xtal_Start_Request(void);
void     Clocks_LFCLK_Xtal_Stop_Request(void);
uint8_t  Clocks_LFCLK_Xtal_Request_Count_Get(void);

void     Clocks_Init(void);




#endif





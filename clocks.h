

#ifndef  _CLOCKS_H_
#define  _CLOCKS_H_


#include "nrf.h"


void     Clock_Struct_Init(void);

//HFXTALCLK Related Functions
void     Clock_HFCLK_Xtal_Start_Request(void);
uint8_t  Clock_HFCLK_Xtal_Started(void);
void     Clock_HFCLK_Xtal_Wait_Until_Ready(void);

void     Clock_HFCLK_Xtal_Stop_Request(void);
uint8_t  Clock_HFCLK_Xtal_Stopped(void);
void     Clock_HFCLK_Xtal_Wait_Until_Stopped(void);

void     Clock_HFCLK_Xtal_Request(void);
void     Clock_HFCLK_Xtal_Release(void);
uint8_t  Clock_HFCLK_Xtal_Request_Count_Get(void);


//LFRCCLK Related Functions
void     Clock_LFCLK_RC_Start(void);
void     Clock_LFCLK_RC_Stop(void);
void     Clock_LFCLK_RC_Request(void);
void     Clock_LFCLK_RC_Release(void);
uint8_t  Clock_LFCLK_RC_Request_Count_Get(void);


//LFXTALCLK Related Functions
void     Clock_LFCLK_Xtal_Start(void);
void     Clock_LFCLK_Xtal_Stop(void);
void     Clock_LFCLK_Xtal_Request(void);
void     Clock_LFCLK_Xtal_Release(void);
uint8_t  Clock_LFCLK_Xtal_Request_Count_Get(void);

void     Clock_Init(void);




#endif





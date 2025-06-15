

#ifndef  _CLOCKS_H_
#define  _CLOCKS_H_


#include "nrf.h"



void     Clock_Struct_Init(void);

void     Clock_HFCLK_Start(void);
void     Clock_HFCLK_Stop(void);
void     Clock_HFCLK_Request(void);
void     Clock_HFCLK_Release(void);
uint8_t  Clock_HFCLK_Request_Count_Get(void);

void     Clock_LFCLK_Start(void);
void     Clock_LFCLK_Stop(void);
void     Clock_LFCLK_Request(void);
void     Clock_LFCLK_Release(void);
uint8_t  Clock_LFCLK_Request_Count_Get(void);

void     Clock_Init(void);




#endif





#ifndef  _RTC_H_
#define  _RTC_H_
#include "nrf.h"

void     RTC_Struct_Init(void);
void     RTC_Clock_Init(void);
void     RTC_RTC0_Reg_Init(void);
void     RTC_RTC0_Disable(void);

void     RTC_RTC1_Set_Timeout(uint32_t val);
void     RTC_RTC1_Clear_Timeout(void);

uint8_t  RTC_RTC0_Event(void);
void     RTC_RTC0_Event_Clear(void);
uint8_t  RTC_RTC1_Event(void);
void     RTC_RTC1_Event_Clear(void);
uint16_t RTC_RTC1_Elapsed_Time_Get(void);
uint32_t RTC_UpTime_Get(void);

void     RTC0_IRQHandler(void);
void     RTC1_IRQHandler(void);

void     RTC_Init(void);



#endif




#ifndef  _TIMEOUT_H_
#define  _TIMEOUT_H_

#include "nrf.h"


void     Timeout_Struct_Init(void);
void     Timeout_Reg_Init(void);
void     RTC0_IRQHandler(void);
void     Timeout_Set_MicroSeconds(uint32_t val);
uint8_t  Timeout_Error_Assign(uint32_t val, uint8_t error_code);
void     Timeout_Error_Force_Assign(uint8_t error_code);
uint8_t  Timeout_Error_Get(void);
void     Timeout_Error_Clear(void);
uint8_t  Timeout_Sticky_Error_Get(void);
void     Timeout_Sticky_Error_Clear(void);
uint8_t  Timeout_Status_Get(void);
void     Timeout_Status_Clear(void);
uint8_t  Timeout_Occured_Flag_Get(void);
void     Timeout_Occured_Flag_Clear(void);
void     Timeout_Arm(void);

void     Timeout_Handler(void);

void     Timeout_Init(void);

#endif



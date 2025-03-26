

#ifndef  _TIMEOUT_H_
#define  _TIMEOUT_H_

#include "nrf.h"


void     Timeout_Struct_Init(void);
void     Timeout_Reg_Init(void);
void     Timeout_Delay_1us(void);
void     Timeout_Delay_us(int32_t delay_us);
void     Timeout_Delay_ms(int32_t delay_ms);
void     Timeout_Set_MicroSeconds(int32_t val);
uint8_t  Timeout_Occured(void);
uint8_t  Timeout_Error_Assign(int32_t val, uint8_t error_code);
void     Timeout_Error_Force_Assign(uint8_t error_code);

uint8_t  Timeout_Error_Get(void);
void     Timeout_Error_Clear(void);
uint8_t  Timeout_Sticky_Error_Get(void);
void     Timeout_Sticky_Error_Clear(void);
int32_t  Timeout_CurrVal_Get(void);
void     Timeout_CurrVal_Clear(void);
int32_t  Timeout_SetVal_Get(void);
void     Timeout_SetVal_Clear(void);
void     Timeout_Status_Clear(void);
void     Timeout_Arm(void);
void     Timeout_Init(void);


#endif



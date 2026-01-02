

#ifndef  _LED_H_
#define  _LED_H_
#include "nrf.h"
#include "cdefs.h"

#define   NRF_DK_BOARD

#ifdef   NRF_DK_BOARD
#define  LED_PIN  (17U)
#else
#define  LED_PIN  (22U)
#endif


void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
uint8_t  LED_Get_State(void);
void     LED_Set_State(uint8_t val);
void     LED_Toggle(void);




#endif



/*
 * File:   led.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver -> LED Library
 * Created on December 15, 2024, 9:40 PM
 */

#include "nrf.h"
#include "led.h"
#include "cdefs.h"


void LED_Init(void){
	NRF_GPIO->DIR |= (uint32_t) (1<<LED_PIN);
	#ifdef NRF_DK_BOARD
	NRF_GPIO->OUT |= (uint32_t) (1<<LED_PIN);
	#else
	NRF_GPIO->OUT &= (uint32_t) (~(1<<LED_PIN));
	#endif
}

void LED_On(void){
	#ifdef NRF_DK_BOARD
	NRF_GPIO->OUT &= (uint32_t) (~(1<<LED_PIN));
	#else
	NRF_GPIO->OUT |= (uint32_t) (1<<LED_PIN);
	#endif
}

void LED_Off(void){
	#ifdef NRF_DK_BOARD
	NRF_GPIO->OUT |= (uint32_t) (1<<LED_PIN);
	#else
	NRF_GPIO->OUT &= (uint32_t) (~(1<<LED_PIN));
	#endif
}

uint8_t LED_Get_State(void){
	if(NRF_GPIO->OUT & (1<<LED_PIN)){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

void LED_Set_State(uint8_t val){
	if(val == LOGIC_HIGH){
		LED_On();
	}
	else if(val == LOGIC_LOW){
		LED_Off();
	}
}

void LED_Toggle(void){
	if(LED_Get_State() == ON){
		LED_Off();
	}
	else{
		LED_On();
	}
}




















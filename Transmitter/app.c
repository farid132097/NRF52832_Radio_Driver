

#include "nrf.h"
#include "app.h"
//#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"

uint32_t loop_cnt = 0, state = 0;
uint8_t  buf[40] = "Hello";

void App_Config(void){
	
	
	//Charge
	NRF_GPIO->DIR |= (uint32_t) (1<<6);
	NRF_GPIO->OUT |= (uint32_t) (1<<6);
	
	//Button
	NRF_GPIO->DIR &= (uint32_t) (~(1<<25));
	
	//Power
	NRF_GPIO->DIR |= (uint32_t) (1<<8);
	NRF_GPIO->OUT &= (uint32_t) (~(1<<8));
	
	//Cap
	if((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == 1){
		NRF_NVMC->CONFIG |= NVMC_CONFIG_WEN_Wen;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){
			//
		}
	  NRF_UICR->NFCPINS = UICR_NFCPINS_PROTECT_Disabled;
	}
	NRF_GPIO->DIR |= (uint32_t) (1<<9);
	NRF_GPIO->OUT &= (uint32_t) (~(1<<9));
	
	Clock_Init();
	Timeout_Init();
	LED_Init();
	Radio_Init();
	
	
	//PWM_Init();
	
}

void App_Mainloop(void){
	
	
	if(Timeout_Occured_Flag_Get()){
		
	  //LED_Toggle();
	
	  Radio_Tx_Packet(buf, 5);
	  Radio_Power_Down();
	
	  Timeout_Set_MicroSeconds(1000000);
	}
	
	Radio_Tx_Complete_Handler();
	
	NRF_POWER->TASKS_LOWPWR = 1;
	__WFE();
	__SEV();
	__WFE();
	
	
	/*
	PWM_Set_Duty((uint16_t)loop_cnt);
	Timeout_Set_MicroSeconds(10000);
	while(Timeout_Occured_Flag_Get() == 0);
	loop_cnt++;
	if(loop_cnt > 999){
		loop_cnt = 0;
	}
	*/
	
}

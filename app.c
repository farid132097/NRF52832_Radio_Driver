

#include "nrf.h"
#include "app.h"
//#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"


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
	
	Timeout_Init();
	
	Radio_Init();
}

void App_Mainloop(void){
	
	Radio_Tx();
	Radio_Power_Down();
	
	Timeout_Arm();
	Timeout_Error_Assign(10000, 10);
	
	
	NRF_POWER->TASKS_LOWPWR = 1;
	__WFE();
	__SEV();
	__WFE();
	
}

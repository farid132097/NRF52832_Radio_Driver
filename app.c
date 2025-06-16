

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
	
	if(NRF_SAADC->ENABLE == 1){
	  Timeout_Arm();
		
		NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;
		
	  NRF_SAADC->CH[0].CONFIG = 0x00020000;

    NRF_SAADC->CH[0].PSELP = 0;
    NRF_SAADC->CH[0].PSELN = 0;

    NRF_SAADC->RESOLUTION = 1;
    NRF_SAADC->RESULT.MAXCNT = 0;
    NRF_SAADC->SAMPLERATE = 0;
	
	  NRF_SAADC->ENABLE = 0;
	  while( NRF_SAADC->ENABLE != 0 );
	}
	
	Radio_Power_Down();
	NRF_POWER->TASKS_LOWPWR = 1;
	__WFE();
	__SEV();
	__WFE();
	
	//Clock_Init();
  //Timeout_Init();
	//Radio_Init();
	//UART_Init(UARTE_BAUDRATE_BAUDRATE_Baud38400);
	//UART_Tx_Text_NL("Debug started");
}

void App_Mainloop(void){
	
	
}

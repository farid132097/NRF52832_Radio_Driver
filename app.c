

#include "nrf.h"
#include "app.h"
//#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"


void App_Config(void){
	
	//Disable Cap Charging
	NRF_GPIO->DIR |= (uint32_t) (1<<6);
	NRF_GPIO->OUT |= (uint32_t) (1<<6);
	
	//Check & stop HFCLK XTAL
	Clock_HFCLK_Stop();
	
	Timeout_Init();
}

void App_Mainloop(void){
	
	NRF_POWER->TASKS_LOWPWR = 1;
	__WFE();
	__SEV();
	__WFE();
	
}

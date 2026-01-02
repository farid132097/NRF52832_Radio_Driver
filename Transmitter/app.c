

#include "nrf.h"
#include "app.h"
//#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"


void App_Config(void){
	
	Clocks_Init();
	Timeout_Init();
	Radio_Init();
	LED_Init();
}

void App_Mainloop(void){
	
	LED_Set_State(ON);
	Radio_Tx();
	LED_Set_State(OFF);
	Timeout_Set_MicroSeconds(1000000);
	while(Timeout_Error_Assign(0) == FALSE);
	
}

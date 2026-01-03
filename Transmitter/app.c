

#include "nrf.h"
#include "app.h"
#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"

static uint8_t buf[32] = "Hello";

void App_Config(void){
	
	Clocks_Init();
	Timeout_Init();
	Radio_Init();
	LED_Init();
}

void App_Mainloop(void){
	
	
	if(Radio_Tx(buf, 5) == SUCCESSFUL){
		LED_Set_State(ON);
		Timeout_Delay(400);
		LED_Set_State(OFF);
	}
	
	
	Timeout_Delay(1000000);
	
}

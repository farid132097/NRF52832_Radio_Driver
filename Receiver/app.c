

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
	
	UART_Init(UARTE_BAUDRATE_BAUDRATE_Baud38400);
}

void App_Mainloop(void){
	
	if(Radio_Rx(10000)){
		LED_Set_State(ON);
		Timeout_Set_MicroSeconds( 500 );
		while( Timeout_Error_Assign(0) == FALSE);
		Timeout_Clear_Assignment();
		
		UART_Tx_Parameter_Hex_NL("CrcSts", Radio_Rx_Crc_Sts());
		UART_Tx_Parameter_Hex_NL("ChksmSts", Radio_Rx_Checksum_Sts());
		UART_Tx_Parameter_Hex_NL("Src", Radio_Rx_Packet_Src_Addr());
		UART_Tx_Parameter_Hex_NL("Dst", Radio_Rx_Packet_Dst_Addr());
		
		
		
		LED_Set_State(OFF);
	}
	
}

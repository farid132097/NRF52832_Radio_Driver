

#include "nrf.h"
#include "app.h"
#include "cdefs.h"
#include "timeout.h"
#include "radio.h"
#include "uart.h"


uint8_t buf[64];

void App_Config(void){
  Timeout_Init();
	Radio_Init();
	UART_Init(UARTE_BAUDRATE_BAUDRATE_Baud115200);
	UART_Tx_Text_NL("Debug started");
}

void App_Mainloop(void){
	if(Radio_Rx_Ack(5000)){
		UART_Tx_Parameter_Hex_SP("SRC",Radio_Rx_Extract_SrcH());
		UART_Tx_Number_Hex_SP(Radio_Rx_Extract_SrcL());
		
		UART_Tx_Parameter_Hex_SP("DST",Radio_Rx_Extract_DstH());
		UART_Tx_Number_Hex_SP(Radio_Rx_Extract_DstL());
		
		UART_Tx_Text_NL("Received packet");
	}
	
	if(Timeout_Sticky_Error_Get() == NULL){
		UART_Tx_Parameter_Hex_NL("StickyErr", Timeout_Sticky_Error_Get());
		Timeout_Sticky_Error_Clear();
	}
}

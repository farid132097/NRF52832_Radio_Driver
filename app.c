

#include "nrf.h"
#include "app.h"
#include "cdefs.h"
#include "timeout.h"
#include "radio.h"
#include "uart.h"


void App_Config(void){
  Timeout_Init();
	Radio_Init();
	UART_Init(UARTE_BAUDRATE_BAUDRATE_Baud38400);
	UART_Tx_Text_NL("Debug started");
}

void App_Mainloop(void){
	if(Radio_Rx_Ack(5000)){
		UART_Tx_Parameter_Hex_SP("SRC",Radio_Rx_Extract_SrcAddr());
		UART_Tx_Parameter_Hex_SP("DST",Radio_Rx_Extract_DstAddr());
		
		UART_Tx_Parameter_SP("UpTime", (Radio_Rx_Get_Data_Buf(0)<<24) | (Radio_Rx_Get_Data_Buf(1)<<16) | (Radio_Rx_Get_Data_Buf(2)<<8) | (Radio_Rx_Get_Data_Buf(3)) );
		UART_Tx_Parameter_SP("VCap", (Radio_Rx_Get_Data_Buf(4)<<8) | (Radio_Rx_Get_Data_Buf(5)) );
		UART_Tx_Parameter_NL("VChrg", (Radio_Rx_Get_Data_Buf(6)<<8) | (Radio_Rx_Get_Data_Buf(7)) );
	}
	
	if(Timeout_Sticky_Error_Get() == NULL){
		UART_Tx_Parameter_Hex_NL("StickyErr", Timeout_Sticky_Error_Get());
		Timeout_Sticky_Error_Clear();
	}
}

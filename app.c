

#include "nrf.h"
#include "app.h"
#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"


void App_Config(void){
	Clock_Init();
  Timeout_Init();
	Radio_Init();
	UART_Init(UARTE_BAUDRATE_BAUDRATE_Baud38400);
	UART_Tx_Text_NL("Debug started");
}

void App_Mainloop(void){
	if(Radio_Rx_Ack(5000)){
		UART_Tx_Parameter_Hex_SP("SRC", Radio_Rx_SrcAddr_Get());
		UART_Tx_Parameter_Hex_SP("DST", Radio_Rx_DstAddr_Get());
		UART_Tx_Parameter_Hex_SP("CRC", Radio_Rx_CRC16_Get());
		
		UART_Tx_Parameter_SP("UpTime", (Radio_Rx_Data_Buf_Get(0)<<24) | (Radio_Rx_Data_Buf_Get(1)<<16) | (Radio_Rx_Data_Buf_Get(2)<<8) | (Radio_Rx_Data_Buf_Get(3)) );
		UART_Tx_Parameter_SP("VCap",   (Radio_Rx_Data_Buf_Get(4)<<8)  | (Radio_Rx_Data_Buf_Get(5)) );
		UART_Tx_Parameter_NL("VChrg",  (Radio_Rx_Data_Buf_Get(6)<<8)  | (Radio_Rx_Data_Buf_Get(7)) );
	}
	
	if(Timeout_Sticky_Error_Get() == NULL){
		UART_Tx_Parameter_Hex_NL("StickyErr", Timeout_Sticky_Error_Get());
		Timeout_Sticky_Error_Clear();
	}
}

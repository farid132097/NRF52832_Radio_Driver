

#include "nrf.h"
#include "app.h"
//#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"

uint8_t  buf[32] = "123456789ABCDEFGH";
uint32_t uptime, heater;
int16_t  temp = 0, target_temp = 0, send_target_temp = 0;

void App_Config(void){
	
	Clocks_Init();
	Timeout_Init();
	Radio_Init();
	LED_Init();
	
	UART_Init(UARTE_BAUDRATE_BAUDRATE_Baud38400);
}

void App_Mainloop(void){
	
	send_target_temp = UART_Last_Received_Byte();
	
	buf[0] = send_target_temp >> 8;
	buf[1] = send_target_temp & 0xFF;
	if(Radio_Rx_Send_Ack(buf, 2, 10000)){
		
		//extract uptime
		uptime   = Radio_Buf_Get(0);
		uptime <<= 8;
		uptime  |= Radio_Buf_Get(1);
		uptime <<= 8;
		uptime  |= Radio_Buf_Get(2);
		uptime <<= 8;
		uptime  |= Radio_Buf_Get(3);
		
		//extract heater sts
		heater   = Radio_Buf_Get(4);
		
		//extract target temp
		target_temp   = Radio_Buf_Get(5);
		target_temp <<= 8;
		target_temp  |= Radio_Buf_Get(6);
		
		//extract temp
		temp     = Radio_Buf_Get(7);
		temp   <<= 8;
		temp    |= Radio_Buf_Get(8);
		
		LED_Set_State(ON);
		//UART_Tx_Parameter_Hex_SP("SrcAddr", Radio_Rx_Packet_Src_Addr());
		UART_Tx_Parameter_SP("UpTime", uptime);
		UART_Tx_Parameter_SP("SentTargetTemp", send_target_temp);
		UART_Tx_Parameter_SP("TargetTemp", target_temp);
		UART_Tx_Parameter_SP("Heater", heater);
		UART_Tx_Parameter_NL("CurrentTemp", temp);
		LED_Set_State(OFF);
	}
	
}

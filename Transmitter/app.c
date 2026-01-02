

#include "nrf.h"
#include "app.h"
//#include "cdefs.h"
#include "timeout.h"
#include "clocks.h"
#include "radio.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"

uint8_t  x = 0 ;
uint16_t crc = 0;

void App_Config(void){
	
	Clock_Init();
	Timeout_Init();
	LED_Init();
	Radio_Active();
	
	
}

void App_Mainloop(void){
	
	for(uint8_t i=0; i<32; i++){
		Radio_Tx_Packet_Set(0, i);
	}
	
	Radio_Tx_Packet_Set(17, 0);
	
	Radio_Tx_Packet_Set(0, 1);
	Radio_Tx_Packet_Set(0, 2);
	Radio_Tx_Packet_Set(0, 3);
	Radio_Tx_Packet_Set(x++, 4);
	
	Radio_Tx_Packet_Set(0, 5);
	Radio_Tx_Packet_Set(0, 6);
	
	Radio_Tx_Packet_Set(0, 7);
	Radio_Tx_Packet_Set(0, 8);
	
	Radio_Tx_Packet_Set(0, 9);
	Radio_Tx_Packet_Set(0, 10);
	
	Radio_Tx_Packet_Set(0, 11);
	
	crc = Radio_CRC_Calculate_Tx_Buf(0, 12);
	
	Radio_Tx_Packet_Set(crc >> 8, 12);
	Radio_Tx_Packet_Set(crc & 0xFF, 13);
	
	Radio_Tx_Packet_Set(14, 29);
	
	crc = Radio_CRC_Calculate_Tx_Buf(0, 30);
	Radio_Tx_Packet_Set(crc >> 8, 30);
	Radio_Tx_Packet_Set(crc & 0xFF, 31);
	Radio_Tx();
	
	
	Timeout_Set_MicroSeconds(1000000);
	while(Timeout_Error_Assign(0) == FALSE);
	Timeout_Clear_Events();
	
	
}

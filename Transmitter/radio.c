

/*
 * File:   radio.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver : NRF52832 RADIO Library
 * Created : December 15, 2024, 9:30 PM
 * Last Modified : 13 May, 2025, Rev2.4
 */
 

#include "nrf.h"
#include "radio.h"
#include "cdefs.h"
#include "clocks.h"
#include "timeout.h"


__attribute__((aligned(4)))
static uint8_t tx_packet[32];


uint16_t Radio_CRC_Calculate_Byte(uint16_t crc, uint8_t data){
	uint16_t temp = data;
	temp <<= 8;
  crc = crc ^ temp;
  for(uint8_t i = 0; i < 8; i++){
    if(crc & 0x8000){
			temp   = crc;
			temp <<= 0x01;
			temp  ^= 0x1021;
	    crc = temp;
	  }
    else{
	    crc <<= 1;
	  }
  }
  return crc;
}

uint16_t Radio_CRC_Calculate_Block(uint8_t *buf, uint8_t start, uint8_t end){
  uint16_t crc = 0;
  for(uint8_t i = start; i < end; i++){
    crc = Radio_CRC_Calculate_Byte(crc,buf[i]);
  }
  return crc;
}


uint16_t Radio_CRC_Calculate_Tx_Buf(uint8_t start, uint8_t end){
  uint16_t crc = 0;
  crc = Radio_CRC_Calculate_Block(tx_packet, start, end);
  return crc;
}


void Radio_HFCLK_Start(void){
  if( (NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)) == 0 ){
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
		
		Timeout_Set_MicroSeconds(1000);
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0){
			//Standard startup time for XTAl is 360uS
		  if(Timeout_Error_Assign(ERROR_RADIO_HFCLK_START_FAILED)){
			  break;
		  }
	  }
		Timeout_Clear_Events();
		
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
		if( (NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)) == 0 ){
			Timeout_Error_Force_Assign(ERROR_RADIO_HFCLK_START_FAILED);
		}
  }
}

void Radio_HFCLK_Stop(void){
	if( NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) ){
	  NRF_CLOCK->TASKS_HFCLKSTOP = 1;
	  while( NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) );
	}
}


void Radio_Reg_Init(void){
	  NRF_RADIO->TXPOWER     = RADIO_TXPOWER_TXPOWER_0dBm;
	  NRF_RADIO->FREQUENCY   = 2;
		NRF_RADIO->MODE        = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
	  NRF_RADIO->BASE0       = 0x4B43;
		NRF_RADIO->PREFIX0     = 0x41;
	  NRF_RADIO->TXADDRESS   = 0;
	  NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->PCNF0       = (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) |
                             (1 << RADIO_PCNF0_S0LEN_Pos)   |
                             (1 << RADIO_PCNF0_S1LEN_Pos);
	  NRF_RADIO->PCNF1       = (32 <<RADIO_PCNF1_MAXLEN_Pos)  |
	                           (32 <<RADIO_PCNF1_STATLEN_Pos) |
	                           (2  <<RADIO_PCNF1_BALEN_Pos)   |
	                           (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)|
		                         (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
	  NRF_RADIO->SHORTS      = 0x00000000;
	  NRF_RADIO->CRCCNF      = 0;
	  NRF_RADIO->MODECNF0    = (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos)|
                             (RADIO_MODECNF0_RU_Fast    << RADIO_MODECNF0_RU_Pos);
	  NRF_RADIO->EVENTS_ADDRESS = 0;
	  NRF_RADIO->EVENTS_PAYLOAD = 0;
}

void Radio_Power_Enable(void){
	NRF_RADIO->POWER = 1;
}

void Radio_Power_Disable(void){
	NRF_RADIO->POWER = 0;
}

void Radio_Active(void){
	Radio_Power_Enable();
	Radio_HFCLK_Start();
	Radio_Reg_Init();
	//Clock_HFCLK_Xtal_Wait_Until_Ready();
}

void Radio_Power_Down(void){
	Radio_Power_Disable();
	Radio_HFCLK_Stop();
	//Radio.RegInit = INCOMPLETE;
}


void Radio_Mode_Disable(void){
	if( (NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Disabled ){
    NRF_RADIO->EVENTS_DISABLED = 0;
	  NRF_RADIO->TASKS_DISABLE = 1;
		
		//Timeout_Set_MicroSeconds(300);
	  while(NRF_RADIO->EVENTS_DISABLED == 0){
	    //if(Timeout_Error_Assign(ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED)){
			//  break;
		  //}
	  }
		//Timeout_Clear_Events();
	}
}

void Radio_Mode_Tx(void){
	if( ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_TxIdle) && 
		  ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Tx)   )  {
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->TASKS_TXEN = 1;
		
		Timeout_Set_MicroSeconds(500);
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign(ERROR_RADIO_MODE_SWITCH_TX_FAILED)){
			  break;
		  }
    }
		Timeout_Clear_Events();
  }
}

void Radio_Mode_Rx(void){
	if( ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_RxIdle) &&
  		((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Rx)   )  {
	  NRF_RADIO->EVENTS_READY =0;
	  NRF_RADIO->TASKS_RXEN = 1;
		
		Timeout_Set_MicroSeconds(500);
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign(ERROR_RADIO_MODE_SWITCH_RX_FAILED)){
			  break;
		  }
    }
		Timeout_Clear_Events();
	}
}


void Radio_Start_Task(int32_t delay){
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;
	
	Timeout_Set_MicroSeconds((uint32_t)delay);
	while(NRF_RADIO->EVENTS_END == 0){
	  if(Timeout_Error_Assign(ERROR_RADIO_TASK_TIMEOUT_OCCURED)){
			break;
		}
	}
	Timeout_Clear_Events();
}



void Radio_Tx(void){
	Radio_Mode_Tx();
	NRF_RADIO->PACKETPTR = (uint32_t)tx_packet;
	Radio_Start_Task(2000);

  //NRF_RADIO->EVENTS_END = 0;
  //NRF_RADIO->TASKS_DISABLE = 1;
  //while(!NRF_RADIO->EVENTS_DISABLED);
  //NRF_RADIO->EVENTS_DISABLED = 0;
}

void Radio_Tx_Packet_Set(uint8_t val, uint8_t index){
	tx_packet[index] = val;
}

uint8_t Radio_Tx_Packet_Get(uint8_t index){
	return tx_packet[index];
}

void Radio_Init(void){
	//Radio_Struct_Init();
	Radio_Reg_Init();
	Radio_Mode_Disable();
	
	#ifdef RADIO_DEBUG_ENABLED
	#warning Radio debug enabled
	#endif
}








/*
 * File:   radio.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver -> RADIO Library
 * Created on December 15, 2024, 9:30 PM
 */

#include "nrf.h"
#include "radio.h"
#include "cdefs.h"
#include "timeout.h"

typedef struct radio_t{
	uint8_t RegInit;
}radio_t;

static radio_t Radio;
	

void Radio_Struct_Init(void){
	Radio.RegInit = INCOMPLETE;
}

void Radio_HFCLK_Start(void){
  if((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) != CLOCK_HFCLKSTAT_SRC_Xtal){
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
		Timeout_Arm();
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0){
		  if(Timeout_Error_Assign(1000, ERROR_RADIO_HFCLK_START_FAILED)){
			  break;
		  }
	  }
		
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
		if((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) != CLOCK_HFCLKSTAT_SRC_Xtal){
			Timeout_Error_Force_Assign(ERROR_RADIO_HFCLK_START_FAILED);
		}
  }
}

void Radio_HFCLK_Stop(void){
	if( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) != CLOCK_HFCLKSTAT_STATE_NotRunning){
	  NRF_CLOCK->TASKS_HFCLKSTOP = 1;
	  Timeout_Arm();
	  while( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) != CLOCK_HFCLKSTAT_STATE_NotRunning ){
		  if(Timeout_Error_Assign(1000, ERROR_RADIO_HFCLK_STOP_FAILED)){
			  break;
		  }
	  }
	}
}

void Radio_Reg_Init(void){
	if(Radio.RegInit == INCOMPLETE){
	  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;
	  NRF_RADIO->FREQUENCY = 10;
	  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit<<RADIO_MODE_MODE_Pos);
	  NRF_RADIO->PREFIX0 = 0x11223344;
	  NRF_RADIO->BASE0   = 0x11111111;
	  NRF_RADIO->TXADDRESS = 0;
	  NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->PCNF0  = 0;
	  NRF_RADIO->PCNF1  = (40<<RADIO_PCNF1_MAXLEN_Pos)  |
	                      (36 <<RADIO_PCNF1_STATLEN_Pos)|
	                      (4 <<RADIO_PCNF1_BALEN_Pos)   |
	                      (RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos);
	  NRF_RADIO->SHORTS = 0x00000000;
	  NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos;
	  NRF_RADIO->MODECNF0 = (1<<9)|(1<<0);
	  NRF_RADIO->CRCINIT = 0xFFFFFF;
	  NRF_RADIO->CRCPOLY = 0x11021;
	  NRF_RADIO->EVENTS_ADDRESS = 0;
	  NRF_RADIO->EVENTS_PAYLOAD = 0;
		Radio.RegInit = COMPLETE;
	}
}

void Radio_Power_Enable(void){
	if(NRF_RADIO->POWER != RADIO_POWER_POWER_Enabled){
	  NRF_RADIO->POWER = 1;
		Timeout_Arm();
	  while(NRF_RADIO->POWER == 0){
		  if(Timeout_Error_Assign(1000, ERROR_RADIO_POWER_ENABLE_FAILED)){
			  break;
		  }
	  }
  }
}

void Radio_Power_Disable(void){
	if(NRF_RADIO->POWER == 1){
	  NRF_RADIO->POWER = 0;
		Timeout_Arm();
	  while(NRF_RADIO->POWER == 1){
		  if(Timeout_Error_Assign(1000, ERROR_RADIO_POWER_DISABLE_FAILED)){
			  break;
		  }
	  }
  }
}

void Radio_Active(void){
	Radio_Power_Enable();
	Radio_HFCLK_Start();
	Radio_Reg_Init();
}

void Radio_Power_Down(void){
	Radio_Power_Disable();
	Radio_HFCLK_Stop();
	Radio.RegInit = INCOMPLETE;
}


void Radio_Mode_Disable(void){
	if(NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled){
    NRF_RADIO->EVENTS_DISABLED = 0;
	  NRF_RADIO->TASKS_DISABLE = 1;
		Timeout_Arm();
	  while(NRF_RADIO->EVENTS_DISABLED == 0){
	    if(Timeout_Error_Assign(1000, ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED)){
			  break;
		  }
	  }
	}
}

void Radio_Mode_Tx(void){
	Radio_Active();
	if( ((NRF_RADIO->STATE == RADIO_STATE_STATE_TxIdle) || (NRF_RADIO->STATE == RADIO_STATE_STATE_Tx)) == 0 ){
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->TASKS_TXEN = 1;
		Timeout_Arm();
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign(1000, ERROR_RADIO_MODE_SWITCH_TX_FAILED)){
			  break;
		  }
    }
  }
}

void Radio_Mode_Rx(void){
	Radio_Active();
	if( ((NRF_RADIO->STATE == RADIO_STATE_STATE_RxIdle) || (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx)) == 0 ){
	  NRF_RADIO->EVENTS_READY =0;
	  NRF_RADIO->TASKS_RXEN = 1;
		Timeout_Arm();
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign(1000, ERROR_RADIO_MODE_SWITCH_RX_FAILED)){
			  break;
		  }
    }
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->EVENTS_CRCOK = 0;
	}
}


void Radio_Start_Task(int32_t delay){
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;
	Timeout_Arm();
	while(NRF_RADIO->EVENTS_END == 0){
	  if(Timeout_Error_Assign((int32_t)delay, ERROR_RADIO_TASK_TIMEOUT_OCCURED)){
			break;
		}
	}
}



uint8_t Radio_Tx(uint8_t *buf, uint8_t len){
	uint8_t sts  = TRUE;
	Timeout_Arm();
	Radio_Mode_Tx();
	if(Timeout_Error_Get() != NULL){
		Radio_Mode_Disable();
		return FALSE;
	}
	
	buf[0] = len;
	NRF_RADIO->PACKETPTR = (uint32_t)buf;
	Radio_Start_Task(1000);
	
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	return sts;
}

uint8_t Radio_Rx(uint8_t *buf, int32_t timeout){
	uint8_t sts  = SUCCESSFUL;
	Timeout_Arm();
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	
	NRF_RADIO->PACKETPTR = (uint32_t)buf;
	Radio_Start_Task(timeout);
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	
	if(NRF_RADIO->EVENTS_CRCOK != 1){
		Timeout_Error_Force_Assign(ERROR_RADIO_CRC_NOT_OK);
		return FALSE;
	}
	else{
		NRF_RADIO->EVENTS_CRCOK = 0;
	}
	
	return sts;
}
	
uint8_t Radio_Tx_Ack(uint8_t *buf, uint8_t len){
	uint8_t sts = FAILED;
	if(Radio_Tx(buf, len) == SUCCESSFUL){
		sts = Radio_Rx(buf, 800);
	}
	return sts;
}
	

uint8_t Radio_Rx_Ack(uint8_t *buf, int32_t timeout){
	uint8_t sts = TRUE;
	
	Radio_Rx(buf, timeout);
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	if(NRF_RADIO->CRCSTATUS != 1){
		return FALSE;
	}
	else{
		NRF_RADIO->EVENTS_CRCOK = 0;
		NRF_RADIO->EVENTS_ADDRESS = 0;
		NRF_RADIO->EVENTS_PAYLOAD = 0;
	}
	Radio_Tx(buf, 5);
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	return sts;
}




void Radio_Init(void){
	Radio_Struct_Init();
	Radio_Reg_Init();
	Radio_Mode_Disable();
	
	#ifdef RADIO_DEBUG_ENABLED
	#warning Radio debug enabled
	#endif
}









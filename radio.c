

/*
 * File:   radio.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver : RADIO Library
 * Created : December 15, 2024, 9:30 PM
 * Rev2.1 
 * Last Modified : 28 Mar, 2025
 *
 */

#include "nrf.h"
#include "radio.h"
#include "cdefs.h"
#include "timeout.h"

#define  RADIO_BUF_SIZE 36

typedef struct packet_t{
	uint8_t  Length;
	uint8_t  PID;
	uint8_t  DstLoad;
	uint8_t  Reserved1;
	uint32_t SRCH;
	uint32_t SRCL;
	uint32_t DSTH;
	uint32_t DSTL;
	uint8_t  Buf[RADIO_BUF_SIZE];
}packet_t;

typedef struct radio_t{
	uint8_t  RegInit;
	uint8_t  Reserved5;
	uint8_t  Reserved6;
	uint8_t  Reserved7;
	packet_t TxPacket;
	packet_t RxPacket;
}radio_t;

static radio_t Radio;
	

void Radio_Struct_Init(void){
	Radio.RegInit          = INCOMPLETE;
	Radio.TxPacket.Length  = 0;
	Radio.TxPacket.PID     = 0;
	Radio.TxPacket.DstLoad = TRUE; //Target Destination Loaded
	Radio.TxPacket.SRCH    = OWN_DEV_ADDRESS_QWORD_H;
	Radio.TxPacket.SRCL    = OWN_DEV_ADDRESS_QWORD_L;
	Radio.TxPacket.DSTH    = REC_DEV_ADDRESS_QWORD_H;
	Radio.TxPacket.DSTL    = REC_DEV_ADDRESS_QWORD_L;
	Radio.TxPacket.Buf[0]  = Radio.TxPacket.Length;
	Radio.TxPacket.Buf[1]  = Radio.TxPacket.PID;
	
	//Load 64 Bit Source Address
	Radio.TxPacket.Buf[2]  = (Radio.TxPacket.SRCH >> 24) & 0xFF;
	Radio.TxPacket.Buf[3]  = (Radio.TxPacket.SRCH >> 16) & 0xFF;
	Radio.TxPacket.Buf[4]  = (Radio.TxPacket.SRCH >>  8) & 0xFF;
	Radio.TxPacket.Buf[5]  = (Radio.TxPacket.SRCH >>  0) & 0xFF;
	Radio.TxPacket.Buf[6]  = (Radio.TxPacket.SRCL >> 24) & 0xFF;
	Radio.TxPacket.Buf[7]  = (Radio.TxPacket.SRCL >> 16) & 0xFF;
	Radio.TxPacket.Buf[8]  = (Radio.TxPacket.SRCL >>  8) & 0xFF;
	Radio.TxPacket.Buf[9]  = (Radio.TxPacket.SRCL >>  0) & 0xFF;
	
	//Load 64 Bit Destination Address
	Radio.TxPacket.Buf[10] = (Radio.TxPacket.DSTH >> 24) & 0xFF;
	Radio.TxPacket.Buf[11] = (Radio.TxPacket.DSTH >> 16) & 0xFF;
	Radio.TxPacket.Buf[12] = (Radio.TxPacket.DSTH >>  8) & 0xFF;
	Radio.TxPacket.Buf[13] = (Radio.TxPacket.DSTH >>  0) & 0xFF;
	Radio.TxPacket.Buf[14] = (Radio.TxPacket.DSTL >> 24) & 0xFF;
	Radio.TxPacket.Buf[15] = (Radio.TxPacket.DSTL >> 16) & 0xFF;
	Radio.TxPacket.Buf[16] = (Radio.TxPacket.DSTL >>  8) & 0xFF;
	Radio.TxPacket.Buf[17] = (Radio.TxPacket.DSTL >>  0) & 0xFF;
	
	//Clear Only Data Sections of Tx Buf
	for(uint8_t i=18; i<RADIO_BUF_SIZE; i++){
		Radio.TxPacket.Buf[i] = 0;
	}
	
	//Clear All Rx Buf
	for(uint8_t i=0; i<RADIO_BUF_SIZE; i++){
		Radio.RxPacket.Buf[i] = 0;
	}
}

void Radio_Tx_Set_Dst_Addr(uint32_t dstH, uint32_t dstL){
	Radio.TxPacket.DSTH = dstH;
	Radio.TxPacket.DSTL = dstL;
}

void Radio_Tx_Load_Dst_Addr(void){
	if(Radio.TxPacket.DstLoad == FALSE){
	  Radio.TxPacket.Buf[10] = (Radio.TxPacket.DSTH >> 24) & 0xFF;
	  Radio.TxPacket.Buf[11] = (Radio.TxPacket.DSTH >> 16) & 0xFF;
	  Radio.TxPacket.Buf[12] = (Radio.TxPacket.DSTH >>  8) & 0xFF;
	  Radio.TxPacket.Buf[13] = (Radio.TxPacket.DSTH >>  0) & 0xFF;
	  Radio.TxPacket.Buf[14] = (Radio.TxPacket.DSTL >> 24) & 0xFF;
	  Radio.TxPacket.Buf[15] = (Radio.TxPacket.DSTL >> 16) & 0xFF;
	  Radio.TxPacket.Buf[16] = (Radio.TxPacket.DSTL >>  8) & 0xFF;
	  Radio.TxPacket.Buf[17] = (Radio.TxPacket.DSTL >>  0) & 0xFF;
	  Radio.TxPacket.DstLoad = TRUE; //Dest Load Complete
	}
}

void Radio_Tx_Copy_Dst_Addr(void){
	for(uint8_t i=0; i<8; i++){
		//For Ack packet, Destination of TX device will be Source address of the RX device
		Radio.TxPacket.Buf[10+i] = Radio.RxPacket.Buf[2+i];
	}
	//Because of ACK packet, Target Dst Addr not loaded
	Radio.TxPacket.DstLoad = FALSE; 
}

void Radio_Tx_Set_Len(uint8_t len){
	Radio.TxPacket.Buf[0] = len;
	Radio.TxPacket.Length = len;
}

void Radio_Tx_Set_PID(uint8_t pid){
	Radio.TxPacket.Buf[1] = pid;
	Radio.TxPacket.PID    = pid;
}

void Radio_Tx_Set_Buf(uint8_t index, uint8_t data){
	Radio.TxPacket.Buf[index] = data;
}

void Radio_Tx_Set_Data_Buf(uint8_t index, uint8_t data){
	Radio.TxPacket.Buf[index + 18] = data;
}

uint32_t Radio_Rx_Extract_SrcH(void){
	uint8_t i;
	uint32_t temp = 0;
	for(i=0; i<4; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[2 + i];
	}
	return temp;
}

uint32_t Radio_Rx_Extract_SrcL(void){
	uint8_t i;
	uint32_t temp = 0;
	for(i=0; i<4; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[6 + i];
	}
	return temp;
}

uint32_t Radio_Rx_Extract_DstH(void){
	uint8_t i;
	uint32_t temp = 0;
	for(i=0; i<4; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[10 + i];
	}
	return temp;
}

uint32_t Radio_Rx_Extract_DstL(void){
	uint8_t i;
	uint32_t temp = 0;
	for(i=0; i<4; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[14 + i];
	}
	return temp;
}

uint8_t Radio_Rx_Get_Buf(uint8_t index){
	return Radio.RxPacket.Buf[index];
}

uint8_t Radio_Rx_Get_Data_Buf(uint8_t index){
	return Radio.RxPacket.Buf[index + 18];
}

void Radio_HFCLK_Start(void){
  if((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) != CLOCK_HFCLKSTAT_SRC_Xtal){
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
		Timeout_Arm();
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0){
		  if(Timeout_Error_Assign(300, ERROR_RADIO_HFCLK_START_FAILED)){
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
		  if(Timeout_Error_Assign(300, ERROR_RADIO_HFCLK_STOP_FAILED)){
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
		  if(Timeout_Error_Assign(300, ERROR_RADIO_POWER_ENABLE_FAILED)){
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
		  if(Timeout_Error_Assign(300, ERROR_RADIO_POWER_DISABLE_FAILED)){
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
	    if(Timeout_Error_Assign(300, ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED)){
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
		  if(Timeout_Error_Assign(300, ERROR_RADIO_MODE_SWITCH_TX_FAILED)){
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
		  if(Timeout_Error_Assign(300, ERROR_RADIO_MODE_SWITCH_RX_FAILED)){
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



uint8_t Radio_Tx(void){
	uint8_t sts  = TRUE;
	Timeout_Arm();
	Radio_Mode_Tx();
	if(Timeout_Error_Get() != NULL){
		Radio_Mode_Disable();
		return FALSE;
	}
	
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.TxPacket.Buf;
	Radio_Start_Task(300);
	
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	return sts;
}

uint8_t Radio_Rx(int32_t timeout){
	uint8_t sts  = SUCCESSFUL;
	Timeout_Arm();
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.RxPacket.Buf;
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

	
uint8_t Radio_Tx_Ack(void){
	uint8_t sts = FAILED;
	if(Radio_Tx() == SUCCESSFUL){
		sts = Radio_Rx(100);
		if( (sts == SUCCESSFUL) && (Radio_Rx_Extract_DstH() == Radio.TxPacket.SRCH) && (Radio_Rx_Extract_DstL() == Radio.TxPacket.SRCL)){
			return SUCCESSFUL;
		}
		return FAILED;
	}
	return sts;
}
	

uint8_t Radio_Rx_Ack(int32_t timeout){
	uint8_t sts = TRUE;
	
	Radio_Rx(timeout);
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
	Radio_Tx_Copy_Dst_Addr();
	Radio_Tx();
	Radio_Tx_Load_Dst_Addr();
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









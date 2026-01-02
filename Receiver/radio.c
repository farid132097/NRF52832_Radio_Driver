

/*
 * File:   radio.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver : RADIO Library
 * Created : December 15, 2024, 9:30 PM
 * Last Modified : 12 May, 2025, Rev2.3
 *
 */

#include "nrf.h"
#include "radio.h"
#include "cdefs.h"
#include "timeout.h"
#include "datahandler.h"
#include "clocks.h"
#include "config.h"
#include "crc.h"

//Max retry if tx is failed
#define  RADIO_TX_FAILED_RETRY       ( 5U)

//Tx retry will be turned off if consecutive
//attempts are failed (including retry)
#define  RADIO_TX_RETRY_DIS_FAIL_ATT ( 5U)

#define  RADIO_FRAME_BUF_SIZE        (36U)
#define  RADIO_FRAME_LEN_POS         ( 0U)
#define  RADIO_FRAME_LEN_SIZE        ( 1U)
#define  RADIO_FRAME_PID_POS         ( 1U)
#define  RADIO_FRAME_PID_SIZE        ( 1U)
#define  RADIO_FRAME_SRC_ADDR_POS    ( 2U)
#define  RADIO_FRAME_SRC_ADDR_SIZE   ( 8U)
#define  RADIO_FRAME_DST_ADDR_POS    (10U)
#define  RADIO_FRAME_DST_ADDR_SIZE   ( 8U)
#define  RADIO_FRAME_CRC16_POS       (18U)
#define  RADIO_FRAME_CRC16_SIZE      ( 2U)
#define  RADIO_FRAME_USER_DATA_POS   (20U)
#define  RADIO_FRAME_USER_DATA_SIZE  (16U)

//Frame Format:
//Len(1 Byte) + PID(1 Byte) + SrcAddr(8 Byte) + DstAddr(8 Byte) + CRC16(2 Byte) + Data(16 Byte)
//CRC16 Includes all bytes including CrcL and CrcH bytes considering 0x00

typedef struct packet_t{
	uint8_t  Length;
	uint8_t  PID;
	uint8_t  LastPID;
	uint8_t  Reserved0[5];
	
	uint64_t LastSender;
	uint64_t SrcAddr;
	uint64_t DstAddr;
	
	uint16_t CRC16;
	uint8_t  CRCSts;
	uint8_t  Buf[RADIO_FRAME_BUF_SIZE];
	uint8_t  Reserved1[1];
}packet_t;

typedef struct radio_t{
	uint8_t  RegInit;
	uint8_t  RetryEnable;
	uint8_t  RetryCnt;
	uint8_t  FaildAttempts;
	uint8_t  LastPktSts;
	uint8_t  PowerDown;
	uint8_t  Reserved2[2];
	
	packet_t TxPacket;
	packet_t RxPacket;
}radio_t;

static radio_t Radio;
	

void Radio_Struct_Init(void){
	Radio.RegInit           = INCOMPLETE;
	Radio.RetryEnable       = TRUE;
	Radio.RetryCnt          = 0;
	Radio.FaildAttempts     = 0;
	Radio.LastPktSts        = SUCCESSFUL;
	Radio.PowerDown         = TRUE;
	Radio.TxPacket.Length   = 0;
	Radio.TxPacket.PID      = 0;
}


void Radio_Reg_Init(void){
	if(Radio.RegInit == INCOMPLETE){
	  NRF_RADIO->TXPOWER     = RADIO_TXPOWER_TXPOWER_Pos4dBm;
	  NRF_RADIO->FREQUENCY   = 10;
		NRF_RADIO->MODE        = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
	  NRF_RADIO->PREFIX0     = 0x11223344;
	  NRF_RADIO->BASE0       = 0x11111111;
	  NRF_RADIO->TXADDRESS   = 0;
	  NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->PCNF0       = 0;
	  NRF_RADIO->PCNF1       = (36 <<RADIO_PCNF1_MAXLEN_Pos)  |
	                           (32 <<RADIO_PCNF1_STATLEN_Pos) |
	                           (4  <<RADIO_PCNF1_BALEN_Pos)   |
	                           (RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos);
	  NRF_RADIO->SHORTS      = 0x00000000;
	  NRF_RADIO->CRCCNF      = RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos;
	  NRF_RADIO->MODECNF0    = (1<<9)|(1<<0);
	  NRF_RADIO->CRCINIT     = 0xFFFFFF;
	  NRF_RADIO->CRCPOLY     = 0x11021;
	  NRF_RADIO->EVENTS_ADDRESS = 0;
	  NRF_RADIO->EVENTS_PAYLOAD = 0;
		Radio.RegInit = COMPLETE;
	}
}


void Radio_Power_Enable(void){
	if(NRF_RADIO->POWER != RADIO_POWER_POWER_Enabled){
	  NRF_RADIO->POWER = 1;
  }
}

void Radio_Power_Disable(void){
	if(NRF_RADIO->POWER == RADIO_POWER_POWER_Enabled){
	  NRF_RADIO->POWER = 0;
  }
}

void Radio_Active(void){
	if(Radio.PowerDown == TRUE){
	  Radio_Power_Enable();
	  Clocks_HFCLK_Xtal_Start_Request();
	  Radio_Reg_Init();
		Radio.PowerDown = FALSE;
	}
}

void Radio_Power_Down(void){
	if(Radio.PowerDown == FALSE){
	  Radio_Power_Disable();
	  Clocks_HFCLK_Xtal_Stop_Request();
	  Radio.RegInit = INCOMPLETE;
		Radio.PowerDown = TRUE;
	}
}


void Radio_Mode_Disable(void){
	if(NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled){
    NRF_RADIO->EVENTS_DISABLED = 0;
	  NRF_RADIO->TASKS_DISABLE = 1;
		Timeout_Set_MicroSeconds(500);
	  while(NRF_RADIO->EVENTS_DISABLED == 0){
	    if(Timeout_Error_Assign( ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED )){
			  break;
		  }
	  }
		Timeout_Clear_Assignment();
	}
}

void Radio_Mode_Tx(void){
	Radio_Active();
	if( ((NRF_RADIO->STATE == RADIO_STATE_STATE_TxIdle) || (NRF_RADIO->STATE == RADIO_STATE_STATE_Tx)) == 0 ){
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->TASKS_TXEN = 1;
		Timeout_Set_MicroSeconds(500);
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign( ERROR_RADIO_MODE_SWITCH_TX_FAILED )){
			  break;
		  }
    }
		Timeout_Clear_Assignment();
  }
}

void Radio_Mode_Rx(void){
	Radio_Active();
	if( ((NRF_RADIO->STATE == RADIO_STATE_STATE_RxIdle) || (NRF_RADIO->STATE == RADIO_STATE_STATE_Rx)) == 0 ){
	  NRF_RADIO->EVENTS_READY =0;
	  NRF_RADIO->TASKS_RXEN = 1;
		Timeout_Set_MicroSeconds(500);
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign( ERROR_RADIO_MODE_SWITCH_RX_FAILED )){
			  break;
		  }
    }
		Timeout_Clear_Assignment();
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->EVENTS_CRCOK = 0;
	}
}

uint8_t Radio_Tx(void){
	uint8_t sts  = TRUE;
	Timeout_Error_Clear();
	Radio_Mode_Tx();
	if(Timeout_Error_Get() != NULL){
		Radio_Mode_Disable();
		return FALSE;
	}
	
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.TxPacket.Buf;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;
	Timeout_Set_MicroSeconds(1000);
	while(NRF_RADIO->EVENTS_END == 0){
	  if(Timeout_Error_Assign( ERROR_RADIO_TX_TASK_TIMEOUT )){
			break;
		}
	}
	Timeout_Clear_Assignment();
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	return sts;
}

uint8_t Radio_Rx(uint32_t timeout){
	uint8_t sts  = SUCCESSFUL;
	Timeout_Error_Clear();
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.RxPacket.Buf;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_START = 1;
	Timeout_Set_MicroSeconds( timeout );
	while(NRF_RADIO->EVENTS_END == 0){
	  if(Timeout_Error_Assign( ERROR_RADIO_RX_TASK_TIMEOUT )){
			break;
		}
	}
	Timeout_Clear_Assignment();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	
	if(NRF_RADIO->EVENTS_CRCOK != 1){
		Timeout_Error_Force_Assign(ERROR_RADIO_CRC_NOT_OK);
		return FALSE;
	}
	else{
		NRF_RADIO->EVENTS_CRCOK = 0;
		Radio.RxPacket.Length = Radio.RxPacket.Buf[RADIO_FRAME_LEN_POS];
		if(Radio.RxPacket.Length > RADIO_FRAME_USER_DATA_SIZE){
			Radio.RxPacket.Length = RADIO_FRAME_USER_DATA_SIZE;
		}
		Radio.RxPacket.PID = Radio.RxPacket.Buf[RADIO_FRAME_PID_POS];
		Radio.RxPacket.CRC16   = Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS];
		Radio.RxPacket.CRC16 <<= 8;
		Radio.RxPacket.CRC16  |= Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS+1];
	}
	
	return sts;
}



void Radio_Init(void){
	Radio_Struct_Init();
	Radio_Active();
	Radio_Power_Down();
}






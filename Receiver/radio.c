

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
#include "arq.h"

#define  RADIO_PACKET_LEN          (32U)
#define  RADIO_CRC_CALC_LEN        (RADIO_PACKET_LEN- 2)
#define  RADIO_CRC16H_POS          (RADIO_PACKET_LEN- 1)
#define  RADIO_CRC16L_POS          (RADIO_PACKET_LEN- 2)
#define  RADIO_CHECKSUM_POS        (RADIO_PACKET_LEN- 3)
#define  RADIO_DATA_LEN_POS        (RADIO_PACKET_LEN- 4)
#define  RADIO_PACKET_INFO_POS     (RADIO_PACKET_LEN- 5)
#define  RADIO_DST_ADDR3_POS       (RADIO_PACKET_LEN- 6)
#define  RADIO_DST_ADDR2_POS       (RADIO_PACKET_LEN- 7)
#define  RADIO_DST_ADDR1_POS       (RADIO_PACKET_LEN- 8)
#define  RADIO_DST_ADDR0_POS       (RADIO_PACKET_LEN- 9)
#define  RADIO_SRC_ADDR3_POS       (RADIO_PACKET_LEN-10)
#define  RADIO_SRC_ADDR2_POS       (RADIO_PACKET_LEN-11)
#define  RADIO_SRC_ADDR1_POS       (RADIO_PACKET_LEN-12)
#define  RADIO_SRC_ADDR0_POS       (RADIO_PACKET_LEN-13)

#define  OWN_DEVICE_ADDRESS0       ((OWN_DEVICE_ADDRESS>> 0) & 0xFF)
#define  OWN_DEVICE_ADDRESS1       ((OWN_DEVICE_ADDRESS>> 8) & 0xFF)
#define  OWN_DEVICE_ADDRESS2       ((OWN_DEVICE_ADDRESS>>16) & 0xFF)
#define  OWN_DEVICE_ADDRESS3       ((OWN_DEVICE_ADDRESS>>24) & 0xFF)

#define  DST_DEVICE_ADDRESS0       ((DST_DEVICE_ADDRESS>> 0) & 0xFF)
#define  DST_DEVICE_ADDRESS1       ((DST_DEVICE_ADDRESS>> 8) & 0xFF)
#define  DST_DEVICE_ADDRESS2       ((DST_DEVICE_ADDRESS>>16) & 0xFF)
#define  DST_DEVICE_ADDRESS3       ((DST_DEVICE_ADDRESS>>24) & 0xFF)


//Frame Format:
//Len(1 Byte) + PID(1 Byte) + SrcAddr(8 Byte) + DstAddr(8 Byte) + CRC16(2 Byte) + Data(16 Byte)
//CRC16 Includes all bytes including CrcL and CrcH bytes considering 0x00

typedef struct packet_t{
	uint8_t  Length;
	uint8_t  PID;
	uint8_t  LastPID;
	uint8_t  PacketInfo;
	uint32_t SrcAddr;
	uint32_t DstAddr;
	
	uint16_t CRC16;
	uint8_t  Checksum8;
	uint8_t  CRCSts;
	uint8_t  ChksmSts;
	uint8_t  Buf[RADIO_PACKET_LEN];
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
	  NRF_RADIO->PCNF1       = (32 <<RADIO_PCNF1_MAXLEN_Pos)  |
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
	if((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Disabled){
    NRF_RADIO->EVENTS_DISABLED = 0;
	  NRF_RADIO->TASKS_DISABLE = 1;
		Timeout_Set_MicroSeconds(1000);
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
	if( ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_TxIdle) && 
		  ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Tx)   )  {
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->TASKS_TXEN = 1;
		Timeout_Set_MicroSeconds(1000);
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
	if( ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_RxIdle) &&
  		((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Rx)   )  {
	  NRF_RADIO->EVENTS_READY =0;
	  NRF_RADIO->TASKS_RXEN = 1;
		Timeout_Set_MicroSeconds(1000);
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

uint8_t Radio_Tx(uint8_t *buf, uint8_t len){
	uint8_t sts  = TRUE;
	Timeout_Error_Clear();
	Radio_Mode_Tx();
	if(Timeout_Error_Get() != NULL){
		Radio_Mode_Disable();
		return FALSE;
	}
	
	for(uint8_t i=0; i<len; i++){
		Radio.TxPacket.Buf[i] = buf[i];
	}
	Radio.TxPacket.Buf[RADIO_SRC_ADDR0_POS]   = OWN_DEVICE_ADDRESS0;
	Radio.TxPacket.Buf[RADIO_SRC_ADDR1_POS]   = OWN_DEVICE_ADDRESS1;
	Radio.TxPacket.Buf[RADIO_SRC_ADDR2_POS]   = OWN_DEVICE_ADDRESS2;
	Radio.TxPacket.Buf[RADIO_SRC_ADDR3_POS]   = OWN_DEVICE_ADDRESS3;
	Radio.TxPacket.Buf[RADIO_DST_ADDR0_POS]   = DST_DEVICE_ADDRESS0;
	Radio.TxPacket.Buf[RADIO_DST_ADDR1_POS]   = DST_DEVICE_ADDRESS1;
	Radio.TxPacket.Buf[RADIO_DST_ADDR2_POS]   = DST_DEVICE_ADDRESS2;
	Radio.TxPacket.Buf[RADIO_DST_ADDR3_POS]   = DST_DEVICE_ADDRESS3;
	Radio.TxPacket.Buf[RADIO_PACKET_INFO_POS] = 0x00;
	Radio.TxPacket.Buf[RADIO_DATA_LEN_POS]    = len;
	Radio.TxPacket.Buf[RADIO_CHECKSUM_POS]    = 0x00;
	Radio.TxPacket.CRC16 = ARQ_CRC16_Calculate_Block(Radio.TxPacket.Buf, 0, RADIO_CRC_CALC_LEN);
	Radio.TxPacket.Buf[RADIO_CRC16L_POS]      = Radio.TxPacket.CRC16 & 0xFF;
	Radio.TxPacket.Buf[RADIO_CRC16H_POS]      = Radio.TxPacket.CRC16 >> 8;
	Radio.TxPacket.Checksum8 = ARQ_Checksum8_Calculate_Block(Radio.TxPacket.Buf, 0, RADIO_PACKET_LEN);
	Radio.TxPacket.Buf[RADIO_CHECKSUM_POS]    = Radio.TxPacket.Checksum8;
	
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
	uint8_t  chksm = 0xFF, sts  = SUCCESSFUL;
	uint16_t crc = 0;
	Radio.RxPacket.ChksmSts = FALSE;
	Radio.RxPacket.CRCSts = FALSE;
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
		//clear crc event
		NRF_RADIO->EVENTS_CRCOK = 0;
		
		//extract source address
		Radio.RxPacket.SrcAddr   = Radio.RxPacket.Buf[RADIO_SRC_ADDR3_POS];
		Radio.RxPacket.SrcAddr <<= 8;
		Radio.RxPacket.SrcAddr  |= Radio.RxPacket.Buf[RADIO_SRC_ADDR2_POS];
		Radio.RxPacket.SrcAddr <<= 8;
		Radio.RxPacket.SrcAddr  |= Radio.RxPacket.Buf[RADIO_SRC_ADDR1_POS];
		Radio.RxPacket.SrcAddr <<= 8;
		Radio.RxPacket.SrcAddr  |= Radio.RxPacket.Buf[RADIO_SRC_ADDR0_POS];
		
		//extract destination address
		Radio.RxPacket.DstAddr   = Radio.RxPacket.Buf[RADIO_DST_ADDR3_POS];
		Radio.RxPacket.DstAddr <<= 8;
		Radio.RxPacket.DstAddr  |= Radio.RxPacket.Buf[RADIO_DST_ADDR2_POS];
		Radio.RxPacket.DstAddr <<= 8;
		Radio.RxPacket.DstAddr  |= Radio.RxPacket.Buf[RADIO_DST_ADDR1_POS];
		Radio.RxPacket.DstAddr <<= 8;
		Radio.RxPacket.DstAddr  |= Radio.RxPacket.Buf[RADIO_DST_ADDR0_POS];
		
		//extract packet info
		Radio.RxPacket.PacketInfo = Radio.RxPacket.Buf[RADIO_PACKET_INFO_POS];
		
		//extract data len
		Radio.RxPacket.Length = Radio.RxPacket.Buf[RADIO_DATA_LEN_POS];
		
		//extract checksum
		Radio.RxPacket.Checksum8 = Radio.RxPacket.Buf[RADIO_CHECKSUM_POS];
		
		//extract crc
		Radio.RxPacket.CRC16  = Radio.RxPacket.Buf[RADIO_CRC16H_POS];
		Radio.RxPacket.CRC16<<= 8;
		Radio.RxPacket.CRC16 |= Radio.RxPacket.Buf[RADIO_CRC16L_POS];
		
		chksm = ARQ_Checksum8_Calculate_Block(Radio.RxPacket.Buf, 0, RADIO_PACKET_LEN);
		if(chksm == 0){
			Radio.RxPacket.ChksmSts = TRUE;
		}
		
		Radio.RxPacket.Buf[RADIO_CHECKSUM_POS] = 0;
		crc = ARQ_CRC16_Calculate_Block(Radio.RxPacket.Buf, 0, RADIO_CRC_CALC_LEN);
		if(crc == Radio.RxPacket.CRC16){
			Radio.RxPacket.CRCSts = TRUE;
		}
	}
	return sts;
}

uint8_t Radio_Tx_Get_Ack(uint8_t *buf, uint8_t len){
	uint8_t  sts = FAILED;
	if(Radio_Tx(buf, len) == SUCCESSFUL){
		sts = Radio_Rx(500);
		if( (sts == SUCCESSFUL) && (Radio.RxPacket.DstAddr == Radio.TxPacket.SrcAddr) ){
			return SUCCESSFUL;
		}
		return FAILED;
	}
	return sts;
}
	

uint8_t Radio_Rx_Send_Ack(int32_t timeout){
	uint16_t crc = 0, chksm = 0, sts = FALSE;
	
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
	
	sts = TRUE;
	
	if( (Radio.RxPacket.DstAddr == Radio.TxPacket.SrcAddr) && (sts == TRUE) ){
		crc = ARQ_CRC16_Calculate_Block(Radio.RxPacket.Buf, 0, RADIO_CRC_CALC_LEN);
		if(crc == Radio.RxPacket.CRC16){
			Radio.RxPacket.CRCSts = TRUE;
			chksm = ARQ_Checksum8_Calculate_Block(Radio.RxPacket.Buf, 0, RADIO_PACKET_LEN);
			//Radio_Tx_Copy_Dst_Addr();
	    //Radio_Tx();
		  //Radio_Tx_Reload_Dst_Addr();
			//if( (Radio.RxPacket.PID == Radio.RxPacket.LastPID) && (Radio.RxPacket.LastSender == Radio.RxPacket.SrcAddr) ){
			//	return FALSE;
			//}
			//Radio.RxPacket.LastPID = Radio.RxPacket.PID;
			//Radio.RxPacket.LastSender = Radio.RxPacket.SrcAddr;
		}
		else{
			Radio.RxPacket.CRCSts = FALSE;
			return FALSE;
		}
	}
	else{
		return FALSE;
	}
		
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	return TRUE;
}


void Radio_Init(void){
	Radio_Struct_Init();
	Radio_Active();
	Radio_Power_Down();
}






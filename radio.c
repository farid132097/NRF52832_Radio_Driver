

/*
 * File:   radio.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver : RADIO Library
 * Created : December 15, 2024, 9:30 PM
 * Last Modified : 28 Mar, 2025, Rev2.2
 *
 */

#include "nrf.h"
#include "radio.h"
#include "cdefs.h"
#include "timeout.h"

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
	uint8_t  FaildAttempts;
	uint8_t  Reserved2[5];
	
	packet_t TxPacket;
	packet_t RxPacket;
}radio_t;

static radio_t Radio;
	

void Radio_Struct_Init(void){
	Radio.RegInit           = INCOMPLETE;
	Radio.RetryEnable       = TRUE;
	Radio.FaildAttempts     = 0;
	Radio.TxPacket.Length   = 0;
	Radio.TxPacket.PID      = 0;
	Radio.TxPacket.SrcAddr  = OWN_DEV_ADDRESS_QWORD_H;
	Radio.TxPacket.SrcAddr<<=32;
	Radio.TxPacket.SrcAddr |= OWN_DEV_ADDRESS_QWORD_L;
	Radio.TxPacket.DstAddr  = REC_DEV_ADDRESS_QWORD_H;
	Radio.TxPacket.DstAddr<<= 32;
	Radio.TxPacket.DstAddr |= REC_DEV_ADDRESS_QWORD_L;
	
	Radio.TxPacket.Buf[0]  = Radio.TxPacket.Length;
	Radio.TxPacket.Buf[1]  = Radio.TxPacket.PID;
	
	//Load 64 Bit Source Address
	Radio.TxPacket.Buf[2]  = (Radio.TxPacket.SrcAddr >> 56) & 0xFF;
	Radio.TxPacket.Buf[3]  = (Radio.TxPacket.SrcAddr >> 48) & 0xFF;
	Radio.TxPacket.Buf[4]  = (Radio.TxPacket.SrcAddr >> 40) & 0xFF;
	Radio.TxPacket.Buf[5]  = (Radio.TxPacket.SrcAddr >> 32) & 0xFF;
	Radio.TxPacket.Buf[6]  = (Radio.TxPacket.SrcAddr >> 24) & 0xFF;
	Radio.TxPacket.Buf[7]  = (Radio.TxPacket.SrcAddr >> 16) & 0xFF;
	Radio.TxPacket.Buf[8]  = (Radio.TxPacket.SrcAddr >>  8) & 0xFF;
	Radio.TxPacket.Buf[9]  = (Radio.TxPacket.SrcAddr >>  0) & 0xFF;
	
	//Load 64 Bit Destination Address
	Radio.TxPacket.Buf[10] = (Radio.TxPacket.DstAddr >> 56) & 0xFF;
	Radio.TxPacket.Buf[11] = (Radio.TxPacket.DstAddr >> 48) & 0xFF;
	Radio.TxPacket.Buf[12] = (Radio.TxPacket.DstAddr >> 40) & 0xFF;
	Radio.TxPacket.Buf[13] = (Radio.TxPacket.DstAddr >> 32) & 0xFF;
	Radio.TxPacket.Buf[14] = (Radio.TxPacket.DstAddr >> 24) & 0xFF;
	Radio.TxPacket.Buf[15] = (Radio.TxPacket.DstAddr >> 16) & 0xFF;
	Radio.TxPacket.Buf[16] = (Radio.TxPacket.DstAddr >>  8) & 0xFF;
	Radio.TxPacket.Buf[17] = (Radio.TxPacket.DstAddr >>  0) & 0xFF;
	
	
	//Clear Only Data Sections of Tx Buf
	for(uint8_t i=RADIO_FRAME_USER_DATA_POS; i<RADIO_FRAME_BUF_SIZE; i++){
		Radio.TxPacket.Buf[i] = 0;
	}
	
	//Clear All Rx Buf
	for(uint8_t i=0; i<RADIO_FRAME_BUF_SIZE; i++){
		Radio.RxPacket.Buf[i] = 0;
	}
}

void Radio_Tx_Set_Dst_Addr(uint64_t dst_addr){
	//Optimize using loops and load LSByte First
	Radio.TxPacket.DstAddr = dst_addr;
	Radio.TxPacket.Buf[10] = (Radio.TxPacket.DstAddr >> 56) & 0xFF;
	Radio.TxPacket.Buf[11] = (Radio.TxPacket.DstAddr >> 48) & 0xFF;
	Radio.TxPacket.Buf[12] = (Radio.TxPacket.DstAddr >> 40) & 0xFF;
	Radio.TxPacket.Buf[13] = (Radio.TxPacket.DstAddr >> 32) & 0xFF;
	Radio.TxPacket.Buf[14] = (Radio.TxPacket.DstAddr >> 24) & 0xFF;
	Radio.TxPacket.Buf[15] = (Radio.TxPacket.DstAddr >> 16) & 0xFF;
	Radio.TxPacket.Buf[16] = (Radio.TxPacket.DstAddr >>  8) & 0xFF;
	Radio.TxPacket.Buf[17] = (Radio.TxPacket.DstAddr >>  0) & 0xFF;
}

void Radio_Tx_Reload_Dst_Addr(void){
	Radio.TxPacket.Buf[10] = (Radio.TxPacket.DstAddr >> 56) & 0xFF;
	Radio.TxPacket.Buf[11] = (Radio.TxPacket.DstAddr >> 48) & 0xFF;
	Radio.TxPacket.Buf[12] = (Radio.TxPacket.DstAddr >> 40) & 0xFF;
	Radio.TxPacket.Buf[13] = (Radio.TxPacket.DstAddr >> 32) & 0xFF;
	Radio.TxPacket.Buf[14] = (Radio.TxPacket.DstAddr >> 24) & 0xFF;
	Radio.TxPacket.Buf[15] = (Radio.TxPacket.DstAddr >> 16) & 0xFF;
	Radio.TxPacket.Buf[16] = (Radio.TxPacket.DstAddr >>  8) & 0xFF;
	Radio.TxPacket.Buf[17] = (Radio.TxPacket.DstAddr >>  0) & 0xFF;
}


void Radio_Tx_Copy_Dst_Addr(void){
	for(uint8_t i=0; i<8; i++){
		//For Ack packet, Destination of TX device will be Source address of the RX device
		Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS+i] = Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS+i];
	}
}

void Radio_Tx_Set_Len(uint8_t len){
	Radio.TxPacket.Buf[0] = len;
	Radio.TxPacket.Length = len;
}

void Radio_Tx_Set_PID(uint8_t pid){
	Radio.TxPacket.Buf[1] = pid;
	Radio.TxPacket.PID    = pid;
}

void Radio_Len_PID_Update(void){
	Radio.TxPacket.Buf[0] = Radio.TxPacket.Length;
	Radio.TxPacket.Buf[1] = Radio.TxPacket.PID;
}

//Should be used after each data packet reception
void Radio_Tx_Clear_Data_Buf(void){
	for(uint8_t i = RADIO_FRAME_USER_DATA_POS; i<RADIO_FRAME_BUF_SIZE; i++){
		Radio.TxPacket.Buf[i] = 0;
	}
}


void Radio_Tx_Set_Data_Buf(uint8_t index, uint8_t data){
	if(index < RADIO_FRAME_USER_DATA_SIZE){
	  Radio.TxPacket.Buf[index + RADIO_FRAME_USER_DATA_POS] = data;
	}
}

uint64_t Radio_Rx_Extract_SrcAddr(void){
	uint8_t i;
	uint64_t temp = 0;
	for(i=0; i<8; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + i];
	}
	Radio.RxPacket.SrcAddr = temp;
	return temp;
}


uint64_t Radio_Rx_Extract_DstAddr(void){
	uint8_t i;
	uint64_t temp = 0;
	for(i=0; i<8; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + i];
	}
	Radio.RxPacket.DstAddr = temp;
	return temp;
}


uint8_t Radio_Rx_Get_Data_Buf(uint8_t index){
	return Radio.RxPacket.Buf[index + RADIO_FRAME_USER_DATA_POS];
}


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
	  NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_Pos4dBm;
	  NRF_RADIO->FREQUENCY = 10;
	  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_2Mbit<<RADIO_MODE_MODE_Pos);
	  NRF_RADIO->PREFIX0   = 0x11223344;
	  NRF_RADIO->BASE0     = 0x11111111;
	  NRF_RADIO->TXADDRESS   = 0;
	  NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->PCNF0    = 0;
	  NRF_RADIO->PCNF1    = (36 <<RADIO_PCNF1_MAXLEN_Pos)  |
	                        (32 <<RADIO_PCNF1_STATLEN_Pos) |
	                        (4  <<RADIO_PCNF1_BALEN_Pos)   |
	                        (RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos);
	  NRF_RADIO->SHORTS   = 0x00000000;
	  NRF_RADIO->CRCCNF   = RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos;
	  NRF_RADIO->MODECNF0 = (1<<9)|(1<<0);
	  NRF_RADIO->CRCINIT  = 0xFFFFFF;
	  NRF_RADIO->CRCPOLY  = 0x11021;
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
		Radio.RxPacket.Length = Radio.RxPacket.Buf[RADIO_FRAME_LEN_POS];
		if(Radio.RxPacket.Length > RADIO_FRAME_USER_DATA_SIZE){
			Radio.RxPacket.Length = RADIO_FRAME_USER_DATA_SIZE;
		}
		Radio.RxPacket.PID = Radio.RxPacket.Buf[RADIO_FRAME_PID_POS];
		Radio_Rx_Extract_SrcAddr();
		Radio_Rx_Extract_DstAddr();
		Radio.RxPacket.CRC16   = Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS];
		Radio.RxPacket.CRC16 <<= 8;
		Radio.RxPacket.CRC16  |= Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS+1];
	}
	
	return sts;
}

	
uint8_t Radio_Tx_Ack(void){
	uint8_t  sts = FAILED;
	if(Radio_Tx() == SUCCESSFUL){
		sts = Radio_Rx(150);
    //Ack sent from the destination address to src address
		if( (sts == SUCCESSFUL) && (Radio.RxPacket.DstAddr == Radio.TxPacket.SrcAddr) ){
			return SUCCESSFUL;
		}
		return FAILED;
	}
	return sts;
}
	

uint8_t Radio_Rx_Ack(int32_t timeout){
	uint16_t crc = 0;
	
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
	
	if( Radio.RxPacket.DstAddr == Radio.TxPacket.SrcAddr ){
		Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS]   = 0;
		Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS+1] = 0;
		crc = Radio_CRC_Calculate_Block(Radio.RxPacket.Buf, 0, RADIO_FRAME_USER_DATA_POS + Radio.RxPacket.Length);
		if(crc == Radio.RxPacket.CRC16){
			Radio.RxPacket.CRCSts = TRUE;
			Radio_Tx_Copy_Dst_Addr();
	    Radio_Tx();
		  Radio_Tx_Reload_Dst_Addr();
			if(Radio.RxPacket.PID == Radio.RxPacket.LastPID){
				return FALSE;
			}
			Radio.RxPacket.LastPID = Radio.RxPacket.PID;
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

uint8_t Radio_Tx_Packet(uint8_t *buf, uint8_t len){
	uint8_t  cnt = 0;
	uint16_t crc = 0;
	
	if(len > RADIO_FRAME_USER_DATA_SIZE){
		len = RADIO_FRAME_USER_DATA_SIZE;
	}
	Radio.TxPacket.Length = len;
	Radio.TxPacket.PID++;
	Radio_Len_PID_Update();
	for(uint8_t i = RADIO_FRAME_USER_DATA_POS; i < (RADIO_FRAME_USER_DATA_POS + len); i++){
		Radio.TxPacket.Buf[i] = buf[cnt];
		cnt++;
	}
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS] = 0;
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS+1] = 0;
	crc = Radio_CRC_Calculate_Block(Radio.TxPacket.Buf, 0, RADIO_FRAME_USER_DATA_POS+len);
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS] = (crc>>8) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS+1] = crc & 0xFF;
	
	if(Radio.RetryEnable == TRUE){
	  for(cnt = 0; cnt < RADIO_TX_FAILED_RETRY; cnt++){
		  if(Radio_Tx_Ack() == SUCCESSFUL){
				Radio.FaildAttempts = 0;
			  return SUCCESSFUL;
		  }
		  else{
			  Timeout_Delay_us(100);
		  }
	  }
		Radio.FaildAttempts++;
		if(Radio.FaildAttempts >= RADIO_TX_RETRY_DIS_FAIL_ATT){
			Radio.FaildAttempts = 0;
			Radio.RetryEnable = FALSE;
		}
  }
	else{
		//Transmit packet once
		if(Radio_Tx_Ack()){
			//If ack received, enable retry
			Radio.RetryEnable = TRUE;
			return TRUE;
		}
		return FALSE;
	}
	
	return FAILED;
}


uint64_t Radio_Rx_SrcAddr_Get(void){
	return Radio.RxPacket.SrcAddr;
}

uint64_t Radio_Rx_DstAddr_Get(void){
	return Radio.RxPacket.DstAddr;
}

uint16_t Radio_Rx_CRC16_Get(void){
	return Radio.RxPacket.CRC16;
}

uint8_t Radio_Rx_CRCSts_Get(void){
	return Radio.RxPacket.CRCSts;
}

uint8_t Radio_Rx_RetryEn_Get(void){
	return Radio.RetryEnable;
}


void Radio_Init(void){
	Radio_Struct_Init();
	Radio_Reg_Init();
	Radio_Mode_Disable();
	
	#ifdef RADIO_DEBUG_ENABLED
	#warning Radio debug enabled
	#endif
}









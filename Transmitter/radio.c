

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


//Max retry if tx is failed
#define  RADIO_TX_FAILED_RETRY       ( 5U)

//Tx retry will be turned off if consecutive
//attempts are failed (including retry)
#define  RADIO_TX_RETRY_DIS_FAIL_ATT ( 5U)



#define  RADIO_FRAME_LEN_POS         ( 0U)
#define  RADIO_FRAME_PID_POS         ( 1U)
#define  RADIO_FRAME_SRC_ADDR_POS    ( 2U)
#define  RADIO_FRAME_DST_ADDR_POS    ( 6U)
#define  RADIO_FRAME_CRC16_POS       (10U)

#define  RADIO_FRAME_LEN_SIZE        ( 1U)
#define  RADIO_FRAME_PID_SIZE        ( 1U)
#define  RADIO_FRAME_SRC_ADDR_SIZE   ( 4U)
#define  RADIO_FRAME_DST_ADDR_SIZE   ( 4U)
#define  RADIO_FRAME_CRC16_SIZE      ( 2U)

#define  RADIO_FRAME_BUF_SIZE        (20U)

#define  RADIO_FRAME_USER_DATA_POS   (12U)
#define  RADIO_FRAME_USER_DATA_SIZE  ( 8U)

//Frame Format:
//Len(1 Byte) + PID(1 Byte) + SrcAddr(4 Byte) + DstAddr(4 Byte) + CRC16(2 Byte) + Data(8 Byte)
//CRC16 Includes all bytes including CrcL and CrcH bytes considering 0x00

typedef struct packet_t{
	uint32_t LastSender;
	uint32_t SrcAddr;
	uint32_t DstAddr;
	uint16_t CRC16;
	uint8_t  CRCSts;
	uint8_t  Buf[RADIO_FRAME_BUF_SIZE];
}packet_t;


typedef struct radio_t{
	packet_t TxPacket;
	packet_t RxPacket;
	
	uint8_t  RegInit;
	uint8_t  RetryEnable;
	uint8_t  FaildAttempts;
	uint8_t  Reserved;
}radio_t;

static radio_t Radio;



void Radio_Struct_Init(void){
	Radio.RegInit           = INCOMPLETE;
	Radio.RetryEnable       = TRUE;
	Radio.FaildAttempts     = 0;
	Radio.TxPacket.SrcAddr  = OWN_DEV_ADDRESS_DWORD;
	Radio.TxPacket.DstAddr  = REC_DEV_ADDRESS_DWORD;
	
	//Clear Only Data Sections of Tx Buf
	for(uint8_t i = 0; i<RADIO_FRAME_BUF_SIZE; i++){
		Radio.TxPacket.Buf[i] = 0;
	}
	
	//Clear All Rx Buf
	for(uint8_t i = 0; i<RADIO_FRAME_BUF_SIZE; i++){
		Radio.RxPacket.Buf[i] = 0;
	}
	Radio.RxPacket.LastSender = 0x00000000;
	
	//Load 32 Bit Source Address
	Radio.TxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 0] = (OWN_DEV_ADDRESS_DWORD >> 24) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 1] = (OWN_DEV_ADDRESS_DWORD >> 16) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 2] = (OWN_DEV_ADDRESS_DWORD >>  8) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 3] = (OWN_DEV_ADDRESS_DWORD >>  0) & 0xFF;
	
	//Load 32 Bit Destination Address
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 0] = (REC_DEV_ADDRESS_DWORD >> 24) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 1] = (REC_DEV_ADDRESS_DWORD >> 16) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 2] = (REC_DEV_ADDRESS_DWORD >>  8) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 3] = (REC_DEV_ADDRESS_DWORD >>  0) & 0xFF;
}

void Radio_Tx_Set_Dst_Addr(uint32_t dst_addr){
	Radio.TxPacket.DstAddr = dst_addr;
	Radio_Tx_Reload_Dst_Addr();
}

void Radio_Tx_Reload_Dst_Addr(void){
	//Avoid loops for faster execution
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 0] = (Radio.TxPacket.DstAddr >> 24) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 1] = (Radio.TxPacket.DstAddr >> 16) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 2] = (Radio.TxPacket.DstAddr >>  8) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 3] = (Radio.TxPacket.DstAddr >>  0) & 0xFF;
}


void Radio_Tx_Copy_Dst_Addr(void){
	//Avoid loops for faster execution
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 0] = Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 0];
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 1] = Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 1];
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 2] = Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 2];
	Radio.TxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + 3] = Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + 3];
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

uint32_t Radio_Rx_Extract_SrcAddr(void){
	uint8_t i;
	uint32_t temp = 0;
	for(i = 0; i < 4; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[RADIO_FRAME_SRC_ADDR_POS + i];
	}
	Radio.RxPacket.SrcAddr = temp;
	return temp;
}


uint32_t Radio_Rx_Extract_DstAddr(void){
	uint8_t i;
	uint32_t temp = 0;
	for(i = 0; i < 4; i++){
		temp <<= 8;
		temp  |= Radio.RxPacket.Buf[RADIO_FRAME_DST_ADDR_POS + i];
	}
	Radio.RxPacket.DstAddr = temp;
	return temp;
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
	if(Radio.RegInit == INCOMPLETE){
	  NRF_RADIO->TXPOWER     = RADIO_TXPOWER_TXPOWER_Pos4dBm;
	  NRF_RADIO->FREQUENCY   = 10;
		//NRF_RADIO->MODE        = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);
	  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
		//NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos);
	  NRF_RADIO->PREFIX0     = 0x11223344;
	  NRF_RADIO->BASE0       = 0x11111111;
	  NRF_RADIO->TXADDRESS   = 0;
	  NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->PCNF0       = 0;
	  NRF_RADIO->PCNF1       = (20 <<RADIO_PCNF1_MAXLEN_Pos)   |
	                           (1 <<RADIO_PCNF1_STATLEN_Pos)   |
	                           (1  <<RADIO_PCNF1_BALEN_Pos)    |
	                           (RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos);
	  NRF_RADIO->SHORTS      = 0x00000000;
	  NRF_RADIO->CRCCNF      = RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos;
	  NRF_RADIO->MODECNF0    = (1<<9)|(1<<0);
	  NRF_RADIO->CRCINIT     = 0xFFFFFF;
	  NRF_RADIO->CRCPOLY     = 0x11021;
	  NRF_RADIO->EVENTS_ADDRESS = 0;
	  NRF_RADIO->EVENTS_PAYLOAD = 0;
		Radio.RegInit = COMPLETE;
	}
}

void Radio_Power_Enable(void){
	if( (NRF_RADIO->POWER & (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos)) == 0 ){
	  NRF_RADIO->POWER = 1;
		
		Timeout_Set_MicroSeconds(300);
	  while(NRF_RADIO->POWER == 0){
		  if(Timeout_Error_Assign(ERROR_RADIO_POWER_ENABLE_FAILED)){
			  break;
		  }
	  }
		Timeout_Clear_Events();
  }
}

void Radio_Power_Disable(void){
	if( NRF_RADIO->POWER & (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos) ){
	  NRF_RADIO->POWER = 0;
		
		Timeout_Set_MicroSeconds(300);
	  while(NRF_RADIO->POWER == 1){
		  if(Timeout_Error_Assign(ERROR_RADIO_POWER_DISABLE_FAILED)){
			  break;
		  }
	  }
		Timeout_Clear_Events();
  }
}

void Radio_Active(void){
	Radio_Power_Enable();
	Clock_HFCLK_Xtal_Start_Request();
	Radio_Reg_Init();
	Clock_HFCLK_Xtal_Wait_Until_Ready();
}

void Radio_Power_Down(void){
	Radio_Power_Disable();
	Radio_HFCLK_Stop();
	Radio.RegInit = INCOMPLETE;
}


void Radio_Mode_Disable(void){
	if( (NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Disabled ){
    NRF_RADIO->EVENTS_DISABLED = 0;
	  NRF_RADIO->TASKS_DISABLE = 1;
		
		Timeout_Set_MicroSeconds(300);
	  while(NRF_RADIO->EVENTS_DISABLED == 0){
	    if(Timeout_Error_Assign(ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED)){
			  break;
		  }
	  }
		Timeout_Clear_Events();
	}
}

void Radio_Mode_Tx(void){
	if( ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_TxIdle) || ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Tx) ){
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->TASKS_TXEN = 1;
		
		Timeout_Set_MicroSeconds(300);
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign(ERROR_RADIO_MODE_SWITCH_TX_FAILED)){
			  break;
		  }
    }
		Timeout_Clear_Events();
  }
}

void Radio_Mode_Rx(void){
	if( ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_RxIdle) || ((NRF_RADIO->STATE & RADIO_STATE_STATE_Msk) != RADIO_STATE_STATE_Rx) ){
	  NRF_RADIO->EVENTS_READY =0;
	  NRF_RADIO->TASKS_RXEN = 1;
		
		Timeout_Set_MicroSeconds(300);
	  while(NRF_RADIO->EVENTS_READY == 0){
		  if(Timeout_Error_Assign(ERROR_RADIO_MODE_SWITCH_RX_FAILED)){
			  break;
		  }
    }
		Timeout_Clear_Events();
		
	  NRF_RADIO->EVENTS_READY = 0;
	  NRF_RADIO->EVENTS_CRCOK = 0;
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



uint8_t Radio_Tx(void){
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.TxPacket.Buf;
	Radio_Start_Task(450);
	
	Radio_Mode_Rx();
	if(Timeout_Error_Get() != NULL){
		return FALSE;
	}
	return TRUE;
}

void Radio_Tx_Low_Power(void){
	Radio_Power_Enable();
	Clock_HFCLK_Xtal_Start_Request();
	Radio_Reg_Init();
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.TxPacket.Buf;
	Radio_Mode_Tx();
	Clock_HFCLK_Xtal_Wait_Until_Ready();
	Radio_Start_Task(450);
}


uint8_t Radio_Rx(int32_t timeout){
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
	
	return SUCCESSFUL;
}

	
uint8_t Radio_Tx_Ack(void){
	uint8_t  sts = FAILED;
	if( SUCCESSFUL){
		Radio_Tx_Low_Power();
		sts = Radio_Rx(200);
    //Ack sent from the destination address to src address
		if( (sts == SUCCESSFUL) && (Radio.RxPacket.DstAddr == Radio.TxPacket.SrcAddr) ){
			return SUCCESSFUL;
		}
		return FAILED;
	}
	return sts;
}
	

uint8_t Radio_Rx_Ack(int32_t timeout){
	uint16_t crc = 0, sts = FALSE;
	
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
	
	//Check Valid Node Address before ack
	#ifdef ALLOW_ANY_SRC_ADDR_TO_SEND_DATA
	sts = TRUE;
	#else
	for(uint8_t i=0; i<ALLOWED_SRC_ADDRESSESS; i++){
		if(Radio.RxPacket.SrcAddr == allowed_node_addr[i]){
			sts = TRUE;
			break;
		}
	}
	#endif
	
	if( (Radio.RxPacket.DstAddr == Radio.TxPacket.SrcAddr) && (sts == TRUE) ){
		Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS]   = 0;
		Radio.RxPacket.Buf[RADIO_FRAME_CRC16_POS+1] = 0;
		crc = Radio_CRC_Calculate_Block(Radio.RxPacket.Buf, 0, RADIO_FRAME_USER_DATA_POS + Radio.RxPacket.Length);
		if(crc == Radio.RxPacket.CRC16){
			Radio.RxPacket.CRCSts = TRUE;
			Radio_Tx_Copy_Dst_Addr();
	    Radio_Tx();
		  Radio_Tx_Reload_Dst_Addr();
			if( (Radio.RxPacket.PID == Radio.RxPacket.LastPID) && (Radio.RxPacket.LastSender == Radio.RxPacket.SrcAddr) ){
				return FALSE;
			}
			Radio.RxPacket.LastPID = Radio.RxPacket.PID;
			Radio.RxPacket.LastSender = Radio.RxPacket.SrcAddr;
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
	
	Timeout_Error_Clear();
	
	Radio_Power_Enable();
	Clock_HFCLK_Xtal_Start_Request();
	Radio_Reg_Init();
	
	if(len > RADIO_FRAME_USER_DATA_SIZE){
		len = RADIO_FRAME_USER_DATA_SIZE;
	}
	Radio.TxPacket.Buf[RADIO_FRAME_LEN_POS] = len;
	Radio.TxPacket.Buf[RADIO_FRAME_PID_POS]++;
	
	for(uint8_t i = RADIO_FRAME_USER_DATA_POS; i < (RADIO_FRAME_USER_DATA_POS + len); i++){
		Radio.TxPacket.Buf[i] = buf[cnt];
		cnt++;
	}
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS] = 0;
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS+1] = 0;
	crc = Radio_CRC_Calculate_Block(Radio.TxPacket.Buf, 0, RADIO_FRAME_USER_DATA_POS + len);
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS] = (crc>>8) & 0xFF;
	Radio.TxPacket.Buf[RADIO_FRAME_CRC16_POS+1] = crc & 0xFF;
	
	NRF_RADIO->PACKETPTR = (uint32_t)Radio.TxPacket.Buf;
	Clock_HFCLK_Xtal_Wait_Until_Ready();
	Radio_Mode_Tx();
	Radio_Start_Task(1000);
	
	//Radio_Rx(200);
	return FAILED;
}


void Radio_Tx_Complete_Handler(void){
	if(Radio.TxComplete == TRUE){
		Radio.TxComplete = FALSE;
		Radio_Power_Down();
	}
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






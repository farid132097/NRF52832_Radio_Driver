

#ifndef  _RADIO_H_
#define  _RADIO_H_
#include "nrf.h"


//#define  OWN_DEVICE_ADDRESS    (0x00000010UL)
//#define  DST_DEVICE_ADDRESS    (0xFFFFFFFFUL)



enum radio_error_t{
	       ERROR_RADIO_NO_ERROR                   = 0x00,
	       ERROR_RADIO_HFCLK_START_FAILED         = 0x01,
	       ERROR_RADIO_HFCLK_STOP_FAILED          = 0x02,
	       ERROR_RADIO_POWER_ENABLE_FAILED        = 0x03,
	       ERROR_RADIO_POWER_DISABLE_FAILED       = 0x04,
	       ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED = 0x05,
	       ERROR_RADIO_MODE_SWITCH_TX_FAILED      = 0x06,
	       ERROR_RADIO_MODE_SWITCH_RX_FAILED      = 0x07,
	       ERROR_RADIO_TX_TASK_TIMEOUT            = 0x08,
	       ERROR_RADIO_RX_TASK_TIMEOUT            = 0x09,
	       ERROR_RADIO_CRC_NOT_OK                 = 0x0A,
	       ERROR_RADIO_DATA_TX_FAILED             = 0x0B,
	       ERROR_RADIO_TX_SUCCESS_NO_ACK          = 0x0C,
	       ERROR_RADIO_RNG_NUM_GENERATE_FAILED    = 0x0D
};

void     Radio_Struct_Init(void);
void     Radio_Power_Enable(void);
void     Radio_Power_Disable(void);
void     Radio_Active(void);
void     Radio_Power_Down(void);

void     Radio_Mode_Disable(void);
void     Radio_Mode_Tx(void);
void     Radio_Mode_Rx(void);

uint8_t  Radio_Tx(uint8_t *buf, uint8_t len);
uint8_t  Radio_Rx(uint32_t timeout);


uint8_t  Radio_Tx_Get_Ack(uint8_t *buf, uint8_t len);
uint8_t  Radio_Rx_Send_Ack(int32_t timeout);

uint8_t  Radio_Rx_Crc_Sts(void);
uint8_t  Radio_Rx_Checksum_Sts(void);
uint32_t Radio_Rx_Packet_Src_Addr(void);
uint32_t Radio_Rx_Packet_Dst_Addr(void);


void     Radio_Init(void);





#endif




/*
 * File:   radio.h
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver : NRF52832 RADIO Library
 * Created : December 15, 2024, 9:30 PM
 * Last Modified : 13 May, 2025, Rev2.4
 */


#ifndef  _RADIO_H_
#define  _RADIO_H_
#include "nrf.h"


/*
 * Device address for this node (TX)
 * Device address must not be    (0x0000000000000000ULL) or 
 *                               (0xFFFFFFFFFFFFFFFFULL)
 */
#define  OWN_DEV_ADDRESS_QWORD   (0xFFFFFFFFFFFFFFFFULL)



/*
 * Receiver address
 * Default Receiver address      (0xFFFFFFFFFFFFFFFFULL)
 */
#define  REC_DEV_ADDRESS_QWORD   (0xFFFFFFFFFFFFFFFFULL)


//Uncomment if any Node can send data to this device
#define  ALLOW_ANY_SRC_ADDR_TO_SEND_DATA


enum radio_error_t{
	       ERROR_RADIO_NO_ERROR                   = 0x00,
	       ERROR_RADIO_HFCLK_START_FAILED         = 0x01,
	       ERROR_RADIO_HFCLK_STOP_FAILED          = 0x02,
	       ERROR_RADIO_POWER_ENABLE_FAILED        = 0x03,
	       ERROR_RADIO_POWER_DISABLE_FAILED       = 0x04,
	       ERROR_RADIO_MODE_SWITCH_DISABLE_FAILED = 0x05,
	       ERROR_RADIO_MODE_SWITCH_TX_FAILED      = 0x06,
	       ERROR_RADIO_MODE_SWITCH_RX_FAILED      = 0x07,
	       ERROR_RADIO_TASK_TIMEOUT_OCCURED       = 0x08,
	       ERROR_RADIO_CRC_NOT_OK                 = 0x09,
	       ERROR_RADIO_DATA_TX_FAILED             = 0x0A,
	       ERROR_RADIO_TX_SUCCESS_NO_ACK          = 0x0B
};

void     Radio_Struct_Init(void);
void     Radio_Tx_Set_Dst_Addr(uint64_t dst_addr);
void     Radio_Tx_Reload_Dst_Addr(void);

void     Radio_Tx_Copy_Dst_Addr(void);
void     Radio_Tx_Set_Len(uint8_t len);
void     Radio_Tx_Set_PID(uint8_t pid);
void     Radio_Len_PID_Update(void);

void     Radio_Tx_Clear_Data_Buf(void);
void     Radio_Tx_Set_Data_Buf(uint8_t index, uint8_t data);
uint64_t Radio_Rx_Extract_SrcAddr(void);
uint64_t Radio_Rx_Extract_DstAddr(void);
uint8_t  Radio_Rx_Data_Buf_Get(uint8_t index);
uint8_t  Radio_Rx_Data_Len_Get(void);
uint8_t  Radio_Rx_Buf_Get(uint8_t index);
uint8_t  Radio_Rx_Len_Get(void);
uint16_t Radio_CRC_Calculate_Byte(uint16_t crc, uint8_t data);
uint16_t Radio_CRC_Calculate_Block(uint8_t *buf, uint8_t start, uint8_t end);

void     Radio_HFCLK_Start(void);
void     Radio_HFCLK_Stop(void);
void     Radio_Reg_Init(void);
void     Radio_Power_Enable(void);
void     Radio_Power_Disable(void);
void     Radio_Active(void);
void     Radio_Power_Down(void);
void     Radio_Mode_Disable(void);
void     Radio_Mode_Tx(void);
void     Radio_Mode_Rx(void);
void     Radio_Start_Task(int32_t delay);
uint8_t  Radio_Tx(void);
void     Radio_Tx_Low_Power(void);
uint8_t  Radio_Rx(int32_t timeout);
uint8_t  Radio_Tx_Ack(void);
uint8_t  Radio_Rx_Ack(int32_t timeout);
uint8_t  Radio_Tx_Packet(uint8_t *buf, uint8_t len);

uint64_t Radio_Rx_SrcAddr_Get(void);
uint64_t Radio_Rx_DstAddr_Get(void);
uint16_t Radio_Rx_CRC16_Get(void);
uint8_t  Radio_Rx_CRCSts_Get(void);
uint8_t  Radio_Rx_RetryEn_Get(void);

void     Radio_Init(void);



#endif



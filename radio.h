

#ifndef  _RADIO_H_
#define  _RADIO_H_
#include "nrf.h"

/*
 * Device address for this node (TX)
 * Device address must not be 0x0000000000000000 or 0xFFFFFFFFFFFFFFFF
 */
#define  OWN_DEV_ADDRESS_QWORD_H (0xF0000000UL)
#define  OWN_DEV_ADDRESS_QWORD_L (0x12345678UL)



/*
 * Receiver address
 * Receiver address must be 0xFFFFFFFFFFFFFFFF
 */
#define  REC_DEV_ADDRESS_QWORD_H (0xFFFFFFFFUL)
#define  REC_DEV_ADDRESS_QWORD_L (0xFFFFFFFFUL)



enum{
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
void     Radio_Tx_Set_Dst_Addr(uint32_t dstH, uint32_t dstL);
void     Radio_Tx_Load_Dst_Addr(void);
void     Radio_Tx_Copy_Dst_Addr(void);
void     Radio_Tx_Set_Len(uint8_t len);
void     Radio_Tx_Set_PID(uint8_t len);
void     Radio_Tx_Set_Buf(uint8_t index, uint8_t data);
void     Radio_Tx_Set_Data_Buf(uint8_t index, uint8_t data);
uint32_t Radio_Rx_Extract_DstH(void);
uint32_t Radio_Rx_Extract_DstL(void);
void     Radio_HFCLK_Start(void);
void     Radio_HFCLK_Stop(void);
void     Radio_Reg_Init(void);
void     Radio_Power_Disable(void);
void     Radio_Power_Enable(void);

void     Radio_Active(void);
void     Radio_Power_Down(void);

void     Radio_Mode_Disable(void);
void     Radio_Mode_Tx(void);
void     Radio_Mode_Rx(void);
void     Radio_Start_Task(int32_t delay);

uint8_t  Radio_Tx(void);
uint8_t  Radio_Rx(int32_t timeout);

uint8_t  Radio_Tx_Ack(void);
uint8_t  Radio_Rx_Ack(int32_t timeout);

void     Radio_Init(void);




#endif



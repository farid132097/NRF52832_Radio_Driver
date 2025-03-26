

#ifndef  _RADIO_H_
#define  _RADIO_H_
#include "nrf.h"

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
	ERROR_RADIO_CRC_NOT_OK                 = 0x09
};

void     Radio_Struct_Init(void);
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
uint8_t  Radio_Tx(uint8_t *buf, uint8_t len);
uint8_t  Radio_Rx(uint8_t *buf, int32_t timeout);
uint8_t  Radio_Tx_Ack(uint8_t *buf, uint8_t len);
uint8_t  Radio_Rx_Ack(uint8_t *buf, int32_t timeout);

void     Radio_Init(void);




#endif



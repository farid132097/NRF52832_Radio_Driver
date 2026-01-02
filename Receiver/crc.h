

#ifndef  _RADIO_H_
#define  _RADIO_H_
#include "nrf.h"


uint16_t  CRC_Calculate_Byte(uint16_t crc, uint8_t data);
uint16_t  CRC_Calculate_Block(uint8_t *buf, uint8_t start, uint8_t len);



#endif




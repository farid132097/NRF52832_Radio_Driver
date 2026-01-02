


#include "nrf.h"
#include "cdefs.h"
#include "crc.h"



uint16_t CRC_Calculate_Byte(uint16_t crc, uint8_t data){
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

uint16_t CRC_Calculate_Block(uint8_t *buf, uint8_t start, uint8_t len){
  uint16_t crc = 0;
  for(uint8_t i = start; i < len; i++){
    crc = CRC_Calculate_Byte(crc, buf[i]);
  }
  return crc;
}



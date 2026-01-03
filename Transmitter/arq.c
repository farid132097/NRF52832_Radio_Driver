


#include "nrf.h"
#include "cdefs.h"
#include "arq.h"

//Automatic Repeat Request (ARQ)
//For data integrity check


uint16_t ARQ_CRC16_Calculate_Byte(uint16_t crc, uint8_t data){
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

uint16_t ARQ_CRC16_Calculate_Block(uint8_t *buf, uint8_t start, uint8_t len){
  uint16_t crc = 0;
  for(uint8_t i = start; i < len; i++){
    crc = ARQ_CRC16_Calculate_Byte(crc, buf[i]);
  }
  return crc;
}

uint8_t   ARQ_Checksum8_Calculate_Block(uint8_t *buf, uint8_t start, uint8_t len){
	uint8_t chksm = 0;
  for(uint8_t i = start; i < len; i++){
    chksm += buf[i];
  }
  return (uint8_t)(0x100 - chksm);
}



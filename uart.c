

/*
 * File:   uart.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver -> UART Library
 * Created on December 12, 2024, 9:44 PM
 */

#include "nrf.h"
#include "uart.h"


#define  UART_DOUBLE_SPEED
#define  UART_ENABLE_TX    
//#define  UART_ENABLE_RX    
//#define  UART_ENABLE_RX_INT

#define  UART_BUFFER_SIZE            64U
#define  UART_RX_PCKT_CMPLT_DELAY    50U



//#define  UART_CRC_ENABLE     //Uncomment if packet validation by CRC is needed
//#define  UART_CRC_SUPPLIER     //Uncomment if CRC is by Supplier Protocol
#define  UART_CRC_XMODEM     //Uncomment if CRC is by X-MODEM Protocol



//Define Software Error Codes
#define  UART_RX_ERR_NO_ERR          0x00U
#define  UART_RX_ERR_FRAMING         0x01U
#define  UART_RX_ERR_OVERRUN         0x02U
#define  UART_RX_ERR_READ_INCOMPLETE 0x10U


typedef struct uart_timer_t{
  volatile uint8_t   Enabled;
  volatile uint8_t   ResetVal;
}uart_timer_t;

typedef struct uart_rx_packet_t{
  volatile uint16_t  CalculatedCRC;
  volatile uint16_t  ReceivedCRC;
  volatile uint8_t   CRCStatus;
  volatile uint8_t   DataAvailable;
  volatile uint8_t   DataReadComplete;
	volatile uint8_t   Reserved;
}uart_rx_packet_t;

typedef struct uart_t{
  volatile uint8_t   Error;
  uint8_t            Digits[8];
  uint8_t            InputNumDigits;

  volatile uint8_t   LastRxByte;
  volatile uint8_t   Buf[UART_BUFFER_SIZE];
	volatile uint8_t   Reserved;
  volatile uint16_t  BufSize;
  volatile uint16_t  BufIndex;
  
  uart_timer_t       Timer;
  
  uart_rx_packet_t   RxPacket;
}uart_t;


enum{
  UART_FALSE = 0,
  UART_TRUE  = 1,
  UART_NULL  = 0
};


static uart_t UART;






/*******************UART Structure Functions Start****************/

void UART_Struct_Init(void){
  UART.Error = 0;
  for(uint8_t i = 0; i < 8; i++){
    UART.Digits[i] = UART_NULL;
  }
  UART.InputNumDigits = UART_NULL;
  UART.LastRxByte = UART_NULL;
  UART.BufSize = UART_BUFFER_SIZE;
  UART.BufIndex = 0;
  for(uint8_t i = 0; i < UART.BufSize; i++){
    UART.Buf[i] = UART_NULL;
  }
}

void UART_RX_Packet_Struct_Init(void){
  UART.RxPacket.CalculatedCRC    = UART_NULL;
  UART.RxPacket.ReceivedCRC      = UART_NULL;
  UART.RxPacket.CRCStatus        = UART_FALSE;
  UART.RxPacket.DataAvailable    = UART_FALSE;
  UART.RxPacket.DataReadComplete = UART_TRUE;
}

/********************UART Structure Functions End*****************/









/*********************UART Init Functions Start******************/

void UART_Config_GPIO(void){
  //add gpio config
}

void UART_Config_Clock(void){
  //add clock config
}

void UART_Config_BAUD_Rate(uint32_t baud_rate){
  NRF_UARTE0->BAUDRATE = baud_rate;
}

void UART_Enable(void){
	if((NRF_UARTE0->ENABLE & UARTE_ENABLE_ENABLE_Msk) != UARTE_ENABLE_ENABLE_Enabled){
		NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled;
	}
}

void UART_Disable(void){
	if((NRF_UARTE0->ENABLE & UARTE_ENABLE_ENABLE_Msk) == UARTE_ENABLE_ENABLE_Enabled){
		NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
	}
}

void UART_Config_Tx(void){
  NRF_UARTE0->PSEL.TXD = UART_TX_PIN;
	NRF_UARTE0->PSEL.TXD|= UARTE_PSEL_TXD_CONNECT_Connected;
	UART_Enable();
}


void UART_Config_Rx(void){
  NRF_UARTE0->PSEL.RXD = UART_RX_PIN;
	NRF_UARTE0->PSEL.RXD|= UARTE_PSEL_RXD_CONNECT_Connected;
	UART_Enable();
}

void UART_Config_Rx_Interrupt(void){
  //add rx int config
}

void UART_Clear_Interrupt_Flag(void){
  //Clear flag if necessary
}

void UART_Tx_Byte(uint8_t val){
  uint8_t tx_ptr[2];
	tx_ptr[0] = val;
	
	NRF_UARTE0->TXD.PTR = (uint32_t)tx_ptr;
	NRF_UARTE0->TXD.MAXCNT = 1;
	NRF_UARTE0->EVENTS_ENDTX = 0;
	NRF_UARTE0->TASKS_STARTTX = 1;
	while(NRF_UARTE0->EVENTS_ENDTX == 0){
		//add timeout
	}
}

uint8_t UART_Rx_Byte(void){
  volatile uint8_t val = 0;
  //rx byte
  return val;
}



//add uart int handler vector
//call UART_ISR_Handler() inside ISR

/**********************UART Init Functions End*******************/









/********************UART Timer Functions Start*****************/

void UART_Timer_Struct_Init(void){
  UART.Timer.Enabled = UART_FALSE;
  UART.Timer.ResetVal = UART_NULL;
}

void UART_Timer_Init(void){
  //config uart timer for auto packet validation
	//calculate UART.Timer.ResetVal if overflow int is used
  #if UART_RX_PCKT_CMPLT_DELAY<20U
    #warning UART_RX_PCKT_CMPLT_DELAY value < 20
  #endif
}

void UART_Timer_Enable(void){
  //uart timer enable
}

void UART_Timer_Disable(void){ 
  //uart timer disable
}

uint8_t UART_Timer_Get_Status(void){
  return UART.Timer.Enabled;
}

uint16_t UART_Timer_Get_Val(void){
  //return current timer val
	return 0;
}


void UART_Timer_Value_Reset(void){
	//reset timer val if compare mode selected
	//set timer val to UART.Timer.ResetVal if Overflow int is used
}

void UART_Timer_Clear_Interrupt_Flag(void){
  //Clear flag if necessary
}

//add uart timer int handler vector
//call UART_Timer_ISR_Handler() inside ISR




/*********************UART Timer Functions End******************/









/********************Buffer Tx Functions Start*******************/

void UART_Tx_Buf(uint8_t *data, uint8_t len){
  for(uint16_t i = 0; i < len; i++){
	UART_Tx_Byte( data[i] );
  }
}

/*********************Buffer Tx Functions End********************/









/*******************End Char Functions Start******************/

void UART_Tx_NL(void){
  UART_Tx_Byte('\r');
  UART_Tx_Byte('\n');
}

void UART_Tx_SP(void){
  UART_Tx_Byte(' ');
}

void UART_Tx_CM(void){
  UART_Tx_Byte(',');
}

/*******************End Char Functions End*******************/









/*********************Text Functions Start*******************/

void UART_Tx_Text(char *str){
  uint8_t i = 0;
  while(str[i] != '\0'){
    UART_Tx_Byte(str[i]);
    i++;
  }
}

void UART_Tx_Text_NL(char *str){
  UART_Tx_Text(str);
  UART_Tx_NL();
}

void UART_Tx_Text_SP(char *str){
  UART_Tx_Text(str);
  UART_Tx_SP();
}

void UART_Tx_Text_CM(char *str){
  UART_Tx_Text(str);
  UART_Tx_CM();
}

/*********************Text Functions End********************/









/*********************Number Functions Start********************/

void UART_Determine_Digit_Numbers(uint32_t num){
  uint8_t i = 0;
  if(num == 0){
    UART.Digits[0] = 0;
    UART.InputNumDigits = 1;
  }else{
    while(num != 0){
      UART.Digits[i] = num%10;
      num /= 10;
      i++;
    }
	UART.InputNumDigits = i;
  }
}

void UART_Tx_Number_Digits(void){
  for(uint8_t i = UART.InputNumDigits; i > 0; i--){
    uint8_t temp = i;
    temp -= 1;
    temp  = UART.Digits[temp];
    temp += 48;
    UART_Tx_Byte(temp);
  }
}

void UART_Tx_Number(int32_t num){
  if(num < 0){
    UART_Tx_Byte('-');
	  num = -num;
  }
  UART_Determine_Digit_Numbers((uint32_t)num);
  UART_Tx_Number_Digits();
}

void UART_Tx_Number_Hex(uint32_t val){
  uint16_t hex_digit, index = 0, loop_counter = 0;
  if(val <= 0xFF){
    index = 8;
    loop_counter = 2;
  }else if(val <= 0xFFFF){
    index = 16;
    loop_counter = 4;     
  }else{
    index = 32;
    loop_counter = 8;
  }
  UART_Tx_Byte('0');
  UART_Tx_Byte('x');
  for(uint8_t i = 0; i < loop_counter; i++){
	index -= 4;
	hex_digit = (uint8_t)((val>>index) & 0x0F);
	if(hex_digit > 9){
	  hex_digit += 55;
	}
	else{
	  hex_digit += 48;
	}
	UART_Tx_Byte((uint8_t)hex_digit);
  }
}

void UART_Tx_Number_Bin(uint32_t val){
  uint8_t loop_counter = 0;
  if(val <= 0xFF){
    loop_counter = 7;
  }else if(val <= 0xFFFF){
    loop_counter = 15;     
  }else{
    loop_counter = 31;
  }
  
  UART_Tx_Byte('0');
  UART_Tx_Byte('b');
  for(int i = loop_counter; i >= 0; i--){
    if( (val>>i) & 1){
      UART_Tx_Byte( 49 );   
    }else{
      UART_Tx_Byte( 48 );         
    }
  }
}

/*********************Number Functions End*********************/









/************Number with End Char Functions Start**************/

void UART_Tx_Number_NL(int32_t num){
  UART_Tx_Number(num);
  UART_Tx_NL();
}

void UART_Tx_Number_SP(int32_t num){
  UART_Tx_Number(num);
  UART_Tx_SP();
}

void UART_Tx_Number_CM(int32_t num){
  UART_Tx_Number(num);
  UART_Tx_CM();
}

/*************Number with End Char Functions End***************/









/**********Hex Number with End Char Functions Start************/

void UART_Tx_Number_Hex_NL(uint32_t num){
  UART_Tx_Number_Hex(num);
  UART_Tx_NL();
}

void UART_Tx_Number_Hex_SP(uint32_t num){
  UART_Tx_Number_Hex(num);
  UART_Tx_SP();
}

void UART_Tx_Number_Hex_CM(uint32_t num){
  UART_Tx_Number_Hex(num);
  UART_Tx_CM();
}

/***********Hex Number with End Char Functions End*************/









/**********Bin Number with End Char Functions Start************/

void UART_Tx_Number_Bin_NL(uint32_t num){
  UART_Tx_Number_Bin(num);
  UART_Tx_NL();
}

void UART_Tx_Number_Bin_SP(uint32_t num){
  UART_Tx_Number_Bin(num);
  UART_Tx_SP();
}

void UART_Tx_Number_Bin_CM(uint32_t num){
  UART_Tx_Number_Bin(num);
  UART_Tx_CM();
}

/***********Bin Number with End Char Functions End*************/







/************Number with Parameter Functions Start*************/

void UART_Tx_Parameter_NL(char *name, int32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_NL(num);
}

void UART_Tx_Parameter_SP(char *name, int32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_SP(num);
}

void UART_Tx_Parameter_CM(char *name, int32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_CM(num);
}

/*************Number with Parameter Functions End**************/









/**********Hex Number with Parameter Functions Start***********/

void UART_Tx_Parameter_Hex_NL(char *name, uint32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Hex_NL(num);
}

void UART_Tx_Parameter_Hex_SP(char *name, uint32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Hex_SP(num);
}

void UART_Tx_Parameter_Hex_CM(char *name, uint32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Hex_CM(num);
}

/***********Hex Number with Parameter Functions End************/









/**********Bin Number with Parameter Functions Start***********/

void UART_Tx_Parameter_Bin_NL(char *name, uint32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Bin_NL(num);
}

void UART_Tx_Parameter_Bin_SP(char *name, uint32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Bin_SP(num);
}

void UART_Tx_Parameter_Bin_CM(char *name, uint32_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Bin_CM(num);
}

/***********Bin Number with Parameter Functions End************/









/*******************UART Buffer Functions Start***************/

void UART_Buf_Flush(void){
  for(uint8_t i = 0; i < UART_BUFFER_SIZE; i++){
	UART.Buf[i] = 0;
  }
  UART.BufIndex = 0;
}

uint8_t UART_Buf_Get(uint16_t index){
  return UART.Buf[index];
}

uint16_t UART_Buf_Get_Index(void){
  return UART.BufIndex;
}

/********************UART Buffer Functions End****************/









/*******************UART Data Functions Start****************/

uint8_t UART_Data_Available(void){
  return UART.RxPacket.DataAvailable;
}

uint16_t UART_Data_Len_Get(void){
  return UART_Buf_Get_Index();
}

uint16_t UART_Data_Calculated_CRC_Get(void){
  return UART.RxPacket.CalculatedCRC;
}

uint16_t UART_Data_Received_CRC_Get(void){
  return UART.RxPacket.ReceivedCRC;
}

uint8_t UART_Data_CRC_Status_Get(void){
  return UART.RxPacket.CRCStatus;
}

uint8_t UART_Data_Read_Complete_Status(void){
  return UART.RxPacket.DataReadComplete;
}

void UART_Data_Clear_Available_Flag(void){
  UART.RxPacket.DataAvailable = UART_FALSE;
}

void UART_Data_Clear_Read_Complete_Flag(void){
  UART_Buf_Flush();
  UART.RxPacket.DataReadComplete = UART_TRUE;
}


void UART_Data_Copy_Buf(uint8_t *buf){
  for(uint16_t i = 0; i < UART_Data_Len_Get(); i++){
	buf[i] = UART_Buf_Get(i);
  }
}


void UART_Data_Print_Buf(void){
  for(uint16_t i = 0; i < UART_Data_Len_Get(); i++){
	UART_Tx_Byte( UART_Buf_Get(i) );
  }
  UART_Tx_NL();
}

/********************UART Data Functions End*****************/









/******************Error Code Functions Start****************/

uint8_t UART_Error_Code_Get(void){
  return UART.Error;
}

void UART_Error_Code_Clear(void){
  UART.Error = 0;
}

/******************Error Code Functions End******************/









/***************UART ISR Handler Functions Start************/

void UART_ISR_Handler(void){
  UART_Clear_Interrupt_Flag();
  UART.LastRxByte = (uint8_t)UART_Rx_Byte();
  if(UART.Error == 0x00){
    UART.Buf[UART.BufIndex] = UART.LastRxByte;
    UART.BufIndex++;
    if(UART.BufIndex >= UART.BufSize){
      UART.BufIndex = 0;
    }
  }
  else{
    UART.LastRxByte = UART_NULL;
  }
  
  UART_Timer_Value_Reset();
  if(UART.Timer.Enabled == UART_FALSE){
	UART_Timer_Enable();
	UART.Timer.Enabled = UART_TRUE;
  }
  
}

void UART_Timer_ISR_Handler(void){
  UART_Timer_Clear_Interrupt_Flag();
  if(UART.Timer.Enabled == UART_TRUE){
    UART_Timer_Disable();
	UART.Timer.Enabled = UART_FALSE;
  }
  
  if(UART_Buf_Get_Index() != UART_NULL){
    
	if(UART.RxPacket.DataReadComplete == UART_FALSE){
	  UART.Error = UART_RX_ERR_READ_INCOMPLETE;
	}
    UART_RX_Packet_CRC_Check();
    #ifdef UART_CRC_ENABLE
	if(UART.RxPacket.CRCStatus == UART_TRUE){
	  UART.RxPacket.DataAvailable = UART_TRUE;
	}
	else{
	  UART_Buf_Flush();
	  UART.RxPacket.DataAvailable = UART_FALSE;
	}
	#else
	UART.RxPacket.DataAvailable = UART_TRUE;
	#endif
	
	UART.RxPacket.DataReadComplete = UART_FALSE;
  }
}

/****************UART ISR Handler Functions End*************/









/******************UART CRC Functions Start****************/

#ifdef   UART_CRC_SUPPLIER

uint16_t CRCTalbe[16] = {
 0x0000, 0xCC01, 0xD801, 0x1400,
 0xF001, 0x3C00, 0x2800, 0xE401,
 0xA001, 0x6C00, 0x7800, 0xB401,
 0x5000, 0x9C01, 0x8801, 0x4400
};


uint16_t UART_CRC_Calculate_Block(uint8_t *buf, uint8_t len){
 uint16_t crc = 0xFFFF, i;
 uint8_t  Data;
 for (i = 0; i < len; i++) {
  Data = *buf++;
  crc = CRCTalbe[(Data ^ crc) & 0x0f] ^ (crc >> 4);
  crc = CRCTalbe[((Data >> 4) ^ crc) & 0x0f] ^ (crc >> 4);
 }
 crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF);
 return crc;
}

#endif

#ifdef   UART_CRC_XMODEM

uint16_t UART_CRC_Calculate_Byte(uint16_t crc, uint8_t data){
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

uint16_t UART_CRC_Calculate_Block(volatile uint8_t *buf, uint8_t len){
  uint16_t crc = 0;
  for(uint8_t i = 0; i < len; i++){
    crc = UART_CRC_Calculate_Byte(crc,buf[i]);
  }
  return crc;
}
#endif

/*******************UART CRC Functions End*****************/









/*************UART RX Packet Functions Start***************/

void UART_RX_Packet_CRC_Check(void){
	uint8_t  temp = 0 ;
  uint16_t crc_calc = 0, crc_recv = 0;
  if( UART_Data_Len_Get() >= 2){
		temp  = (uint8_t)UART_Data_Len_Get();
		temp -= 2;
    crc_calc   =  UART_CRC_Calculate_Block(UART.Buf, temp);
    crc_recv   =  UART_Buf_Get(UART_Data_Len_Get() - 2);
    crc_recv <<= 8;
    crc_recv  |= UART_Buf_Get(UART_Data_Len_Get() - 1);
  }
  UART.RxPacket.CalculatedCRC = crc_calc;
  UART.RxPacket.ReceivedCRC = crc_recv;
  if( UART.RxPacket.CalculatedCRC == UART.RxPacket.ReceivedCRC ){
    UART.RxPacket.CRCStatus = UART_TRUE;
  }
  else{
    UART.RxPacket.CRCStatus = UART_FALSE;
  }
}

void UART_RX_Packet_Read_Complete(void){
  UART_Buf_Flush();
  UART_Data_Clear_Available_Flag();
  UART_Data_Clear_Read_Complete_Flag();
}

/**************UART RX Packet Functions End****************/









/*****************UART Init Functions Start****************/

void UART_Init(uint32_t baud){
  UART_Struct_Init();
  UART_RX_Packet_Struct_Init();
  UART_Timer_Struct_Init();
  
  UART_Config_GPIO();
  UART_Config_Clock();
  UART_Config_BAUD_Rate(baud);
  
  #ifdef UART_ENABLE_TX  
  UART_Config_Tx();
  #endif
  
  #ifdef UART_ENABLE_RX
  UART_Config_Rx();
  #endif
  
  #ifdef UART_ENABLE_RX_INT
  UART_Config_Rx_Interrupt();
  #endif
  
  UART_Timer_Init();
  UART_Buf_Flush();
}

/******************UART Init Functions End*****************/



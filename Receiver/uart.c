

/*
 * File:   uart.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * LL Driver -> UART Library
 * Rev 3.3 (23 Apr, 2025)
 * Created on December 12, 2024, 9:44 PM
 */

//Resources used in this header
//NRF_UART0 with Interrupt Priority 0
//NRF_TIMER0 With Interrupt Priority 1
//NRF_TIMER1 With Interrupt Priority 2


//add includes according to hardware
#include "nrf.h"
#include "cdefs.h"

//must be included
#include "uart.h"


#define  UART_DOUBLE_SPEED
#define  UART_ENABLE_TX    
#define  UART_ENABLE_RX    
#define  UART_ENABLE_RX_INT

#define  UART_BUFFER_SIZE               (    64U)
#define  UART_RX_PCKT_CMPLT_DELAY       (  500UL)
#define  UART_RX_ACK_MAX_DELAY          (20000UL)
#define  UART_INT_PRIORITY              (     0U)
#define  UART_TIMEOUT_INT_PRIORITY      (     1U)
#define  UART_ACK_INT_PRIORITY          (     2U)

#define  UART_CRC_ENABLE     //Uncomment if packet validation by CRC is needed
#define  UART_CRC_XMODEM     //Uncomment if X-MODEM CRC Protocol is used
//#define  UART_CRC_ALTERNATE     //Uncomment if Alternate CRC Protocol is used




//Define Software Error Codes
#define  UART_RX_ERR_NO_ERR             (0x00U)
#define  UART_RX_ERR_FRAMING            (0x01U)
#define  UART_RX_ERR_OVERRUN            (0x02U)
#define  UART_RX_ERR_READ_INCOMPLETE    (0x10U)


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
  volatile uint8_t   Reserved0;
}uart_rx_packet_t;

typedef struct uart_t{
  volatile uint8_t   Error;
	volatile uint8_t   StickyError;
	volatile uint8_t   Reserved1;
  uint8_t            Digits[8];
  uint8_t            InputNumDigits;

  volatile uint8_t   LastRxByte;
  volatile uint8_t   Buf[UART_BUFFER_SIZE];
  volatile uint8_t   AckTimeout;
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
  UART.Error = UART_NULL;
	UART.StickyError = UART_NULL;
  for(uint8_t i = 0; i < 8; i++){
    UART.Digits[i] = UART_NULL;
  }
  UART.InputNumDigits = UART_NULL;
  UART.LastRxByte = UART_NULL;
  UART.BufSize = UART_BUFFER_SIZE;
	UART.AckTimeout = UART_FALSE;
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
	NRF_UART0->BAUDRATE = baud_rate;
}


void UART_Config_Tx(void){
	NRF_UART0->PSELTXD = UART_TX_PIN;
	if( (NRF_UART0->ENABLE & UART_ENABLE_ENABLE_Msk) != UART_ENABLE_ENABLE_Enabled ){
		NRF_UART0->ENABLE |= UART_ENABLE_ENABLE_Enabled;
	}
}


void UART_Config_Rx(void){
	NRF_UART0->PSELRXD = UART_RX_PIN;
	if( (NRF_UART0->ENABLE & UART_ENABLE_ENABLE_Msk) != UART_ENABLE_ENABLE_Enabled ){
		NRF_UART0->ENABLE |= UART_ENABLE_ENABLE_Enabled;
	}
	
}

void UART_Config_Rx_Interrupt(void){
  //add rx int config
	NRF_UART0->EVENTS_RXDRDY = 0;
	NRF_UART0->INTENSET |= UART_INTENSET_RXDRDY_Msk;
	NVIC_SetPriority(UARTE0_UART0_IRQn, UART_INT_PRIORITY);
	NVIC_EnableIRQ(UARTE0_UART0_IRQn);
	NRF_UART0->TASKS_STARTRX = 1;
}

void UART_Clear_Interrupt_Flag(void){
  //Clear flag if necessary
	NRF_UART0->EVENTS_RXDRDY = 0;
}

void UART_Tx_Byte(uint8_t val){
	NRF_UART0->EVENTS_TXDRDY = 0;
	NRF_UART0->TXD = val;
	NRF_UART0->TASKS_STARTTX = 1;
	while(NRF_UART0->EVENTS_TXDRDY == 0){
		//add timeout
	}
}

uint8_t UART_Rx_Byte(void){
  return (uint8_t)NRF_UART0->RXD;
}


void UARTE0_UART0_IRQHandler(void){
	if(NRF_UART0->EVENTS_RXDRDY){
	  UART_ISR_Handler();
	}
}

/**********************UART Init Functions End*******************/









/********************UART Timer Functions Start*****************/

void UART_Timer_Struct_Init(void){
  UART.Timer.Enabled  = UART_FALSE;
  UART.Timer.ResetVal = UART_NULL;
}

void UART_Timer_Init(void){
  //config comm timer for auto packet validation
	NRF_TIMER0->TASKS_STOP = 1;
	while(NRF_TIMER0->TASKS_STOP == 1){
		//add timeout
	}
	//Prescaler 2^N = 2^4 = 16; 16MHz/16 = 1MHz
	NRF_TIMER0->PRESCALER = 4;
	NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit<<TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER0->CC[0] = UART_RX_PCKT_CMPLT_DELAY;
	NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled<<TIMER_INTENSET_COMPARE0_Pos;
	NVIC_SetPriority(TIMER0_IRQn, UART_TIMEOUT_INT_PRIORITY);
	NVIC_EnableIRQ(TIMER0_IRQn);
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->TASKS_START = 1;
}

void UART_Timer_Enable(void){
	NRF_TIMER0->TASKS_START = 1;
}

void UART_Timer_Disable(void){
	NRF_TIMER0->TASKS_STOP = 1;
	while(NRF_TIMER0->TASKS_STOP == 1){
		//add timeout
	}
}

uint8_t UART_Timer_Get_Status(void){
  return UART.Timer.Enabled;
}

uint16_t UART_Timer_Get_Val(void){
  //return current timer val
	NRF_TIMER0->TASKS_CAPTURE[1] = 1;
	return (uint16_t)NRF_TIMER0->CC[1];
}


void UART_Timer_Value_Reset(void){
  //reset timer val if compare mode selected
  //set timer val to UART.Timer.ResetVal if Overflow int is used
	NRF_TIMER0->TASKS_CLEAR = 1;
}

void UART_Timer_Clear_Interrupt_Flag(void){
  //Clear flag if necessary
	NRF_TIMER0->EVENTS_COMPARE[0] = 0;
}

//add comm timer interrupt handler vector
//call UART_Timer_ISR_Handler() inside ISR

void TIMER0_IRQHandler(void){
	if(NRF_TIMER0->EVENTS_COMPARE[0]){
	  UART_Timer_ISR_Handler();
	}
}


/*********************UART Timer Functions End******************/









/*****************COMM Ack Timer Functions Start****************/

void UART_Ack_Timer_Struct_Init(void){
  
}

void UART_Ack_Timer_Init(void){
	NRF_TIMER1->TASKS_STOP = 1;
	while(NRF_TIMER1->TASKS_STOP == 1){
		//add timeout
	}
	//Prescaler 2^N = 2^4 = 16; 16MHz/16 = 1MHz
	NRF_TIMER1->PRESCALER = 4;
	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_24Bit<<TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER1->CC[0] = UART_RX_ACK_MAX_DELAY;
	NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled<<TIMER_INTENSET_COMPARE0_Pos;
	NVIC_SetPriority(TIMER1_IRQn, UART_ACK_INT_PRIORITY);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NRF_TIMER1->TASKS_CLEAR = 1;
}

void UART_Ack_Timer_Enable(void){
  NRF_TIMER1->TASKS_START = 1;
}

void UART_Ack_Timer_Disable(void){ 
  NRF_TIMER1->TASKS_STOP = 1;
	while(NRF_TIMER1->TASKS_STOP == 1){
		//add timeout
	}
}

uint8_t UART_Ack_Timer_Get_Status(void){
  return 0;
}

uint16_t UART_Ack_Timer_Get_Val(void){
  NRF_TIMER1->TASKS_CAPTURE[1] = 1;
	return (uint16_t)NRF_TIMER1->CC[1];
}


void UART_Ack_Timer_Value_Reset(void){
  NRF_TIMER1->TASKS_CLEAR = 1;
}

void UART_Ack_Timer_Clear_Interrupt_Flag(void){
  NRF_TIMER1->EVENTS_COMPARE[0] = 0;
}

void TIMER1_IRQHandler(void){
	if(NRF_TIMER1->EVENTS_COMPARE[0]){
	  UART_Ack_Timer_Clear_Interrupt_Flag();
		UART_Ack_Timer_Disable();
		UART.AckTimeout = UART_TRUE;
	}
}



/*******************COMM Ack Timer Functions End*****************/









/********************Buffer Tx Functions Start*******************/

void UART_Tx_Buf(volatile uint8_t *data, uint8_t start, uint8_t len){
	len += start;
  for(uint16_t i = start; i < len; i++){
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

void UART_Tx_Number_Hex32_Raw(uint32_t val){
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

void UART_Tx_Number_Hex32(uint32_t val){
  UART_Tx_Byte('0');
  UART_Tx_Byte('x');
  UART_Tx_Number_Hex32_Raw(val);
}

void UART_Tx_Number_Hex(uint64_t val){
  UART_Tx_Number_Hex32(val >> 32);
  UART_Tx_Number_Hex32_Raw(val & 0xFFFFFFFF);
}

void UART_Tx_Number_Bin32_Raw(uint32_t val){
  uint8_t loop_counter = 0;
  if(val <= 0xFF){
    loop_counter = 7;
  }else if(val <= 0xFFFF){
    loop_counter = 15;     
  }else{
    loop_counter = 31;
  }
  
  for(int i = loop_counter; i >= 0; i--){
    if( (val>>i) & 1){
      UART_Tx_Byte( 49 );   
    }else{
      UART_Tx_Byte( 48 );         
    }
  }
}

void UART_Tx_Number_Bin32(uint32_t val){
  UART_Tx_Byte('0');
  UART_Tx_Byte('b');
  UART_Tx_Number_Bin32_Raw(val);
}

void UART_Tx_Number_Bin(uint64_t val){
  UART_Tx_Number_Bin32(val >> 32);
  UART_Tx_Number_Bin32_Raw(val & 0xFFFFFFFF);
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

void UART_Tx_Number_Hex_NL(uint64_t num){
  UART_Tx_Number_Hex(num);
  UART_Tx_NL();
}

void UART_Tx_Number_Hex_SP(uint64_t num){
  UART_Tx_Number_Hex(num);
  UART_Tx_SP();
}

void UART_Tx_Number_Hex_CM(uint64_t num){
  UART_Tx_Number_Hex(num);
  UART_Tx_CM();
}

/***********Hex Number with End Char Functions End*************/









/**********Bin Number with End Char Functions Start************/

void UART_Tx_Number_Bin_NL(uint64_t num){
  UART_Tx_Number_Bin(num);
  UART_Tx_NL();
}

void UART_Tx_Number_Bin_SP(uint64_t num){
  UART_Tx_Number_Bin(num);
  UART_Tx_SP();
}

void UART_Tx_Number_Bin_CM(uint64_t num){
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

void UART_Tx_Parameter_Hex_NL(char *name, uint64_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Hex_NL(num);
}

void UART_Tx_Parameter_Hex_SP(char *name, uint64_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Hex_SP(num);
}

void UART_Tx_Parameter_Hex_CM(char *name, uint64_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Hex_CM(num);
}

/***********Hex Number with Parameter Functions End************/









/**********Bin Number with Parameter Functions Start***********/

void UART_Tx_Parameter_Bin_NL(char *name, uint64_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Bin_NL(num);
}

void UART_Tx_Parameter_Bin_SP(char *name, uint64_t num){
  UART_Tx_Text(name);
  UART_Tx_SP();
  UART_Tx_Number_Bin_SP(num);
}

void UART_Tx_Parameter_Bin_CM(char *name, uint64_t num){
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

uint8_t  UART_Last_Received_Byte(void){
	return UART.LastRxByte;
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
  UART.RxPacket.DataReadComplete = UART_TRUE;
}

uint8_t UART_Ack_Timeout_Status_Get(void){
	if(UART.AckTimeout == UART_TRUE){
		UART.AckTimeout = UART_FALSE;
		return UART_TRUE;
	}
	return UART_FALSE;
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
		if(UART.StickyError == UART_NULL){
	    UART.StickyError = UART.Error;
	  }
	  UART.Error = UART_NULL;
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
  
  UART_Timer_ISR_Executables();
}

void UART_Timer_ISR_Executables(void){
  if(UART_Buf_Get_Index() != UART_NULL){
	  if(UART.RxPacket.DataReadComplete == UART_FALSE){
	    UART.Error = UART_RX_ERR_READ_INCOMPLETE;
		  if(UART.StickyError == UART_NULL){
	      UART.StickyError = UART.Error;
	    }
	    UART.Error = UART_NULL;
	  }
	
    UART_RX_Packet_CRC_Check();
	
    #ifdef UART_CRC_ENABLE
	  if(UART.RxPacket.CRCStatus == UART_TRUE){
	    UART.RxPacket.DataAvailable = UART_TRUE;
	    UART.RxPacket.DataReadComplete = UART_FALSE;
	    UART_RX_Packet_Disassemble();
	  }
	  else{
	    UART_RX_Packet_Read_Complete();
	  }
	  #else
	  UART.RxPacket.DataAvailable = UART_TRUE;
	  UART.RxPacket.DataReadComplete = UART_FALSE;
    UART_RX_Packet_Disassemble();
	  #endif
  }
}

/****************UART ISR Handler Functions End*************/









/******************UART CRC Functions Start****************/

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

#ifdef   UART_CRC_ALTERNATE

uint16_t CRCTable[16] = {
  0x0000, 0xCC01, 0xD801, 0x1400,
  0xF001, 0x3C00, 0x2800, 0xE401,
  0xA001, 0x6C00, 0x7800, 0xB401,
  0x5000, 0x9C01, 0x8801, 0x4400
};


uint16_t UART_CRC_Calculate_Block(volatile uint8_t *buf, uint8_t len){
  uint16_t crc = 0xFFFF, i;
  uint8_t  Data;
  for (i = 0; i < len; i++) {
    Data = *buf++;
    crc = CRCTable[(Data ^ crc) & 0x0f] ^ (crc >> 4);
    crc = CRCTable[((Data >> 4) ^ crc) & 0x0f] ^ (crc >> 4);
  }
  crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF);
  return crc;
}

#endif

/*******************UART CRC Functions End*****************/









/*************UART RX Packet Functions Start***************/

void UART_RX_Packet_CRC_Check(void){
  uint8_t  temp = 0;
  uint16_t crc_calc = 0, crc_recv = 0;
  UART.RxPacket.CRCStatus = UART_FALSE;
  if( UART_Data_Len_Get() > 2){
    temp  = (uint8_t)UART_Data_Len_Get();
	  temp -= 2;
    crc_calc   =  UART_CRC_Calculate_Block(UART.Buf, temp);
    crc_recv   =  UART_Buf_Get(UART_Data_Len_Get() - 2);
    crc_recv <<= 8;
    crc_recv  |= UART_Buf_Get(UART_Data_Len_Get() - 1);
	  UART.RxPacket.CalculatedCRC = crc_calc;
    UART.RxPacket.ReceivedCRC = crc_recv;
    if( UART.RxPacket.CalculatedCRC == UART.RxPacket.ReceivedCRC ){
      UART.RxPacket.CRCStatus = UART_TRUE;
	  }
  }
}

void UART_RX_Packet_Disassemble(void){
  //Disassemble packet
	
}


void UART_RX_Packet_Read_Complete(void){
  UART_Buf_Flush();
  UART_Data_Clear_Available_Flag();
  UART_Data_Clear_Read_Complete_Flag();
  UART_Error_Code_Clear();
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
  UART_Timer_Value_Reset();
  UART_Timer_Enable();
  UART_Buf_Flush();
	
	//UART_Ack_Timer_Struct_Init();
	//UART_Ack_Timer_Init();
	
}

/******************UART Init Functions End*****************/



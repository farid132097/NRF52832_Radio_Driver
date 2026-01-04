

#include "nrf.h"
#include "timeout.h"
#include "clocks.h"
#include "cdefs.h"
#include "rtc.h"

typedef struct timeout_t{
	uint8_t          Error;
	uint8_t          StickyError;
}timeout_t;

static timeout_t Timeout;



void Timeout_Struct_Init(void){
	Timeout.Error       = NULL;
	Timeout.StickyError = NULL;
}

void Timeout_Reg_Init(void){
	Clocks_LFCLK_RC_Start_Request();
}



void Timeout_Set_MicroSeconds(uint32_t val){
  RTC_RTC1_Set_Timeout( val );
}

uint8_t Timeout_Error_Assign(uint8_t error_code){
	if( RTC_RTC1_Event() == TRUE ){
		Timeout.Error = error_code;
		if(Timeout.StickyError == NULL){
			Timeout.StickyError = error_code;
		}
		RTC_RTC1_Event_Clear();
		return TRUE;
	}
	else{
		return FALSE;
	}
}

void Timeout_Error_Force_Assign(uint8_t error_code){
	Timeout.Error = error_code;
	if(Timeout.StickyError == NULL){
		Timeout.StickyError = error_code;
	}
}

void Timeout_Clear_Assignment(void){
	RTC_RTC1_Clear_Timeout();
	RTC_RTC1_Event_Clear();
}

void Timeout_Delay(uint32_t val){
	Timeout_Set_MicroSeconds(val);
	while(Timeout_Error_Assign(0) == FALSE);
	Timeout_Clear_Assignment();
}

uint8_t Timeout_Error_Get(void){
	return Timeout.Error;
}

void Timeout_Error_Clear(void){
	Timeout.Error = NULL;
}

uint8_t Timeout_Sticky_Error_Get(void){
	return Timeout.StickyError;
}

void Timeout_Sticky_Error_Clear(void){
	Timeout.StickyError = NULL;
}

uint16_t Timeout_Elapsed_Time_Get(void){
	return RTC_RTC1_Elapsed_Time_Get();
}

void Timeout_Init(void){
	Timeout_Struct_Init();
	Timeout_Reg_Init();
}



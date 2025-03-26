

#include "nrf.h"
#include "timeout.h"
#include "cdefs.h"

typedef struct timeout_t{
	int16_t   SetStatus;
	uint8_t   Error;
	uint8_t   StickyError;
	int32_t   SetVal;
	int32_t   CurrVal;
}timeout_t;

static timeout_t Timeout;



void Timeout_Struct_Init(void){
	Timeout.SetStatus   = FALSE;
	Timeout.Error       = NULL;
	Timeout.StickyError = NULL;
	Timeout.SetVal      = NULL;
	Timeout.CurrVal     = NULL;
}

void Timeout_Reg_Init(void){
	//Initialize registers if necessary
}

void Timeout_Delay_1us(void){
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

void Timeout_Delay_us(int32_t delay_us){
	for(int32_t i=0;i<delay_us;i++){
		Timeout_Delay_1us();
	}
}

void Timeout_Delay_ms(int32_t delay_ms){
	int32_t delay_cycles = delay_ms;
	delay_cycles *= 1000;
	for(int32_t i=0;i<delay_cycles;i++){
		Timeout_Delay_1us();
	}
}

void Timeout_Set_MicroSeconds(int32_t val){
	if(Timeout.SetStatus == FALSE){
		Timeout.SetVal = val;
		Timeout.SetStatus = TRUE;
	}
}

uint8_t Timeout_Occured(void){
	if(Timeout.CurrVal >= Timeout.SetVal){
		Timeout.SetStatus = FALSE;
		return TRUE;
	}
	else{
		Timeout_Delay_1us();
	  Timeout.CurrVal++;
		return FALSE;
	}
}

uint8_t Timeout_Error_Assign(int32_t val, uint8_t error_code){
	Timeout_Set_MicroSeconds(val);
	if(Timeout_Occured()){
		Timeout.Error = error_code;
		if(Timeout.StickyError == NULL){
			Timeout.StickyError = error_code;
		}
		return TRUE;
	}
	else{
		return FALSE;
	}
}

void Timeout_Error_Force_Assign(uint8_t error_code){
	Timeout.Error = error_code;
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

int32_t Timeout_CurrVal_Get(void){
	return Timeout.CurrVal;
}

void Timeout_CurrVal_Clear(void){
	Timeout.CurrVal = NULL;
}

int32_t Timeout_SetVal_Get(void){
	return Timeout.SetVal;
}

void Timeout_SetVal_Clear(void){
	Timeout.SetVal = NULL;
}

void Timeout_Status_Clear(void){
	Timeout.SetStatus = FALSE;
}

void Timeout_Arm(void){
	Timeout_Status_Clear();
	Timeout_SetVal_Clear();
	Timeout_CurrVal_Clear();
	Timeout_Error_Clear();
}

void Timeout_Init(void){
	Timeout_Struct_Init();
	Timeout_Reg_Init();
}



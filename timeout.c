

#include "nrf.h"
#include "timeout.h"
#include "clocks.h"
#include "cdefs.h"

typedef struct timeout_t{
	int16_t          SetStatus;
	uint8_t          Error;
	uint8_t          StickyError;
	volatile uint8_t Occured;
}timeout_t;

static timeout_t Timeout;



void Timeout_Struct_Init(void){
	Timeout.SetStatus   = FALSE;
	Timeout.Error       = NULL;
	Timeout.StickyError = NULL;
	Timeout.Occured     = FALSE;
}

void Timeout_Reg_Init(void){
	// Enable the LFCLK
  Clock_LFCLK_Request();

  // No prescaling, 32.768 kHz
  NRF_RTC0->PRESCALER = 0;
  // Disable all events	
  NRF_RTC0->EVTENCLR = 0xFFFFFFFF;
	// Disable all interrupts
  NRF_RTC0->INTENCLR = 0xFFFFFFFF; 

  // Enable IRQ in NVIC
  NVIC_EnableIRQ(RTC0_IRQn);

  // Start RTC0
  NRF_RTC0->TASKS_START = 1;
}

void RTC0_IRQHandler(void) {
  if (NRF_RTC0->EVENTS_COMPARE[0]) {
		// Clear event flag
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    // Disable interrupt
    NRF_RTC0->INTENCLR = RTC_INTENCLR_COMPARE0_Msk;
		// Set flag for main loop
    Timeout.Occured = TRUE;
		// Clear Flag for next time setting
		Timeout.SetStatus = FALSE;
  }
}



void Timeout_Set_MicroSeconds(int32_t val){
	if(Timeout.SetStatus == FALSE){
		uint32_t now = NRF_RTC0->COUNTER;
		// Unit tick is 30.5 uS
		val /= 30;
		// Set compare with wraparound
    NRF_RTC0->CC[0] = (now + val) & 0xFFFFFF;
		// Clear event flag
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    // Enable interrupt
    NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Msk;
		Timeout.Occured = FALSE;
		Timeout.SetStatus = TRUE;
	}
}

uint8_t Timeout_Error_Assign(int32_t val, uint8_t error_code){
	Timeout_Set_MicroSeconds(val);
	if( Timeout_Occured_Flag_Get() == TRUE ){
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
	if(Timeout.StickyError == NULL){
		Timeout.StickyError = error_code;
	}
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

uint8_t Timeout_Status_Get(void){
	return Timeout.SetStatus;
}

void Timeout_Status_Clear(void){
	Timeout.SetStatus = FALSE;
}

uint8_t Timeout_Occured_Flag_Get(void){
	return Timeout.Occured;
}

void Timeout_Occured_Flag_Clear(void){
	Timeout.Occured = FALSE;
}

void Timeout_Arm(void){
	Timeout_Error_Clear();
	Timeout_Occured_Flag_Clear();
}


__WEAK void Timeout_Handler(void){
	//while(1){
		//add function for robust implementation
	//}
}


void Timeout_Init(void){
	Timeout_Struct_Init();
	Timeout_Reg_Init();
}



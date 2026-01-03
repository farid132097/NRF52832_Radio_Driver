

#include "nrf.h"
#include "rtc.h"
#include "clocks.h"
#include "cdefs.h"


typedef struct rtc_t{
	volatile uint32_t UpTime;
	volatile uint32_t IntervalTime;
	volatile uint32_t RTC1Val;
	uint8_t           RTC0Configured;
	uint8_t           RTC1Configured;
	uint8_t           RTC1Event;
	uint8_t           Reserved1;
}rtc_t;

static rtc_t RTC;

void RTC_Struct_Init(void){
	RTC.UpTime = 0;
	RTC.IntervalTime = 0;
	RTC.RTC1Val = 0;
	RTC.RTC0Configured = FALSE;
	RTC.RTC1Configured = FALSE;
	RTC.RTC1Event = FALSE;
}



void RTC_Clock_Init(void){
	Clocks_LFCLK_RC_Start_Request();
}


void RTC_RTC0_Reg_Init(void){
	if(RTC.RTC0Configured == FALSE){
	  if(NRF_RTC0->TASKS_START == 1){
	    NRF_RTC0->TASKS_STOP = 1;
	  }
	  NRF_RTC0->PRESCALER = 0;
	  NRF_RTC0->CC[0] = 32768;
	  NRF_RTC0->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
	  NRF_RTC0->INTENSET = (RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos);
		NVIC_ClearPendingIRQ(RTC0_IRQn);
		NVIC_SetPriority(RTC0_IRQn, 0);
	  NVIC_EnableIRQ(RTC0_IRQn);
	  NRF_RTC0->TASKS_START = 1;
		RTC.RTC0Configured = TRUE;
  }
}

void RTC_RTC0_Disable(void){
	if(NRF_RTC0->TASKS_START == 1){
	  NRF_RTC0->TASKS_STOP = 1;
		NVIC_DisableIRQ(RTC0_IRQn);
		NVIC_ClearPendingIRQ(RTC0_IRQn);
	}
	if(NRF_CLOCK->EVENTS_LFCLKSTARTED == 1){
		NRF_CLOCK->TASKS_LFCLKSTOP = 1;
	}
}


void RTC_RTC1_Set_Timeout(uint32_t val){
	if(RTC.RTC1Configured == FALSE){
	  if(NRF_RTC1->TASKS_START == 1){
	    NRF_RTC1->TASKS_STOP = 1;
	  }
		NRF_RTC1->TASKS_CLEAR = 1;
	  NRF_RTC1->PRESCALER = 0;
		val /= 31;  //1 tick = 30.5uS (31uS Approx.)
	  NRF_RTC1->CC[0] = val;
	  NRF_RTC1->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
	  NRF_RTC1->INTENSET = (RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos);
		NVIC_ClearPendingIRQ(RTC1_IRQn);
		NVIC_SetPriority(RTC1_IRQn, 1);
	  NVIC_EnableIRQ(RTC1_IRQn);
	  NRF_RTC1->TASKS_START = 1;
		RTC.RTC1Configured = TRUE;
	}
}


void RTC_RTC1_Clear_Timeout(void){
	if(RTC.RTC1Configured == TRUE){
		NRF_RTC1->TASKS_STOP = 1;
		RTC.RTC1Val = NRF_RTC1->COUNTER;
    NRF_RTC1->EVTENCLR = RTC_EVTENCLR_COMPARE0_Clear << RTC_EVTENCLR_COMPARE0_Pos;
		NRF_RTC1->INTENCLR = RTC_INTENCLR_COMPARE0_Clear << RTC_INTENCLR_COMPARE0_Pos;
    NVIC_DisableIRQ(RTC1_IRQn);
		NVIC_ClearPendingIRQ(RTC1_IRQn);
		RTC.RTC1Configured = FALSE;
	}
}


uint8_t RTC_RTC1_Timeout_Event(void){
	return RTC.RTC1Event;
}

void RTC_RTC1_Timeout_Event_Clear(void){
	RTC.RTC1Event = FALSE;
}

uint16_t RTC_RTC1_Elapsed_Time_Get(void){
	return RTC.RTC1Val;
}

uint32_t RTC_UpTime_Get(void){
	return RTC.UpTime;
}


uint32_t RTC_Interval_Time_Get(void){
	return RTC.IntervalTime;
}


void RTC_Interval_Time_Clear(void){
	RTC.IntervalTime = 0;
}






void RTC0_IRQHandler(void){
	if(NRF_RTC0->EVENTS_COMPARE[0] == 1){
		NRF_RTC0->EVENTS_COMPARE[0] = 0;
	  NRF_RTC0->TASKS_CLEAR = 1;
	  RTC.UpTime++;
	  RTC.IntervalTime++;
	}
}


void RTC1_IRQHandler(void){
	if(NRF_RTC1->EVENTS_COMPARE[0] == 1){
	  NRF_RTC1->EVENTS_COMPARE[0] = 0;
		RTC_RTC1_Clear_Timeout();
	  NRF_RTC1->TASKS_CLEAR = 1;
		RTC.RTC1Event = TRUE;
		
	}
}


void RTC_Init(void){
	RTC_Struct_Init();
	RTC_Clock_Init();
	RTC_RTC0_Reg_Init();
}


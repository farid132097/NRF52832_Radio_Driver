

#include "nrf.h"
#include "clocks.h"
#include "cdefs.h"

typedef struct clocks_t{
	uint8_t HFCLKSts;
	uint8_t LFCLKSts;
}clocks_t;

clocks_t Clocks;


void Clock_Struct_Init(void){
	Clocks.HFCLKSts = STOPPED;
	Clocks.LFCLKSts = STOPPED;
}

void Clock_HFCLK_Start(void){
	if((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) == CLOCK_HFCLKSTAT_SRC_RC){
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
  }
}

void Clock_HFCLK_Stop(void){
	if( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) != CLOCK_HFCLKSTAT_STATE_NotRunning){
	  NRF_CLOCK->TASKS_HFCLKSTOP = 1;
	  while( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) != CLOCK_HFCLKSTAT_STATE_NotRunning );
	}
}

void Clock_HFCLK_Request(void){
	if(Clocks.HFCLKSts == STOPPED){
		//enable clock module
		Clock_HFCLK_Start();
	  Clocks.HFCLKSts++;
	}
	else if(Clocks.HFCLKSts != 0xFF){
    Clocks.HFCLKSts++;
	}
}

void Clock_HFCLK_Release(void){
	if(Clocks.HFCLKSts > 0){
		Clocks.HFCLKSts--;
		if(Clocks.HFCLKSts == STOP){
			//disable clock module
			Clock_HFCLK_Stop();
	  }
	}
}

uint8_t Clock_HFCLK_Request_Count_Get(void){
	return Clocks.HFCLKSts;
}





void Clock_LFCLK_Start(void){
	if( (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) == CLOCK_LFCLKSTAT_STATE_NotRunning ){
	  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
	  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_LFCLKSTART = 1;
	  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
	}
}

void Clock_LFCLK_Stop(void){
	if( (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) == (CLOCK_LFCLKSTAT_STATE_Running << CLOCK_LFCLKSTAT_STATE_Pos) ){
		NRF_CLOCK->TASKS_LFCLKSTOP = 1;
	}
}


void Clock_LFCLK_Request(void){
	if(Clocks.LFCLKSts == STOPPED){
		//enable clock module
		Clock_LFCLK_Start();
		Clocks.LFCLKSts++;
	}
	else if(Clocks.LFCLKSts != 0xFF){
    Clocks.LFCLKSts++;
	}
}

void Clock_LFCLK_Release(void){
	if(Clocks.LFCLKSts > 0){
		Clocks.LFCLKSts--;
		if(Clocks.LFCLKSts == STOP){
			//disable clock module
			Clock_LFCLK_Stop();
		}
	}
}

uint8_t Clock_LFCLK_Request_Count_Get(void){
	return Clocks.LFCLKSts;
}



void Clock_Init(void){
	Clock_Struct_Init();
}
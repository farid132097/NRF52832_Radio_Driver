

#include "nrf.h"
#include "clocks.h"
#include "cdefs.h"

typedef struct clocks_t{
	uint8_t HFCLKSts;
	uint8_t LFCLKSts;
}clocks_t;

static clocks_t Clocks;


void Clock_Struct_Init(void){
	Clocks.HFCLKSts = STOPPED;
	Clocks.LFCLKSts = STOPPED;
}



//////////////////////////////////////HFCLK Related Functions Start/////////////////////////////////////////

void Clock_HFCLK_Start_Request(void){
	if( (NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)) == 0){
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  }
}

uint8_t Clock_HFCLK_Start_Complete(void){
	return (uint8_t)NRF_CLOCK->EVENTS_HFCLKSTARTED;
}

void Clock_HFCLK_Wait_Until_Ready(void){
	while(Clock_HFCLK_Start_Complete() == 0);
}



void Clock_HFCLK_Stop_Request(void){
	if( NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) ){
	  NRF_CLOCK->TASKS_HFCLKSTOP = 1;
	  while( NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) );
	}
}

uint8_t Clock_HFCLK_Stop_Complete(void){
	if( NRF_CLOCK->HFCLKSTAT & (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) ){
		return FALSE;
	}
	else{
		return TRUE;
	}
}

void Clock_HFCLK_Wait_Until_Stopped(void){
	while(Clock_HFCLK_Stop_Complete() == 0);
}



void Clock_HFCLK_Request(void){
	if(Clocks.HFCLKSts == STOPPED){
		//enable clock module
		Clock_HFCLK_Start_Request();
		Clock_HFCLK_Wait_Until_Ready();
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
			Clock_HFCLK_Stop_Request();
			Clock_HFCLK_Wait_Until_Stopped();
	  }
	}
}

uint8_t Clock_HFCLK_Request_Count_Get(void){
	return Clocks.HFCLKSts;
}

///////////////////////////////////////HFCLK Related Functions End//////////////////////////////////////////









//////////////////////////////////////LFCLK Related Functions Start/////////////////////////////////////////

void Clock_LFCLK_Start(void){
	if( (NRF_CLOCK->LFCLKSTAT & (CLOCK_LFCLKSTAT_SRC_Xtal << CLOCK_LFCLKSTAT_SRC_Pos)) == 0 ){
		//LFCLK source is 32.768kHz crystal, startup time 250ms
	  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
		
		//LFCLK source is 32.768kHz RC Oscillator, startup time 0.6ms
		//NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);
	  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_LFCLKSTART = 1;
	  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
	}
}

void Clock_LFCLK_Stop(void){
	if( NRF_CLOCK->LFCLKSTAT & (CLOCK_LFCLKSTAT_SRC_Xtal << CLOCK_LFCLKSTAT_SRC_Pos) ){
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

///////////////////////////////////////LFCLK Related Functions End//////////////////////////////////////////




void Clock_Init(void){
	Clock_Struct_Init();
}





#include "nrf.h"
#include "clocks.h"
#include "cdefs.h"

typedef struct clocks_t{
	uint8_t HFXTALCLKSts;
	uint8_t LFRCCLKSts;
	uint8_t LFXTALCLKSts;
}clocks_t;

static clocks_t Clocks;


void Clocks_Struct_Init(void){
	Clocks.HFXTALCLKSts = STOPPED;
	Clocks.LFRCCLKSts   = STOPPED;
	Clocks.LFXTALCLKSts = STOPPED;
}



////////////////////////////////////HFXTALCLK Related Functions Start///////////////////////////////////////

void Clocks_HFCLK_Xtal_Start(void){
	if( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) != (CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) ||
		  (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk)   != (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)      ){
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  }
}

uint8_t Clocks_HFCLK_Xtal_Started(void){
	return (uint8_t)NRF_CLOCK->EVENTS_HFCLKSTARTED;
}

void Clocks_HFCLK_Xtal_Wait_Until_Ready(void){
	while(Clocks_HFCLK_Xtal_Started() == 0);
}



void Clocks_HFCLK_Xtal_Stop(void){
	if( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) == (CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) &&
		  (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk)   == (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)      ){
	  NRF_CLOCK->TASKS_HFCLKSTOP = 1;
	}
}

uint8_t Clocks_HFCLK_Xtal_Stopped(void){
	if( (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) == (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos) ){
		return FALSE;
	}
	else{
		return TRUE;
	}
}

void Clocks_HFCLK_Xtal_Wait_Until_Stopped(void){
	while(Clocks_HFCLK_Xtal_Stopped() == 0);
}



void Clocks_HFCLK_Xtal_Start_Request(void){
	if(Clocks.HFXTALCLKSts == STOPPED){
		Clocks_HFCLK_Xtal_Start();
		Clocks_HFCLK_Xtal_Wait_Until_Ready();
	  Clocks.HFXTALCLKSts++;
	}
	else if(Clocks.HFXTALCLKSts != 0xFF){
    Clocks.HFXTALCLKSts++;
	}
}

void Clocks_HFCLK_Xtal_Stop_Request(void){
	if(Clocks.HFXTALCLKSts > 0){
		Clocks.HFXTALCLKSts--;
		if(Clocks.HFXTALCLKSts == STOP){
			//disable clock module
			Clocks_HFCLK_Xtal_Stop();
			Clocks_HFCLK_Xtal_Wait_Until_Stopped();
	  }
	}
}

uint8_t Clocks_HFCLK_Xtal_Request_Count_Get(void){
	return Clocks.HFXTALCLKSts;
}

/////////////////////////////////////HFXTALCLK Related Functions End////////////////////////////////////////









/////////////////////////////////////LFRCCLK Related Functions Start////////////////////////////////////////

void Clocks_LFCLK_RC_Start(void){
	if( (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) != (CLOCK_LFCLKSTAT_STATE_Running<<CLOCK_LFCLKSTAT_STATE_Pos) || 
		  (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSRC_SRC_Msk)    != (CLOCK_LFCLKSTAT_SRC_RC << CLOCK_LFCLKSTAT_SRC_Pos)      ){
		//LFCLK source is 32.768kHz RC Oscillator, startup time 0.6ms
		NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);
	  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_LFCLKSTART = 1;
	  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
	}
}

void Clocks_LFCLK_RC_Stop(void){
	if( (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) == (CLOCK_LFCLKSTAT_STATE_Running<<CLOCK_LFCLKSTAT_STATE_Pos) ){
		NRF_CLOCK->TASKS_LFCLKSTOP = 1;
	}
}

void Clocks_LFCLK_RC_Start_Request(void){
	if(Clocks.LFRCCLKSts == STOPPED){
		//enable clock module
		Clocks_LFCLK_RC_Start();
		Clocks.LFRCCLKSts++;
	}
	else if(Clocks.LFRCCLKSts != 0xFF){
    Clocks.LFRCCLKSts++;
	}
}

void Clocks_LFCLK_RC_Stop_Request(void){
	if(Clocks.LFRCCLKSts > 0){
		Clocks.LFRCCLKSts--;
		if(Clocks.LFRCCLKSts == STOP){
			//disable clock module
			Clocks_LFCLK_RC_Stop();
		}
	}
}

uint8_t Clocks_LFCLK_RC_Request_Count_Get(void){
	return Clocks.LFRCCLKSts;
}

/////////////////////////////////////LFRCCLK Related Functions End////////////////////////////////////////









////////////////////////////////////LFXTALCLK Related Functions Start///////////////////////////////////////

void Clocks_LFCLK_Xtal_Start(void){
	if( (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) != (CLOCK_LFCLKSTAT_STATE_Running<<CLOCK_LFCLKSTAT_STATE_Pos) || 
		  (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSRC_SRC_Msk)    != (CLOCK_LFCLKSTAT_SRC_Xtal << CLOCK_LFCLKSTAT_SRC_Pos)    ){
		//LFCLK source is 32.768kHz crystal, startup time 250ms
	  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
	  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	  NRF_CLOCK->TASKS_LFCLKSTART = 1;
	  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
	}
}

void Clocks_LFCLK_Xtal_Stop(void){
	if( (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) == (CLOCK_LFCLKSTAT_STATE_Running<<CLOCK_LFCLKSTAT_STATE_Pos) ){
		NRF_CLOCK->TASKS_LFCLKSTOP = 1;
	}
}

void Clocks_LFCLK_Xtal_Start_Request(void){
	if(Clocks.LFXTALCLKSts == STOPPED){
		//enable clock module
		Clocks_LFCLK_Xtal_Start();
		Clocks.LFXTALCLKSts++;
	}
	else if(Clocks.LFXTALCLKSts != 0xFF){
    Clocks.LFXTALCLKSts++;
	}
}

void Clocks_LFCLK_Xtal_Stop_Request(void){
	if(Clocks.LFXTALCLKSts > 0){
		Clocks.LFXTALCLKSts--;
		if(Clocks.LFXTALCLKSts == STOP){
			//disable clock module
			Clocks_LFCLK_Xtal_Stop();
		}
	}
}

uint8_t Clocks_LFCLK_Xtal_Request_Count_Get(void){
	return Clocks.LFXTALCLKSts;
}

/////////////////////////////////////LFXTALCLK Related Functions End////////////////////////////////////////





void Clocks_Init(void){
	Clocks_Struct_Init();
}



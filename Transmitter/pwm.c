

#include "nrf.h"
//#include "cdefs.h"

#define PWM_OUTPUT_PIN   27
#define PWM_FREQUENCY_HZ 1000
#define PWM_CLOCK_HZ     1000000  // 1 MHz resolution
#define PWM_TOP          (PWM_CLOCK_HZ / PWM_FREQUENCY_HZ)
#define PWM_DUTY         (PWM_TOP / 3) // 50% duty

uint16_t pwm_seq[4] = {
	PWM_DUTY|(1<<15), 10, 10, 10
};

void PWM_Init(void){
  // 1. Set GPIO as output
  NRF_GPIO->DIRSET = (1 << PWM_OUTPUT_PIN);
  NRF_PWM0->PSEL.OUT[0] = (PWM_OUTPUT_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->PSEL.OUT[1] = (PWM_OUTPUT_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  
  NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  NRF_PWM0->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
  NRF_PWM0->COUNTERTOP = (PWM_TOP << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
  NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                      (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  NRF_PWM0->SEQ[0].PTR = (uint32_t)(pwm_seq);
  NRF_PWM0->SEQ[0].CNT = 4;
	//NRF_PWM0->SEQ[0].CNT = 1;
  NRF_PWM0->SEQ[0].REFRESH = 0;
  NRF_PWM0->SEQ[0].ENDDELAY = 0;
	
  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
	
	NRF_PWM0->TASKS_SEQSTART[0] = 1;
	
	
}

void PWM_Set_Duty(uint16_t val){
	pwm_seq[0] = val|(1<<15);
	NRF_PWM0->TASKS_SEQSTART[0] = 1;
}
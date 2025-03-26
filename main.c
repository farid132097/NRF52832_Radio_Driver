

/*
 * File:   main.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * NRF52832 Raio Driver
 * Created on March 26, 2025, 03:38 PM
 */


#include "nrf.h"
#include "app.h"

int main(void){
	
	App_Config();
	//add WDT Config
	
	while(1){
		
		while(1){
			
			//reload WDT
			App_Mainloop();
			
		}
		
	}
	
}



/*
 * ECE 153B - Winter 2020
 *
 * Name(s):
 * Section:
 * Lab: 2A
 */

#include "stm32l476xx.h"


/*
Msk: 11
_1: 10
_0: 01

*/

void PWM_Init() {
	// Enable GPIO Port E Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN_Msk;
	
	// Enable TIM1 Clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk;
	
	// Configure PE8  
	GPIOE->MODER &=  ~(GPIO_MODER_MODE8_Msk); 	//reset since two bits are configrued
	GPIOE->MODER |=  GPIO_MODER_MODE8_1;  			//alternative mode (MODE8_1 has value 2UL which is 10 for AFIO)
	
	GPIOE->OSPEEDR  &=  ~(GPIO_OSPEEDR_OSPEED8_Msk); //reset since two bits are configrued
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk;      //set to very high output speed(11)
	
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk); 		 //set to no pull-up, no pull-down
	
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL8_0;			//shift 1<<0 to AFR[1] to set AFPH8[3:0] as 0001 which corrsponds to AF1 on manual
	//Now PE8 is in AFIO1 mode
	
	// Configure PWM Output for TIM1 CH 1N
	
	TIM1->CR1 &= ~(1<<TIM_CR1_DIR_Pos);		//on page 805 of manual; reset bit 4 to set as upcounter
	TIM1->CR1 &= ~(TIM_CR1_ARPE_Msk);
	TIM1->PSC &= ~0xFF;										//For this part of the lab, we will just use the default 4 MHz system clock. on page 826
	TIM1->ARR =  400-1; 									//we wish to run the timer in frequency of 10kHz, so 4MHz/10_000 = 400; 
																				//Period = 1/4MHz * 400 = 0.1 ms
	TIM1->CCMR1 |= ~TIM_CCMR1_OC1M_Msk;   //"Clear the output compare mode bits for channel 1"
	TIM1->CCMR1 |= 110 << TIM_CCMR1_OC1M_Pos; // Set the output compare mode bits to PWM mode 1
  TIM1->CCMR1 &= ~(1<<16);
	
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE_Msk;	 //Enable output preload for channel 1 page 819 on manual
	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
	
	TIM1->CCER &= ~TIM_CCER_CC1NP_Msk;    //reset this bit to enable OC1E active high for polarity, page 823
	TIM1->CCER |= TIM_CCER_CC1NE_Msk;
	TIM1->BDTR |=  TIM_BDTR_MOE_Msk;			//P 831, TIM1/TIM8 break and dead-time register (TIMx_BDTR)				
	
	TIM1->CR1 |= 1 <<TIM_CR1_CEN_Pos;			//P806 1<<0 , enable the timer
	// TODO
}
 
int main() {
	// Initialization - We will use the default 4 MHz clock
	PWM_Init();
	
	// Periodic Dimming
	int i;
	
	while(1) {
		// TODO (changing duty cycle, etc.)
		volatile uint32_t dim;
		
		for(dim = 399; dim > 0 ;dim --){
			for(int i = 0;i<2000;i++);
			TIM1->CCR1 = dim;
		}
		for(dim = 0; dim < 400 ;dim ++){
			for(int i = 0;i<2000;i++);
			TIM1->CCR1 = dim;
		}
		
		}
	
	
}
	



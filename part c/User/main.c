/*
 * ECE 153B - Winter 2020
 *
 * Name(s):
 * Section:
 * Lab: 2C
 */
 
#include <stdio.h> 
 
#include "stm32l476xx.h"
#include "lcd.h"
#include "led.h"

uint32_t volatile timeInterval = -1;
uint32_t volatile distance= -1;
uint32_t volatile count = 0;
uint32_t volatile in = 0;

void Input_Capture_Setup() {
	//PB6
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN_Msk;
	
	GPIOB->MODER &=  ~GPIO_MODER_MODE6_Msk;
	GPIOB->MODER |=  0b10 << GPIO_MODER_MODE6_Pos;  //10: Alternate function mode
	
	
	//configure TIM4_CH1 AF2
	GPIOB->AFR[0] |= 0b0010 << GPIO_AFRL_AFSEL6_Pos;
	//no pull up/down (00)
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk;
	
	//Enable timer 4 in RCC APB1ENRx
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN_Msk;
	
	//Enable auto reload preload in the control register
	TIM4->CR1 |= TIM_CR1_ARPE_Msk; //1	
	
	//set the auto reload value to its maximum value
	TIM4->ARR |= 65535;
	
	//Set the prescaler
	TIM4->PSC  = 0;
	//In the capture/compare mode register, set the input capture mode bits such that the input capture is mapped to timer input 1.
	TIM4->CCMR1 |= 0b01 << TIM_CCMR1_CC1S_Pos;		//p908, 01: CC1 channel is configured as input, IC1 is mapped on TI1.
	
	//In the capture/compare enable register, set bits to capture both rising/falling edges and 
	TIM4->CCER |=  0b1 << TIM_CCER_CC1P_Pos; //11, sensitive to both edges
	TIM4->CCER |=  0b1 << TIM_CCER_CC1NP_Pos;
	//enable capturing
	/*
	CC1 channel configured as input: CC1NP/CC1P bits select TI1FP1 and TI2FP1 polarity for
	trigger or capture operations
	*/
	TIM4->CCER |= TIM_CCER_CC1E_Msk;
	
	//In the DMA/Interrupt enable register, enable both interrupt and DMA requests. In addition, enable the update interrupt.
	//TIM4->DIER |= TIM_DIER_CC1DE_Msk | TIM_DIER_CC1IE_Msk | TIM_DIER_UIE_Msk | TIM_DIER_TDE_Msk | TIM_DIER_TIE_Msk; //1, enable
	TIM4->DIER |= TIM_DIER_CC1DE_Msk | TIM_DIER_CC1IE_Msk | TIM_DIER_UIE_Msk ;
	//Enable update generation in the event generation register
	TIM4->EGR |= TIM_EGR_UG_Msk; //0, no action ; 1, ~enable
	
	//Clear the update interrupt flag
	TIM4->SR &= ~TIM_SR_UIF_Msk;
	
	//Set the direction of the counter and enable the counter in the control register.
	TIM4->CR1 &= ~TIM_CR1_DIR_Msk;	 //DIR  = 0upcount
	TIM4->CR1 |= TIM_CR1_CEN_Msk;
	
	//Enable the interrupt (TIM4 IRQn) in the NVIC and set its priority to 2.
	NVIC_EnableIRQ(TIM4_IRQn);			//Enable and mask bits control the NVIC interrupt channel corresponding to EXTI input line 0. 
	NVIC_SetPriority(TIM4_IRQn, 2);
}


void TIM4_IRQHandler(void) {
	if(TIM4->SR & TIM_SR_CC1IF_Msk){
		if((GPIOB->IDR & GPIO_IDR_ID6_Msk) != 0 ){  				//rising edge
			count = 0;
			in = TIM4->CCR1;
			
		}
		else if ( (GPIOB->IDR & (GPIO_IDR_ID6_Msk))== 0){  //falling edge
			//timeInterval = count;
			//timeInterval = count*65535;
			//difference of instruction cycles: CCR1_FALLING - CCR_RISING = TIM4-CCR1 - in
			//timeInterval = (CCR1_FALLING - CCR_RISING) * time/instruction cycle
			//time(us)/instruction cycle = 1/16M * 1000 = 1/16 us/cycle
			timeInterval = (count * 65535 + TIM4->CCR1 - in) / 16;
		}
		TIM4->SR &=  (~TIM_SR_CC1IF_Msk);
	}
	
	if( (TIM4->SR & TIM_SR_UIF_Msk)!=0){
		count++;
		TIM4->SR &= (~TIM_SR_UIF_Msk);
	}
	
}

void Trigger_Setup() {

		//configure PE11
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN_Msk;
	
	 //configure TIM1 CH2 AF1
		GPIOE->MODER &=  ~(GPIO_MODER_MODE11_Msk);
	  GPIOE->MODER |=  GPIO_MODER_MODE11_1;  //10: Alternate function mode
	  GPIOE->AFR[1] |= GPIO_AFRH_AFSEL11_0;
	
	//Set PE11 to no pull-up, no pull-down.
	GPIOE->PUPDR &= ~GPIO_PUPDR_PUPD11_Msk;
	
	//Set the output type of PE11 to push-pull.
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT11_Msk; 
	
	//Set PE11 to very high output speed.
	GPIOE->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED11_Pos;
	
	//Enable timer 1 in RCC APB2ENR.
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk;
	
	//Set the prescaler to 15
	TIM1->PSC = 15;
	//Enable auto reload preload in the control register and set the auto reload value to its maximum value.
	TIM1->CR1 |= TIM_CR1_ARPE_Msk;
	TIM1->ARR |= 65535;
	
	//Set the CCR value that will trigger the sensor. (Hint: What is the timer clock frequency and what kind of signal activates the sensor?)
	TIM1->CCR2 	|= 11000;					//clock frequency = 16MHz/(PSC(15)+1)= 1 MHz
															//ARR is 65535 cycles; pulse period is 0.065536s = 65536 us  
															//we need 10 us to trigger; 
															//CCR1 is set to be at least 11 us. 

	//In the capture/compare modeTIM1 CH2 register, set the output control mode bits such that the timer is in PWM Mode 1 and enable the ouTtput compare preload.
	TIM1->CCMR1 |= 0b0110 << TIM_CCMR1_OC2M_Pos;
	//TIM1->CCMR1 &= ~(1<<24);
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE_Msk;
	TIM1->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;
	
	//Enable the output in the capture/compare enable register
	TIM1->CCER |= TIM_CCER_CC2E_Msk;
	
	//In the break and dead-time register, set the bits for main output enable and off-state selection for run mode.
	TIM1->BDTR |= TIM_BDTR_MOE_Msk;
	TIM1->BDTR |= TIM_BDTR_OSSR_Msk;
	TIM1->BDTR |= TIM_BDTR_OSSI_Msk;
	//Enable update generation in the event generation register.
	TIM1->EGR |= 1;
	//TIM1->EGR |= TIM_EGR_COMG_Msk;
	//TIM1->EGR |= TIM_EGR_CC2G_Msk;
	
	//Enable the update interrupt and clear the update interrupt flag.
	TIM1->DIER |= TIM_DIER_UIE_Msk;
	TIM1->SR &= ~TIM_SR_UIF_Msk;
	
	//Set the direction of the counter and enable the counter in the control register
	TIM1->CR1 &= ~TIM_CR1_DIR_Msk; //COUNT UP
	TIM1->CR1 |= TIM_CR1_CEN_Msk;
}

int main(void) {	
	//LED_Init();
	
	
	
	
	// Enable High Speed Internal Clock (HSI = 16 MHz)
	RCC->CR |= RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI is ready
	
	// Select HSI as system clock source 
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_HSI;
	while ((RCC->CFGR & RCC_CFGR_SWS) == 0); // Wait until HSI is system clock source
  
	// Input Capture Setup
	Input_Capture_Setup();
	
	// Trigger Setup
	Trigger_Setup();

	// Setup LCD
	LCD_Initialization();
	LCD_Clear();
	
	char temp[6];
	while(1) {
		// Code for Part C1 -- Comment out when demoing Part C2
		sprintf(temp,"%6d",((timeInterval)/1000));
		//TIM1->CCR2 	|= 20000;	
		// Code for Part C2 -- Comment out when demoing Part C1
		// TODO
		//timeInterval = 3;
		//sprintf(temp,"%6d",((timeInterval/65535)*4));
		
		//sprintf(temp,"%6d",((timeInterval)/148));
		LCD_DisplayString((uint8_t *) temp);
	}
}

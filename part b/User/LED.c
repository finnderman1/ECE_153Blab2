/*
 * ECE 153B - Winter 2020
 *
 * Name(s):
 * Section:
 * Lab: 2B
 */
 
#include "LED.h"

void LED_Init(void) {
	//reset GPIOAEN, GPIOBEN, GPIOEEN
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOEEN_Msk;// 0x00000013;

	//configure red led PB2
	GPIOB->MODER &=  ~(GPIO_MODER_MODE2_Msk); 	//reset IO mode
	GPIOB->MODER |=  GPIO_MODER_MODE2_0;  			//set to output 
	//GPIOB->MODER |=  GPIO_MODER_MODE2_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT2_Msk); 		//set to push pull
	
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD2_Msk); 		//no pull-up, no pull-down
	
	//configure green led PE8
	GPIOE->MODER &=  ~(GPIO_MODER_MODE8_Msk); 	//reset
	GPIOE->MODER |=  GPIO_MODER_MODE8_0;  			//set to output 

	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT8_Msk); 		//set to push pull
	
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk); 		//no pull-up, no pull-down
}

void Red_LED_Off(void) {
	GPIOB->ODR &= ~GPIO_ODR_OD2_Msk; //turn off pin B2 by resetting ODR reg
}

void Red_LED_On(void) {
	GPIOB->ODR |= GPIO_ODR_OD2_Msk;	 //turn on pin B2 by setting ODR reg
}

void Red_LED_Toggle(void){
	GPIOB->ODR ^= GPIO_ODR_OD2_Msk; //toggle pin B2 by setting ODR reg
}

void Green_LED_Off(void) {
	GPIOE->ODR &= ~GPIO_ODR_OD8_Msk; //turn on pin E8 by resetting ODR regs
}

void Green_LED_On(void) {
	GPIOE->ODR |= GPIO_ODR_OD8_Msk;  //turn on pin E8 by setting ODR regs
}

void Green_LED_Toggle(void) {
	GPIOE->ODR ^= GPIO_ODR_OD8_Msk; //toggle pin E8 by setting ODR reg
}



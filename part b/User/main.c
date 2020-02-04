/*
 * ECE 153B - Winter 2020
 *
 * Name(s):
 * Section:
 * Lab: 2B
 */
 
#include "stm32l476xx.h"

#include "LCD.h"
#include "LED.h"
#include "RTC.h"
#include "SysClock.h"

char strTime[12] = {0};
char strDate[12] = {0};

void RTC_Set_Alarm(void) {
	
	
	
	RTC->CR &= ~(RTC_CR_ALRAE_Msk | RTC_CR_ALRBE_Msk); //Alarm A,B disabled; P1074
	RTC_Disable_Write_Protection();
	/*
	program Alarm A to set off an alarm when the seconds field of the RTC is
	30 seconds; P1081
	*/	
	RTC->ALRMAR &= ~(RTC_ALRMAR_MSK1_Msk | RTC_ALRMAR_MSK2_Msk| RTC_ALRMAR_MSK3_Msk| RTC_ALRMAR_MSK4_Msk);
	//RTC->ALRMAR |=  RTC_ALRMAR_ST_1|RTC_ALRMAR_ST_0; //{ST[2:0],SU[3:0]} = 30 corrsponds to second tens(ST) and seconds units 0110000
	RTC->ALRMAR |=  RTC_ALRMAR_SU_1|RTC_ALRMAR_SU_0;
	/*
	Program Alarm B to set off an alarm every second; p 1082
	*/
	RTC->ALRMBR &= ~(RTC_ALRMBR_MSK1_Msk | RTC_ALRMBR_MSK2_Msk| RTC_ALRMBR_MSK3_Msk| RTC_ALRMBR_MSK4_Msk);
	//RTC->ALRMBR |= RTC_ALRMAR_SU_0; //Second units in BCD format
	
	
	RTC->CR |= (RTC_CR_ALRBIE_Msk | RTC_CR_ALRAIE_Msk | RTC_CR_ALRBE_Msk | RTC_CR_ALRAE_Msk); //Enable the alarm and its interrupt for both Alarm A and Alarm B
	
	RTC_Enable_Write_Protection();
}

void RTC_Alarm_Enable(void) {
	//SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI8;
	//SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PE;
	EXTI->FTSR1 |= EXTI_RTSR1_RT18_Msk; //P335, 1: Rising trigger enabled (for Event and Interrupt) for input line
	EXTI->IMR1 &= ~EXTI_IMR1_IM18_Msk;  //0: Interrupt request from Line x is masked
	EXTI->EMR1 &= ~EXTI_EMR1_EM18_Msk;  //0: Event request from line x is masked
	EXTI->PR1 |= EXTI_PR1_PIF18_Msk; //Clear the pending interrupt in EXTI PRx by writing a 1 to the bit that corresponds to the target EXTI line
	
	NVIC_EnableIRQ(RTC_Alarm_IRQn);			//Enable and mask bits control the NVIC interrupt channel corresponding to EXTI input line 0. 
	NVIC_SetPriority(RTC_Alarm_IRQn, 0);
}

void RTC_Alarm_IRQHandler(void) {
	//EXTI->SWIER1 &= ~EXTI_SWIER1_SWI18_Msk; //clear the alarm event flag, 
																					//This bit is cleared by clearing the corresponding bit in the EXTI_PR register (by writing a 1 into the bit).
																					//so we ignnore this line by commenting out.
	Red_LED_On();
	Green_LED_On();
	EXTI->PR1 |= EXTI_PR1_PIF18;  //This bit is cleared by writing a 1 to the bit.  clear the interrupt pending bit P336
	/*
	if(RTC->ALRMAR & RTC_ALRMAR_MSK1_Msk){
		Red_LED_Toggle();//When Alarm A is triggered, toggle the red LED
	}
	if(RTC->ALRMBR & RTC_ALRMBR_MSK1_Msk){
		Green_LED_Toggle();//When Alarm B is triggered, toggle the green LED
	}
	*/
	
}

int main(void) {	
	System_Clock_Init(); // Switch System Clock = 80 MHz
	LED_Init();
	LCD_Initialization();
	
	RTC_Init();

	RTC_Set_Alarm();
	RTC_Alarm_Enable();
	
	while(1) {
		Get_RTC_Calendar(strTime,strDate);
		LCD_DisplayString(strTime);
		// TODO
		//Red_LED_On();
		//Green_LED_On();
	}
}

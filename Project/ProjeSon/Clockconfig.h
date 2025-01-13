#include "stm32f10x.h"       

void initClockPLL(void)
{
	int FLAG = 0;
		RCC->CFGR &= ~RCC_CFGR_SW;
		if (RCC_CFGR_SWS_PLL && (FLAG==0))
		{			
			FLAG = 1;
		}

			if (FLAG==1){
			RCC->CFGR |= RCC_CFGR_SW_HSI;
			RCC->CFGR &= ~RCC_CFGR_SW;
			while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI){}
			RCC->CR &= ~RCC_CR_PLLON;
			while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLON){}
			RCC->CFGR |= RCC_CFGR_PPRE1_2;	
			RCC->CFGR &= ~RCC_CFGR_PLLMULL;			
			RCC->CFGR |= 	RCC_CFGR_PLLMULL9;
			RCC->CR |= RCC_CR_PLLON; 
			while(!(RCC->CR & RCC_CR_PLLRDY));
			RCC->CFGR |= RCC_CFGR_SW_PLL;
			while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}	
		}
}

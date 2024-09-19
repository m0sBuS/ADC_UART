#include <stm32f10x.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////

/* In this project build on STM32F103C8T6 Bluepill board																							*/
/* PA1 - Analog input																																									*/
/* PA9 - UART Tx																																											*/
/* PA10 - UART Rx																																											*/

////////////////////////////////////////////////////////////////////////////////////////////////////////

/* declarate functions */
void RCCPLL_Init(void);																				//Clock configuration function
void ADC_Init(void);																					//ADC configuration function
void UART_Init(void);																					//UART configuration function 

/* declarate interrupt functions */
void ADC1_2_IRQHandler(void);																	//ADC interrupt function
void USART1_IRQHandler(void);																	//UART interrupt function

/* declarate variables */
static uint16_t ADC_Data = 0;																	//ADC data buffer
static unsigned char USARTBuf = 0;														//UART Rx data buffer
static uint8_t TxON = 0;																			//ADC data transmission flag

/* main program */
int main(void)
{
	RCCPLL_Init();																							//Main clock configuration (72MHz)
	UART_Init();																								//UART configuration
	ADC_Init();																									//ADC configuration
	while(1)
		__WFI();																									//Wait for interrupt
}

/* Definition clock configuration function */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																		//Switching-on extended clock source
	while(!(RCC->CR & RCC_CR_HSERDY))														//HSE readiness waiting loop
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;				//Configuring PLL for multiply HSE by 9 to 72 MHz and divide APB1 periph clocks by 2 to 36 MHz according by reference manual
	RCC->CR |= RCC_CR_PLLON;																		//Switching-on PLL module
	while(!(RCC->CR & RCC_CR_PLLRDY))														//PLL readiness waiting loop
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																//Switching system clock source on PLL
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))											//Sysclock readiness waiting loop
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																		//Switching-off internal clock source
}

/* Definition ADC configuration function */
void ADC_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | 	//Switching-on ports A and C
									RCC_APB2ENR_ADC1EN;													//Switching-on ADC1
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;													//Switching-on Timer 3
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);						//Pin configuration (input and analog mode)
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);					//Led pin init(clear register)
	GPIOC->CRH |= GPIO_CRH_MODE13_1;														//Output mode and Push-Pull GPIO
	GPIOC->ODR |= GPIO_ODR_ODR13;																//Reset pin (reverse polarity LED)
	
	//STM32F103 ADC have short conversion time and can be work at 1 MSPS. But for UART 115200 8N1 streaming 1 us will be so fast and ADC conversion time must be slower
	// Conversion period is Fclk/BAUDRATE*(TxLENGTH + 1) - 1
	// Fclk - CPU clock
	// BAUDRATE - UART baudrate
	// TxLENGTH - Data length 
	// 72000000 / 115200 * (1 + 8 + 1 + 1) - 1 = 6874
	TIM3->ARR = 6874;
	TIM3->CR2 = TIM_CR2_MMS_1;																	//Timer as master configuration (triggering on timer update)
	TIM3->CR1 |= TIM_CR1_CEN;																		//Switching-on timer
	
	RCC->CFGR |= RCC_CFGR_ADCPRE_0 | RCC_CFGR_ADCPRE_1;					//Predivider on 8 according manual (ADC max clock is 14MHz)
	
	NVIC_EnableIRQ(ADC1_2_IRQn);																//ADC Watchdog interrupt enable
	
	ADC1->CR2 = ADC_CR2_ADON;																		//Switching-on ADC1
	ADC1->CR1 = ADC_CR1_AWDEN | ADC_CR1_AWDIE | 								//Watchdog configuration (Channel_1 and threshold and end of conversion interrupt)
							ADC_CR1_EOCIE | ADC_CR1_AWDCH_0;
	ADC1->CR2 |= 	ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG | 					//ADC configuration for Timer 3 TRGO event
								ADC_CR2_ALIGN;																//ADC left-side align 
	ADC1->HTR = 2867;																						//High threshold level Watchdog 4096 * 0.7 = 2867
	ADC1->SQR3 = ADC_SQR3_SQ1_0;																//sequence length for channel 1(2 conversions)
}

/* Definition UART configuration function */
void UART_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;	//Switching-on port A and UART
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |						//USART1 pins configuration (clear register)
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |						//Alternate function for PA9 and fastest output
								GPIO_CRH_CNF10_1;															//Alternate function
	
	NVIC_EnableIRQ(USART1_IRQn);																//USART1 interrupt enable
	
	USART1->BRR = 0x271;																				//115200 baudrate divider from reference manual
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_TE | 						//Rx interrupt and switching-on receiver and transmitter
								USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;																//Switching-on USART1
}

/* Definition ADC interrupt function */
void ADC1_2_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_AWD)																	//Check ADC status register for Watchdog event
	{
		ADC1->SR &= (uint32_t)~ADC_SR_AWD;												//Clear Watchdog flag
		GPIOC->ODR &= (uint32_t)~GPIO_ODR_ODR13;									//LED turn on
	}
	if (ADC1->SR & ADC_SR_EOC)																	//Check ADC status register for end of conversion event
	{
		ADC_Data = (uint16_t)ADC1->DR;														//ADC read
		if (TxON)																									//if ADC data transmission flag is on
			USART1->DR = *(uint8_t*)((uint32_t)&ADC_Data+1);				//Transmitting ADC data (high byte)
	}
}

/* Definition UART interrupt function */
void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_RXNE)															//Check UART status register for non empty receiver buffer event
	{
		USART1->SR &= ~USART_SR_RXNE;															//Clear flag
		USARTBuf = (unsigned char)USART1->DR;											//Data reading
		if (USARTBuf == 'e' || USARTBuf == 'E')										//if received data is ADC data transmission enable
			TxON = 1;																								//ADC data transmission is on
		if (USARTBuf == 'd' || USARTBuf == 'D')										//if received data is ADC data transmission disable
			TxON = 0;																								//ADC data transmission is off
		if (USARTBuf == 'r' || USARTBuf == 'R')										//if received data is threshold LED disable
			GPIOC->ODR |= GPIO_ODR_ODR13;														//Switching-off LED
	}
}

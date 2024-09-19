#include <stm32f10x.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ������������ ���������� STM32F103C8T6 �� �������� ����� Bluepill 																	*/
/* PA1 - ������� ���������� ������																																		*/
/* PA9 - UART Tx																																											*/
/* PA10 - UART Rx																																											*/

////////////////////////////////////////////////////////////////////////////////////////////////////////

//�������������� �������
void RCCPLL_Init(void);																				//������� ������� �������� �������
void ADC_Init(void);																					//������� ������������� ���
void UART_Init(void);																					//������� ������������� UART 

//�������������� ����������
void ADC1_2_IRQHandler(void);																	//������� ���������� �� ���
void USART1_IRQHandler(void);																	//������� ���������� �� UART

//����������� ����������
static uint16_t ADC_Data = 0;																	//������ � ���
static unsigned char USARTBuf = 0;														//����� ������ UART
static uint8_t TxON = 0;																			//���� ���������� �������� ������

//�������� ���������
int main(void)
{
	RCCPLL_Init();																							//������� �������� ������� (72���)
	UART_Init();																								//������������� UART
	ADC_Init();																									//������������� ���
	while(1)
		__WFI();																									//�������� ����������
}

/* ��������� ������� ������� �������� ������� */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																		//��������� �������� ��������� �������
	while(!(RCC->CR & RCC_CR_HSERDY))														//�������� ���������� ������ �������� ��������� �������
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;				//���������������� ���� � ���������� ������� ������� �� 9 �� 72��� � ������� ������� APB1 �� 2 �� 36 ��� �������� reference manual
	RCC->CR |= RCC_CR_PLLON;																		//��������� ����
	while(!(RCC->CR & RCC_CR_PLLRDY))														//�������� ���������� ������ ����
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																//������������ ��������� ������� �� ����
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))											//�������� ���������� ������������
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																		//���������� ����������� ��������� ����������
}

/* ����������� ������� ������������� ��� */
void ADC_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | 	//��������� ������ � � �
									RCC_APB2ENR_ADC1EN;													//��������� ��� 1
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;													//��������� ������� 3
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);						//������������� ����� ��� (����� 1)
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);					//������������� ���������� PC13
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOC->ODR |= GPIO_ODR_ODR13;																//����� ���������� (������� �� ��������� ����������)
	
	//������������ �������� ������ �� Datasheet 1MSPS, �� � ����� ������ ��� ��������� �������� ���������� ���������� ��� ������� �������� UART 115200 8N1
	// ������ ��������� �� ������� Fclk/BAUDRATE*(TxLENGTH + 1) - 1, ���
	// Fclk - �������� �������
	// BAUDRATE - ������� �������� 1 ����
	// TxLENGTH - ����� ������� 
	// 72000000 / 115200 * (1 + 8 + 1 + 1) - 1 = 6874
	TIM3->ARR = 6874;
	TIM3->CR2 = TIM_CR2_MMS_1;																	//������������� ������ �� ������ �� ���������� � ��������� ������� TRGO
	TIM3->CR1 |= TIM_CR1_CEN;																		//�������� ������
	
	RCC->CFGR |= RCC_CFGR_ADCPRE_0 | RCC_CFGR_ADCPRE_1;					//��������� ������������ �������� ������� ��� �� 8, �.�. ������� �� ������ ��������� 14��� �������� reference manual
	
	NVIC_EnableIRQ(ADC1_2_IRQn);																//��������� ���������� �� ������� Watchdog
	
	ADC1->CR2 = ADC_CR2_ADON;																		//��������� ��� 1
	ADC1->CR1 = ADC_CR1_AWDEN | ADC_CR1_AWDIE | 								//���������������� ���������� �� ������� Watchdog � ����� ������ Watchdog
							ADC_CR1_EOCIE | ADC_CR1_AWDCH_0;
	ADC1->CR2 |= 	ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG | 					//���������������� ��� �� ������ �� �������� ��������� ������� (������ TRGO ������� 3)
								ADC_CR2_ALIGN;																//������������ ������ �����
	ADC1->HTR = 2867;																						//��������� �������� ������ ������������ Watchdog 4096 * 0.7 = 2867
	ADC1->SQR3 = ADC_SQR3_SQ1_0;																//������������ ��������� ��� ������ 1(2 ���������)
}

/* ����������� ������� ������������� UART */
void UART_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;	//��������� ����� � � ���������� USART1
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |						//������������� ����� ��� USART1
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |
								GPIO_CRH_CNF10_1;
	
	NVIC_EnableIRQ(USART1_IRQn);																//��������� ���������� �� USART1
	
	USART1->BRR = 0x271;																				//���������� �������� 115200 (����� ������� �� reference manual)
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_TE | 						//��������� ���������� �� ����� ������ � �������� �������� � �����������
								USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;																//��������� USART1
}

/* ����������� ������� ���������� �� ��� */
void ADC1_2_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_AWD)																	//����� �������� ������� �� ������� Watchdog
	{
		ADC1->SR &= (uint32_t)~ADC_SR_AWD;												//����� ����� Watchdog
		GPIOC->ODR &= (uint32_t)~GPIO_ODR_ODR13;									//��������� ����������
	}
	if (ADC1->SR & ADC_SR_EOC)																	//������ DMA (����������)
	{
		ADC_Data = (uint16_t)ADC1->DR;
		if (TxON)
			USART1->DR = *(uint8_t*)((uint32_t)&ADC_Data+1);
	}
}

/* ����������� ������� ���������� �� UART */
void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_RXNE)															//����� �������� ������� �� ������� ������ � ������
	{
		USART1->SR &= ~USART_SR_RXNE;															//����� �����
		USARTBuf = (unsigned char)USART1->DR;											//������ ������ �� ������
		if (USARTBuf == 'e' || USARTBuf == 'E')										//������� ��������� �������� ������
			TxON = 1;
		if (USARTBuf == 'd' || USARTBuf == 'D')										//������� ���������� �������� ������
			TxON = 0;
		if (USARTBuf == 'r' || USARTBuf == 'R')										//������� ������ ����������
			GPIOC->ODR |= GPIO_ODR_ODR13;
	}
}

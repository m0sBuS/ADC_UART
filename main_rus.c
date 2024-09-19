#include <stm32f10x.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Используемое устройство STM32F103C8T6 на макетной плате Bluepill 																	*/
/* PA1 - Входной аналоговый сигнал																																		*/
/* PA9 - UART Tx																																											*/
/* PA10 - UART Rx																																											*/

////////////////////////////////////////////////////////////////////////////////////////////////////////

//Декларирование функций
void RCCPLL_Init(void);																				//Функция задания тактовой частоты
void ADC_Init(void);																					//Функция инициализации АЦП
void UART_Init(void);																					//Функции инициализации UART 

//Декларирование прерываний
void ADC1_2_IRQHandler(void);																	//Функции прерывания по АЦП
void USART1_IRQHandler(void);																	//Функции прерывания по UART

//Определение переменных
static uint16_t ADC_Data = 0;																	//Данные с АЦП
static unsigned char USARTBuf = 0;														//Буфер данных UART
static uint8_t TxON = 0;																			//Флаг разрешение передачи данных

//Основная программа
int main(void)
{
	RCCPLL_Init();																							//Задание тактовой частоты (72МГц)
	UART_Init();																								//Инициализация UART
	ADC_Init();																									//Инициализация АЦП
	while(1)
		__WFI();																									//Ожидание прерывания
}

/* Опредение функции задания тактовой частоты */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																		//Включение внешнего тактового сигнала
	while(!(RCC->CR & RCC_CR_HSERDY))														//Ожидание готовности работы внешнего тактового сигнала
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;				//Конфигурирование ФАПЧ с умножением входной частоты на 9 до 72МГц и деление частоты APB1 на 2 до 36 МГц согласно reference manual
	RCC->CR |= RCC_CR_PLLON;																		//Включение ФАПЧ
	while(!(RCC->CR & RCC_CR_PLLRDY))														//Ожидание готовности работы ФАПЧ
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																//Переключение тактового сигнала на ФАПЧ
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))											//Ожидание готовности переключения
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																		//Отключение внутреннего тактового генератора
}

/* Определение функции инициализации АЦП */
void ADC_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | 	//Включение портов А и С
									RCC_APB2ENR_ADC1EN;													//Включение АЦП 1
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;													//Включение таймера 3
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);						//Инициализация порта АЦП (Канал 1)
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);					//Инициализация светодиода PC13
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOC->ODR |= GPIO_ODR_ODR13;																//Сброс светодиода (распаян на обратоной полярности)
	
	//Максимальный тактовый сигнал по Datasheet 1MSPS, но в нашем случае для потоковой передачи необходимо настроится под частоты передачи UART 115200 8N1
	// Период считается по формуле Fclk/BAUDRATE*(TxLENGTH + 1) - 1, где
	// Fclk - Тактовая частота
	// BAUDRATE - Частота передачи 1 бода
	// TxLENGTH - Длина посылки 
	// 72000000 / 115200 * (1 + 8 + 1 + 1) - 1 = 6874
	TIM3->ARR = 6874;
	TIM3->CR2 = TIM_CR2_MMS_1;																	//Конфигурируем таймер на работу по обновлению и генерацию события TRGO
	TIM3->CR1 |= TIM_CR1_CEN;																		//Включаем таймер
	
	RCC->CFGR |= RCC_CFGR_ADCPRE_0 | RCC_CFGR_ADCPRE_1;					//Установка предделителя тактовой частоты АЦП на 8, т.к. частота не должна превышать 14МГц согласно reference manual
	
	NVIC_EnableIRQ(ADC1_2_IRQn);																//Включение прерывания по событию Watchdog
	
	ADC1->CR2 = ADC_CR2_ADON;																		//Включение АЦП 1
	ADC1->CR1 = ADC_CR1_AWDEN | ADC_CR1_AWDIE | 								//Конфигурирование прерывания по событию Watchdog и выбор канала Watchdog
							ADC_CR1_EOCIE | ADC_CR1_AWDCH_0;
	ADC1->CR2 |= 	ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG | 					//Конфигурирование АЦП по работе от внешнего источника события (сигнал TRGO таймера 3)
								ADC_CR2_ALIGN;																//Выравнивания данных влево
	ADC1->HTR = 2867;																						//Установка верхнего порога срабатывания Watchdog 4096 * 0.7 = 2867
	ADC1->SQR3 = ADC_SQR3_SQ1_0;																//Длительность измерения для Канала 1(2 измерения)
}

/* Определение функции инициализации UART */
void UART_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;	//Включение порта А и интерфейса USART1
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |						//Инициализация порта под USART1
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |
								GPIO_CRH_CNF10_1;
	
	NVIC_EnableIRQ(USART1_IRQn);																//Включение прерывания по USART1
	
	USART1->BRR = 0x271;																				//Подстройка бодрейта 115200 (Взято готовое из reference manual)
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_TE | 						//Включение прерывания по приёму данных и ключение приёмника и передатчика
								USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;																//Включение USART1
}

/* Определение функции прерывания по АЦП */
void ADC1_2_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_AWD)																	//Опрос регистра статуса по событию Watchdog
	{
		ADC1->SR &= (uint32_t)~ADC_SR_AWD;												//Сброс флага Watchdog
		GPIOC->ODR &= (uint32_t)~GPIO_ODR_ODR13;									//Включение светодиода
	}
	if (ADC1->SR & ADC_SR_EOC)																	//Сделаю DMA (Постараюсь)
	{
		ADC_Data = (uint16_t)ADC1->DR;
		if (TxON)
			USART1->DR = *(uint8_t*)((uint32_t)&ADC_Data+1);
	}
}

/* Определение функции прерывания по UART */
void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_RXNE)															//Опрос регистра статуса по наличию данных в буфере
	{
		USART1->SR &= ~USART_SR_RXNE;															//Сброс флага
		USARTBuf = (unsigned char)USART1->DR;											//Чтение данных из буфера
		if (USARTBuf == 'e' || USARTBuf == 'E')										//Условие включения отправки данных
			TxON = 1;
		if (USARTBuf == 'd' || USARTBuf == 'D')										//Условие выключения отправки данных
			TxON = 0;
		if (USARTBuf == 'r' || USARTBuf == 'R')										//Условие сброса светодиода
			GPIOC->ODR |= GPIO_ODR_ODR13;
	}
}

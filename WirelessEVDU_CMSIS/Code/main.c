#include "main.h"

uint8_t ClockInit()
{
	/* ----------- ������ HSE ----------- */
	RCC->CR |= RCC_CR_HSEON;

	for (uint32_t StartUpCounter = 0; ; StartUpCounter++)
	{
	    if (RCC->CR & RCC_CR_HSERDY) break;

	    /* ���� HSE �� �������� - ����� � ������� */
	    if (StartUpCounter > 0x1000)
	    {
			RCC->CR &= ~RCC_CR_HSEON;	// ��������� HSE
			return 1;
	    }
	}

	/* ----------- ��������� � ������ PLL ----------- */
	RCC->CFGR |= RCC_CFGR_PLLMUL8 	// PLLMUL = 8
			  |  RCC_CFGR_PLLDIV2 	// PLLDIV = 2
			  |  RCC_CFGR_PLLSRC;	// ������������ PLL �� HSE
	RCC->CR   |= RCC_CR_PLLON;		// ������ PLL


	for (uint32_t StartUpCounter = 0; ; StartUpCounter++)
	{
		if (RCC->CR & RCC_CR_PLLRDY) break;

		/* ���� HSE �� �������� - ����� � ������� */
		if (StartUpCounter > 0x1000)
		{
			RCC->CR &= ~RCC_CR_HSEON;	// ��������� HSE
			RCC->CR &= ~RCC_CR_PLLON; 	// ��������� PLL
			return 2;
		}
	}

	/*----------- FLASH � �������� ----------- */
	FLASH->ACR  = FLASH_ACR_ACC64;
	FLASH->ACR |= FLASH_ACR_LATENCY 	// 1 ���� �������� Flash, �.�. ������� 32 MHz
			   |  FLASH_ACR_PRFTEN;	 	// ����� ��������������� ������� ���, �.�. ������� > 24 MHz


	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1 	// �������� ���� APB2 = 1
	          |  RCC_CFGR_PPRE1_DIV1 	// �������� ���� APB1 = 1
	          |  RCC_CFGR_HPRE_DIV1; 	// �������� AHB = 1


	/* ----------- ������ ������������ �� PLL ----------- */
	RCC->CFGR |= RCC_CFGR_SW_PLL;								// ������������� �� ������ �� PLL
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);	// �������� ������������

	/* ----------- ���������� ��������� ----------- */
	RCC->CR &= ~RCC_CR_HSION;									// ���������� HSI ����� ������� PLL
	RCC->CR &= ~RCC_CR_MSION;									// ���������� MSI ����� ������� PLL

	/* ����� */
	return 0;
}
void PortInit()
{
	/* ��������� ������������ �� ����� A, B, C � H */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN	// GPIOA Clock Enable
				|  RCC_AHBENR_GPIOBEN	// GPIOB Clock Enable
				|  RCC_AHBENR_GPIOCEN	// GPIOC Clock Enable
				|  RCC_AHBENR_GPIOHEN;	// GPIOH Clock Enable

	/* PA0 - USR_BUT  - GPIO_Input */
	GPIOA->MODER |= (GPIO_INPUT << GPIO_MODER_MODER0_Pos);		// USR_BUT  GPIO_Input

	/* PB7 - GRN_LED - GPIO_Output
	 * PB6 - BLUE_LED - GPIO_Output */
	GPIOB->MODER  |=  (GPIO_OUTPUT << GPIO_MODER_MODER6_Pos)	// GRN_LED GPIO_Output
				  |   (GPIO_OUTPUT << GPIO_MODER_MODER7_Pos);	// BLUE_LED GPIO_Output
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8);	// GRN_LED and BLUE_LED Output push-pull

	/* PC9 - DIR - GPIO_Output, Pull-up */
	GPIOC->MODER   |= (GPIO_OUTPUT << GPIO_MODER_MODER9_Pos); 	// DIR GPIO_Output
	GPIOC->OTYPER  &= ~GPIO_OTYPER_OT_9; 						// DIR Output push-pull
	GPIOC->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR9;					// DIR Very high speed
	GPIOC->PUPDR   |= (0x01 << GPIO_PUPDR_PUPDR9_Pos);			// DIR Pull-up
}
void UART1_Init()
{
	/* GPIO */
	GPIOA->MODER   |= (GPIO_ALTER << GPIO_MODER_MODER9_Pos)		// UART1_TX Alternative Function
				   |  (GPIO_ALTER << GPIO_MODER_MODER10_Pos); 	// UART1_RX Alternative Function
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR9;					// UART1_TX Very high speed
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_9;						// UART1_TX Output push-pull
	GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR10;						// UART1_RX No pull-up, pull-down
	GPIOA->PUPDR   |= (0x01 << GPIO_PUPDR_PUPDR9_Pos);			// UART1_TX Pull-up
	GPIOA->AFR[1]  |= (0x07 << GPIO_AFRH_AFSEL9_Pos)			// UART1_TX AFIO7
				   |  (0x07 << GPIO_AFRH_AFSEL10_Pos);			// UART1_RX AFIO7

	/* UART1 */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 						// ������ ������������ �� ���� � 32���

	USART1->BRR = (F_CPU + UART1_BAUD / 2) / UART1_BAUD;		// 9600
	USART1->CR1 |= (USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE);	// Uart Enable, TX, RX
	//USART1->CR1 |= USART_CR1_RXNEIE;							// RX Interrupt

	NVIC_EnableIRQ(USART1_IRQn); 								//�������� ����������, ��������� ������
}
void UART2_Init()
{
	/* GPIO */
	GPIOA->MODER   |= (GPIO_ALTER << GPIO_MODER_MODER2_Pos)		// UART2_TX Alternative Function
				   |  (GPIO_ALTER << GPIO_MODER_MODER3_Pos);	// UART2_RX Alternative Function
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR2;					// UART2_TX Very high speed
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_2;						// UART2_TX Output push-pull
	GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR3;						// UART2_RX No pull-up, pull-down
	GPIOA->PUPDR   |= (0x01 << GPIO_PUPDR_PUPDR2_Pos);			// UART2_TX Pull-up
	GPIOA->AFR[0]  |= (0x07 << GPIO_AFRL_AFSEL2_Pos)			// UART2_TX AFIO7
				   |  (0x07 << GPIO_AFRL_AFSEL3_Pos);			// UART2_RX AFIO7

	/* UART1 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; 						// ������ ������������ �� ����

	USART2->BRR = (F_CPU + UART2_BAUD / 2) / UART2_BAUD;		// 9600
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;	// Uart Enable, TX, RX
	USART2->CR1 |= USART_CR1_RXNEIE;							// RX Interrupt

	NVIC_EnableIRQ(USART2_IRQn); 								//�������� ����������, ��������� ������
}
void IwdgInit()
{
	IWDG->KR  = 0x5555;		// ������ � ������ ���������
	IWDG->PR  = 5;			// �������� 128
	IWDG->RLR = 1250;		// �������. ��� � 4 �������.
	IWDG->KR  = 0xAAAA;		// ����������
	IWDG->KR  = 0xCCCC;		// �����
}

void IwdgReset()
{
	IWDG->KR = 0xAAAA;
}

int main(void)
{
	ClockInit();
	PortInit();

	SysTick_Config(TimerTick);
	IwdgInit();
	UART1_Init();
	UART2_Init();

	i2cInit();
	fl_get_offset = 3;
	swtimer_t ST_BLINK;
	swTimerSet(&ST_BLINK, 0, 500);

	for (uint8_t i = 0; i < 10; i++)
	{
		PIN_TOGGLE(GRN_LED_Port, GRN_LED_Pin);
		PIN_TOGGLE(BLUE_LED_Port, BLUE_LED_Pin);
		Delay(500000);
	}

	while(1)
	{
		FSM_UART1();
		FSM_UART2();
		if (swTimerCheck(&ST_BLINK)) { PIN_TOGGLE(GRN_LED_Port, GRN_LED_Pin); }
		i2cRead(0x20);
		IwdgReset();
	}

	return 0;
}

void mco_init(void)
{
	/* �������� ������������ - PLL, �������� �� 16 �� A8 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // �������� ������������ ����� �

	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_8;  // ���������� ���� CNF ��� ���� 8. ����� 00 - Push-Pull
	GPIOA->MODER |= (GPIO_ALTER << GPIO_MODER_MODER8_Pos); // ������ ����� ��� 8 �� ���� ����� CNF  = 10 (�������������� �������, Push-Pull)
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL0;	// ���������� ���� MODE ��� ���� 8
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR8;	// ���������� ��� MODE ��� ������ ����. ����� MODE11 = Max Speed 50MHz

	RCC->CFGR |= RCC_CFGR_MCOPRE_DIV16;	// Microcontroller clock output prescaler
	RCC->CFGR |= RCC_CFGR_MCOSEL_SYSCLK;	// Microcontroller clock output selection
}
void Delay(uint32_t Val)
{
	for( ; Val != 0; Val--)
	{
		asm("nop");
	}
}

#include "Interrupt.h"

volatile uint32_t sysTick = 0;

uint32_t GetTick()
{
	return sysTick;
}

void SysTick_Handler(void)
{
	sysTick++;
	millis++;
}

volatile uint8_t ch = 0;

void USART1_IRQHandler(void)
{
	/* Если что-то пришло в буфер */
	if ((USART1->CR1 & USART_CR1_RXNEIE) && (USART1->SR & USART_SR_RXNE))
	{
		USART1->SR &= ~USART_SR_RXNE;	// Сброс флага
		UART1_RXNE_Interrupt();
	}

	/* Если буфер опустошился (отправили байт) */
	if ((USART1->CR1 & USART_CR1_TXEIE) && (USART1->SR & USART_SR_TXE))
	{
		USART1->SR &= ~USART_SR_TXE;	// Сброс флага
		UART1_TXE_Interrupt();
	}

	if ((USART1->CR1 & USART_CR1_TCIE) && (USART1->SR & USART_SR_TC))
	{
		USART1->SR &= ~USART_SR_TC;		// Сброс флага
		tx1Flag = 1;
		START_UART1_READ;
	}
}

void USART2_IRQHandler(void)
{
	if ((USART2->CR1 & USART_CR1_RXNEIE) && (USART2->SR & USART_SR_RXNE))
	{
		USART2->SR &= ~USART_SR_RXNE;	// Сброс флага
		UART2_RXNE_Interrupt();
	}

	/* Если буфер опустошился (отправили байт) */
	if ((USART2->CR1 & USART_CR1_TXEIE) && (USART2->SR & USART_SR_TXE))
	{
		USART2->SR &= ~USART_SR_TXE;	// Сброс флага
		UART2_TXE_Interrupt();
	}

	if ((USART2->CR1 & USART_CR1_TCIE) && (USART2->SR & USART_SR_TC))
	{
		USART2->SR &= ~USART_SR_TC;		// Сброс флага

		tx2Flag = 1;
		START_UART2_READ;
	}
}

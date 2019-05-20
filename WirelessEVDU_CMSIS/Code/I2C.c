#include "I2C.h"

/*
 * PB8 - SCL
 * PB9 - SDA
 * */

void i2cInit(void)
{
	/* RCC */
	RCC->APB1ENR |=	RCC_APB1ENR_I2C1EN;		// Тактирование I2C
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;		// GPIOB Clock Enable

	/* SCL -> PB8 */
	GPIOB->MODER |=  (GPIO_ALTER << GPIO_MODER_MODER8_Pos);		// Альтернативная функция
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8;							// UART1_TX Output push-pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;					// UART1_TX Very high speed
	GPIOB->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL8_Pos);			// SCL AFIO4

	/* SDA -> PB9 */
	GPIOB->MODER |= (GPIO_ALTER << GPIO_MODER_MODER8_Pos);		// Альтернативная функция
	GPIOB->OTYPER |= GPIO_OTYPER_OT_9;							// UART1_TX Output push-pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;					// UART1_TX Very high speed
	GPIOB->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL9_Pos);			// SDA AFIO4

	/* I2C */
	I2C1->CR1 &= ~I2C_CR1_SMBUS;				// I2C Mode
	I2C1->CR2 &= ~I2C_CR2_FREQ;					// Указываем частоту тактирования модуля
	I2C1->CR2 |= 32 << I2C_CR2_FREQ_Pos; 		// 32 MHz
	I2C1->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY); 	// Конфигурируем I2C, standart mode, 100 KHz duty cycle 1/2
	I2C1->CCR |= 156; 							// Задаем частоту работы модуля SCL по формуле 10 000nS/(2*APB1) : 10 000ns/64ns = 156
	I2C1->TRISE = 33; 							// (1000nS / 31nS)+1 [Standart_Mode = 1000nS, Fast_Mode = 300nS, 1/42MHz = 31nS]s
	I2C1->CR1 |= I2C_CR1_PE; 					// Включаем модуль
}


void i2cWrite(uint8_t reg_addr, uint8_t data)
{
    // Стартуем
    I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	(void) I2C2->SR1;

    // Передаем адрес устройства
	I2C2->DR = AT24CXX_ADDR_WRITE;//I2C_ADDRESS(EEPROM_I2C_ADDR, I2C_MODE_WRITE);
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR1;
	(void) I2C2->SR2;

    // Передаем адрес регистра
	I2C2->DR = reg_addr;
	while(!(I2C2->SR1 & I2C_SR1_TXE));

    // Пишем данные
	I2C2->DR = data;
	while(!(I2C2->SR1 & I2C_SR1_BTF));
	I2C2->CR1 |= I2C_CR1_STOP;
}

uint8_t i2cRead(uint8_t reg_addr)
{
	uint8_t data;

	// Стартуем
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	(void) I2C2->SR1;

	// Передаем адрес устройства
	I2C2->DR = AT24CXX_ADDR_WRITE;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR1;
	(void) I2C2->SR2;

	// Передаем адрес регистра
	I2C2->DR = reg_addr;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->CR1 |= I2C_CR1_STOP;

	// Рестарт!!!
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	(void) I2C2->SR1;

	// Передаем адрес устройства, но теперь для чтения
	I2C2->DR = AT24CXX_ADDR_READ;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR1;
	(void) I2C2->SR2;

	// Читаем
	I2C2->CR1 &= ~I2C_CR1_ACK;
	while(!(I2C2->SR1 & I2C_SR1_RXNE));
	data = I2C2->DR;
	I2C2->CR1 |= I2C_CR1_STOP;

	return data;
}


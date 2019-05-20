#include "stm32l1xx.h"
#include "Interrupt.h"
#include "Errors.h"
#include "SwTimer.h"
#include "UART.h"
#include "I2C.h"

#define GPIO_INPUT 0x00
#define GPIO_OUTPUT 0x01
#define GPIO_ALTER 0x02

#define F_CPU 		32000000UL
#define TimerTick  	F_CPU/1000-1	// Нам нужен килогерц

#define UART1_BAUD 9600UL
#define UART2_BAUD 9600UL

#define GRN_LED_Port GPIOB
#define GRN_LED_Pin 7
#define BLUE_LED_Port GPIOB
#define BLUE_LED_Pin 6
#define DIR_Port GPIOC
#define DIR_Pin 9

#define PIN_ON(PORT, PIN)  PORT->BSRR=(GPIO_BSRR_BS_0 << PIN)
#define PIN_OFF(PORT, PIN)  PORT->BSRR=(GPIO_BSRR_BR_0 << PIN)
#define PIN_TOGGLE(PORT, PIN) PORT->ODR ^= (GPIO_ODR_ODR_0 << PIN)


void Delay(uint32_t Val);

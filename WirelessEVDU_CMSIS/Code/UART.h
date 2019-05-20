#ifndef UART_H_
#define UART_H_

#include "main.h"

#define DIR_WRITE  PIN_ON(DIR_Port, DIR_Pin)
#define DIR_READ   PIN_OFF(DIR_Port, DIR_Pin)

#define START_UART1_SEND 	DIR_WRITE; 	tx1BufferIndex = 0; ENABLE_TX1
#define START_UART1_READ 	DIR_READ;  	rx1BufferIndex = 0; ENABLE_RX1
#define START_UART2_SEND 				tx2BufferIndex = 0; ENABLE_TX2
#define START_UART2_READ 				rx2BufferIndex = 0; ENABLE_RX2

#define ENABLE_RX1	USART1->CR1 =   (USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE)
#define ENABLE_TX1	USART1->CR1 =   (USART_CR1_UE | USART_CR1_TE | USART_CR1_TXEIE)
#define DISABLE_RX1	USART1->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE)
#define DISABLE_TX1	USART1->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE)

#define ENABLE_RX2	USART2->CR1 =   (USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE)
#define ENABLE_TX2	USART2->CR1 =   (USART_CR1_UE | USART_CR1_TE | USART_CR1_TXEIE)
#define DISABLE_RX2	USART2->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE)
#define DISABLE_TX2	USART2->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE)

#define DISABLE_UART1 USART1->CR1 = USART_CR1_UE
#define DISABLE_UART2 USART2->CR1 = USART_CR1_UE

#define SIZE_RX1_BUFFER 8
#define SIZE_TX1_BUFFER 4
#define SIZE_RX2_BUFFER 4
#define SIZE_TX2_BUFFER 8

#define NUMBER_TRY_RECIVE_RESPONSE 	20
#define NUMBER_TRY_SET_COEF        	4
#define NUMBER_BT_PARAM 			2		// Кол-во байт в параметрах в пакете Bluetooth
#define NUMBER_MEASURE 				20		// Кол-во замеров для усреднения веса при калибровке

#define SLAVE1_ID 101
#define SLAVE2_ID 102
#define SLAVE3_ID 103

#define accuracy 		((int64_t)10000000)	// 0,0000001
#define accuracy_scale  ((int64_t)100000)	// 0,00001
#define MAX_ALPHA 		((int64_t)65536)	// Максимально возможная альфа для ФНЧ
#define K_ALPHA 		16					// Битность максимальной альфы для ФНЧ

#define com_get_weight      0	// Команда запроса веса
#define com_set_coefficient 1	// Команда установки коэффициента
#define com_get_coefficient 2	// Команда запроса коэффициента

void UART_Init();
void UART1_RXNE_Interrupt();
void UART1_TXE_Interrupt();
void UART2_RXNE_Interrupt();
void UART2_TXE_Interrupt();
void FSM_UART1();
void FSM_UART2();
void RunSetCoefficient();

typedef struct
{
	int64_t weight_1;			// Вес с датчика 1
	int64_t weight_1_offset;	// Вес тары для 1 датчика
	int64_t coefficient_1;		// Коэффициент 1 датчика
	int64_t reserved_offset_1; 	// Калибровочный offset 1 датчика

	int64_t weight_2;			// Вес с датчика 2
	int64_t weight_2_offset;	// Вес тары для 2 датчика
	int64_t coefficient_2;		// Коэффициент 2 датчика
	int64_t reserved_offset_2; 	// Калибровочный offset 2 датчика

	int64_t weight_3;			// Вес с датчика 3
	int64_t weight_3_offset;	// Вес тары для 3 датчика
	int64_t coefficient_3;		// Коэффициент 3 датчика
	int64_t reserved_offset_3; 	// Калибровочный offset 3 датчика

	int64_t total_weight;		// Сумма весов с трех датчиков без учета тары

	int64_t alpha;				// Коэффициент для ФНЧ
	int64_t scale;				// Коэффициент АЦП -> Кг

} DataSensors;

typedef union
{
	int32_t value;
	uint8_t bytes[4];
} DataWeight;

typedef struct
{
	uint8_t command;
	uint8_t id;
	uint8_t param[2];
} bt_command_t;

extern volatile uint8_t tx1BufferIndex;
extern volatile uint8_t rx1BufferIndex;
extern volatile uint8_t rx1Buffer[SIZE_RX1_BUFFER];
extern volatile uint8_t tx1Buffer[SIZE_TX1_BUFFER];
extern volatile uint8_t rx1Flag;
extern volatile uint8_t tx1Flag;
extern volatile uint8_t tx2BufferIndex;
extern volatile uint8_t rx2BufferIndex;
extern volatile uint8_t rx2Buffer[SIZE_RX2_BUFFER];

//extern volatile uint8_t rx2Flag;
extern volatile uint8_t tx2Flag;

extern volatile int64_t millis;

extern DataSensors stDataSensors;
extern uint8_t fl_get_offset;

#endif /* UART_H_ */

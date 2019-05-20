#include "UART.h"

enum { FSM_OFF, STATE_SEND_REQUEST, STATE_RECEIVED_DATA };
enum { ID0,	ID1, DATA0,	DATA1, DATA2, DATA3, MASK, CRC8 };
enum {COMMAND, ID, PARAM1, PARAM2};
enum {
	GET_WEIGHT = 200,
	GET_COEFFICIENT = 202,		// CA
	SET_COEFFICIENT = 204,
	ALPHA = 206,
	CALIBRATION = 208,			// D0
	OFFSET = 210				// D2
};

#define RunCalibration() 	state_calibration = 1
#define CancelCalibration() state_calibration = 0

/* Для прерываний UART'ов */
volatile uint8_t tx1BufferIndex = 0;
volatile uint8_t rx1BufferIndex = 0;
volatile uint8_t tx2BufferIndex = 0;
volatile uint8_t rx2BufferIndex = 0;

volatile uint8_t rx1Flag = 0;
volatile uint8_t tx1Flag = 0;
volatile uint8_t rx2Flag = 0;
volatile uint8_t tx2Flag = 0;

volatile uint8_t rx1Buffer[SIZE_RX1_BUFFER];
volatile uint8_t tx1Buffer[SIZE_TX1_BUFFER];
volatile uint8_t rx2Buffer[SIZE_RX2_BUFFER];
volatile uint8_t tx2Buffer[SIZE_TX2_BUFFER];

/* Буферы для UART1 */
static uint8_t num_not_received_packages[3];	// Массив счетчиков непришедших пакетов датчиков
static uint8_t num_bad_crc_packages[3];			// Массив счетчиков битых пакетов датчиков (не совпал CRC или ID или заместо веса нули)
static uint16_t array_coefficients[3];			// Массив текущих коэффициентов датчиков для изменения и проверки

static uint8_t state_uart_1 = 0;				// Текущее состояние UART2, который общается с BT
static uint8_t state_uart_2 = 0;				// Текущее состояние UART1, который общается с датчиками
static uint8_t state_get_coef = 0;				// Текущее состояние КА опроса коэффициента датчика
static uint8_t state_get_weight = 0;			// Текущее состояние КА опроса веса датчиков
static uint8_t state_set_coef = 0;				// Текущее состояние КА установки нового коэффициента датчика
static uint8_t state_calibration = 0;
static uint8_t index_id = 0;					// Индекс текущего датчика, с которым ведется работа (опрос веса, коэффициента, установка коэффициента)
static uint8_t count_measure = 0;				// Счетчик замеров для усреднения при калибровке
static int64_t mean_weight = 0;					// Средний вес для калибровки
static int64_t reference_weight = 0;			// Эталонный вес для калибровки

uint8_t try_set_coef = 0;						// Кол-во попыток записать коэффициент
uint8_t fl_get_offset = 0;						// Запрос на получение offset
uint8_t fl_start_calibration = 0;				// Запрос старта калибровки
uint8_t fl_data_is_ready = 0;					// Данные с трех датчиков собраны (для калибровки)

static swtimer_t TimerMaxWaitData;				// Таймер ожидания пакета с датчика
static swtimer_t TimerRequestWeight;			// Таймер, по которому опрашиваются датчики
static swtimer_t TimerWaitTx2Flag;				// Таймер ожидания отправки данных

DataWeight dataWeight 	  = {0};				// Конвертор значения АЦП с датчика из битов в число
DataSensors stDataSensors = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	// Основная структура с результатами датчиков - вес, offset, коэффициенты
bt_command_t bt_command;						// Структура, в которой хранится команда и параметры

/* Для интерполяции */
volatile int64_t millis;
volatile int64_t interpolX[9];
volatile uint8_t indexX;
		 int64_t interpolY[9];
		 uint8_t indexY;


/* РАБОТА С БУФЕРОМ */
uint8_t BufLen(const uint8_t *buf)
{
	uint8_t len = 0;

	while (*buf++) len++;

	return len;
}
uint8_t NumLen(int64_t num)
{
	uint8_t len = 0;
	while (num)
	{
		num/= 10;
		len++;
	}

	return len;
}
void BufClear(volatile uint8_t* buf, uint8_t val, int8_t size)
{
	while (size-- > 0) *buf++ = val;
}
void BufCopyBuf(volatile uint8_t *buf1, const uint8_t *buf2)
{
	for (uint8_t i = 0; i < 8; i++)
		buf1[i] = ' ';

	for (uint8_t i = 0; i < BufLen(buf2); i++)
		buf1[i] = buf2[i];

	buf1[7] = '\n';
}
void BufCopyNum(volatile uint8_t *buf1, int64_t num)
{
	uint8_t fl_neg = 0, index = 6;
	if (num < 0) { fl_neg = 1; num *= (int64_t)(-1); }

	for (uint8_t i = 0; i < 8; i++) buf1[i] = '0';
	while (num && index)
	{
		buf1[index--] = '0' + (num % 10);
		num /= 10;
	}

	buf1[0] = (fl_neg) ? '-' : ' ';
	buf1[7] = '\n';

	/*for (uint8_t i = 0; i < 8; i++)
	{
		buf1[i] = (num >> (i * 8)) & 0xFF;
	}*/
}

/* ИНИЦИАЛИЗАЦИЯ */
void UART_Init()
{
	BufClear(rx1Buffer, 0, SIZE_RX1_BUFFER);
	BufClear(tx1Buffer, 0, SIZE_TX1_BUFFER);
	BufClear(rx2Buffer, 0, SIZE_RX2_BUFFER);
	BufClear(tx2Buffer, 0, SIZE_TX2_BUFFER);

	tx1Flag = 0;
	rx1Flag = 0;
	tx1BufferIndex = 0;
	rx1BufferIndex = 0;

	tx2Flag = 0;
	rx2Flag = 0;
	tx2BufferIndex = 0;
	rx2BufferIndex = 0;

	num_bad_crc_packages[0] 		= num_bad_crc_packages[1] 		= num_bad_crc_packages[2] 		= 0;
	num_not_received_packages[0] 	= num_not_received_packages[1] 	= num_not_received_packages[2] 	= 0;

	for (uint8_t i = 0; i < 9; i++)
	{
		interpolX[i] = 0;
		interpolY[i] = 0;
	}

	indexX = 0;
	indexY = 0;
	millis = 0;

	stDataSensors.alpha = 65535;				// !!!
	stDataSensors.scale = accuracy_scale;		// !!!

	state_get_coef = 1;

	DIR_READ;
	ENABLE_RX1;
	ENABLE_RX2;
}

/* ФИЛЬРАЦИЯ */
int64_t Interpolation(int64_t *x, int64_t *y, int64_t x_)
{
	/* Точность 0,0000001 или делить на 10000000*/
	int64_t a, b, c;

	a = (((x_-x[1]) * (x_-x[2])) * accuracy) / ((x[0]-x[1]) * (x[0]-x[2]));
	b = (((x_-x[0]) * (x_-x[2])) * accuracy) / ((x[1]-x[0]) * (x[1]-x[2]));
	c = (((x_-x[0]) * (x_-x[1])) * accuracy) / ((x[2]-x[0]) * (x[2]-x[1]));

	return (a * y[0] + b * y[1] + c * y[2]) / accuracy;
}
int64_t ExpSmoothing(int64_t x, int64_t x_prev)
{
	return ((stDataSensors.alpha * x) + ((MAX_ALPHA - stDataSensors.alpha) * x_prev)) / MAX_ALPHA;
}

/* ДЛЯ FSM UART1 */
uint8_t GetCRC8(uint8_t *buffer, uint8_t lenght)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < lenght; i++)
	{
		uint8_t tmp = buffer[i];

		for (uint8_t j = 0; j < 8; j++)
		{
			if ((tmp ^ crc) & (1 << 0))
			{
				crc >>= 1;  crc ^= 0x0c;  crc |= 0x80;
			}
			else crc >>= 1;
			tmp >>= 1;
		}
	}

	return crc;
}
uint8_t CheckSensorsDataErrors()
{
	if ((rx1Buffer[ID0] != SLAVE1_ID && rx1Buffer[ID0] != SLAVE2_ID && rx1Buffer[ID0] != SLAVE3_ID) ||  // Если неверный ID или
	   (!rx1Buffer[DATA0] && !rx1Buffer[DATA1] && !rx1Buffer[DATA2] && !rx1Buffer[DATA3]) 			|| 	// Заместо показаний АЦП нули
		 GetCRC8((uint8_t*)rx1Buffer, 8) != 0)															// Не совпал CRC
	{
		/* Подсчет кол-ва битых пакетов подряд */
		if (++num_bad_crc_packages[index_id] >= NUMBER_TRY_RECIVE_RESPONSE)
		{
			num_bad_crc_packages[index_id] = 0;

			/* Ошибка: битый пакет */
			switch(index_id)
			{
				case 0: errorsSet(SENSOR1_BAD_CRC); break;
				case 1: errorsSet(SENSOR2_BAD_CRC); break;
				case 2: errorsSet(SENSOR3_BAD_CRC); break;
			}
		}

		return 1;
	}
	else num_bad_crc_packages[index_id] = 0;

	/* Проверка битовой маски на обрыв моста */
	if (rx1Buffer[MASK] & (1 << 1))
	{
		/* Ошибка: обрыв моста */
		switch(index_id)
		{
			case 0: errorsSet(SENSOR1_DEFECTIVE); break;
			case 1: errorsSet(SENSOR2_DEFECTIVE); break;
			case 2: errorsSet(SENSOR3_DEFECTIVE); break;
		}

		return 1;
	}

	return 0;
}
void ResetDataSensors()
{
	index_id = 0;
}
void SendRequestSensors(uint8_t command)
{
	/* Помещение ID текущего ведомого */
	switch(index_id)
	{
		case 0: tx1Buffer[0] = SLAVE1_ID; break;
		case 1: tx1Buffer[0] = SLAVE2_ID; break;
		case 2: tx1Buffer[0] = SLAVE3_ID; break;
	}

	tx1Buffer[1] = command;
	tx1Buffer[2] = (command == com_set_coefficient) ? (array_coefficients[index_id] & 0xFF) : 0;
	tx1Buffer[3] = (command == com_set_coefficient) ? (array_coefficients[index_id] >> 8)   : 0;

	/* Разрешение на отправку данных */
	START_UART1_SEND;
}
void ReadSensorResponse()
{
	dataWeight.bytes[0] = rx1Buffer[DATA0];
	dataWeight.bytes[1] = rx1Buffer[DATA1];
	dataWeight.bytes[2] = rx1Buffer[DATA2];
	dataWeight.bytes[3] = rx1Buffer[DATA3];

	dataWeight.value += 8388607;	// Смещение на середину диапазона

	int64_t coef1 = stDataSensors.coefficient_1 * stDataSensors.scale;
	int64_t coef2 = stDataSensors.coefficient_2 * stDataSensors.scale;
	int64_t coef3 = stDataSensors.coefficient_3 * stDataSensors.scale;

	switch(rx1Buffer[ID0])
	{
		case SLAVE1_ID: if (fl_get_offset) stDataSensors.weight_1_offset = dataWeight.value; interpolY[indexY++] = stDataSensors.weight_1 = ((dataWeight.value - stDataSensors.weight_1_offset) * accuracy_scale * 10) / coef1; 	break;
		case SLAVE2_ID: if (fl_get_offset) stDataSensors.weight_2_offset = dataWeight.value; interpolY[indexY++] = stDataSensors.weight_2 = ((dataWeight.value - stDataSensors.weight_2_offset) * accuracy_scale * 10) / coef2; 	break;
		case SLAVE3_ID: if (fl_get_offset) stDataSensors.weight_3_offset = dataWeight.value; interpolY[indexY++] = stDataSensors.weight_3 = ((dataWeight.value - stDataSensors.weight_3_offset) * accuracy_scale * 10) / coef3;
						if (fl_get_offset)
						{
							if (--fl_get_offset == 0)
							{
								//eeprom_save(E_STRUCT_DATA_SENSORS); WAIT(10);
							}

							indexX = indexY = stDataSensors.total_weight = 0;
						}
						fl_data_is_ready = 1;
		break;
	}
}
void ReadSensorCoefficient()
{
	dataWeight.bytes[0] = rx1Buffer[DATA0];
	dataWeight.bytes[1] = rx1Buffer[DATA1];
	dataWeight.bytes[2] = 0;
	dataWeight.bytes[3] = 0;

	switch(rx1Buffer[ID0])
	{
		case SLAVE1_ID: array_coefficients[0] = stDataSensors.coefficient_1 = dataWeight.value; break;
		case SLAVE2_ID: array_coefficients[1] = stDataSensors.coefficient_2 = dataWeight.value; break;
		case SLAVE3_ID: array_coefficients[2] = stDataSensors.coefficient_3 = dataWeight.value; break;
	}
}
void GetResultWeight()
{
	if (indexY >= 9)
	{
		indexX = indexY = 0;
		if (millis >> 16) millis = 0;

		if (state_calibration) return;	// Если сейчас происходит калибровка - выход

		if (num_not_received_packages[0] || num_not_received_packages[1] || num_not_received_packages[2]) return;

		PIN_TOGGLE(BLUE_LED_Port, BLUE_LED_Pin);

		int64_t x[3], y[3];
		int64_t x_find = (interpolX[0] + interpolX[8]) / (int64_t)2;
		int64_t result = 0;

		for (uint8_t num = 0; num < 3; num++)
		{
			for (uint8_t i = 0, j = num; i < 3; i++, j += 3)
			{
				x[i] = interpolX[j];
				y[i] = interpolY[j];
			}

			result += Interpolation(x, y, x_find);
		}

		if (stDataSensors.total_weight) 	stDataSensors.total_weight = ExpSmoothing(result, stDataSensors.total_weight);
		else								stDataSensors.total_weight = result;												// Для первого запуска
	}
}
void RunSetCoefficient()
{
	index_id = 0;
	state_get_weight = FSM_OFF;
	state_set_coef = STATE_SEND_REQUEST;
}

/* FSM ДЛЯ FSM UART1 */
void SensorsGetWeight()
{
	switch(state_get_weight)
	{
	case FSM_OFF:
		break;

	case STATE_SEND_REQUEST:

		if (swTimerCheck(&TimerRequestWeight)) SendRequestSensors(com_get_weight);

		/* Когда отправка данных завершена */
		if (tx1Flag)
		{
			tx1Flag = 0;
			state_get_weight = STATE_RECEIVED_DATA;
			swTimerSet(&TimerMaxWaitData, 25, 0);	// Таймер на ожидание ответа
		}

	break;

	/* Обработка полученных данных */
	case STATE_RECEIVED_DATA:

		/* Если были получены и обработаны */
		if (rx1Flag)
		{
			rx1Flag = 0;
			state_get_weight = STATE_SEND_REQUEST;

			swTimerReset(&TimerMaxWaitData);

			if (CheckSensorsDataErrors())
			{
				ResetDataSensors();
				indexY = 0;
				indexX = 0;
				stDataSensors.total_weight = 0;
				return;
			}

			ReadSensorResponse();
			num_not_received_packages[index_id] = 0;	// Сброс кол-ва отсутствий ответа с текущего датчика

			/* Если все 3 запроса трем датчикам были отправлены */
			if (++index_id == 3)
			{
				GetResultWeight();
				ResetDataSensors();
			}
		}

		/* Если превышено время ожидания - флаг ошибки данных и переход в другое состояние */
		if (swTimerCheck(&TimerMaxWaitData))
		{
			swTimerReset(&TimerMaxWaitData);
			state_get_weight = STATE_SEND_REQUEST;

			/* Подсчет отсутствий ответов с датчика */
			if (++num_not_received_packages[index_id] >= NUMBER_TRY_RECIVE_RESPONSE)
			{
				num_not_received_packages[index_id] = 0;

				switch(index_id)
				{
					case 0: errorsSet(SENSOR1_NOT_RESPOND); break;
					case 1: errorsSet(SENSOR2_NOT_RESPOND); break;
					case 2: errorsSet(SENSOR3_NOT_RESPOND); break;
				}
			}

			if (++index_id >= 3) ResetDataSensors();
		}

	break;
	}
}
void SensorsGetCoefficient()
{
	switch(state_get_coef)
	{
	case FSM_OFF:
		break;

	case STATE_SEND_REQUEST:

		/* Отправка запроса на чтение коэффициента */
		if (swTimerCheck(&TimerRequestWeight)) SendRequestSensors(com_get_coefficient);

		/* Когда отправка данных завершена */
		if (tx1Flag)
		{
			tx1Flag = 0;
			state_get_coef = 2;
			swTimerSet(&TimerMaxWaitData, 25, 0);	// Таймер на ожидание ответа
		}

		break;

	case STATE_RECEIVED_DATA:

		/* Если были получены и обработаны */
		if (rx1Flag)
		{
			rx1Flag = 0;
			state_get_coef = STATE_SEND_REQUEST;

			swTimerReset(&TimerMaxWaitData);

			if (CheckSensorsDataErrors())
			{
				num_not_received_packages[index_id]++;
				ResetDataSensors();
				return;
			}

			ReadSensorCoefficient();					// Если ошибок в пакете не было - читаем значения
			num_not_received_packages[index_id] = 0;	// Сброс кол-ва отсутствий ответа с текущего датчика

			/* Если все 3 запроса трем датчикам были отправлены */
			if (++index_id >= 3)
			{
				ResetDataSensors();

				/* Если все данные получены без ошибок */
				if (!num_not_received_packages[0] && !num_not_received_packages[1] && !num_not_received_packages[2])
				{
					state_get_coef = FSM_OFF; 														// Закончить работу КА
					state_get_weight = (state_set_coef != FSM_OFF) ? FSM_OFF : STATE_SEND_REQUEST;	// Если КА SensorsSetCoefficient не запушен - запускаем КА SensorsGetWeight
					num_not_received_packages[0] = num_not_received_packages[1] = num_not_received_packages[2] = 0;
				}
				else
				{
					ResetDataSensors();
					state_get_coef = 1;
				}
			}
		}

		/* Если превышено время ожидания - флаг ошибки данных и переход в другое состояние */
		if (swTimerCheck(&TimerMaxWaitData))
		{
			swTimerReset(&TimerMaxWaitData);
			state_get_coef = 1;

			/* Подсчет отсутствий ответов с датчика */
			if (++num_not_received_packages[index_id] >= NUMBER_TRY_RECIVE_RESPONSE)
			{
				num_not_received_packages[0] = num_not_received_packages[1] = num_not_received_packages[2] = 0;
				state_get_coef = FSM_OFF;					// Отключение данного КА
				state_get_weight = STATE_SEND_REQUEST;		// Включение на вес, работа по старым коэффициентам
				index_id = 2;								// Сброс индекса в ноль

				switch(index_id)
				{
					case 0: errorsSet(SENSOR1_NOT_GET_COEF); break;
					case 1: errorsSet(SENSOR2_NOT_GET_COEF); break;
					case 2: errorsSet(SENSOR3_NOT_GET_COEF); break;
				}

				if (state_set_coef != FSM_OFF) errorsSet(NOT_SET_COEF);
			}

			if (++index_id >= 3) ResetDataSensors();
		}
		break;
	}
}
void SensorsSetCoefficient()
{
	switch(state_set_coef)
	{
	case FSM_OFF:
		break;

	case STATE_SEND_REQUEST:

		if (swTimerCheck(&TimerRequestWeight)) SendRequestSensors(com_set_coefficient);

		/* Когда отправка данных завершена */
		if (tx1Flag)
		{
			tx1Flag = 0;

			if (++index_id >= 3)
			{
				index_id = 0;
				state_get_coef = STATE_SEND_REQUEST;		// Включаем КА на опрос значений коэффициентов
				state_set_coef = 2;
			}
		}
		break;

	case 2:
		/* Ожидаем, пока КА SensorsGetCoefficient не отработает */
		if (state_get_coef == FSM_OFF)
		{
			/* Если всё совпало - выходим с успехом */
			if (array_coefficients[0] == stDataSensors.coefficient_1 &&
				array_coefficients[1] == stDataSensors.coefficient_2 &&
				array_coefficients[2] == stDataSensors.coefficient_3)
			{
				try_set_coef = 0;
				state_set_coef = FSM_OFF;
				state_get_weight = STATE_SEND_REQUEST;
				break;
			}

			/* Иначе не совпало - пробуем еще */
			state_set_coef = STATE_SEND_REQUEST;

			/* Если попыток было слишком много - выход с ошибкой */
			if (++try_set_coef >= NUMBER_TRY_SET_COEF)
			{
				errorsSet(NOT_SET_COEF);
				state_set_coef = FSM_OFF;
				state_get_weight = STATE_SEND_REQUEST;
			}
		}
		break;
	}
}
void SensorsCalibration()
{
	switch(state_calibration)
	{

	case FSM_OFF:
		break;

	/* Инициализация */
	case 1:
		count_measure = 0;
		mean_weight = 0;
		state_calibration = 2;
		stDataSensors.scale = accuracy_scale;
		fl_data_is_ready = 0;

	break;

	/* Усреднение веса */
	case 2:
		if (fl_data_is_ready)
		{
			fl_data_is_ready = 0;
			mean_weight += stDataSensors.weight_1 + stDataSensors.weight_2 + stDataSensors.weight_3;
			count_measure++;
		}

		if (count_measure >= NUMBER_MEASURE) state_calibration = 3;
	break;

	/* Высчитывание коэффициента */
	case 3:
		mean_weight = (mean_weight * accuracy_scale) / count_measure;
		stDataSensors.scale = mean_weight / reference_weight;

		state_calibration = 4;
	break;

	/* Конец калибровки */
	case 4:

		indexX = indexY = stDataSensors.total_weight = 0;
		state_calibration = FSM_OFF;

		//eeprom_save(E_SCALE);
	break;

	}
}

/* ДЛЯ FSM UART2 */
void ReadBtCommand()
{
	bt_command.command = rx2Buffer[COMMAND];
	bt_command.id = rx2Buffer[ID];

	for (uint8_t i = 0; i < NUMBER_BT_PARAM; i++)
		bt_command.param[i] = rx2Buffer[PARAM1 + i];

	BufClear(rx2Buffer, 0, SIZE_RX2_BUFFER);
}
void ApplyBtCommand()
{
	switch(bt_command.command)
	{
	case GET_WEIGHT:

		switch(bt_command.id)
		{
		case 0:
			if (errorsCount())
				errorsPrint(tx2Buffer);
			else
				BufCopyNum(tx2Buffer, (bt_command.param[0] == 0) ? stDataSensors.total_weight 	: (stDataSensors.weight_1 + stDataSensors.weight_1_offset +
																								   stDataSensors.weight_2 + stDataSensors.weight_2_offset +
																								   stDataSensors.weight_3 + stDataSensors.weight_3_offset));	break;
		case 1: BufCopyNum(tx2Buffer, (bt_command.param[0] == 0) ? stDataSensors.weight_1		: (stDataSensors.weight_1 + stDataSensors.weight_1_offset));	break;
		case 2: BufCopyNum(tx2Buffer, (bt_command.param[0] == 0) ? stDataSensors.weight_2 		: (stDataSensors.weight_2 + stDataSensors.weight_2_offset));	break;
		case 3: BufCopyNum(tx2Buffer, (bt_command.param[0] == 0) ? stDataSensors.weight_3 		: (stDataSensors.weight_3 + stDataSensors.weight_3_offset));	break;
		}

	break;

	case GET_COEFFICIENT:

		switch(bt_command.id)
		{
		case 0: BufCopyNum(tx2Buffer, stDataSensors.scale); 		break;
		case 1: BufCopyNum(tx2Buffer, stDataSensors.coefficient_1); break;
		case 2: BufCopyNum(tx2Buffer, stDataSensors.coefficient_2); break;
		case 3: BufCopyNum(tx2Buffer, stDataSensors.coefficient_3); break;
		}

	break;

	case SET_COEFFICIENT:

		switch(bt_command.id)
		{
		case 0: stDataSensors.scale   = (bt_command.param[1] << 8) + bt_command.param[0]; break;
		case 1: array_coefficients[0] = (bt_command.param[1] << 8) + bt_command.param[0]; break;
		case 2: array_coefficients[1] = (bt_command.param[1] << 8) + bt_command.param[0]; break;
		case 3: array_coefficients[2] = (bt_command.param[1] << 8) + bt_command.param[0]; break;
		}

		if (bt_command.id) RunSetCoefficient();
		BufCopyBuf(tx2Buffer, (uint8_t*)"OK");

	break;

	case ALPHA:

		switch(bt_command.param[0])
		{
		case 0: BufCopyNum(tx2Buffer, stDataSensors.alpha); break;
		case 1: stDataSensors.alpha = (bt_command.param[1] << 8) + bt_command.param[0]; BufCopyBuf(tx2Buffer, (uint8_t*)"Ok!"); break;
		}

	break;

	case CALIBRATION:

		switch(bt_command.id)
		{
		case 0: RunCalibration(); reference_weight = (bt_command.param[1] << 8) + bt_command.param[0]; 	BufCopyBuf(tx2Buffer, (uint8_t*)"Ok!");	break;
		case 1: CancelCalibration();  																	BufCopyBuf(tx2Buffer, (uint8_t*)"Ok!"); break;
		}

	break;

	case OFFSET:

		switch(bt_command.param[0])
		{
		case 0: fl_get_offset = 3; break;
		case 1:
			stDataSensors.weight_1_offset = stDataSensors.reserved_offset_1;
			stDataSensors.weight_2_offset = stDataSensors.reserved_offset_2;
			stDataSensors.weight_3_offset = stDataSensors.reserved_offset_3;
		break;
		}

		BufCopyBuf(tx2Buffer, (uint8_t*)"Ok!");

	break;

	default: rx2BufferIndex = 0; ENABLE_RX2; break;
	}
}
void ClearBtCommand()
{
	bt_command.command = 0;
	bt_command.id = 0;
	bt_command.param[0] = 0;
	bt_command.param[1] = 0;
}

/* КОНЕЧНЫЕ АВТОМАТЫ */
void FSM_UART1()
{
	switch(state_uart_1)
	{
		/* Инициализация */
		case 0:
			UART_Init();
			swTimerSet(&TimerRequestWeight, 0, 100);
			state_uart_1 = 1;
		break;

		case 1:
			SensorsGetWeight();
			SensorsGetCoefficient();
			SensorsSetCoefficient();
			SensorsCalibration();
		break;
		/* Отправка запроса */
	}
}
void FSM_UART2()
{
	switch(state_uart_2)
	{
	case 0:
		if (rx2Flag)
		{
			rx2Flag = 0;
			state_uart_2 = 1;

			ReadBtCommand();
			swTimerReset(&TimerWaitTx2Flag);
		}
	break;

	case 1:
		//BufClear(tx2Buffer, 0, SIZE_TX2_BUFFER);
		ApplyBtCommand();
		ClearBtCommand();

		state_uart_2 = (state_calibration == 0) ? 2 : 3;
	break;

	/* Разрешение на отправку данных */
	case 2:
		swTimerSet(&TimerWaitTx2Flag, 500, 0);

		state_uart_2   = 4;
		tx2BufferIndex = 0;
		ENABLE_TX2;
	break;

	/* Если калибровка - ждем окончания */
	case 3:
		if (!state_calibration)
		{
			swTimerSet(&TimerWaitTx2Flag, 500, 0);

			state_uart_2 = 4;
			tx2BufferIndex = 0;
			ENABLE_TX2;
		}
	break;

	case 4:
		if (tx2Flag)
		{
			tx2Flag = 0;
			state_uart_2 = 0;
		}

		if (swTimerCheck(&TimerWaitTx2Flag)) state_uart_2 = 0;
	break;
	}
}


/********************************************************************************************************************************/


/* ПРЕРЫВАНИЯ UART */
/* Прерывание на прием данных */
void UART1_RXNE_Interrupt()
{
	if (rx1BufferIndex < SIZE_RX1_BUFFER)
	{
		rx1Buffer[rx1BufferIndex] = (uint8_t)USART1->DR;
		rx1BufferIndex++;
	}

	if (rx1BufferIndex == SIZE_RX1_BUFFER)
	{
		rx1Flag = 1;
		rx1BufferIndex = 0;
		interpolX[indexX++] = millis;	// Сохранение времени текущих показаний с датчика

		DISABLE_RX1;
	}

	//if (rx1BufferIndex > SIZE_RX1_BUFFER) rx1BufferIndex = 0;
}
/* Прерывание по опустошениею UDR (готовность отправки) */
void UART1_TXE_Interrupt()
{
	if (tx1BufferIndex < SIZE_TX1_BUFFER)
	{
		USART1->DR = tx1Buffer[tx1BufferIndex];
		tx1BufferIndex++;
	}

	if (tx1BufferIndex == SIZE_TX1_BUFFER)
	{
		USART1->CR1 &= ~USART_CR1_TXEIE;
		USART1->CR1 |= USART_CR1_TCIE;
	}

	//if (tx1BufferIndex > SIZE_TX1_BUFFER) tx1BufferIndex = 0;
}
/* Прерывание на прием данных */
void UART2_RXNE_Interrupt()
{
	if (rx2BufferIndex < SIZE_RX2_BUFFER)
	{
		rx2Buffer[rx2BufferIndex] = (uint8_t)USART2->DR;
		rx2BufferIndex++;
	}

	if (rx2BufferIndex == SIZE_RX2_BUFFER)
	{
		rx2Flag = 1;
		rx2BufferIndex = 0;

		DISABLE_RX2;
	}

	//if (rx2BufferIndex > SIZE_RX2_BUFFER) rx2BufferIndex = 0;
}
/* Прерывание по опустошениею UDR (готовность отправки) */
void UART2_TXE_Interrupt()
{
	if (tx2BufferIndex < SIZE_TX2_BUFFER)
	{
		USART2->DR = tx2Buffer[tx2BufferIndex];
		tx2BufferIndex++;
	}

	if (tx2BufferIndex == SIZE_TX2_BUFFER)
	{
		USART2->CR1 &= ~USART_CR1_TXEIE;
		USART2->CR1 |= USART_CR1_TCIE;
	}

	if (tx2BufferIndex > SIZE_TX2_BUFFER) tx2BufferIndex = 0;
}


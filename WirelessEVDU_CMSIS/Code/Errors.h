#ifndef ERROROS_H_
#define ERROROS_H_

#include "main.h"

enum 
{
	NO_ERRORS,				// Ошибок нет														0
	SENSOR1_DEFECTIVE,		// Датчик 1 неисправен (присылает неверное значение/обрыв моста)	1
	SENSOR2_DEFECTIVE,		// Датчик 2 неисправен (присылает неверное значение/обрыв моста)	2
	SENSOR3_DEFECTIVE,		// Датчик 3 неисправен (присылает неверное значение/обрыв моста)	3
	SENSOR1_NOT_RESPOND,	// Датчик 1 не отвечает (нет связи)									4
	SENSOR2_NOT_RESPOND,	// Датчик 1 не отвечает (нет связи)									5
	SENSOR3_NOT_RESPOND,	// Датчик 1 не отвечает (нет связи)									6
	NOT_CALIBRATED,			// Неправильно откалибровано (слишком маленький коэфициент)			7
	SENSOR1_BAD_CRC,		// Датчик 1 присылает битые данные									8
	SENSOR2_BAD_CRC,		// Датчик 2 присылает битые данные									9
	SENSOR3_BAD_CRC,		// Датчик 3 присылает битые данные									10
	SENSOR1_NOT_GET_COEF,	// Датчик 1 не присылает коэффициент								11
	SENSOR2_NOT_GET_COEF,	// Датчик 2 не присылает коэффициент								12
	SENSOR3_NOT_GET_COEF,	// Датчик 3 не присылает коэффициент								13
	NOT_SET_COEF,			// Offset или коэффициент не корректный (большой вес)				14

	TOTAL_ERRORS
};

void errorsReset();
void errorsSet(uint8_t error_id);
void errorsPrint(volatile uint8_t *buf);
uint8_t errorsGet(uint8_t error_id);
uint8_t errorsCheck();
uint8_t errorsCount();
uint8_t errorsGetAll();

#endif /* ERROROS_H_ */

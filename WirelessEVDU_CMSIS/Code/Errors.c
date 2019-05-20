#include "Errors.h"

uint32_t Errors;

void errorsReset()
{
    Errors = 0;
}

void errorsSet(uint8_t error_id)
{
    if (error_id >= TOTAL_ERRORS) return;

    Errors |= (uint32_t)1 << error_id;
}
uint8_t errorsGet(uint8_t error_id)
{
    if (error_id >= TOTAL_ERRORS) return 0;

    return (Errors & ((uint32_t)1 << error_id));
}

void errorsPrint(volatile uint8_t *buf)
{
    buf[0] = (Errors >> 24) & 0xFF;
    buf[1] = (Errors >> 16) & 0xFF;
    buf[2] = (Errors >> 8) & 0xFF;
    buf[3] = (Errors >> 0) & 0xFF;
}

uint8_t errorsCheck()
{
    return (Errors) ? 1 : 0;
}

uint8_t errorsCount()
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < TOTAL_ERRORS; i++)
    {
        if (Errors & ((uint32_t)1 << i)) count++;
    }

    return count;
}

uint8_t errorsGetAll()
{
    return Errors;
}

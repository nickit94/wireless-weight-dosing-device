#ifndef SWTIMER_H_
#define SWTIMER_H_

#include "main.h"

typedef struct
{
  uint8_t status;
  uint32_t period;
  uint32_t time;
  uint32_t prevTimeSystem;
} swtimer_t;

void swTimerSet(swtimer_t* pTimer, uint32_t Time, uint32_t Period);
void swTimerReset(swtimer_t* pTimer);
uint8_t swTimerCheck(swtimer_t* pTimer);

#endif /* SWTIMER_H_ */

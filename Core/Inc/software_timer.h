#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define MAX_TIMER 10

extern volatile int timer_counter[MAX_TIMER];
extern volatile int timer_flag[MAX_TIMER];

void setTimer(int index, int duration);
void clearTimer(int index);
int  isTimerExpired(int index);
void timerRun(void);

#endif /* INC_SOFTWARE_TIMER_H_ */

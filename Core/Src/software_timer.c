#include "software_timer.h"

volatile int timer_counter[MAX_TIMER];
volatile int timer_flag[MAX_TIMER];

void setTimer(int index, int duration) {
    if (index < 0 || index >= MAX_TIMER) return;
    timer_counter[index] = duration;
    timer_flag[index] = 0;
}

void clearTimer(int index) {
    if (index < 0 || index >= MAX_TIMER) return;
    timer_counter[index] = 0;
    timer_flag[index] = 0;
}

int isTimerExpired(int index) {
    if (index < 0 || index >= MAX_TIMER) return 0;
    return (timer_flag[index] == 1) ? 1 : 0;
}

void timerRun(void) {
    for (int i = 0; i < MAX_TIMER; i++) {
        if (timer_counter[i] > 0) {
            timer_counter[i]--;
            if (timer_counter[i] <= 0) {
                timer_flag[i] = 1;
            }
        }
    }
}

#include "timer.h"
#include "system.h"

/*Function:  timer1_Initialize
 * - initialize timer 1 for CPU1
 * Argument: none
 * Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
void timer1_Initialize(){

    // period count from N to 0 so it's N+1
    long period_count = SYS_CLK/TIMER1_FREQ - 1;
    CPUTimer_stopTimer(CPUTIMER1_BASE);          // stop before configuring
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);    // no prescaler
    CPUTimer_setPeriod(CPUTIMER1_BASE, period_count);   // set period
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE); // reload period
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);    // enable interrupt
    Interrupt_enable(INT_TIMER1);                 //
    CPUTimer_startTimer(CPUTIMER1_BASE);        // start timer
}



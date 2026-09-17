#ifndef COMMAND_EXECUTION_H_
#define COMMAND_EXECUTION_H_

#include "command_interpreter.h"
#include "motion_planner.h"



// interrupt declaration
__interrupt void cpuTimer0_ISR(void);


// function prototype
int command_execute(CMD *cmdList, int index);

void motor_drivers_initialize(volatile MotionProfile *axes);

void home_axis(int driver, int homeDir);

#endif 



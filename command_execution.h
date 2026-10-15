#ifndef COMMAND_EXECUTION_H_
#define COMMAND_EXECUTION_H_

#include "command_interpreter.h"
#include "motion_planner.h"

// pre-define symbol
#define MAX_WAYPOINTS 50 // max amount of way point accepted 

// data structure
typedef struct {
    float theta1;   // J1 angle (radians)
    float theta2;   // j2 angle (radians)
    float z;        // Z position (mm)
    float delay; // delay in ms
} Waypoint;

typedef struct{
    uint16_t num_waypoints;
    bool teach_mode;
    bool recorded_program;
} TeachProgram;

extern volatile Waypoint program[MAX_WAYPOINTS];
extern volatile TeachProgram teach;

// interrupt declaration
__interrupt void cpuTimer0_ISR(void);


// function prototype
void command_execute(CMD *cmdList, int index);

void motor_drivers_initialize(volatile MotionProfile *axes);

void waypoints_initialize (volatile Waypoint *program, volatile TeachProgram *teach);

void home_axis(int driver, int homeDir);

bool jog_limit_check (volatile JointAngles_t *joints, int index, float target);

#endif 



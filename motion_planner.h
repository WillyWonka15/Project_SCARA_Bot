#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include "kinematics.h"

#define GLOBAL_VMAX 80.0f // steps/sec
#define GLOBAL_ACCEL 30.0f // step/sec^2
#define MOTOR_ANGLE_RESOLUTION 0.1125f // 1/16 microstep, calculate from degree
#define CLOCK_WISE -1
#define COUNTER_CLOCKWISE 1
#define HOME 0
// interrupt decleration
__interrupt void cpuTimer1_ISR(void);


/*enum{
    CLOCK_WISE, COUNTER_CLOCKWISE, HOME
};*/

//
typedef struct{
    int target_steps;       // total distance to move
    int current_step;       // store current step taken per-move 
    long total_steps;         // store total step taken since home. can be pos or neg
    float step_accumulator; // store micro step count
    float v_max;            // target peak velocity
    float accel;            // constant acceleration rate
    long d_accel;           // step required to reach Vmax
    long d_cruise;          // cruise distance at vmax
    float current_vel;      // velocity at this exact moment
    int move_dir;           // direction of the move
    bool is_moving;         // state flag for the interrupt
} MotionProfile;


//
extern volatile MotionProfile joint1; 
extern volatile MotionProfile joint2; 

// Function Prototype
void motion_profile_initialize(volatile MotionProfile *joint1, volatile MotionProfile *joint2);

void plan_move(volatile MotionProfile *joint1, volatile MotionProfile *joint2, volatile JointAngles_t *joints);

void update_velocity_profile(volatile MotionProfile *joint);

void move_complete (volatile MotionProfile *joint1, volatile MotionProfile *joint2, volatile JointAngles_t *joints);





#endif



#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include "kinematics.h"

#define GLOBAL_VMAX 800.0f // steps/sec
#define GLOBAL_VMIN 100.0f
#define GLOBAL_ACCEL 2000.0f // step/sec^2
#define STEPS_PER_DEG  8.888f  
#define MOTOR_ANGLE_RESOLUTION 0.1125f // 1/8 microstep, calculate from degree
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
    long target_steps;       // total distance to move
    long current_step;       // store current step taken per-move 
    long total_steps;         // store total step taken since home. can be pos or neg
    float step_accumulator; // store micro step count
    float v_max;            // target peak velocity
    float accel;            // constant acceleration rate
    long d_accel;           // step required to reach Vmax
    long d_cruise;          // cruise distance at vmax
    float current_vel;      // velocity at this exact moment
    int move_dir;           // direction of the move
    bool is_moving;         // state flag for the interrupt

    // per-axis config, needed for homing
    uint16_t step_gpio;
    uint16_t dir_gpio;
    uint16_t limit_sw_gpio;
    int home_dir;          // direction to move toward limit switch (+1/-1)
    long home_backoff_steps; // how far to back off after fast hit, before slow creep
} MotionProfile;

typedef enum { AXIS_J1 = 0, AXIS_J2 = 1, AXIS_Z = 2, NUM_AXES } axis_id_t;

extern volatile MotionProfile axes[NUM_AXES];
//
//extern volatile MotionProfile joint1; 
//extern volatile MotionProfile joint2; 

// Function Prototype
void motion_profile_initialize(volatile MotionProfile *joint1, volatile MotionProfile *joint2);

void plan_move(volatile MotionProfile *joint1, volatile MotionProfile *joint2, volatile JointAngles_t *joints);

void update_velocity_profile(volatile MotionProfile *joint);

void move_complete (volatile MotionProfile *joint1, volatile MotionProfile *joint2, volatile JointAngles_t *joints);





#endif



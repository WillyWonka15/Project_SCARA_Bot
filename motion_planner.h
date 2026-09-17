#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include "kinematics.h"

#define GLOBAL_VMAX 8000.0f // steps/sec
#define GLOBAL_VMIN 600.0f
#define GLOBAL_ACCEL 12000.0f // step/sec^2
#define Z_VMAX 3200.0f // step/sec
#define Z_ACCEL 6400.0f // step/sec^2

// motor
#define MOTOR_STEPS_PER_REV 400.0f // 0.9 deg motor
#define MICRO_STEPS 8.0f
#define GEAR_RATIO 20  // 20:1 gear ratio
#define MOTOR_ANGLE_RESOLUTION (360.0f / (MOTOR_STEPS_PER_REV * MICRO_STEPS * GEAR_RATIO)) // calculate from degree
//#define MOTOR_ANGLE_RESOLUTION 0.1125f // calculate from degree

#define Z_PITCH_MM 2.0 // 2mm per rev
#define Z_STEPS_PER_MM      (((400 * 8) / Z_PITCH_MM))  // 1600 steps/mm


//
#define CLOCK_WISE -1
#define COUNTER_CLOCKWISE 1
#define HOME 0
// interrupt declaration
__interrupt void cpuTimer1_ISR(void);

//
typedef struct{
    float vmax;             // hold default max velocity for a move
    int32_t target_steps;       // total distance to move
    int32_t current_step;       // store current step taken per-move 
    int32_t total_steps;         // store total step taken since home. can be pos or neg
    float step_accumulator; // store micro step count
    float v_max;            // target peak velocity
    float accel;            // constant acceleration rate
    long d_accel;           // step required to reach Vmax
    long d_cruise;          // cruise distance at vmax
    float current_vel;      // velocity at this exact moment
    int move_dir;           // direction of the move
    bool is_moving;         // state flag for the interrupt

    // motor driver control config
    int16_t motor;
    char motor_address;
} MotionProfile;

typedef struct{
    float x;
    float y;
    float z;
} CurrentPosition;

//
typedef enum { AXIS_J1 = 0, AXIS_J2 = 1, AXIS_Z = 2, NUM_AXES } axis_id_t;

//
extern volatile CurrentPosition coordinate;
extern volatile MotionProfile axes[NUM_AXES];
//
//extern volatile MotionProfile joint1; 
//extern volatile MotionProfile joint2; 

// Function Prototype
void motion_profile_initialize(volatile MotionProfile *axes);

void coordinate_initialize(volatile CurrentPosition *coordinate);

void plan_move(volatile MotionProfile *axes, volatile JointAngles_t *joints, float z_target);

void update_velocity_profile(volatile MotionProfile *axis);

void move_complete (volatile MotionProfile *axes);

float z_mm_to_steps(float mm);

float z_steps_to_mm(float steps);





#endif



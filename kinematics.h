#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define ELBOW_UP 1
#define ELBOW_DOWN -1

//arm length in mm
#define LINK_1_MM (300.0f)
#define LINK_2_MM (300.0f)

// Safety Limits for Testing
#define THETA1_MIN_RAD  (-150.0f * M_PI/180.0f)   // -150 degrees
#define THETA1_MAX_RAD  (150.0f * M_PI/180.0f)   // +150 degrees

#define THETA2_MIN_RAD  (-145.0f * M_PI/180.0f)   //  -145 degrees 
#define THETA2_MAX_RAD  (145.0f * M_PI/180.0f)    // 145 degrees 

#define X_MAX  (270)


//struct
typedef struct {
    float theta1;           // shoulder angle in radian
    float theta2;           // elbow angle in radian
    int elbow_config;         // elbow config
    int prev_elbow_config;    // previous elbow config
    float prev_theta1;      // previous angle in radian
    float prev_theta2;      // previous angle in radian
} JointAngles_t;


extern volatile JointAngles_t joints;

//return true if coordinate are reachable
void kinematics_compute_FK(float theta1, float theta2, float *x, float *y);

bool kinematics_compute_IK(float x, float y, float *theta1, float *theta2, int sign);

bool joints_in_limit(float *theta1, float *theta2);

float angleDiff(float target, float current);

bool solve_auto_config(float x, float y, volatile JointAngles_t *joints);

void joint_angle_initialize(volatile JointAngles_t *joints);

#endif 



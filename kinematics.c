#include "kinematics.h"
#include <math.h>

volatile JointAngles_t joints;

/*Function: kinematics_compute_IK
 * - calculate elbow and shoulder angle from user input coordinate
 * float x, float y, float *theta1, float *theta2, int sign
 * Return: true if the coordinate is reachable false if not
 *
 *
 * Author: WN
 * Date: 2026/05/07

 */
bool kinematics_compute_IK(float x, float y, float *theta1, float *theta2, int sign){

// 1.calculate theta 2, elbow angle
// calculate distance from the origin
float distSq = (x*x) + (y*y);
float dist = sqrtf(distSq);
float maxReach = LINK_1_MM + LINK_2_MM;
// check if distance is out of bound or not
if(dist > maxReach ){
    return false;
}
// cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2)
float cosTheta2 = (distSq - (LINK_1_MM * LINK_1_MM) - (LINK_2_MM * LINK_2_MM)) / (2.0f * LINK_1_MM * LINK_2_MM); 
// safety clamp 
if(cosTheta2 > 1.0f) return false;  //singularity point
if(cosTheta2 < -1.0f) return false; //self collision
// store the value of theta 2
*theta2 = sign * acosf(cosTheta2);
// 2.calculate theta 1, shoulder angle
// theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
float alpha = atan2f(y,x);
float beta = atan2f(LINK_2_MM * sinf(*theta2), LINK_1_MM + LINK_2_MM * cosf(*theta2));
*theta1 = alpha - beta;

 return true;   
}

bool solve_auto_config(float x, float y, volatile JointAngles_t *joints)
{
    float theta1_down = 0.0f;
    float theta2_down = 0.0f;
    float theta1_up = 0.0f;
    float theta2_up = 0.0f;

    bool up = kinematics_compute_IK(x, y, &theta1_up, &theta2_up, ELBOW_UP);
    bool down = kinematics_compute_IK(x, y, &theta1_down, &theta2_down, ELBOW_DOWN);

    bool up_valid = up && joints_in_limit(&theta1_up, &theta2_up);
    bool down_valid = down && joints_in_limit(&theta1_down, &theta2_down);


    // both config is possible
    if (down_valid && up_valid)
    {
        // default at elbow up config 
        /*joints->prev_theta1 = joints-> theta1;
        joints->prev_theta2 = joints-> theta2;
        joints->prev_elbow_config = joints->elbow_config;
        joints->theta1 = theta1_up;
        joints->theta2 = theta2_up;
        joints->elbow_config = ELBOW_UP;*/

        // stay at previous config
        joints->prev_theta1 = joints-> theta1;
        joints->prev_theta2 = joints-> theta2;
        joints->prev_elbow_config = joints->elbow_config;
        if(joints->prev_elbow_config == ELBOW_UP)
        {
            joints->theta1 = theta1_up;
            joints->theta2 = theta2_up;
            joints->elbow_config = ELBOW_UP;
        }
        else if(joints->prev_elbow_config == ELBOW_DOWN)
        {
            joints->theta1 = theta1_down;
            joints->theta2 = theta2_down;
            joints->elbow_config = ELBOW_DOWN;
        }
        return true;
    }
    //if only elbow down is valid
    else if(down_valid)
    {
        // put arm in elbow down config
        joints->prev_theta1 = joints-> theta1;
        joints->prev_theta2 = joints-> theta2;
        joints->prev_elbow_config = joints->elbow_config;
        joints->theta1 = theta1_down;
        joints->theta2 = theta2_down;
        joints->elbow_config = ELBOW_DOWN;

        return true;
    }
    // if only elbow up is valid
    else if(up_valid)
    {
        // default at elbow up config 
        joints->prev_theta1 = joints-> theta1;
        joints->prev_theta2 = joints-> theta2;
        joints->prev_elbow_config = joints->elbow_config;
        joints->theta1 = theta1_up;
        joints->theta2 = theta2_up;
        joints->elbow_config = ELBOW_UP;

        return true;
    }
    // un reachable
    else 
    {
        return false;
    }
}


bool joints_in_limit(float *theta1, float *theta2)
{
    // check theta 1 limit
    if(*theta1 < THETA1_MIN_RAD || *theta1 > THETA1_MAX_RAD)
    {
        return false;
    }
    // check theta 2 limit
    else if(*theta2 < THETA2_MIN_RAD || *theta2 > THETA2_MAX_RAD)
    {
        return false;
    }
    // all joints are in limit
    else return true; 
}

void joint_angle_initialize(volatile JointAngles_t *joints)
{
    joints->theta1 = 0.0f;
    joints->theta2 = 0.0f;
    joints->elbow_config = ELBOW_UP;
    joints->prev_elbow_config = ELBOW_UP;
    joints->prev_theta1 = 0.0f;
    joints->prev_theta2 = 0.0f;
}




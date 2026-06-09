#include "motion_planner.h"
#include "system.h"
#include "timer.h"
#include "tmc2209.h"
#include <math.h>
#include <stdbool.h>

static const float DT = (1.0f / TIMER1_FREQ);
volatile MotionProfile joint1;
volatile MotionProfile joint2;
//volatile bool step_state_joint1 = false;
//volatile bool step_state_joint2 = false;
// static float step_accumulator_j1 = 0.0f;
// static float step_accumulator_j2 = 0.0f;

/*Function: plan-move
 * - plan the velocity profile base on calculated joint angle
 * using the trapezoidal velocity profile
 * Argument: volatile MotionProfile *joint1, volatile MotionProfile *joint2,
 * volatile JointAngles_t *joints Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
void plan_move(volatile MotionProfile *joint1, volatile MotionProfile *joint2,
               volatile JointAngles_t *joints) {
  // calculate delta, how far to move
  int delta1 =
      (int)(joints->theta1 / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f)) -
      joint1->total_steps;
  int delta2 =
      (int)(joints->theta2 / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f)) -
      joint2->total_steps;

  // assign direction base on delta
  joint1->move_dir = (delta1 >= 0) ? COUNTER_CLOCKWISE : CLOCK_WISE;
  joint2->move_dir = (delta2 >= 0) ? COUNTER_CLOCKWISE : CLOCK_WISE;
  // set direction pin for the motor driver 
  if(joint1->move_dir == COUNTER_CLOCKWISE)
  {
    //
    tmc_set_direction(MOTOR_1, MOTOR_CCW);
  }
  else if (joint1->move_dir == CLOCK_WISE) {
   // 
    tmc_set_direction(MOTOR_1, MOTOR_CW);
  }
  if(joint2->move_dir == COUNTER_CLOCKWISE){
    //
    //tmc_set_direction(MOTOR_2, 0);
  }
  else if (joint2->move_dir == CLOCK_WISE) {
    //
    //tmc_set_direction(MOTOR_2, 1);
  }

  //
  joint1->target_steps = abs(delta1);
  joint2->target_steps = abs(delta2);

  // Assing global vmax and accel
  float local_vmax_joint1 = GLOBAL_VMAX;
  float local_vmax_joint2 = GLOBAL_VMAX;
  joint1->accel = GLOBAL_ACCEL;
  joint2->accel = GLOBAL_ACCEL;

  // Calculate distance to reach vmax
  float d_accel_ideal_joint1 =
      (local_vmax_joint1 * local_vmax_joint1) / (2.0f * joint1->accel);
  float d_accel_ideal_joint2 =
      (local_vmax_joint2 * local_vmax_joint2) / (2.0f * joint2->accel);

  // The TRIANGLE check
  if ((2.0f * d_accel_ideal_joint1) > joint1->target_steps) {
    //
    joint1->d_accel = joint1->target_steps / 2;

    // Calculate new lower peak velocity
    local_vmax_joint1 = sqrtf(2.0f * joint1->accel * joint1->d_accel);
  } else {
    joint1->d_accel = d_accel_ideal_joint1;
  }

  if ((2.0f * d_accel_ideal_joint2) > joint2->target_steps) {
    //
    joint2->d_accel = joint2->target_steps / 2;

    // Calculate new lower peak velocity
    local_vmax_joint2 = sqrtf(2.0f * joint2->accel * joint2->d_accel);
  } else {
    joint2->d_accel = d_accel_ideal_joint2;
  }

  // Store the final Vmax
  joint1->v_max = local_vmax_joint1;
  joint2->v_max = local_vmax_joint2;

  // Calculate d cruise
  joint1->d_cruise = joint1->target_steps - (2.0f * joint1->d_accel);
  joint2->d_cruise = joint2->target_steps - (2.0f * joint2->d_accel);

  // Update moving status
  joint1->is_moving = true;
  joint2->is_moving = true;
}

/*Function: update_velocity_profile
 * - update current velocity to match the plan trapezoid velocity profile
 * Argument: volatile MotionProfile *joint
 * Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
void update_velocity_profile(volatile MotionProfile *joint) {
  // find out which part of the velocity profile we're in
  // Acceleration state
  if (joint->current_step <= joint->d_accel) {
    joint->current_vel += joint->accel * DT;
  }
  // Cruise state at v max
  else if (joint->current_step > joint->d_accel &&
           joint->current_step <= joint->d_accel + joint->d_cruise) {
    joint->current_vel = joint->v_max;
  }
  // Deceleration state
  else if (joint->current_step > joint->d_accel + joint->d_cruise &&
           joint->current_step <= joint->target_steps) {
    joint->current_vel -= joint->accel * DT;
  }
}

void move_complete(volatile MotionProfile *joint1,
                   volatile MotionProfile *joint2,
                   volatile JointAngles_t *joints) {

  // reset after a move
  joint1->target_steps = 0;
  joint1->current_step = 0;
  joint1->v_max = 0;
  joint1->accel = 0;
  joint1->d_accel = 0;
  joint1->d_cruise = 0;
  joint1->current_vel = 0;
  joint1->is_moving = false;

  joint2->target_steps = 0;
  joint2->current_step = 0;
  joint2->v_max = 0;
  joint2->accel = 0;
  joint2->d_accel = 0;
  joint2->d_cruise = 0;
  joint2->current_vel = 0;
  joint2->is_moving = false;
}
/*Function:  motion_profile_initialize
 * - initialize value in the MotionProfile struct to know value
 * Argument: volatile MotionProfile *joint1, volatile MotionProfile *joint2
 * Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
void motion_profile_initialize(volatile MotionProfile *joint1,
                               volatile MotionProfile *joint2) {
  joint1->target_steps = 0;
  joint1->current_step = 0;
  joint1->total_steps = 0;
  joint1->step_accumulator = 0;
  joint1->v_max = 0;
  joint1->accel = 0;
  joint1->d_accel = 0;
  joint1->d_cruise = 0;
  joint1->current_vel = 0;
  joint1->move_dir = HOME;
  joint1->is_moving = false;
  //
  joint2->target_steps = 0;
  ;
  joint2->current_step = 0;
  joint2->total_steps = 0;
  joint2->step_accumulator = 0;
  joint2->v_max = 0;
  joint2->accel = 0;
  joint2->d_accel = 0;
  joint2->d_cruise = 0;
  joint2->current_vel = 0;
  joint2->move_dir = HOME;
  joint2->is_moving = false;
}

/*Function:  cpuTimer1_ISR
 * - Interrupt of timer 1 going at user specify rate, to update and drive the
 * motor Argument: none Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
void cpuTimer1_ISR(void) {
  //step_state_joint1 = !step_state_joint1;
  //step_state_joint2 = !step_state_joint2;
  // 1. Trapezoidal velocity math for Joint 1
  if (joint1.is_moving) {
    update_velocity_profile(
        &joint1); // update current veloccity based on current step taken
    joint1.step_accumulator +=
        joint1.current_vel *
        DT; // step accumulator increase faster or slower depend on current vel
    if (joint1.step_accumulator >= 1.0f) {
      // toggle driver pin
      tmc_step(MOTOR_1);
      
      // reset once move is initiate, -1 to keep motor from drifting
      joint1.step_accumulator -= 1.0f;
      // increase or decrease total step counter base on direction
      joint1.total_steps += joint1.move_dir;
      // increase current step counter
      joint1.current_step++;
      if (joint1.current_step >= joint1.target_steps) {
        joint1.is_moving = false;
      }
    }
  }

  // 2. Repeat for Joint 2 (Keeps links perfectly in sync)
  if (joint2.is_moving) {
    update_velocity_profile(
        &joint2); // update current veloccity based on current step taken
    joint2.step_accumulator +=
        joint2.current_vel *
        DT; // step accumulator increase faster or slower depend on current vel
    if (joint2.step_accumulator >= 1.0f) {
      // toggle driver pin
      //tmc_step(MOTOR_2);
      // reset once move is initiate, -1 to keep motor from drifting
      joint2.step_accumulator -= 1.0f;
      // increase or decrease total step counter base on direction
      joint2.total_steps += joint2.move_dir;
      // increase current step counter
      joint2.current_step++;
      if (joint2.current_step >= joint2.target_steps) {
        joint2.is_moving = false;
      }
    }
  }
  // clear interrup flag
  CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
}

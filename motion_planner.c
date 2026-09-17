#include "motion_planner.h"
#include "system.h"
#include "timer.h"
#include "tmc2209.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

static const float DT = (1.0f / TIMER1_FREQ);
volatile MotionProfile axes[NUM_AXES];
volatile CurrentPosition coordinate;

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
void plan_move(volatile MotionProfile *axes, volatile JointAngles_t *joints,
               float z_target) {

  // J1/J2 have limited range (±150°/±145°) — every legal angle is unambiguous,
  // so use the raw difference. Wrapping to ±180° can pick a shorter path that
  // sweeps through the out-of-range dead zone.
  float diff1 = joints->theta1 - joints->prev_theta1;
  float diff2 = joints->theta2 - joints->prev_theta2;
  //
  float z_target_steps = z_mm_to_steps(z_target);

  // calculate delta, how far to move
  int32_t delta1 =
      (int32_t)roundf(diff1 / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f));
  int32_t delta2 =
      (int32_t)roundf(diff2 / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f));
  int32_t delta3 = (int32_t)roundf(z_target_steps - axes[AXIS_Z].total_steps);

  // assign direction base on delta
  // Only flip J1 (the upside-down one), leave homing alone:
  uint16_t dir1 = (delta1 >= 0) ? COUNTER_CLOCKWISE : CLOCK_WISE;
  if (J1_DIR_INVERT) dir1 = -dir1;
  axes[AXIS_J1].move_dir = dir1;

  uint16_t dir2 = (delta2 >= 0) ? COUNTER_CLOCKWISE : CLOCK_WISE;
  if (J2_DIR_INVERT) dir2 = -dir2;
  axes[AXIS_J2].move_dir = dir1;

  axes[AXIS_Z].move_dir = (delta3 >= 0) ? COUNTER_CLOCKWISE : CLOCK_WISE;

  //
  axes[AXIS_J1].target_steps = labs(delta1);
  axes[AXIS_J2].target_steps = labs(delta2);
  axes[AXIS_Z].target_steps = labs(delta3);

  ///////////////////////////////////////////////
  int i = 0;

  // set per-axis vmax and accel before the loop
  axes[AXIS_J1].vmax = GLOBAL_VMAX;
  axes[AXIS_J1].accel = GLOBAL_ACCEL;

  axes[AXIS_J2].vmax = GLOBAL_VMAX;
  axes[AXIS_J2].accel = GLOBAL_ACCEL;

  axes[AXIS_Z].vmax = Z_VMAX;
  axes[AXIS_Z].accel = Z_ACCEL;

  // Calculate the traperzoidal velocity profile
  for (i = 0; i < NUM_AXES; i++) {
    // set vmin
    axes[i].current_vel = GLOBAL_VMIN;
    // set direction pin for the motor driver
    if (axes[i].move_dir == COUNTER_CLOCKWISE) {
      //
      tmc_set_direction(axes[i].motor, MOTOR_CCW);
    } else if (axes[i].move_dir == CLOCK_WISE) {
      //
      tmc_set_direction(axes[i].motor, MOTOR_CW);
    }

    // Assing global vmax and accel
    float local_vmax = axes[i].vmax;
    axes[i].accel = GLOBAL_ACCEL;

    // Calculate distance to reach vmax
    float d_accel_ideal = (local_vmax * local_vmax) / (2.0f * axes[i].accel);

    // The TRIANGLE check
    if ((2.0f * d_accel_ideal) > axes[i].target_steps) {
      //
      axes[i].d_accel = axes[i].target_steps / 2;

      // Calculate new lower peak velocity
      local_vmax = sqrtf(2.0f * axes[i].accel * axes[i].d_accel);
    } else {
      axes[i].d_accel = d_accel_ideal;
    }
    // Store the final Vmax
    axes[i].v_max = local_vmax;

    // Calculate d cruise
    axes[i].d_cruise = axes[i].target_steps - (2.0f * axes[i].d_accel);
  }

  // enable motor drivers
  tmc_enable(MOTOR_1);
  tmc_enable(MOTOR_2);
  // tmc_enable(MOTOR_Z);

  DEVICE_DELAY_US(100);

  // Update moving status
  axes[AXIS_J1].is_moving = true;
  axes[AXIS_J2].is_moving = true;
  //  axes[AXIS_Z].is_moving = true;
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
void update_velocity_profile(volatile MotionProfile *axis) {
  // find out which part of the velocity profile we're in
  // Acceleration state
  if (axis->current_step <= axis->d_accel) {
    axis->current_vel += axis->accel * DT;
  }
  // Cruise state at v max
  else if (axis->current_step > axis->d_accel &&
           axis->current_step <= axis->d_accel + axis->d_cruise) {
    axis->current_vel = axis->v_max;
  }
  // Deceleration state
  else if (axis->current_step > axis->d_accel + axis->d_cruise &&
           axis->current_step <= axis->target_steps) {
    axis->current_vel -= axis->accel * DT;
  }
}

void move_complete(volatile MotionProfile *axes) {
  // disnable motor driver once move is done
  tmc_disable(MOTOR_1);
  tmc_disable(MOTOR_2);
  // tmc_disable(MOTOR_Z);

  int i = 0;
  //  reset after a move
  for (i = 0; i < NUM_AXES; i++) {
    axes[i].target_steps = 0;
    axes[i].current_step = 0;
    axes[i].v_max = 0;
    axes[i].accel = 0;
    axes[i].d_accel = 0;
    axes[i].d_cruise = 0;
    axes[i].current_vel = 0;
    axes[i].is_moving = false;
  }
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
void motion_profile_initialize(volatile MotionProfile *axes) {
  int i = 0;
  for (i = 0; i < NUM_AXES; i++) {
    axes[i].vmax = 0;
    axes[i].target_steps = 0;
    axes[i].current_step = 0;
    axes[i].total_steps = 0;
    axes[i].step_accumulator = 0;
    axes[i].v_max = 0;
    axes[i].accel = 0;
    axes[i].d_accel = 0;
    axes[i].d_cruise = 0;
    axes[i].current_vel = 0;
    axes[i].move_dir = HOME;
    axes[i].is_moving = false;
  }

  axes[AXIS_J1].motor = MOTOR_1;
  axes[AXIS_J2].motor = MOTOR_2;
  axes[AXIS_Z].motor = MOTOR_Z;

  axes[AXIS_J1].motor_address = TMC_ADDR_0;
  axes[AXIS_J2].motor_address = TMC_ADDR_1;
  axes[AXIS_Z].motor_address = TMC_ADDR_Z;
}

void coordinate_initialize(volatile CurrentPosition *coordinate) {
  coordinate->x = 0;
  coordinate->y = 0;
  coordinate->z = 0;
}

float z_mm_to_steps(float mm) { return mm * Z_STEPS_PER_MM; }

float z_steps_to_mm(float steps) { return steps / Z_STEPS_PER_MM; }

/*Function:  cpuTimer1_ISR
 * - Interrupt of timer 1 going at user specify rate, to update and drive the
 * motor Argument: none Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
void cpuTimer1_ISR(void) {
  int i = 0;
  for (i = 0; i < NUM_AXES; i++) {
    if (axes[i].is_moving) {
      update_velocity_profile(&axes[i]); // update current veloccity based
                                         // on current step taken
      axes[i].step_accumulator +=
          axes[i].current_vel * DT; // step accumulator increase faster or
                                    // slower depend on current vel
      if (axes[i].step_accumulator >= 1.0f) {
        // toggle driver pin
        tmc_step(axes[i].motor);
        // reset once move is initiate, -1 to keep motor from drifting
        axes[i].step_accumulator -= 1.0f;
        // increase or decrease total step counter base on direction
        axes[i].total_steps += axes[i].move_dir;
        // increase current step counter
        axes[i].current_step++;
        if (axes[i].current_step >= axes[i].target_steps) {
          axes[i].is_moving = false;
          axes[i].current_vel = 0.0f; // prevent phantom steps
          axes[i].step_accumulator = 0.0f;
        }
      }
    }
  }
  // tmc_step(MOTOR_1);
  //   clear interrup flag
  CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
}

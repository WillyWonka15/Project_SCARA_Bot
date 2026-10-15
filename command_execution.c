/*Function:  command_execute
 * - excute command base on what command user sent
 * Argument: CMD *cmdList, int index
 *CMD *cmdList contain all needed information from user
 * Return: none
 *
 *
 * Author: WN
 * Date: 2026/05/25
 */
#include "command_execution.h"
#include "homing_gpio.h"
#include "kinematics.h"
#include "system.h"
#include "tmc2209.h"
#include "usci.h"
#include <stdbool.h>
#include <stdio.h>
#include <strings.h>

// Timer0 ISR — runs every 1ms continuously
volatile uint16_t switch_state[3] = {0, 0, 0};
volatile Waypoint program[MAX_WAYPOINTS];
volatile TeachProgram teach;

/*Function: command_execute
 * - execute the command sent by the user
 * Argument: CMD *cmdList, int index
 * CMD *cmdList: hold the command argument from the user
 * int index: index of user command
 * Return: none
 *
 *
 * Author: WN
 * Date: 2026/10/01
 */
void command_execute(CMD *cmdList, int index) {

  char buffer[300] = {0};

  switch (index) {
  case INDEX_MOVE: {
    // compute the joint angle from user x and y
    bool i = solve_auto_config(cmdList[index].args[0], cmdList[index].args[1],
                               &joints);
    if (i == false) {
      strcpy(buffer, "\r\nOut of range move\r\n");
      SCIA_TXstr(buffer);
      break;
    }

    // check if z is within limit
    float z_target = cmdList[index].args[2];
    if (z_target < Z_MIN_MM || z_target > Z_MAX_MM) {
      strcpy(buffer, "\r\nZ out of range\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    // assign value once pass the limit check
    joints.prev_z = joints.z;
    joints.z = z_target;

    // plan velocity profile
    plan_move(axes, &joints, z_target);

    // loop until both joint have stop moving
    while (axes[AXIS_J1].is_moving || axes[AXIS_J2].is_moving ||
           axes[AXIS_Z].is_moving) {
    }
    // update coordinate after move

    // reset speed after move
    move_complete(axes);
    strcpy(buffer, "\r\nDone moving\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  case INDEX_HOME: {
    // ===== PHASE 1: home each axis to its switch =====
    // Z first — retract up to avoid collision during rotation
    home_axis(MOTOR_Z, TMC2_HOME_DIR);
    home_axis(MOTOR_1, TMC0_HOME_DIR);
    home_axis(MOTOR_2, TMC1_HOME_DIR);

    // ===== Set all references to switch positions =====
    joints.prev_theta1 = J1_SWITCH_ANGLE_RAD;
    joints.prev_theta2 = J2_SWITCH_ANGLE_RAD;
    joints.prev_z = Z_HOME_MM;

    // Sync step counters to switch positions
    axes[AXIS_J1].total_steps = (int32_t)roundf(
        J1_SWITCH_ANGLE_RAD / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f));
    axes[AXIS_J2].total_steps = (int32_t)roundf(
        J2_SWITCH_ANGLE_RAD / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f));
    axes[AXIS_Z].total_steps = z_mm_to_steps(Z_SWITCH_POSITION);

    // ===== PHASE 2: coordinated move to home =====
    joints.theta1 = J1_HOME_ANGLE_RAD; // home angles
    joints.theta2 = J2_HOME_ANGLE_RAD; // home angles
    joints.z = Z_HOME_MM;

    // move back to 0 deg position
    plan_move(axes, &joints, Z_HOME_MM);

    // loop until both joint have stop moving
    while (axes[AXIS_J1].is_moving || axes[AXIS_J2].is_moving ||
           axes[AXIS_Z].is_moving) {
    }
    // reset speed after move
    move_complete(axes);
    strcpy(buffer, "\r\nDone homing\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  case INDEX_JOG: {
    // check limit of move before plan move
    bool i =
        jog_limit_check(&joints, cmdList[index].joint, cmdList[index].args[0]);
    if (i == false) {
      strcpy(buffer, "\r\nOut of range jog\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    // plan single move
    plan_single_move(axes, &joints, cmdList[index].joint);

    //
    while (axes[AXIS_J1].is_moving || axes[AXIS_J2].is_moving ||
           axes[AXIS_Z].is_moving) {
    }
    // update coordinate after move

    // reset speed after move
    move_complete(axes);
    strcpy(buffer, "\r\nDone jogging\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  case INDEX_TEACH: {
    if (teach.teach_mode == true) {
      strcpy(buffer, "\r\nAlready in teach mode\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    // set teach mode flag to true
    teach.teach_mode = true;
    // start fresh everytime
    teach.num_waypoints = 0;
    teach.recorded_program = false;
    strcpy(buffer, "\r\nEnter teaching mode\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  case INDEX_END: {
    // set teach mode flag to false if already in teach mode
    if (teach.teach_mode == true) {
      teach.teach_mode = false;
      strcpy(buffer, "\r\nExiting teach mode\r\n");
      SCIA_TXstr(buffer);
      // set recorded program if recorded somethign
      if (teach.num_waypoints > 0) {
        teach.recorded_program = true;
        //sprintf(buffer, "\r\nTaught %d points\r\n", teach.num_waypoints);
        //SCIA_TXstr(buffer);
      }
    } else {
      strcpy(buffer, "\r\nNot in teach mode\r\n");
      SCIA_TXstr(buffer);
    }
    break;
  }
  case INDEX_RECORD: {
    if (teach.teach_mode == false) {
      strcpy(buffer, "\r\nNot in teach mode\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    if (teach.num_waypoints >= MAX_WAYPOINTS) {
      strcpy(buffer, "\r\nProgram full\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    // record current position
    program[teach.num_waypoints].theta1 = joints.theta1;
    program[teach.num_waypoints].theta2 = joints.theta2;
    program[teach.num_waypoints].z = joints.z;
    // increase counter
    teach.num_waypoints += 1;
    //
    //sprintf(buffer, "Recorded theta1: %.3f theta2: %.3f z: %.3f",joints.theta1, joints.theta2, joints.z);
    //SCIA_TXstr(buffer);
    break;
  }
  case INDEX_ADD_DELAY: {
    if (teach.teach_mode == false) {
      strcpy(buffer, "\r\nNot in teach mode\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    if (teach.num_waypoints == 0) {
      strcpy(buffer, "\r\nNo point to add delay\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    int16_t delay_ms = cmdList[index].args[0];
    if (delay_ms < 0) {
      strcpy(buffer, "\r\nDelay can not be a negative number\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    program[teach.num_waypoints - 1].delay = delay_ms;
    break;
  }
  case INDEX_RUN: {
    if (teach.recorded_program == false) {
      strcpy(buffer, "\r\nNo program recorded\r\n");
      SCIA_TXstr(buffer);
      break;
    }
    int i;
    for (i = 0; i < teach.num_waypoints; i++) {
      // assign angle
      joints.prev_theta1 = joints.theta1;
      joints.prev_theta2 = joints.theta2;
      joints.prev_z = joints.z;
      joints.theta1 = program[i].theta1;
      joints.theta2 = program[i].theta2;
      joints.z = program[i].z;

      // plan move
      plan_move(axes, &joints, program[i].z);
      while (axes[AXIS_J1].is_moving || axes[AXIS_J2].is_moving ||
             axes[AXIS_Z].is_moving) {
      }
      // reset speed after move
      move_complete(axes);
      // add delay if there is any
      if (program[i].delay > 0) {
        DEVICE_DELAY_US((uint32_t)program[i].delay * 1000);
      }
    }
    strcpy(buffer, "\r\nPlayback done\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  }
  // Temporary
  //strcpy(buffer, "\r\n");
  //SCIA_TXstr(buffer);
}

/*Function: home_axis
 * - home a single axis to limit switch then do a slow approach and stop
 * Argument: int driver, int homeDir
 * int driver: the motor to initiate home sequence
 * int homeDir: the direction moving to limit switch
 * Return: none
 *
 *
 * Author: WN
 * Date: 2026/10/01
 */
void home_axis(int driver, int homeDir) {
  // Phase 1 — fast approach
  tmc_set_direction(driver, homeDir);
  tmc_enable(driver);
  while (!switch_state[driver]) {
    tmc_step(driver);
    DEVICE_DELAY_US(HOMING_FAST_DELAY_US);
  }

  DEVICE_DELAY_US(200000);

  // Back off — move away from switch
  tmc_set_direction(driver, !homeDir);
  uint16_t k;
  for (k = 0; k < HOMING_BACKOFF_STEPS; k++) {
    tmc_step(driver);
    DEVICE_DELAY_US(HOMING_FAST_DELAY_US);
  }

  DEVICE_DELAY_US(200000);

  // Phase 2 — slow creep back to switch
  tmc_set_direction(driver, homeDir);
  while (!switch_state[driver]) {
    tmc_step(driver);
    DEVICE_DELAY_US(HOMING_SLOW_DELAY_US);
  }
  tmc_disable(driver);
  // This point is precise home
}

/*Function: jog_limit_check
 * - check the limit for a jog command
 * Argument: volatile JointAngles_t *joints, int index, float target
 * volatile JointAngles_t *joints: struct holding all the angle and z postion
 * int index: the joint to jog
 * float target: the amount to move for this jog command
 * Return: true if target is reachable and false if not
 *
 *
 * Author: WN
 * Date: 2026/10/01
 */
bool jog_limit_check(volatile JointAngles_t *joints, int index, float target) {
  bool i = true;
  float theta1;
  float theta2;
  float z;
  switch (index) {
  case AXIS_J1: {
    joints->prev_theta1 = joints->theta1;
    joints->theta1 += target * M_PI / 180.0f;
    theta1 = joints->theta1;
    theta2 = joints->theta2;
    i = joints_in_limit(&theta1, &theta2);
    break;
  }
  case AXIS_J2: {
    joints->prev_theta2 = joints->theta2;
    joints->theta2 += target * M_PI / 180.0f;
    theta1 = joints->theta1;
    theta2 = joints->theta2;
    i = joints_in_limit(&theta1, &theta2);
    break;
  }
  case AXIS_Z: {
    z = joints->z;
    z += target;
    if (z < Z_MIN_MM || z > Z_MAX_MM) {
      i = false;
      break;
    }
    joints->prev_z = joints->z;
    joints->z = z;
    break;
  }
  }
  return i;
}

/*Function: motor_drivers_initialize
 * - initialize the motor driver board for all 3 axis
 * Argument: volatile MotionProfile *axes
 * volatile MotionProfile *axes: holding motor information to be use to
 * initialize Return: None
 *
 *
 * Author: WN
 * Date: 2026/10/01
 */
void motor_drivers_initialize(volatile MotionProfile *axes) {
  // initialize pin use to communicate with tmc2209
  tmc_gpio_init();
  int microstep[3] = {5, 5, 7};
  int i = 0;
  for (i = 0; i < NUM_AXES; i++) {
    // Step 1 — enable UART interface
    // PDN_DISABLE bit 6 = 1 + Spreadcycle
    tmc_write_reg(axes[i].motor_address, TMC_REG_GCONF, 0x000000C4);
    DEVICE_DELAY_US(100);

    // NODECONF register 0x03
    // SENDDELAY bits [7:4] — set to 2 = 8 bit times delay
    tmc_write_reg(axes[i].motor_address, TMC_REG_NODECONF, 0x00000200);
    DEVICE_DELAY_US(100);

    // set IHOLD and IRUN
    tmc_set_current(axes[i].motor_address, 18, 8);
    DEVICE_DELAY_US(100);

    // 1/8 microstep
    tmc_set_microsteps(axes[i].motor_address, microstep[i]);
    DEVICE_DELAY_US(100);

    // TPOWERDOWN: delay before standstill power-down
    // Set to max so it never powers down mid-homing:
    tmc_write_reg(axes[i].motor_address, TMC_REG_TPOWERDOWN, 0x14);
    DEVICE_DELAY_US(100);

    // put motor driver in reset up on start up
    tmc_disable(axes[i].motor);

    // reset step pin
    GPIO_writePin(axes[i].motor_step_pin, 0);

    // default direction pin to CCW
    GPIO_writePin(axes[i].motor_dir_pin, MOTOR_CCW);

    GPIO_writePin(axes[i].motor_en_pin, 1);
  }
}

/*Function: waypoints_initialize
 * - initialize the waypoints data struct using for recording and playback
 * Argument: volatile Waypoint *program, volatile TeachProgram *teach
 * volatile Waypoint *program: holding the position and delay recorded
 * volatile TeachProgram *teach: holding flags
 * Return: None
 *
 *
 * Author: WN
 * Date: 2026/10/01
 */
void waypoints_initialize(volatile Waypoint *program,
                          volatile TeachProgram *teach) {
  int i = 0;
  teach->num_waypoints = 0;
  teach->teach_mode = false;
  teach->recorded_program = false;
  for (i = 0; i <= MAX_WAYPOINTS; i++) {
    program[i].theta1 = 0;
    program[i].theta2 = 0;
    program[i].z = 0;
    program[i].delay = 0;
  }
}

/*Function: cpuTimer0_ISR
 * - timer 0 running at low frequency checking for limit switch, confirm trigger
 * and release by 8 stable samples Argument: None Return: None
 *
 *
 * Author: WN
 * Date: 2026/10/01
 */
void cpuTimer0_ISR(void) {
  static uint16_t history[3] = {0, 0, 0};
  uint32_t pins[3] = {GPIO_HOME_JOINT1, GPIO_HOME_JOINT2, GPIO_HOME_Z};

  uint16_t i;
  for (i = 0; i < 3; i++) {
    // shift in latest reading
    history[i] = (history[i] << 1) | GPIO_readPin(pins[i]);
    history[i] &= 0xFF;

    // 8 stable HIGH samples → confirmed triggered
    if (history[i] == 0xFF)
      switch_state[i] = 1;
    // 8 stable LOW samples → confirmed released
    else if (history[i] == 0x00)
      switch_state[i] = 0;
  }

  CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

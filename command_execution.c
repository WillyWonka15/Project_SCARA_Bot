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

int command_execute(CMD *cmdList, int index) {

  char buffer[MAX_CHAR_ARG] = {0};

  switch (index) {
  case INDEX_MOVE: {
    // compute the joint angle from user x and y
    bool i = solve_auto_config(cmdList[index].args[0], cmdList[index].args[1],
                               &joints);
    if (i == false) {
      sprintf(buffer, "\r\n out of range  move\r\n");
      SCIA_TXstr(buffer);
      break;
    }

    // check if z is within limit
    float z_target = cmdList[index].args[2];
    if (z_target < Z_MIN_MM || z_target > Z_MAX_MM) {
      sprintf(buffer, "\r\n Z out of range\r\n");
      SCIA_TXstr(buffer);
      break;
    }

    // plan velocity profile
    plan_move(axes, &joints, z_target);

    // loop until both joint have stop moving
    while (axes[AXIS_J1].is_moving || axes[AXIS_J2].is_moving ||
           axes[AXIS_Z].is_moving) {
    }
    //
    /*float theta1 = axes[AXIS_J1].total_steps * MOTOR_ANGLE_RESOLUTION;
    float theta2 = axes[AXIS_J2].total_steps * MOTOR_ANGLE_RESOLUTION;*/

    // update coordinate after move

    // reset speed after move
    move_complete(axes);
    sprintf(buffer, "\r\nDone moving\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  case INDEX_HOME: {
    // ===== PHASE 1: home each axis to its switch =====
    // Z first — retract up to avoid collision during rotation
    // home_axis(MOTOR_Z, TMC2_HOME_DIR);
    home_axis(MOTOR_1, TMC0_HOME_DIR);
    home_axis(MOTOR_2, TMC1_HOME_DIR);

    // ===== Set all references to switch positions =====
    joints.prev_theta1 = J1_SWITCH_ANGLE_RAD;
    joints.prev_theta2 = J2_SWITCH_ANGLE_RAD;

    // Sync step counters to switch positions
    axes[AXIS_J1].total_steps = (int32_t)roundf(
        J1_SWITCH_ANGLE_RAD / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f));
    axes[AXIS_J2].total_steps = (int32_t)roundf(
        J2_SWITCH_ANGLE_RAD / (MOTOR_ANGLE_RESOLUTION * M_PI / 180.0f));
    axes[AXIS_Z].total_steps =
        // z_mm_to_steps(Z_SWITCH_POSITION);

        // ===== PHASE 2: coordinated move to home =====
        joints.theta1 = J1_HOME_ANGLE_RAD; // home angles
    joints.theta2 = J2_HOME_ANGLE_RAD;     // home angles

    // move back to 0 deg position
    plan_move(axes, &joints, Z_HOME_MM);

    // loop until both joint have stop moving
    while (axes[AXIS_J1].is_moving || axes[AXIS_J2].is_moving ||
           axes[AXIS_Z].is_moving) {
    }

    // reset speed after move
    move_complete(axes);
  }
  case INDEX_JOG: {
    break;
  }
  }

  return 1;
}

void home_axis(int driver, int homeDir) {
  // Phase 1 — fast approach
  tmc_set_direction(driver, homeDir);
  tmc_enable(driver);
  while (switch_state[driver] == 0) {
    tmc_step(driver);
    DEVICE_DELAY_US(HOMING_FAST_DELAY_US);
  }

  // Back off — move away from switch
  tmc_set_direction(driver, !homeDir);
  uint16_t k;
  for (k = 0; k < HOMING_BACKOFF_STEPS; k++) {
    tmc_step(driver);
    DEVICE_DELAY_US(HOMING_FAST_DELAY_US);
  }

  // Phase 2 — slow creep back to switch
  tmc_set_direction(driver, homeDir);
  while (switch_state[driver] == 0) {
    tmc_step(driver);
    DEVICE_DELAY_US(HOMING_SLOW_DELAY_US);
  }
  tmc_disable(driver);
  // This point is precise home
}

void motor_drivers_initialize(volatile MotionProfile *axes) {
  // initialize pin use to communicate with tmc2209
  tmc_gpio_init();

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

    tmc_set_current(axes[i].motor_address, 26, 13);
    DEVICE_DELAY_US(100);

    // 1/8 microstep
    tmc_set_microsteps(axes[i].motor_address, 5);
    DEVICE_DELAY_US(100);
  }

  // Right before the move to 0°, read BOTH registers:
  uint32_t gconf2 = tmc_read_reg(0x01, TMC_REG_GCONF);
  DEVICE_DELAY_US(100);
  uint32_t chop2 = tmc_read_reg(0x01, TMC_REG_CHOPCONF);

  uint16_t mstep_sel = (gconf2 >> 7) & 0x01;
  uint16_t mres = (chop2 >> 24) & 0x0F;

  // reset step pin
  GPIO_writePin(TMC0_STEP_PIN, 0);
  GPIO_writePin(TMC1_STEP_PIN, 0);
  // GPIO_writePin(TMC2_STEP_PIN, 0);

  // default dir pin to CW
  GPIO_writePin(TMC0_DIR_PIN, MOTOR_CCW);
  GPIO_writePin(TMC1_DIR_PIN, MOTOR_CCW);
  // GPIO_writePin(TMC2_DIR_PIN, MOTOR_CCW);
}

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

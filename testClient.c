/*
 *
 * main.c
 *
 *
 *
 *
 */
#include "command_execution.h"
#include "motion_planner.h"
#include "system.h"
#include "timer.h"
#include "tmc2209.h"
#include "usci.h"
#include <stddef.h>

void main(void) {

  // disable global interrupt for initialize
  Interrupt_disableGlobal();

  // initialize device to a known state, run first to lock in the clk speed
  Device_init();
  Device_initGPIO();
  Board_init();

  // interupt section
  // enable PIE
  Interrupt_enablePIE();
  //
  Interrupt_initModule();
  //
  Interrupt_initVectorTable();
  //
  Interrupt_register(INT_TIMER1, &cpuTimer1_ISR);
  //
  Interrupt_enable(INT_TIMER1);

  // initialize timer1 for velocity profile
  timer1_Initialize();

  // initialize SCIA module for UART to laptop
  SCIA_initialize();
  // initialize SCIB module for UART to laptop
  SCIB_initialize();

  // initialize pin use to communicate with tmc2209
  tmc_gpio_init();
  // Step 1 — enable UART interface
  // PDN_DISABLE bit 6 = 1 + Spreadcycle
  tmc_write_reg(0x00, TMC_REG_GCONF, 0x00000044);
  DEVICE_DELAY_US(100);

  // NODECONF register 0x03
  // SENDDELAY bits [7:4] — set to 2 = 8 bit times delay
  tmc_write_reg(0x00, 0x03, 0x00000200);
  DEVICE_DELAY_US(100);

  uint16_t ifcnt_before = tmc_verify_uart(TMC_ADDR_0);
  DEVICE_DELAY_US(100);

  tmc_set_current(TMC_ADDR_0, 26, 13);
  DEVICE_DELAY_US(100);

  // 1/8 microstep
  tmc_set_microsteps(TMC_ADDR_0, 5);
  DEVICE_DELAY_US(100);

  // Read IFCNT after writes — should be higher than before
  uint16_t ifcnt_after = tmc_verify_uart(TMC_ADDR_0);
  DEVICE_DELAY_US(100);

  // reset step pin
  GPIO_writePin(TMC0_STEP_PIN, 0);

  // default dir pin to CW
  GPIO_writePin(TMC0_DIR_PIN, MOTOR_CCW);

  // testing the command interpreter engine
  // local variable
  CMD cmdList[NUM_COMMANDS];
  int index = -1;
  int inputError = -1;
  char userInput[MAX_CHAR_ARG] = {0};

  // initialize command
  command_Inittialize(cmdList);
  //
  motion_profile_initialize(&joint1, &joint2);
  //
  joint_angle_initialize(&joints);

  // initialize global interrupt
  Interrupt_enableGlobal();
  ERTM; // enable real time interrupt

  while (1) {
    // queue user to select a command
    command_Selection();
    while (1) {
      // get user input
      inputError = command_Getter(userInput);
      // skip parsing if user enter more character than allowed
      if (inputError == -1) {
        continue;
      }
      index = command_Parser(cmdList, userInput);
      // exit the loop once successfully parse a command
      if (index != -1) {
        break;
      }
    }
    command_execute(cmdList, index);
    DEVICE_DELAY_US(1000000);
  }

}

// testing single motor
/*void main(void) {

  // disable global interrupt for initialize
  Interrupt_disableGlobal();

  Device_init();
  Board_init();

  Interrupt_enablePIE();
  //
  Interrupt_initModule();
  //
  Interrupt_initVectorTable();
  //
  Interrupt_register(INT_TIMER1, &cpuTimer1_ISR);
  //
  Interrupt_enable(INT_TIMER1);

  // initialize timer1 for velocity profile
  timer1_Initialize();

  // initialize SCIA module for UART to laptop
  SCIA_initialize();
  // initialize SCIB module for UART to laptop
  SCIB_initialize();

  tmc_gpio_init();

  // Step 1 — enable UART interface
  // PDN_DISABLE bit 6 = 1 + Spreadcycle
  tmc_write_reg(0x00, TMC_REG_GCONF, 0x00000044);
  DEVICE_DELAY_US(100);

  // NODECONF register 0x03
  // SENDDELAY bits [7:4] — set to 2 = 8 bit times delay
  tmc_write_reg(0x00, 0x03, 0x00000200);
  DEVICE_DELAY_US(100);

  uint16_t ifcnt_before = tmc_verify_uart(TMC_ADDR_0);
  DEVICE_DELAY_US(100);

  tmc_set_current(TMC_ADDR_0, 26, 13);
  DEVICE_DELAY_US(100);

  // 1/8 microstep
  tmc_set_microsteps(TMC_ADDR_0, 5);
  DEVICE_DELAY_US(100);

  // Read IFCNT after writes — should be higher than before
  uint16_t ifcnt_after = tmc_verify_uart(TMC_ADDR_0);
  DEVICE_DELAY_US(100);

  // reset step pin
  GPIO_writePin(TMC0_STEP_PIN, 0);

  // default dir pin to CW
  GPIO_writePin(TMC0_DIR_PIN, MOTOR_CCW);

  // testing
  tmc_enable(MOTOR_1);

  // initialize global interrupt
  Interrupt_enableGlobal();
  ERTM; // enable real time interrupt

  DEVICE_DELAY_US(5000000);

  tmc_disable(MOTOR_1);
}*/

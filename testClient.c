/*
 *
 * main.c
 *
 *
 *
 *
 */
#include "command_execution.h"
#include "system.h"
#include "timer.h"
#include "tmc2209.h"
#include "usci.h"
#include "homing_gpio.h"
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
  Interrupt_register(INT_TIMER0, &cpuTimer0_ISR);

  // initialize timer1 for velocity profile
  timer1_Initialize();
  
  // initialize timer0 for sampling limit switch reading
  timer0_Initialize();


  // initialize SCIA module for UART to laptop
  SCIA_initialize();
  // initialize SCIB module for motor control
  SCIB_initialize();

  // inititialize GPIO use for homing
  homing_gpio_initialize();

  // local variable
  CMD cmdList[NUM_COMMANDS];
  int index = -1;
  int inputError = -1;
  char userInput[MAX_CHAR_ARG] = {0};

  // initialize data struct
  command_Inittialize(cmdList);

  // initialize motion profile, information for each axis
  motion_profile_initialize(axes);

  // initialize coordinate tracking
  coordinate_initialize(&coordinate);

  // initialize joints angle tracking
  joint_angle_initialize(&joints);

  // initialize motor driver board
  motor_drivers_initialize(axes);

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

  //Interrupt_enablePIE();
  //
  //Interrupt_initModule();
  //
  //Interrupt_initVectorTable();
  //
  //Interrupt_register(INT_TIMER1, &cpuTimer1_ISR);
  //
  //Interrupt_enable(INT_TIMER1);

  // initialize timer1 for velocity profile
  //timer1_Initialize();

  // initialize SCIA module for UART to laptop
  SCIA_initialize();
  // initialize SCIB module for UART to laptop
  SCIB_initialize();

  tmc_gpio_init();

  // Step 1 — enable UART interface
  // PDN_DISABLE bit 6 = 1 + Spreadcycle
  tmc_write_reg(TMC_ADDR_1, TMC_REG_GCONF, 0x000000C4);
  DEVICE_DELAY_US(100);

  // NODECONF register 0x03
  // SENDDELAY bits [7:4] — set to 2 = 8 bit times delay
  tmc_write_reg(TMC_ADDR_1, 0x03, 0x00000200);
  DEVICE_DELAY_US(100);

  uint16_t ifcnt_before = tmc_verify_uart(TMC_ADDR_1);
  DEVICE_DELAY_US(100);

  tmc_set_current(TMC_ADDR_1, 26, 13);
  DEVICE_DELAY_US(100);

  // 1/8 microstep
  tmc_set_microsteps(TMC_ADDR_1, 5);
  DEVICE_DELAY_US(100);

  // Read IFCNT after writes — should be higher than before
  uint16_t ifcnt_after = tmc_verify_uart(TMC_ADDR_1);
  DEVICE_DELAY_US(100);

  // reset step pin
  GPIO_writePin(TMC1_STEP_PIN, 0);

  // default dir pin to CW
  GPIO_writePin(TMC1_DIR_PIN, MOTOR_CCW);

  // 1. Read CHOPCONF
  uint32_t chopconf = tmc_read_reg(TMC_ADDR_1, TMC_REG_CHOPCONF);

  // 2. Extract mres (bits 27:24)
  uint16_t mres = (chopconf >> 24) & 0x0F;

  // testing
  //tmc_enable(MOTOR_2);

  // initialize global interrupt
  //Interrupt_enableGlobal();
  //ERTM; // enable real time interrupt

  //DEVICE_DELAY_US(5000000);

  //tmc_disable(MOTOR_2);
}*/

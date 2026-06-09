/*
 *
 * main.c
 *
 *
 *
 *
 */
#include "command_execution.h"
#include "gpio_init.h"
#include "motion_planner.h"
#include "system.h"
#include "timer.h"
#include "tmc2209.h"
#include "usci.h"


void main(void) {

  #ifdef _FLASH
memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
       (size_t)((uint32_t)&RamfuncsLoadEnd -
                (uint32_t)&RamfuncsLoadStart));
#endif
  // disable global interrupt for initialize
  Interrupt_disableGlobal();

  // initialize device to a known state, run first to lock in the clk speed
  Device_init();
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

  // initialize GPIO pin for SCIA
  SCIA_GPIOinit();
  // initialize GPIO pin for SCIB
  SCIB_GPIOinit();

  tmc_gpio_init();

  // NODECONF register 0x03
  // SENDDELAY bits [7:4] — set to 2 = 8 bit times delay
  tmc_write_reg(0x00, 0x03, 0x00000200);

  // set up current and microstep
  tmc_set_current(TMC_ADDR_0, 26, 13);
  tmc_set_microsteps(TMC_ADDR_0, 5);

  // reset step pin
  GPIO_writePin(TMC0_STEP_PIN, 0);
  // GPIO_writePin(TMC1_STEP_PIN, 0);

  // default dir pin to 0
  GPIO_writePin(TMC0_DIR_PIN, MOTOR_CW);
  // GPIO_writePin(TMC1_DIR_PIN, 0);

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
    while (1) {
      // queue user to select a command
      command_Selection();
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
    // execute commadn
    command_execute(cmdList, index);
  }

  // int test = 0;
}

/*void main(void) {

  Device_init();
  Board_init();

  // interupt section
  // enable PIE
  Interrupt_enablePIE();
  //
  Interrupt_initModule();

  // initialize SCIA module for UART to laptop
  SCIA_initialize();
  // initialize SCIB module for UART to laptop
  SCIB_initialize();

  tmc_gpio_init();

  tmc_enable(MOTOR_1);

  // NODECONF register 0x03
  // SENDDELAY bits [7:4] — set to 2 = 8 bit times delay
  tmc_write_reg(0x00, 0x03, 0x00000200);

  // Read IFCNT before any writes — should be 0
  uint16_t ifcnt_before = tmc_verify_uart(TMC_ADDR_0);

  // Write something to driver
  tmc_set_current(TMC_ADDR_0, 18, 9);
  tmc_set_microsteps(TMC_ADDR_0, 4);
  tmc_set_direction(MOTOR_1, 1);


  // Read IFCNT after writes — should be higher than before
  uint16_t ifcnt_after = tmc_verify_uart(TMC_ADDR_0);

  // If ifcnt_after > ifcnt_before → UART is working ✅

}*/

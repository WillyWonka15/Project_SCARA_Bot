#include "command_execution.h"
#include "kinematics.h"
#include "motion_planner.h"
#include "usci.h"
#include "tmc2209.h"
#include <stdbool.h>
#include <stdio.h>
#include <strings.h>


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
    // plan velocity profile
    plan_move(&joint1, &joint2, &joints);
    // loop until both joint have stop moving
    while (joint1.is_moving || joint2.is_moving) {
    }
    //
    float theta1 = joint1.total_steps * MOTOR_ANGLE_RESOLUTION;
    float theta2 = joint2.total_steps * MOTOR_ANGLE_RESOLUTION;
    move_complete(&joint1, &joint2, &joints);
    sprintf(buffer, "\r\nDone moving\r\n");
    SCIA_TXstr(buffer);
    break;
  }
  case INDEX_HOME: {

    // figure out which elbow, direction the robot need to use to move back

    // initiate the move until hit limit switch

    // reset parameter once home is done
    motion_profile_initialize(&joint1, &joint2);
  case INDEX_USER_DRIVE:
    break;
  }
  }

  return 1;
}

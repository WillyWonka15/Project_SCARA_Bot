#include "command_interpreter.h"
#include "system.h"
#include "usci.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

/*Function: usciInit
 * - initialize the structure CMD that is use to store user input and all of the
 * mode for SCARA BOT Argument: *cmd Return: NONE
 *
 *
 * Author: WN
 * Date: 2026/04/05
 * */
void command_Inittialize(CMD *cmdList) {
  int i = 0;
  int j = 0;
  int16_t nArgs[NUM_COMMANDS] = {CMD0_NARGS, CMD1_NARGS, CMD2_NARGS, CMD3_NARGS,
                                 CMD4_NARGS, CMD5_NARGS, CMD6_NARGS, CMD7_NARGS};

  // initialize the command name
  cmdList[INDEX_MOVE].name = CMD0;
  cmdList[INDEX_HOME].name = CMD1;
  cmdList[INDEX_JOG].name = CMD2;
  cmdList[INDEX_TEACH].name = CMD3;
  cmdList[INDEX_END].name = CMD4;
  cmdList[INDEX_RECORD].name = CMD5;
  cmdList[INDEX_ADD_DELAY].name = CMD6;
  cmdList[INDEX_RUN].name = CMD7;
  // initialize the argument array and the number of argument for each command
  for (j = 0; j < NUM_COMMANDS; j++) {
    cmdList[j].nargs = nArgs[j];
    cmdList[j].joint = -1;
    for (i = 0; i < MAX_ARG; i++) {
      cmdList[j].args[i] = 0;
    }
  }
}
/*Function: command_selection
 * - display a list ofcommand with description for user to choose from
 * Argument: none
 * Return: NONE
 *
 *
 * Author: WN
 * Date: 2026/04/05
 * */
void command_Selection(void) {
  char buffer[MAX_CHAR_ARG] = {0};
  // Clean the screen everytime running
  ANSI_sequence(CLEAR_SCREEN_SEQUENCE);
  ANSI_sequence(CURSOR_HOME_SEQUENCE);

  strcpy(buffer, "Please choose from the list of the following commands:");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "MOVE X Y Z");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: allow user to move TCP to their desire x & y & z "
                 "coordiate\r\nArgument: X - x coordinate in mm\r\n          Y - y "
                 "coordinate in mm\r\n          Z- z absolute distance in mm");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "HOME");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: home the TCP to x:0 and y:0 also hard reset the "
                 "coordinate values\r\nArgument: NONE");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "JOG Joint Distance");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(
      buffer,
      "Function: allow user to drive any axis by increment or decrement value "
      "of the input\r\nArgument: Joint - J1,J2,Z to choose axis to jog "
      "\r\n          Distance - angle input in degree or z distance in mm");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "TEACH");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer,
         "Function: Mark the start of teaching mode\r\nArgument: NONE");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "END");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: Mark the end of teaching mode\r\nArgument: NONE");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "RECORD");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: Record the last move "
                 "in teach mode\r\nArgument: NONE");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "ADD_DELAY time");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: add a delay in ms in between last recorded move "
                 "and next record move\r\nArgument: time - A number "
                 "represent the delay time in millisecond");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "RUN");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: play back recorded move in teach mode\r\nArgument: None");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);
}
/*Function: command_Parser
 * - Take input line from user then tokenize it and validate
 * Argument: CMD *cmdList, char *userInput
 * - CMD *cmdList point to the array that contain the preset command name as
 * well as the number of argument
 * - char *userInput is the array contain what user type in
 * Return: index number that match with what command user input or -1 if there
 * was an error
 *
 * Author: WN
 * Date: 2026/04/30
 * */
int command_Parser(CMD *cmdList, char *userInput) {
  //
  char buffer[MAX_BUFF_SIZE] = {0};
  // local variable
  int i = 0;
  int index = 0;
  float val = 0;
  // local variable use for tokenization
  char seps[] = " ";
  char *tok = NULL;
  char *pEnd = NULL;

  // tokenize the first part of the input string
  tok = strtok(userInput, seps);
  // if no command name were found
  if (tok == NULL) {
    strcpy(buffer, "\r\nDid not find any command name, please try again.\r\n");
    SCIA_TXstr(buffer);
    return -1;
  }
  // validate command name if found something
  else {
    index = commmand_Validate(cmdList, tok);
    // check for error
    if (index == -1) {
      strcpy(buffer, "\r\nWrong command name, please try again.\r\n");
      SCIA_TXstr(buffer);
      return -1;
    }
  }

  // validate joint input, for JOG command only
  if (index == INDEX_JOG) {
    tok = strtok(NULL, seps);
    int joint = parseJoint(tok);
    if (joint == -1) {
      strcpy(buffer, "\r\nwrong input to select axis.\r\n");
      SCIA_TXstr(buffer);
      return -1;
    } else {
      cmdList[index].joint = joint;
    }
  }

  // once know the index of the command, continue to store the argument
  for (i = 0; i < cmdList[index].nargs; i++) {
    // keep tokenize the input string
    tok = strtok(NULL, seps);
    // if found no trailing argument
    if (tok == NULL) {
      strcpy(buffer, "\r\nPlease enter the argument.\r\n");
      SCIA_TXstr(buffer);
      return -1;
    }
    // if found something, continue to validate it
    else {
      errno = 0;
      val = strtof(tok, &pEnd);
      // check if there are extra character behind
      if (*pEnd != '\0') {
        strcpy(buffer, "\r\nExtra character behind argument.\r\n");
        SCIA_TXstr(buffer);
        return -1;
      }
      // over flown character
      else if (errno == ERANGE) {
        strcpy(buffer, "\r\nOverflown input.\r\n");
        SCIA_TXstr(buffer);
        return -1;
      }
      // check for argument value range
      // if input value biger than int16 range (change to the x & y range of
      //  robot)
      else if (val < -32768 || val > 32768) {
        strcpy(buffer, "\r\nInput value outside of int16 range.\r\n");
        SCIA_TXstr(buffer);
        return -1;
      }
      // store agument once pass validation
      cmdList[index].args[i] = val;
    }
  }
  return index;
}

/*Function: command_validate
 * - compare the command the user input versus the preset command to determine
 which command it is by return the index matching with that command
 * Argument: CMD *cmdList, char *cmdName
 *
 * Return: -1 if user did not enter any valid command name, if user enter valid
 command then return: *0 for MOVE *1 for HOME *2 for USER_DRIVE
 *
 * Author: WN
 * Date: 2026/04/22
 * */
int commmand_Validate(CMD *cmdList, char *cmdName) {
  int found_command = -1;
  int i = 0;

  // loop to check the name against all command name possible, return matching
  // index if found
  for (i = 0; i < NUM_COMMANDS; i++) {
    // found_command will be 0 if found a match
    found_command = strcmp(cmdName, cmdList[i].name);
    // exit the loop immediately if found a matching command
    if (found_command == 0) {
      return i;
    }
  }
  // return -1 if did not find any matching command
  return -1;
}
/*Function: error_handling
 * process and send feed for user base on the error code receive
 * Argument: ParseError errorCode
 * ParseError errorcode contain the error code need to be check
 *
 * Return: 0 if there is no error and 1 if there is an error
 *
 * Author: WN
 * Date: 2026/04/22
 * */
int error_Handling(ParseError errorCode) {
  int i = 0;
  char buffer[MAX_CHAR_ARG] = {0};

  switch (errorCode) {

  case PARSE_OK:
    i = 0;
    break;

  case PARSE_ERROR:
    ANSI_sequence(CURSOR_DOWN_1LINE);
    strcpy(buffer, "Did not find any command, please try again.");
    SCIA_TXstr(buffer);
    ANSI_sequence(CURSOR_DOWN_1LINE);
    i = 1;
    break;

  case PARSE_INVALID_CMD:
    ANSI_sequence(CURSOR_DOWN_1LINE);
    strcpy(buffer, "Invalid command name, please try again.");
    SCIA_TXstr(buffer);
    ANSI_sequence(CURSOR_DOWN_1LINE);
    i = 1;
    break;

  case PARSE_INVALID_ARG:
    ANSI_sequence(CURSOR_DOWN_1LINE);
    strcpy(buffer, "Invalid argument, please try again.");
    SCIA_TXstr(buffer);
    ANSI_sequence(CURSOR_DOWN_1LINE);
    i = 1;
    break;

  case PARSE_OVERFLOW:
    ANSI_sequence(CURSOR_DOWN_1LINE);
    strcpy(buffer, "Overflow commmand, please try again.");
    SCIA_TXstr(buffer);
    ANSI_sequence(CURSOR_DOWN_1LINE);
    i = 1;
    break;
  }

  return i;
}
/*Function: command_Getter
 * use function in usci.c to get and store user input commadn into an array
 * Argument: char *userInput
 *
 *
 * Return: 0 if there is no error and -1 if there is an error
 *
 * Author: WN
 * Date: 2026/04/22
 * */
int command_Getter(char *userInput) {
  // local variable
  int error = 0;
  char buffer[MAX_BUFF_SIZE] = {0};

  // call the function to get user input
  error = SCIA_RXstr(userInput);
  // if user input is longer than max character allowed, set error flag
  if (error == -1) {
    strcpy(buffer, "\r\nInput exceed allowed characters.\r\n");
    SCIA_TXstr(buffer);
    return -1;
  } 
  else {
    return 0;
  }
}

int parseJoint(const char *tok) {
  if (strcmp(tok, "J1") == 0)
    return 0;
  if (strcmp(tok, "J2") == 0)
    return 1;
  if (strcmp(tok, "Z") == 0)
    return 2;
  return -1; // invalid
}

/*Function: argument_Validate
 * validate val base on which index command it is
 * Argument: long val, int index
 *
 *
 * Return: 0 if value is within range and 1 if it's out of range
 *
 * Author: WN
 * Date: 2026/04/22
 * */
int argument_Validate(long val, int index) {
  int i = 0;

  switch (index) {
  case INDEX_HOME:
    i = 0;
    break;

  case INDEX_MOVE:
    // needed robot design to bound X and Y value, visit later
    i = 0;
    break;

  case INDEX_JOG:
    i = 0;
    break;
  }
  return i;
}

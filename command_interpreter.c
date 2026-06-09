#include "command_interpreter.h"
#include "system.h"
#include "usci.h"
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
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
  // initialize the command name
  cmdList[INDEX_MOVE].name = CMD0;
  cmdList[INDEX_HOME].name = CMD1;
  cmdList[INDEX_USER_DRIVE].name = CMD2;
  // initialize the number of arguments in each command
  cmdList[INDEX_MOVE].nargs = CMD0_NARGS;
  cmdList[INDEX_HOME].nargs = CMD1_NARGS;
  cmdList[INDEX_USER_DRIVE].nargs = CMD2_NARGS;
  // initialize the argument array
  for (i = 0; i < MAX_ARG; i++) {
    cmdList[INDEX_MOVE].args[i] = 0;
    cmdList[INDEX_HOME].args[i] = 0;
    cmdList[INDEX_USER_DRIVE].args[i] = 0;
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
  // ANSI_sequence(CLEAR_SCREEN_SEQUENCE);
  // ANSI_sequence(CURSOR_HOME_SEQUENCE);

  strcpy(buffer, "Please choose from the list of the following commands:");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "MOVE X_coordinate, Y_coordinate");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer,
         "Function: allow user to move TCP to their desire x & y coordiate.");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "HOME");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: home the TCP to x:0 and y:0 also hard reset the "
                 "coordinate values");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "USER DRIVE speed");
  SCIA_TXstr(buffer);
  ANSI_sequence(CURSOR_DOWN_1LINE);

  strcpy(buffer, "Function: allow user to drive the TCP with their desire "
                 "speed from 0 to 100");
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
  long val = 0;
  // local variable use for tokenization
  char seps[] = " ";
  char *tok = NULL;
  char *pEnd = NULL;
  // error code
  ParseError errorCode = PARSE_OK;

  // tokenize the first part of the input string
  tok = strtok(userInput, seps);
  // if no command name were found
  if (tok == NULL) {
    errorCode = PARSE_ERROR;
    // check error code
    if (error_Handling(errorCode)) {
      return -1;
    }
  }
  // validate command name if found something
  else {
    index = commmand_Validate(cmdList, tok, &errorCode);
    // check error code
    if (error_Handling(errorCode)) {
      return -1;
    }
  }

  // once know the index of the command, continue to store the argument
  for (i = 0; i < cmdList[index].nargs; i++) {
    // keep tokenize the input string
    tok = strtok(NULL, seps);
    // test
    sprintf(buffer, "\r\ninput is %s\r\n", tok);
    SCIA_TXstr(buffer);
    // if found no trailing argument
    if (tok == NULL) {
      errorCode = PARSE_INVALID_ARG;
      // check error code
      if (error_Handling(errorCode)) {
        return -1;
      }
    }
    // if found something, continue to validate it
    else {
      errno = 0; // 
      val = strtol(tok, &pEnd, 10);
      // check if there are extra character behind
      if (*pEnd != '\0') {
        errorCode = PARSE_INVALID_ARG;
        // check error code
        if (error_Handling(errorCode)) {
          return -1;
        }
      }
      // over flown character
      else if (errno == ERANGE) {
        errorCode = PARSE_OVERFLOW;
        // check error code
        if (error_Handling(errorCode)) {
          return -1;
        }
      }
      //check for argument value range 
      //if input value biger than int16 range (change to the x & y range of
      // robot)
      else if (val < -32768 || val > 32768) {
        errorCode = PARSE_OVERFLOW;
        // check error code
        if (error_Handling(errorCode)) {
          return -1;
        }
      }
      //check if value is in operating range of the robot design
      /*if(argument_Validate(val, index))
      {
        errorCode = PARSE_OVERFLOW;
        if (error_Handling(errorCode)) {
          return -1;
        }
      }*/
      // store agument once pass validation
      cmdList[index].args[i] = (float)val;
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
int commmand_Validate(CMD *cmdList, char *cmdName, ParseError *errorCode) {
  int found_command = -1;
  int i = 0;

  // loop to check the name against all command name possible, return matching
  // index if found
  for (i = 0; i < NUM_COMMANDS; i++) {
    // found_command will be 0 if found a match
    found_command = strcmp(cmdName, cmdList[i].name);
    // exit the loop immediately if found a matching command
    if (found_command == 0) {
      *errorCode = PARSE_OK;
      return i;
    }
  }
  // return -1 if did not find any matching command, also set error state
  *errorCode = PARSE_INVALID_CMD;
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
  ParseError errorCode = PARSE_OK;

  // call the function to get user input
  error = SCIA_RXstr(userInput);
  // if user input is longer than max character allowed, set error flag
  if (error == -1) {
    errorCode = PARSE_OVERFLOW;
  }

  if (error_Handling(errorCode)) {
    return -1;
  } else {
    return 0;
  }
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
int argument_Validate(long val, int index)
{
  int i = 0;

  switch (index) {
  case INDEX_HOME:
  i = 0;
  break;

  case INDEX_MOVE:
  //needed robot design to bound X and Y value, visit later
  i= 0;
  break;

  case INDEX_USER_DRIVE:
  if (val > 0 && val <= 100)
  {
    i = 0;
  }
  else {
    i = 1;
  }
  break;
  }
return i;
}


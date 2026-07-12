/*
 * command_interpreter.h
 *
 *  Created on: 2026/04/05
 *  Author: Will Nguyen
 *  Description: Get user input and process it.
 *
 */
#ifndef COMMAND_INTERPRETER_H_
#define COMMAND_INTERPRETER_H_

#include<stdint.h>
//#include "usci.h"


#define MAX_CHAR_ARG 100
#define MAX_ARG 3
#define CMD0 "MOVE"
#define CMD1 "HOME"
#define CMD2 "USER_DRIVE"
#define CMD0_NARGS 3
#define CMD1_NARGS 0
#define CMD2_NARGS 1
#define X_MAX_RANGE_POS 50
#define Y_MAX_RANGE_POS 50
#define X_MAX_RANGE_NEG -50
#define Y_MAX_RANGE_NEG -50
//GLOBAL VARIABLE

//structure, use to store user input data
typedef struct CMD{
    const char *name;
    int nargs;
    float args[MAX_ARG];
}CMD;
//Enum of error code
typedef enum{
    PARSE_OK,
    PARSE_ERROR,
    PARSE_INVALID_CMD,
    PARSE_INVALID_ARG,
    PARSE_OVERFLOW
} ParseError;
//Enum of command list 
enum COMMAND_LIST_INDEX 
{
    INDEX_MOVE, INDEX_HOME, INDEX_USER_DRIVE, NUM_COMMANDS
};
//FUNCTION PROTOTYPE
/*
 */
void command_Inittialize(CMD *cmdList);
/*
*/
void command_Selection(void);
/*
*/
int command_Getter(char *userInput);
/*
*/
int command_Parser(CMD *cmdList, char *cmdName);
/*
*/
int commmand_Validate(CMD *cmdList, char *cmdName, ParseError *errorCode);
/*
*/
int argument_Validate(long val, int index);
/*
*/
int error_Handling(ParseError errorCode);

#endif






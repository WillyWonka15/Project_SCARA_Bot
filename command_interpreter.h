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


#define MAX_CHAR_ARG 300
#define MAX_ARG 3

#define CMD0_NARGS 3
#define CMD1_NARGS 0
#define CMD2_NARGS 1
#define CMD3_NARGS 0
#define CMD4_NARGS 0
#define CMD5_NARGS 0
#define CMD6_NARGS 1
#define CMD7_NARGS 0

#define CMD0 "MOVE"
#define CMD1 "HOME"
#define CMD2 "JOG"
#define CMD3 "TEACH"
#define CMD4 "END"
#define CMD5 "RECORD"
#define CMD6 "ADD_DELAY"
#define CMD7 "RUN"

#define X_MAX_RANGE_POS 50
#define Y_MAX_RANGE_POS 50
#define X_MAX_RANGE_NEG -50
#define Y_MAX_RANGE_NEG -50
//GLOBAL VARIABLE

//structure, use to store user input data
typedef struct CMD{
    const char *name;
    int nargs;
    int joint;          // 0=J1, 1=J2, 2=Z (-1 = none/unused)
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
    INDEX_MOVE, INDEX_HOME, INDEX_JOG, INDEX_TEACH, INDEX_END, INDEX_RECORD, INDEX_ADD_DELAY, INDEX_RUN, NUM_COMMANDS
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
int commmand_Validate(CMD *cmdList, char *cmdName);
/*
*/
int argument_Validate(long val, int index);
/*
*/
int error_Handling(ParseError errorCode);
/*
*/
int parseJoint(const char *tok);
#endif






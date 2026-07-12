/*
 * usci.h
 *
 *  Created on: 2025/10/02
 *  Author: Will Nguyen
 *  Description: Header for SCI protocol, this will define the configuration
 * function as well as the receive and transmit.
 *
 */
#ifndef USCI_H_
#define USCI_H_

#include <stdint.h>
// #include<stdbool.h>

// Pre-define constant
#define CLEAR_SCREEN_SEQUENCE 0
#define CURSOR_BACKSPACE_SEQUENCE 1
#define CURSOR_HOME_SEQUENCE 2
#define CURSOR_DOWN_1LINE 3
#define BACKSPACE_DELETE_SEQUENCE 4
// Normally, the SYSCLK start up at 200Mhz. Low speed clock is default at /4
// from SYSCLK
#define LSPCLK 25000000
// BAUD rate
#define BAUD 115200
// Max buffer size for user input
#define MAX_BUFF_SIZE 100
/*
 *
 */
void SCIA_initialize(void);
/*
 *
 */
void SCIB_initialize(void);
/*
 *
 */
void SCIA_GPIOinit(void);
/*
 *
 */
void SCIB_GPIOinit(void);
/*
 *
 */
void SCIA_TXstr(char *txByte);
/*
 *
 */
int16_t SCIA_RXstr(char *rxStr);
/*
 *
 */
// void ansiSequenceExecute(int16_t mode);
/*
 *
 */
void ANSI_sequence(int16_t mode);

#endif

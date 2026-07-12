/*
 * usci.c
 *
 *  Created on: 2025/10/02
 *  Author: Will Nguyen
 *  Description: Source for SCI protocol, this will define the configuration function as well as the receive and transmit.
 *
 */
#include "usci.h"
#include "system.h"
#include<strings.h>
#include<stdio.h>

/*Function: usciInit
 * - initialize the SCI module use for UART, in this case we'll use SCI_A as it is integrated to the USB on the device
 * Argument: NONE
 * Return: NONE
 *
 *
 * Author: WN
 * Date: 2026/03/26
 * */
void SCIA_initialize(void)
{
    //Put SCIA in reset for configuration
    SCI_performSoftwareReset(SCIA_BASE);

    //disable FIFO fucntionality
    SCI_disableFIFO(SCIA_BASE);

    //Configured the SCI module using built in function from driverlib.h
    SCI_setConfig(SCIA_BASE, SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ), BAUD,
                  SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE);

    SCI_disableLoopback(SCIA_BASE);

    // initialize GPIO pin for SCIA
    SCIA_GPIOinit();
    
    //enable module
    //SCI_performSoftwareReset(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
}

void SCIB_initialize(void)
{
    //Put SCIA in reset for configuration
    SCI_performSoftwareReset(SCIB_BASE);

    //Configured the SCI module using built in function from driverlib.h
    SCI_setConfig(SCIB_BASE, SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ), BAUD,
                 SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE| SCI_CONFIG_PAR_NONE);

    // enable FIFO
    SCI_enableFIFO(SCIB_BASE);

    SCI_disableLoopback(SCIB_BASE);

    // initialize GPIO pin for SCIB
    SCIB_GPIOinit();
    
    //enable module
    SCI_enableModule(SCIB_BASE);
    SCI_performSoftwareReset(SCIB_BASE);
}

/*Function: usciTXstr
 * - send a string of data through UART
 * Argument: txByte pointer point to the array of characters to send
 * Return: NONE
 *
 *
 * Author: WN
 * Date: 2025/01/28

 */
void SCIA_TXstr(char *txByte)
{

    int16_t i = 0;
    //loop through the array while, stop only when reach the null character at end of string
    while (txByte[i] != '\0')
    {
        //check if transmiter is busy or not, maybe not needed
        while(SCI_isTransmitterBusy(SCIA_BASE));
        //clear buffer
        
        //this function send character thru TX
        SCI_writeCharBlockingNonFIFO(SCIA_BASE, (uint16_t)txByte[i]);
        i++;
    }

}
/*Function: usciRXstr
 * - Receive input string from user
 * Argument: txStr pointer point to receive array
 * Return: -1 if user input is longer than MAX_BUFF_SIZE defined in .h file, other wise return 0
 *
 *
 * Author: WN
 * Date: 2025/02/01

 */
int16_t SCIA_RXstr(char *rxStr){
    //counter
    int16_t i = 0;
    int16_t ret = 0;
    //local receiver string
    char rxBuff[MAX_BUFF_SIZE] = {0};

    while (1)
    {
        rxBuff[i] = SCI_readCharBlockingNonFIFO(SCIA_BASE);
        //if user not enter "enter" key, move on to receive character
        if (rxBuff[i] != 0x0d)
        {
            //if user pass the limit of receive buffer
            if (i >= MAX_BUFF_SIZE)
            {
                ret = -1;
                return ret;
            }
            //if user does not press back space
            if (rxBuff[i] != 0x08)
            {
                //echo character back
                SCI_writeCharBlockingNonFIFO(SCIA_BASE, (uint16_t)rxBuff[i]);
                //increase pointer
                i++;
            }
            //if user press back space
            else if (rxBuff[i] == 0x08)
            {
                //if user already input something then move counter back by 1
                if (i > 0)
                {
                    i--;
                }
                //clear character and move cursor back by 1 space
                ANSI_sequence(CURSOR_BACKSPACE_SEQUENCE);
                ANSI_sequence(BACKSPACE_DELETE_SEQUENCE);
            }
        }
        //exit the loop if user hit enter key
        else if (rxBuff[i] == 0x0d)
        {
            i++;
            break;
        }
    }

   //insert Null character at the end of receive string if there is somethign in the receive buffer
   if(i>0)
   {
    i--;
   }
    rxBuff[i] = '\0';
    //copy string
    strcpy(rxStr, rxBuff);
    ret = 0;
    //flush input buffer by reset both RX and TX channel
    SCI_resetChannels(SCIA_BASE);
    return ret;
}

/*Function: SCIA_GPIOinit
 * - initialize the GPIO pin for SCIA module
 * Argument: NONE
 * Return: NONE
 *
 *
 * Author: WN
 * Date: 2026/03/26
 * */
void SCIA_GPIOinit(void)
{  
    // set controller core
    GPIO_setControllerCore(42,GPIO_CORE_CPU1);
    GPIO_setControllerCore(43,GPIO_CORE_CPU1);

    //Set pin to peripheral mode
    GPIO_setPinConfig(GPIO_42_SCITXDA);
    GPIO_setPinConfig(GPIO_43_SCIRXDA);

    //set pin direction 
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    
    //set pin as floating i/p and o/p
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
}


void SCIB_GPIOinit(void)
{  
    // set controller core
    GPIO_setControllerCore(18,GPIO_CORE_CPU1);
    GPIO_setControllerCore(19,GPIO_CORE_CPU1);

    //Set pin to peripheral mode
    GPIO_setPinConfig(GPIO_18_SCITXDB);
    GPIO_setPinConfig(GPIO_19_SCIRXDB);

    //set pin direction 
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    
    //set pin as floating i/p and o/p
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    //qualification mode 
    GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);
}

/*Function: ANSIsequence
 * - execute ANSI sequence on UART interface
 * Argument: mode
 * Return: none
 *
 *
 * Author: WN
 * Date: 2025/02/18

 */
void ANSI_sequence(int16_t mode)
{
    char ansiSequence[10];
    switch (mode)
    {
    case CLEAR_SCREEN_SEQUENCE:
        strcpy(ansiSequence, "\x1B[2J");
        SCIA_TXstr(ansiSequence);
        break;

    case CURSOR_BACKSPACE_SEQUENCE:
        strcpy(ansiSequence, "\x1B[1D");
        SCIA_TXstr(ansiSequence);
        break;

    case CURSOR_HOME_SEQUENCE:
        strcpy(ansiSequence, "\x1B[H");
        SCIA_TXstr(ansiSequence);
        break;

    case CURSOR_DOWN_1LINE:
        strcpy(ansiSequence, "\r\n");
        SCIA_TXstr(ansiSequence);
        break;

    case BACKSPACE_DELETE_SEQUENCE:
        strcpy(ansiSequence, "\x1B[0J");
        SCIA_TXstr(ansiSequence);
        break;
    }
}

/*
 * gpio_init.c
 *
 *  Created on: 2026/03/26
 *  Author: Will Nguyen
 *  Description: initialize the GPIO for different module in the F28379D device
 *
 */
#include "system.h"
#include "gpio_init.h"

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


void GPIO_Init_Output(void)
{
    // motor 1 drive pin
    GPIO_setPinConfig(GPIO_0_GPIO0);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // motor 2 drive pin
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // motor 1 direction pin
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // motor 2 direction pin
    GPIO_setPinConfig(GPIO_3_GPIO3);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
}

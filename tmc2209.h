#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
//
// Driver addresses (set by MS1/MS2 pins)
//
#define TMC_ADDR_0 0x00 // MS1=GND, MS2=GND
#define TMC_ADDR_1 0x01 // MS1=3.3V, MS2=GND

//
// Register addresses
//
#define TMC_REG_GCONF 0x00
#define TMC_REG_IFCNT 0x02
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_CHOPCONF 0x6C
#define TMC_REG_SGTHRS 0x40
#define TMC_REG_PWMCONF 0x70



//
// GPIO pin defines — change to match your wiring
//
// Driver 1
#define MOTOR_1 0
#define TMC0_STEP_PIN 0
#define TMC0_DIR_PIN 1
#define TMC0_EN_PIN 2

// Driver 2
#define MOTOR_2 1
#define TMC1_STEP_PIN 3
#define TMC1_DIR_PIN 4
#define TMC1_EN_PIN 5

#define MOTOR_CW 1
#define MOTOR_CCW 0
//
// Function declarations
//
void     tmc_gpio_init      (void);

void     tmc_write_reg      (uint16_t addr, uint16_t reg, uint32_t value);
uint32_t tmc_read_reg       (uint16_t addr, uint16_t reg);

void     tmc_set_current    (uint16_t addr, uint16_t irun, uint16_t ihold);
void     tmc_set_microsteps (uint16_t addr, uint16_t mres);
uint16_t  tmc_verify_uart    (uint16_t addr);

void     tmc_enable         (uint16_t driver);
void     tmc_disable        (uint16_t driver);
void     tmc_set_direction  (uint16_t driver, uint16_t dir);
void     tmc_step           (uint16_t driver);

void delay_200ns(void);


#endif // TMC2209_H



#include "tmc2209.h"
#include "system.h"
#include "usci.h"

// ─────────────────────────────────────────
// Internal helper — not exposed in header
// ─────────────────────────────────────────

uint16_t tmc_crc(uint16_t *data, uint16_t len) {
  uint16_t crc = 0;
  uint16_t i, j;

  for (i = 0; i < len; i++) {
    uint16_t byte = data[i] & 0xFF; // mask to lower 8 bits
    for (j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (byte & 0x01))
        crc = (crc << 1) ^ 0x07;
      else
        crc = (crc << 1);
      byte >>= 1;
    }
    crc &= 0xFF; // keep only lower 8 bits
  }
  return crc;
}

// Returns the STEP pin for a given driver index
static uint32_t get_step_pin(uint16_t driver) {
  return (driver == 0) ? TMC0_STEP_PIN : TMC1_STEP_PIN;
}

static uint32_t get_dir_pin(uint16_t driver) {
  return (driver == 0) ? TMC0_DIR_PIN : TMC1_DIR_PIN;
}

static uint32_t get_en_pin(uint16_t driver) {
  return (driver == 0) ? TMC0_EN_PIN : TMC1_EN_PIN;
}

// ─────────────────────────────────────────
// GPIO init for STEP DIR EN pins
// ─────────────────────────────────────────

void tmc_gpio_init(void) {
  uint32_t pins[6] = {TMC0_STEP_PIN, TMC0_DIR_PIN, TMC0_EN_PIN,
                      TMC1_STEP_PIN, TMC1_DIR_PIN, TMC1_EN_PIN};
  uint16_t i;
  for (i = 0; i < 6; i++) {
    GPIO_setDirectionMode(pins[i], GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(pins[i], GPIO_PIN_TYPE_STD);
    GPIO_writePin(pins[i], 0);
  }

  // Disable both drivers on startup (EN active LOW, so write HIGH = disabled)
  GPIO_writePin(TMC0_EN_PIN, 1);
  GPIO_writePin(TMC1_EN_PIN, 1);
}

// ─────────────────────────────────────────
// UART write and read
// ─────────────────────────────────────────

void tmc_write_reg(uint16_t addr, uint16_t reg, uint32_t value) {
  uint16_t packet[8];

  packet[0] = 0x05;
  packet[1] = addr;
  packet[2] = reg | 0x80; // write flag
  packet[3] = (value >> 24) & 0xFF;
  packet[4] = (value >> 16) & 0xFF;
  packet[5] = (value >> 8) & 0xFF;
  packet[6] = (value) & 0xFF;
  packet[7] = tmc_crc(packet, 7);

  uint16_t i;
  for (i = 0; i < 8; i++)
    SCI_writeCharBlockingFIFO(SCIB_BASE, packet[i]);
}

uint32_t tmc_read_reg(uint16_t addr, uint16_t reg) {
  uint16_t packet[4];

  packet[0] = 0x05;
  packet[1] = addr;
  packet[2] = reg;
  packet[3] = tmc_crc(packet, 3);

  uint16_t i;
  uint16_t buffer[4];
  for (i = 0; i < 4; i++)
    SCI_writeCharBlockingFIFO(SCIB_BASE, packet[i]);

  DEVICE_DELAY_US(100);
  for (i = 0; i < 4; i++)
    buffer[i] = SCI_readCharBlockingFIFO(SCIB_BASE);

  uint16_t response[8];
  for (i = 0; i < 8; i++)
    response[i] = SCI_readCharBlockingFIFO(SCIB_BASE);

  uint32_t value = ((uint32_t)response[3] << 24) |
                   ((uint32_t)response[4] << 16) |
                   ((uint32_t)response[5] << 8) | ((uint32_t)response[6]);

  return value;
}

// ─────────────────────────────────────────
// Configuration functions
// ─────────────────────────────────────────

void tmc_set_current(uint16_t addr, uint16_t irun, uint16_t ihold) {
  // IHOLD_IRUN register layout:
  // bits [12:8]  = IRUN
  // bits [4:0]   = IHOLD
  uint32_t value = ((uint32_t)irun << 8) | ihold;
  tmc_write_reg(addr, TMC_REG_IHOLD_IRUN, value);
}

void tmc_set_microsteps(uint16_t addr, uint16_t mres) {
  // CHOPCONF register — set mres bits [27:24]
  // Base value 0x10000053 is safe default for NEMA17
  uint32_t value = 0x10000053;
  value |= ((uint32_t)mres << 24);
  tmc_write_reg(addr, TMC_REG_CHOPCONF, value);
}

uint16_t tmc_verify_uart(uint16_t addr) {
  // Write to driver then read IFCNT
  // IFCNT increments on every successful UART write
  // If value > 0 after our writes, UART is working
  uint32_t ifcnt = tmc_read_reg(addr, TMC_REG_IFCNT);
  return (uint16_t)(ifcnt & 0xFF);
}

// ─────────────────────────────────────────
// Motor control functions
// ─────────────────────────────────────────

void tmc_enable(uint16_t driver) {
  // EN is active LOW — pull LOW to enable
  GPIO_writePin(get_en_pin(driver), 0);
}

void tmc_disable(uint16_t driver) {
  // Pull HIGH to disable
  GPIO_writePin(get_en_pin(driver), 1);
}

void tmc_set_direction(uint16_t driver, uint16_t dir) {
  GPIO_writePin(get_dir_pin(driver), dir);
}

void delay_200ns(void) {
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
  NOP;
}

void tmc_step(uint16_t driver) {
  uint32_t pin = get_step_pin(driver);

// 20 NOPs = 200ns at 100MHz
#define NOP __asm(" NOP")

  GPIO_writePin(pin, 1);
  delay_200ns();

  GPIO_writePin(pin, 0);
  delay_200ns();
}

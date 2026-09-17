#include "homing_gpio.h"
#include "system.h"

void homing_gpio_initialize() {
  uint32_t pins[3] = {GPIO_HOME_JOINT1, GPIO_HOME_JOINT2, GPIO_HOME_Z};
  uint16_t i;
  for (i = 0; i < 3; i++) {
    GPIO_setDirectionMode(pins[i], GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(pins[i], GPIO_PIN_TYPE_PULLUP);
  }
}

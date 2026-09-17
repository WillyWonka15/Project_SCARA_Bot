#ifndef HOMING_GPIO_H_
#define HOMING_GPIO_H_

#define GPIO_HOME_JOINT1 61
#define GPIO_HOME_JOINT2 123
#define GPIO_HOME_Z 122

#define HOMING_FAST_DELAY_US  700    // ~60 RPM fast approach
#define HOMING_SLOW_DELAY_US  3750   // ~15 RPM slow creep
#define HOMING_BACKOFF_STEPS  533   // steps to back off between phases

#define J1_SWITCH_ANGLE_RAD  (-127.0f * M_PI / 180.0f)
#define J1_HOME_ANGLE_RAD (0.0f)
#define J2_SWITCH_ANGLE_RAD  (-160.0f * M_PI / 180.0f)
#define J2_HOME_ANGLE_RAD (0.0f)
#define Z_HOME_MM (0.0f)

void homing_gpio_initialize();



#endif






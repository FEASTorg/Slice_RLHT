#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "config.h"

// ----- General BREAD -----
#define ESTOP 2
#define LED_PIN 5

// ----- Timing constants -----
#define SERIAL_UPDATE_TIME_MS 1000
#define THERMO_UPDATE_TIME_MS 300

#if (RLHT_HW_GEN == 1)
#define RLHT_HAS_STATUS_LED 0
#elif (RLHT_HW_GEN == 2)
#define RLHT_HAS_STATUS_LED 1
#else
#error "Unsupported RLHT_HW_GEN value"
#endif

// ----- RLHT Specific -----
#define TC_CS1 10
#define TC_CS2 11
#define TC_DATA 12
#define TC_CLK 13

#define RELAY1 6
#define RELAY2 7

#endif // HARDWARE_CONFIG_H

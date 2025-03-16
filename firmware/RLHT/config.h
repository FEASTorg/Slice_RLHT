#ifndef CONFIG_H
#define CONFIG_H

// define the I2C address for this SLICE device
#define I2C_ADR 10
#define TYPE_ID 1 // SLICE type ID for RLHT!!!

// timing constants
#define THERMO_UPDATE_TIME_MS 300
#define SERIAL_UPDATE_TIME_MS 1000

//  Uncomment to enable additional debugging messages:
// #define SLICE_DEBUG 1

#ifdef SLICE_DEBUG

#define SLICE_DEBUG_PRINT(...) \
    Serial.print(__VA_ARGS__)

#define SLICE_DEBUG_PRINTLN(...) \
    Serial.println(__VA_ARGS__)

#else
#define SLICE_DEBUG_PRINT(...)
#define SLICE_DEBUG_PRINTLN(...)
#endif

#endif // CONFIG_H
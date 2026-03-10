#ifndef CONFIG_H
#define CONFIG_H

#ifndef RLHT_HW_GEN
#define RLHT_HW_GEN 2
#endif

#if (RLHT_HW_GEN == 1)
#define I2C_ADR 10
#elif (RLHT_HW_GEN == 2)
#define I2C_ADR 10
#else
#error "Unsupported RLHT_HW_GEN value"
#endif

#define TYPE_ID 1 // SLICE type ID for RLHT

#define VERSION "1.0.0" // firmware version, update when making changes

// Uncomment to enable additional debugging messages:
#define SLICE_DEBUG 1

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

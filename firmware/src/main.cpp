#include <Arduino.h>
#include <FastLED.h>
#include <PID_v1.h>
#include <max6675.h>

#include <crumbs.h>
#include <crumbs_arduino.h>
#include <bread/rlht_ops.h>

#include "config.h"
#include "config_hardware.h"
#include "globals.h"

crumbs_context_t ctx;
volatile bool estopTriggered = false;
CRGB led;

RLHT_SLICE slice;
Timing timing = {0};

MAX6675 thermocouple1(TC_CLK, TC_CS1, TC_DATA);
MAX6675 thermocouple2(TC_CLK, TC_CS2, TC_DATA);

PID relay1PID(&(slice.relayHeater1.inputTemperature), &(slice.relayHeater1.relayOnTime), &(slice.relayHeater1.setpointTemperature),
              slice.relayHeater1.Kp, slice.relayHeater1.Ki, slice.relayHeater1.Kd, DIRECT);
PID relay2PID(&(slice.relayHeater2.inputTemperature), &(slice.relayHeater2.relayOnTime), &(slice.relayHeater2.setpointTemperature),
              slice.relayHeater2.Kp, slice.relayHeater2.Ki, slice.relayHeater2.Kd, DIRECT);

void setup()
{
    setupSlice();
    setupRLHT();
    delay(1000);
}

void loop()
{
    pollEStop();
    measureThermocouples();
    relayControlLogic();
    serialCommands();
    printSerialOutput();
}

void setupSlice()
{
    int rc;

    Serial.begin(115200);

    crumbs_arduino_init_peripheral(&ctx, I2C_ADR);

    rc = crumbs_register_handler(&ctx, RLHT_OP_SET_MODE, handler_set_mode, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_SET_MODE"));

    rc = crumbs_register_handler(&ctx, RLHT_OP_SET_SETPOINTS, handler_set_setpoints, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_SET_SETPOINTS"));

    rc = crumbs_register_handler(&ctx, RLHT_OP_SET_PID, handler_set_pid, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_SET_PID"));

    rc = crumbs_register_handler(&ctx, RLHT_OP_SET_PERIODS, handler_set_periods, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_SET_PERIODS"));

    rc = crumbs_register_handler(&ctx, RLHT_OP_SET_TC_SELECT, handler_set_tc_select, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_SET_TC_SELECT"));

    rc = crumbs_register_handler(&ctx, RLHT_OP_SET_OPEN_DUTY, handler_set_open_duty, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_SET_OPEN_DUTY"));

    rc = crumbs_register_reply_handler(&ctx, 0x00, reply_version, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register version reply handler"));

    rc = crumbs_register_reply_handler(&ctx, RLHT_OP_GET_STATE, reply_get_state, nullptr);
    if (rc != 0)
        SLICE_DEBUG_PRINTLN(F("CRUMBS: Failed to register RLHT_OP_GET_STATE reply handler"));

#if RLHT_HAS_STATUS_LED
    FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);
    FastLED.setBrightness(50);
    led = CRGB::Blue;
    FastLED.show();
#endif

    pinMode(ESTOP, INPUT);
    attachInterrupt(digitalPinToInterrupt(ESTOP), estopISR, CHANGE);

    SLICE_DEBUG_PRINTLN(F("RLHT SLICE INITIALIZED"));
    SLICE_DEBUG_PRINTLN(F("VERSION: " VERSION));
#if (RLHT_HW_GEN == 1)
    SLICE_DEBUG_PRINTLN(F("HW Profile: Gen1"));
#else
    SLICE_DEBUG_PRINTLN(F("HW Profile: Gen2"));
#endif
}

void setupRLHT()
{
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);

    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    relay1PID.SetOutputLimits(0, slice.relayHeater1.relayPeriod);
    relay2PID.SetOutputLimits(0, slice.relayHeater2.relayPeriod);

    relay1PID.SetMode(AUTOMATIC);
    relay2PID.SetMode(AUTOMATIC);

    timing.lastThermoRead = millis();
    timing.lastSerialPrint = millis();
    timing.relay1Start = millis();
    timing.relay2Start = millis();
}

void pollEStop()
{
    if (estopTriggered)
    {
        processEStop();
        estopTriggered = false;
    }

#if RLHT_HAS_STATUS_LED
    led = slice.eStop ? CRGB::Red : CRGB::Green;
    FastLED.show();
#endif
}

void estopISR()
{
    estopTriggered = true;
}

void processEStop()
{
    if (digitalRead(ESTOP) == HIGH)
    {
        slice.relayHeater1.setpointTemperature = 0;
        slice.relayHeater2.setpointTemperature = 0;
        slice.relayHeater1.relayOnTime = 0;
        slice.relayHeater2.relayOnTime = 0;

        digitalWrite(RELAY1, LOW);
        digitalWrite(RELAY2, LOW);

        slice.relay1State = false;
        slice.relay2State = false;
        slice.eStop = true;

        SLICE_DEBUG_PRINTLN(F("ESTOP PRESSED!"));
    }
    else
    {
        slice.eStop = false;
        SLICE_DEBUG_PRINTLN(F("ESTOP RELEASED!"));
    }
}

void measureThermocouples()
{
    if (millis() - timing.lastThermoRead >= THERMO_UPDATE_TIME_MS)
    {
        slice.temperature1 = thermocouple1.readCelsius();
        slice.temperature2 = thermocouple2.readCelsius();
        timing.lastThermoRead = millis();
    }
}

void relayControlLogic()
{
    if (slice.eStop)
    {
        digitalWrite(RELAY1, LOW);
        digitalWrite(RELAY2, LOW);
        slice.relay1State = false;
        slice.relay2State = false;
        return;
    }

    if (slice.mode == CLOSED_LOOP)
    {
        relay1PID.SetMode(AUTOMATIC);
        relay2PID.SetMode(AUTOMATIC);

        relay1PID.SetTunings(slice.relayHeater1.Kp, slice.relayHeater1.Ki, slice.relayHeater1.Kd);
        relay2PID.SetTunings(slice.relayHeater2.Kp, slice.relayHeater2.Ki, slice.relayHeater2.Kd);

        switch (slice.relayHeater1.thermocoupleSelect)
        {
        case 1:
            slice.relayHeater1.inputTemperature = slice.temperature1;
            break;
        case 2:
            slice.relayHeater1.inputTemperature = slice.temperature2;
            break;
        default:
            break;
        }

        if (isnan(slice.relayHeater1.inputTemperature))
            slice.relayHeater1.relayOnTime = 0;
        else
            relay1PID.Compute();

        switch (slice.relayHeater2.thermocoupleSelect)
        {
        case 1:
            slice.relayHeater2.inputTemperature = slice.temperature1;
            break;
        case 2:
            slice.relayHeater2.inputTemperature = slice.temperature2;
            break;
        default:
            break;
        }

        if (isnan(slice.relayHeater2.inputTemperature))
            slice.relayHeater2.relayOnTime = 0;
        else
            relay2PID.Compute();
    }
    else if (slice.mode == OPEN_LOOP)
    {
        relay1PID.SetMode(MANUAL);
        relay2PID.SetMode(MANUAL);
    }
    else
    {
        digitalWrite(RELAY1, LOW);
        digitalWrite(RELAY2, LOW);
        slice.relay1State = false;
        slice.relay2State = false;
        SLICE_DEBUG_PRINTLN(F("ERROR INVALID MODE, ENTERED UNKNOWN STATE!"));
        return;
    }

    if (slice.relayHeater1.relayOnTime < 0)
        slice.relayHeater1.relayOnTime = 0;
    if (slice.relayHeater2.relayOnTime < 0)
        slice.relayHeater2.relayOnTime = 0;

    if (slice.relayHeater1.relayOnTime > slice.relayHeater1.relayPeriod)
        slice.relayHeater1.relayOnTime = slice.relayHeater1.relayPeriod;
    if (slice.relayHeater2.relayOnTime > slice.relayHeater2.relayPeriod)
        slice.relayHeater2.relayOnTime = slice.relayHeater2.relayPeriod;

    actuateRelay(RELAY1, timing.relay1Start, (unsigned long)slice.relayHeater1.relayPeriod, (unsigned long)slice.relayHeater1.relayOnTime, slice.relay1State);
    actuateRelay(RELAY2, timing.relay2Start, (unsigned long)slice.relayHeater2.relayPeriod, (unsigned long)slice.relayHeater2.relayOnTime, slice.relay2State);
}

void actuateRelay(uint8_t relayPin, unsigned long &relayStart, unsigned long relayPeriod, unsigned long relayOnTime, bool &relayState)
{
    unsigned long currentTime = millis();

    if (relayPeriod == 0)
    {
        digitalWrite(relayPin, LOW);
        relayState = false;
        return;
    }

    if (currentTime - relayStart > relayPeriod)
    {
        relayStart += relayPeriod;
    }

    relayState = ((currentTime - relayStart) < relayOnTime);
    digitalWrite(relayPin, relayState ? HIGH : LOW);
}

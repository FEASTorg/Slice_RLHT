#include "globals.h"
#include "config.h"

void printSerialOutput()
{
    if (millis() - timing.lastSerialPrint >= SERIAL_UPDATE_TIME_MS)
    {
        SLICE_DEBUG_PRINT(F("ControlMode: "));
        SLICE_DEBUG_PRINT(slice.mode == CLOSED_LOOP ? F("CLOSED_LOOP") : F("OPEN_LOOP"));
        SLICE_DEBUG_PRINT(F(", T1:"));
        SLICE_DEBUG_PRINT(slice.temperature1);
        SLICE_DEBUG_PRINT(F(", T2:"));
        SLICE_DEBUG_PRINT(slice.temperature2);

        SLICE_DEBUG_PRINT(F(", PID1:"));
        SLICE_DEBUG_PRINT(slice.relayHeater1.Kp);
        SLICE_DEBUG_PRINT(F(","));
        SLICE_DEBUG_PRINT(slice.relayHeater1.Ki);
        SLICE_DEBUG_PRINT(F(","));
        SLICE_DEBUG_PRINT(slice.relayHeater1.Kd);

        SLICE_DEBUG_PRINT(F(", Relay1Input:"));
        SLICE_DEBUG_PRINT(slice.relayHeater1.inputTemperature);
        SLICE_DEBUG_PRINT(F(", Setpoint1:"));
        SLICE_DEBUG_PRINT(slice.relayHeater1.setpointTemperature);
        SLICE_DEBUG_PRINT(F(", onTime1:"));
        SLICE_DEBUG_PRINT((int)slice.relayHeater1.relayOnTime);
        SLICE_DEBUG_PRINT(F(", rPeriod1:"));
        SLICE_DEBUG_PRINT(slice.relayHeater1.relayPeriod);

        SLICE_DEBUG_PRINT(F(", PID2:"));
        SLICE_DEBUG_PRINT(slice.relayHeater2.Kp);
        SLICE_DEBUG_PRINT(F(","));
        SLICE_DEBUG_PRINT(slice.relayHeater2.Ki);
        SLICE_DEBUG_PRINT(F(","));
        SLICE_DEBUG_PRINT(slice.relayHeater2.Kd);

        SLICE_DEBUG_PRINT(F(", Relay2Input:"));
        SLICE_DEBUG_PRINT(slice.relayHeater2.inputTemperature);
        SLICE_DEBUG_PRINT(F(", Setpoint2:"));
        SLICE_DEBUG_PRINT(slice.relayHeater2.setpointTemperature);
        SLICE_DEBUG_PRINT(F(", onTime2:"));
        SLICE_DEBUG_PRINT((int)slice.relayHeater2.relayOnTime);
        SLICE_DEBUG_PRINT(F(", rPeriod2:"));
        SLICE_DEBUG_PRINT(slice.relayHeater2.relayPeriod);
        SLICE_DEBUG_PRINT(F(", Thermo Select Relay 1:"));
        SLICE_DEBUG_PRINT(slice.relayHeater1.thermocoupleSelect);
        SLICE_DEBUG_PRINT(F(", Thermo Select Relay 2:"));
        SLICE_DEBUG_PRINT(slice.relayHeater2.thermocoupleSelect);
        SLICE_DEBUG_PRINT(F(", ESTOP:"));
        SLICE_DEBUG_PRINTLN(slice.eStop);

        timing.lastSerialPrint = millis();
    }
}

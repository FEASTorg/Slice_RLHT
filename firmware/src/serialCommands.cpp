#include <Arduino.h>

#include "globals.h"

void serialCommands()
{
    if (Serial.available() <= 0)
        return;

    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith(F("MODE=")))
    {
        String mode = command.substring(5);
        mode.toUpperCase();
        if (mode.equals(F("CLOSED_LOOP")) || mode.equals(F("0")))
        {
            slice.mode = CLOSED_LOOP;
            Serial.println(F("Mode set to CLOSED_LOOP"));
        }
        else if (mode.equals(F("OPEN_LOOP")) || mode.equals(F("1")))
        {
            slice.mode = OPEN_LOOP;
            Serial.println(F("Mode set to OPEN_LOOP"));
        }
        else
        {
            Serial.println(F("Invalid mode. Use CLOSED_LOOP/0 or OPEN_LOOP/1"));
        }
    }
    else if (command.startsWith(F("R1TEMP=")))
    {
        if (slice.mode == CLOSED_LOOP)
        {
            double value = command.substring(7).toFloat();
            if (value >= 0 && value <= 500)
            {
                slice.relayHeater1.setpointTemperature = value;
                Serial.print(F("Relay 1 setpoint: "));
                Serial.println(value);
            }
            else
            {
                Serial.println(F("Temp out of range (0-500C)"));
            }
        }
        else
        {
            Serial.println(F("R1 temp only in CLOSED_LOOP"));
        }
    }
    else if (command.startsWith(F("R2TEMP=")))
    {
        if (slice.mode == CLOSED_LOOP)
        {
            double value = command.substring(7).toFloat();
            if (value >= 0 && value <= 500)
            {
                slice.relayHeater2.setpointTemperature = value;
                Serial.print(F("Relay 2 setpoint: "));
                Serial.println(value);
            }
            else
            {
                Serial.println(F("Temp out of range (0-500C)"));
            }
        }
        else
        {
            Serial.println(F("R2 temp only in CLOSED_LOOP"));
        }
    }
    else if (command.startsWith(F("R1TIME=")))
    {
        if (slice.mode == OPEN_LOOP)
        {
            double value = command.substring(7).toFloat();
            if (value >= 0 && value <= slice.relayHeater1.relayPeriod)
            {
                slice.relayHeater1.relayOnTime = value;
                Serial.print(F("R1 time: "));
                Serial.print(value);
                Serial.println(F("ms"));
            }
            else
            {
                Serial.print(F("Time range 0-"));
                Serial.print(slice.relayHeater1.relayPeriod);
                Serial.println(F("ms"));
            }
        }
        else
        {
            Serial.println(F("R1 time only in OPEN_LOOP"));
        }
    }
    else if (command.startsWith(F("R2TIME=")))
    {
        if (slice.mode == OPEN_LOOP)
        {
            double value = command.substring(7).toFloat();
            if (value >= 0 && value <= slice.relayHeater2.relayPeriod)
            {
                slice.relayHeater2.relayOnTime = value;
                Serial.print(F("R2 time: "));
                Serial.print(value);
                Serial.println(F("ms"));
            }
            else
            {
                Serial.print(F("Time range 0-"));
                Serial.print(slice.relayHeater2.relayPeriod);
                Serial.println(F("ms"));
            }
        }
        else
        {
            Serial.println(F("R2 time only in OPEN_LOOP"));
        }
    }
    else if (command.startsWith(F("R1TC=")))
    {
        int value = command.substring(5).toInt();
        if (value == 1 || value == 2)
        {
            slice.relayHeater1.thermocoupleSelect = (uint8_t)value;
            Serial.print(F("R1 TC: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("TC select 1 or 2"));
        }
    }
    else if (command.startsWith(F("R2TC=")))
    {
        int value = command.substring(5).toInt();
        if (value == 1 || value == 2)
        {
            slice.relayHeater2.thermocoupleSelect = (uint8_t)value;
            Serial.print(F("R2 TC: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("TC select 1 or 2"));
        }
    }
    else if (command.startsWith(F("R1KP=")))
    {
        double value = command.substring(5).toFloat();
        if (value >= 0)
        {
            slice.relayHeater1.Kp = value;
            Serial.print(F("R1 Kp: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("Kp must be >= 0"));
        }
    }
    else if (command.startsWith(F("R1KI=")))
    {
        double value = command.substring(5).toFloat();
        if (value >= 0)
        {
            slice.relayHeater1.Ki = value;
            Serial.print(F("R1 Ki: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("Ki must be >= 0"));
        }
    }
    else if (command.startsWith(F("R1KD=")))
    {
        double value = command.substring(5).toFloat();
        if (value >= 0)
        {
            slice.relayHeater1.Kd = value;
            Serial.print(F("R1 Kd: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("Kd must be >= 0"));
        }
    }
    else if (command.startsWith(F("R2KP=")))
    {
        double value = command.substring(5).toFloat();
        if (value >= 0)
        {
            slice.relayHeater2.Kp = value;
            Serial.print(F("R2 Kp: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("Kp must be >= 0"));
        }
    }
    else if (command.startsWith(F("R2KI=")))
    {
        double value = command.substring(5).toFloat();
        if (value >= 0)
        {
            slice.relayHeater2.Ki = value;
            Serial.print(F("R2 Ki: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("Ki must be >= 0"));
        }
    }
    else if (command.startsWith(F("R2KD=")))
    {
        double value = command.substring(5).toFloat();
        if (value >= 0)
        {
            slice.relayHeater2.Kd = value;
            Serial.print(F("R2 Kd: "));
            Serial.println(value);
        }
        else
        {
            Serial.println(F("Kd must be >= 0"));
        }
    }
    else if (command.startsWith(F("R1PERIOD=")))
    {
        int value = command.substring(9).toInt();
        if (value >= 100 && value <= 10000)
        {
            slice.relayHeater1.relayPeriod = value;
            relay1PID.SetOutputLimits(0, slice.relayHeater1.relayPeriod);
            Serial.print(F("R1 period: "));
            Serial.print(value);
            Serial.println(F("ms"));
        }
        else
        {
            Serial.println(F("Period range 100-10000ms"));
        }
    }
    else if (command.startsWith(F("R2PERIOD=")))
    {
        int value = command.substring(9).toInt();
        if (value >= 100 && value <= 10000)
        {
            slice.relayHeater2.relayPeriod = value;
            relay2PID.SetOutputLimits(0, slice.relayHeater2.relayPeriod);
            Serial.print(F("R2 period: "));
            Serial.print(value);
            Serial.println(F("ms"));
        }
        else
        {
            Serial.println(F("Period range 100-10000ms"));
        }
    }
    else if (command.startsWith(F("HELP")) || command.startsWith(F("?")))
    {
        Serial.println(F("Commands:"));
        Serial.println(F("MODE=CLOSED_LOOP/OPEN_LOOP"));
        Serial.println(F("R1TEMP=<val> - R1 setpoint (CLOSED)"));
        Serial.println(F("R2TEMP=<val> - R2 setpoint (CLOSED)"));
        Serial.println(F("R1TIME=<val> - R1 time ms (OPEN)"));
        Serial.println(F("R2TIME=<val> - R2 time ms (OPEN)"));
        Serial.println(F("R1TC=1/2 - R1 thermocouple"));
        Serial.println(F("R2TC=1/2 - R2 thermocouple"));
        Serial.println(F("R1KP/KI/KD=<val> - R1 PID"));
        Serial.println(F("R2KP/KI/KD=<val> - R2 PID"));
        Serial.println(F("R1PERIOD=<val> - R1 period ms"));
        Serial.println(F("R2PERIOD=<val> - R2 period ms"));
    }
    else
    {
        Serial.println(F("Invalid cmd! Type HELP"));
    }
}

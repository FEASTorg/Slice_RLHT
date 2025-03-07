#include <Wire.h>
#include "max6675.h"
#include <FastLED.h>
#include <PID_v1.h>

// internal libs
#include "hardware_config.h"
#include "config.h"
#include "wire_comms.h"

// init timing vars
long lastThermoRead = 0;
long lastSerialPrint = 0;

// initialize the relay timing variables
unsigned long relay1StartTime;
unsigned long relay2StartTime;

// init var for emergency stop trigger
bool E_STOP = false;

// create pbject for the LED
CRGB led;
int hue = 0;

// initialize the Thermocouples
MAX6675 thermocouple1(TC_CLK, TC_CS1, TC_DATA);
MAX6675 thermocouple2(TC_CLK, TC_CS2, TC_DATA);

// Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID relay1PID(&(RLHT.relay1Input), &(RLHT.rOnTime_1), &(RLHT.heatSetpoint_1), RLHT.Kp_1, RLHT.Ki_1, RLHT.Kd_1, DIRECT);
PID relay2PID(&(RLHT.relay2Input), &(RLHT.rOnTime_2), &(RLHT.heatSetpoint_2), RLHT.Kp_2, RLHT.Ki_2, RLHT.Kd_2, DIRECT);

enum Mode
{
  CONTROL,
  WRITE
} currentMode = CONTROL;

void setup()
{
  // initialize serial communication
  Serial.begin(115200);

  // initialize the LED
  FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);

  // initialize the estop
  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estop, CHANGE);

  // initialize the relay pins
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  // tell the PID to range between 0 and the full window size
  relay1PID.SetOutputLimits(0, RLHT.rPeriod_1);
  relay2PID.SetOutputLimits(0, RLHT.rPeriod_2);

  // turn the PID on
  relay1PID.SetMode(AUTOMATIC);
  relay2PID.SetMode(AUTOMATIC);

  // initialize the relay start times
  relay1StartTime = millis();
  relay2StartTime = millis();

  // small delay before entering loop
  delay(1000);
}

void loop()
{

  // read the thermocouples
  measureThermocouples();

  // handle serial commands
  serialCommandHandler();

  // print output to serial
  printSerialOutput();

  if (!E_STOP && currentMode == CONTROL)
  {
    setPIDTunings();
    // assign thermocouple readings to relay inputs
    switch (RLHT.thermoSelect[0])
    {
    case 1: // thermocouple 1 to relay 1
      RLHT.relay1Input = RLHT.thermo1;
      break;
    case 2: // thermocouple 2 to relay 1
      RLHT.relay1Input = RLHT.thermo2;
      break;
    }
    if (isnan(RLHT.relay1Input))
      RLHT.rOnTime_1 = 0;
    else
      relay1PID.Compute();

    switch (RLHT.thermoSelect[1])
    {
    case 1: // thermocouple 1 to relay 2
      RLHT.relay2Input = RLHT.thermo1;
      break;
    case 2: // thermocouple 2 to relay 2
      RLHT.relay2Input = RLHT.thermo2;
      break;
    }
    if (isnan(RLHT.relay2Input))
      RLHT.rOnTime_2 = 0;
    else
      relay2PID.Compute();

    actuateRelays();
  }
  else if (!E_STOP && currentMode == WRITE)
  {
    actuateRelays();
  }
  else
  {
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);
    Serial.println(F("ERROR, UNKNOWN STATE!"));
  }
}

void estop()
{
  if (digitalRead(ESTOP) == HIGH) // when set to HIGH state
  {
    led = CRGB::Red;
    FastLED.show();

    // save states
    RLHT_old.heatSetpoint_1 = RLHT.heatSetpoint_1;
    RLHT_old.heatSetpoint_2 = RLHT.heatSetpoint_2;
    RLHT_old.rOnTime_1 = RLHT.rOnTime_1;
    RLHT_old.rOnTime_2 = RLHT.rOnTime_2;

    // set critical states to zero and turn off relays
    RLHT.heatSetpoint_1 = 0;
    RLHT.heatSetpoint_2 = 0;
    RLHT.rOnTime_1 = 0;
    RLHT.rOnTime_2 = 0;

    // turn off relays
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    // set the ESTOP flag
    E_STOP = true;
    Serial.println("ESTOP PRESSED!");
  }
  else // when ESTOP state is LOW i.e. normal operation
  {
    led = CRGB::Black;
    FastLED.show();

    // reassign old states
    RLHT_old.heatSetpoint_1 = RLHT_old.heatSetpoint_1;
    RLHT_old.heatSetpoint_2 = RLHT_old.heatSetpoint_2;
    RLHT_old.rOnTime_1 = RLHT_old.rOnTime_1;
    RLHT_old.rOnTime_2 = RLHT_old.rOnTime_2;

    // clear the ESTOP flag
    E_STOP = false;
    Serial.println("ESTOP RELEASED!");
  }
}

void measureThermocouples()
{
  if (millis() - lastThermoRead >= THERMO_UPDATE_TIME_MS)
  {
    RLHT.thermo1 = thermocouple1.readCelsius();
    RLHT.thermo2 = thermocouple2.readCelsius();
    lastThermoRead = millis();
  }
}

void actuateRelays()
{
  if (RLHT.heatSetpoint_1 == 0 && currentMode == CONTROL)
    RLHT.rOnTime_1 = 0;
  if (RLHT.heatSetpoint_2 == 0 && currentMode == CONTROL)
    RLHT.rOnTime_2 = 0;
  // Relay 1
  if (millis() - relay1StartTime > RLHT.rPeriod_1)
  { // time to shift the Relay Window
    relay1StartTime += RLHT.rPeriod_1;
  }
  if ((int)(RLHT.rOnTime_1) > millis() - relay1StartTime)
    digitalWrite(RELAY1, HIGH);
  else
    digitalWrite(RELAY1, LOW);

  // Relay 2
  if (millis() - relay2StartTime > RLHT.rPeriod_2)
  { // time to shift the Relay Window
    relay2StartTime += RLHT.rPeriod_2;
  }
  if ((int)(RLHT.rOnTime_2) > millis() - relay2StartTime)
    digitalWrite(RELAY2, HIGH);
  else
    digitalWrite(RELAY2, LOW);
}

void printSerialOutput()
{
  if (millis() - lastSerialPrint >= SERIAL_UPDATE_TIME_MS)
  {
    Serial.print(F("Mode: "));
    Serial.print(currentMode == CONTROL ? "CONTROL" : "WRITE");
    Serial.print(F(", T1:"));
    Serial.print(RLHT.thermo1);
    Serial.print(F(", T2:"));
    Serial.print(RLHT.thermo2);

    Serial.print(F(", PID1:"));
    Serial.print(RLHT.Kp_1);
    Serial.print(F(","));
    Serial.print(RLHT.Ki_1);
    Serial.print(F(","));
    Serial.print(RLHT.Kd_1);

    Serial.print(F(", Relay1Input:"));
    Serial.print(RLHT.relay1Input);
    Serial.print(F(", Setpoint1:"));
    Serial.print(RLHT.heatSetpoint_1);
    Serial.print(F(",onTime1:"));
    Serial.print((int)RLHT.rOnTime_1);

    Serial.print(F(", PID2:"));
    Serial.print(RLHT.Kp_2);
    Serial.print(F(","));
    Serial.print(RLHT.Ki_2);
    Serial.print(F(","));
    Serial.print(RLHT.Kd_2);

    Serial.print(F(", Relay2Input:"));
    Serial.print(RLHT.relay2Input);
    Serial.print(F(", Setpoint2:"));
    Serial.print(RLHT.heatSetpoint_2);
    Serial.print(F(", onTime2:"));
    Serial.println((int)RLHT.rOnTime_2);

    lastSerialPrint = millis();
  }
}

void serialCommandHandler()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n'); // Read serial input until newline
    command.trim();                                // Remove whitespace

    Serial.println("Received command: " + command);

    char cmdType = command.charAt(0); // First character determines command type

    if (cmdType == 'M') // Mode
    {
      int mode = command.charAt(2) - '0'; // 0 = CONTROL, 1 = WRITE

      if (mode == 0)
      {
        currentMode = CONTROL;
        relay1PID.SetMode(AUTOMATIC); // Turn on PID
        relay2PID.SetMode(AUTOMATIC);
        Serial.println(F("Switched to CONTROL mode"));
      }
      else if (mode == 1)
      {
        currentMode = WRITE;
        relay1PID.SetMode(WRITE); // Turn off PID
        relay2PID.SetMode(WRITE);
        Serial.println(F("Switched to WRITE mode"));
      }
    }
    else if (cmdType == 'W') // Write relay input
    {
      if (currentMode == WRITE)
      {
        int relay = command.charAt(2) - '0';           // 1 or 2
        double input = command.substring(4).toFloat(); // Extract relay input

        // limit it from 0 to 100
        input = (input < 0) ? 0 : (input > 100) ? 100
                                                : input;
        // cast the input to an integer
        int newOnTime = (int)input;

        if (relay == 1)
        {
          // scale from 0 to 100 to 0 to rPeriod
          newOnTime = map(newOnTime, 0, 100, 0, RLHT.rPeriod_1);
          RLHT.rOnTime_1 = newOnTime;
          Serial.print(F("Relay 1 input updated to: "));
        }
        else if (relay == 2)
        {
          // scale from 0 to 100 to 0 to rPeriod
          newOnTime = map(newOnTime, 0, 100, 0, RLHT.rPeriod_2);
          RLHT.rOnTime_2 = newOnTime;
          Serial.print(F("Relay 2 input updated to: "));
        }
        Serial.println(input);
      }
      else
      {
        Serial.println(F("ERROR: Not in WRITE mode!"));
      }
    }
    else if (cmdType == 'H') // Heater Setpoint
    {
      int heater = command.charAt(2) - '0';             // Heater number (1 or 2)
      double setpoint = command.substring(4).toFloat(); // Extract temperature setpoint

      if (heater == 1)
      {
        RLHT.heatSetpoint_1 = setpoint;
        Serial.print(F("Heater 1 setpoint updated to: "));
      }
      else if (heater == 2)
      {
        RLHT.heatSetpoint_2 = setpoint;
        Serial.print(F("Heater 2 setpoint updated to: "));
      }
      Serial.println(setpoint);
    }
    else if (cmdType == 'P') // PID Tuning
    {
      int heater = command.charAt(2) - '0';
      int firstComma = command.indexOf(',', 4);
      int secondComma = command.indexOf(',', firstComma + 1);
      int thirdComma = command.indexOf(',', secondComma + 1);

      double Kp = command.substring(4, firstComma).toFloat();
      double Ki = command.substring(firstComma + 1, secondComma).toFloat();
      double Kd = command.substring(secondComma + 1, thirdComma).toFloat();

      if (heater == 1)
      {
        RLHT.Kp_1 = Kp;
        RLHT.Ki_1 = Ki;
        RLHT.Kd_1 = Kd;
        relay1PID.SetTunings(Kp, Ki, Kd);
        Serial.print("PID Heater 1 updated: ");
      }
      else if (heater == 2)
      {
        RLHT.Kp_2 = Kp;
        RLHT.Ki_2 = Ki;
        RLHT.Kd_2 = Kd;
        relay2PID.SetTunings(Kp, Ki, Kd);
        Serial.print("PID Heater 2 updated: ");
      }
      Serial.print("Kp = ");
      Serial.print(Kp);
      Serial.print(", Ki = ");
      Serial.print(Ki);
      Serial.print(", Kd = ");
      Serial.println(Kd);
    }
    else if (cmdType == 'S') // Thermo select
    {
      int relay = command.charAt(2) - '0';
      char thermo = command.charAt(4);

      if (relay == 1)
      {
        RLHT.thermoSelect[0] = thermo;
        Serial.print("ThermoSelect 1 updated: ");
      }
      else if (relay == 2)
      {
        RLHT.thermoSelect[1] = thermo;
        Serial.print("ThermoSelect 2 updated: ");
      }
      Serial.println(thermo);
    }
    else
    {
      Serial.println("Invalid command!");
    }
  }
}

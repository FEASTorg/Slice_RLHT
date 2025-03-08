/**
 * @file RLHT.ino
 * @brief RLHT firmware for the BREAD system.
 * @author Cameron K. Brooks 2025
 *
 */

// RLHT specific libraries
#include <PID_v1.h>
#include "max6675.h"

// shared BREAD libs
#include <FastLED.h>

// BREAD specific libs
#include <CRUMBS.h>

// internal libs
#include "hardware_config.h"
#include "config.h"

/**
 * @def SLICE_PRINT
 * @brief Uncomment to enable print messages.
 */

#define SLICE_PRINT 1

#ifdef SLICE_PRINT

#define SLICE_PRINT(...) \
  Serial.print(__VA_ARGS__)

#define SLICE_PRINTLN(...) \
  Serial.println(__VA_ARGS__)

#else
#define SLICE_PRINT(...)
#define SLICE_PRINTLN(...)
#endif

/**
 * @def SLICE_DEBUG
 * @brief Uncomment to enable additional debugging messages.
 */

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

// CRUMBS start
CRUMBS crumbsSlice(false, I2C_ADR); // Slave mode, I2C address 0x08

// flag for emergency stop
volatile bool estopTriggered = false;

// create object for the LED
CRGB led;

struct RelayHeater
{
  double setpointTemperature = 0; // set desired temperature for heater 1 (if using relay as heater actuator)
  int thermocoupleSelect = 0;     // select which thermocouple to use for the relay
  double inputTemperature = 0;    // input to relay 1 taken from selected thermocouple
  double relayOnTime = 0;         // time relay is on in ms (0-period)
  int relayPeriod = 1000;         // duration of relay 1 cycle in ms (1000ms = 1s)
  double Kp = 1;                  // PID proportional gain for heater 1
  double Ki = 0;                  // PID integral gain for heater 1
  double Kd = 0;                  // PID derivative gain for heater 1
};

struct RLHT_SLICE
{
  double temperature1;      // thermocouple 1 measurement
  double temperature2;      // thermocouple 2 measurement
  RelayHeater relayHeater1; // relay heater 1
  RelayHeater relayHeater2; // relay heater 2
  bool eStop = false;       // emergency stop flag
} slice;

struct Timing
{
  long lastThermoRead;
  long lastSerialPrint;
  unsigned long relay1Start;
  unsigned long relay2Start;
} timing = {0};

enum Mode
{
  CONTROL,
  WRITE
} currentMode = CONTROL;

// initialize the Thermocouples
MAX6675 thermocouple1(TC_CLK, TC_CS1, TC_DATA);
MAX6675 thermocouple2(TC_CLK, TC_CS2, TC_DATA);

// Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID relay1PID(&(slice.relayHeater1.inputTemperature), &(slice.relayHeater1.relayOnTime), &(slice.relayHeater1.setpointTemperature), slice.relayHeater1.Kp, slice.relayHeater1.Ki, slice.relayHeater1.Kd, DIRECT);
PID relay2PID(&(slice.relayHeater2.inputTemperature), &(slice.relayHeater2.relayOnTime), &(slice.relayHeater2.setpointTemperature), slice.relayHeater2.Kp, slice.relayHeater2.Ki, slice.relayHeater2.Kd, DIRECT);

void setup()
{
  // the general slice setup
  setupSlice();

  // the RLHT specific setup
  setupRLHT();

  // small delay before entering loop
  delay(1000);
}

void setupSlice()
{
  // initialize serial communication
  Serial.begin(115200);

  // Initialize CRUMBS communication
  crumbsSlice.begin();

  crumbsSlice.onRequest(handleRequest); // Register callback for data requests
  crumbsSlice.onReceive(handleMessage); // Register callback for received messages

  // initialize the LED
  FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);

  // initialize the estop
  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estopISR, CHANGE);
}

void setupRLHT()
{
  // initialize the relay pins
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  // tell the PID to range between 0 and the full window size
  relay1PID.SetOutputLimits(0, slice.relayHeater1.relayPeriod);
  relay2PID.SetOutputLimits(0, slice.relayHeater2.relayPeriod);

  // turn the PID on
  relay1PID.SetMode(AUTOMATIC);
  relay2PID.SetMode(AUTOMATIC);

  // initialize the relay start times
  timing.relay1Start = millis();
  timing.relay2Start = millis();
}

void loop()
{

  // poll the emergency stop
  pollEStop();

  // read the thermocouples
  measureThermocouples();

  // main control logic
  relayControlLogic();

  // print output to serial
  printSerialOutput();
}

void pollEStop()
{
  if (estopTriggered)
  {
    // Update LED to indicate emergency stop
    led = CRGB::Red;
    FastLED.show();
    processEStop();
    estopTriggered = false; // Reset flag after handling
  }
  else
  {
    // Update LED to indicate normal operation
    led = CRGB::Green;
    FastLED.show();
  }
}

void estopISR()
{
  estopTriggered = true;
}

void processEStop()
{
  // Read the current state of the estop pin if needed for debouncing/validation
  if (digitalRead(ESTOP) == HIGH)
  {
    // Disable PID and turn off relays safely
    slice.relayHeater1.setpointTemperature = 0;
    slice.relayHeater2.setpointTemperature = 0;
    slice.relayHeater1.relayOnTime = 0;
    slice.relayHeater2.relayOnTime = 0;
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    // Set the emergency stop flag in the system
    slice.eStop = true;
    SLICE_DEBUG_PRINTLN("ESTOP PRESSED!");
  }
  else
  {
    // If the button/switch is released, restore or handle recovery
    led = CRGB::Green;
    FastLED.show();

    // Optionally restore previous settings or require a manual reset
    slice.eStop = false;
    SLICE_DEBUG_PRINTLN("ESTOP RELEASED!");
  }
}

/**
 * @brief Callback function to handle received CRUMBSMessages from the Master.
 *
 * @param message The received CRUMBSMessage.
 */
void handleMessage(CRUMBSMessage &message)
{
  // Debug print the received message
  SLICE_DEBUG_PRINTLN(F("Slice: Received Message:"));
  SLICE_DEBUG_PRINT(F("typeID: "));
  SLICE_DEBUG_PRINTLN(message.typeID);
  SLICE_DEBUG_PRINT(F("commandType: "));
  SLICE_DEBUG_PRINTLN(message.commandType);
  SLICE_DEBUG_PRINT(F("data: "));
  for (int i = 0; i < 6; i++)
  {
    SLICE_DEBUG_PRINT(message.data[i]);
    SLICE_DEBUG_PRINT(F(" "));
  }
  SLICE_DEBUG_PRINTLN();
  SLICE_DEBUG_PRINT(F("errorFlags: "));
  SLICE_DEBUG_PRINTLN(message.errorFlags);

  // Check if typeID matches the expected value
  if (message.typeID != TYPE_ID)
  {
    SLICE_DEBUG_PRINTLN(F("Slice: Type ID mismatch."));
    return;
  }

  // Process the message based on commandType
  switch (message.commandType)
  {
  case 0:
    // CommandType 0: Data Request Format Command
    SLICE_DEBUG_PRINTLN(F("Slice: Data Request Format Command Received."));

    // implement

    break;

  case 1:
    // CommandType 1: Change Mode Command
    currentMode = (Mode)message.data[0]; // Cast data to Mode enum where 0 = CONTROL, 1 = WRITE
    SLICE_DEBUG_PRINT(F("Slice: Mode Change Command Received. Mode: "));
    SLICE_DEBUG_PRINTLN(currentMode == CONTROL ? F("CONTROL") : F("WRITE"));
    break;

  case 2:
    // CommandType 2: Change Setpoint Command
    slice.relayHeater1.setpointTemperature = message.data[0]; // Set desired temperature for heater 1
    slice.relayHeater2.setpointTemperature = message.data[1]; // Set desired temperature for heater 2
    SLICE_DEBUG_PRINT(F("Slice: Setpoint Change Command Received. Setpoint1: "));
    SLICE_DEBUG_PRINT(slice.relayHeater1.setpointTemperature);
    SLICE_DEBUG_PRINT(F(", Setpoint2: "));
    SLICE_DEBUG_PRINTLN(slice.relayHeater2.setpointTemperature);
    break;

  case 3:
    // CommandType 3: Change PID Tuning Command
    slice.relayHeater1.Kp = message.data[0]; // Set PID proportional gain for heater 1
    slice.relayHeater1.Ki = message.data[1]; // Set PID integral gain for heater 1
    slice.relayHeater1.Kd = message.data[2]; // Set PID derivative gain for heater 1
    slice.relayHeater2.Kp = message.data[3]; // Set PID proportional gain for heater 2
    slice.relayHeater2.Ki = message.data[4]; // Set PID integral gain for heater 2
    slice.relayHeater2.Kd = message.data[5]; // Set PID derivative gain for heater 2
    SLICE_DEBUG_PRINT(F("Slice: PID Tuning Change Command Received. Kp1: "));
    SLICE_DEBUG_PRINT(slice.relayHeater1.Kp);
    SLICE_DEBUG_PRINT(F(", Ki1: "));
    SLICE_DEBUG_PRINT(slice.relayHeater1.Ki);
    SLICE_DEBUG_PRINT(F(", Kd1: "));
    SLICE_DEBUG_PRINT(slice.relayHeater1.Kd);
    SLICE_DEBUG_PRINT(F(", Kp2: "));
    SLICE_DEBUG_PRINT(slice.relayHeater2.Kp);
    SLICE_DEBUG_PRINT(F(", Ki2: "));
    SLICE_DEBUG_PRINT(slice.relayHeater2.Ki);
    SLICE_DEBUG_PRINT(F(", Kd2: "));
    SLICE_DEBUG_PRINTLN(slice.relayHeater2.Kd);
    break;

  case 4:
    // CommandType 4: Change Relay Period Command
    slice.relayHeater1.relayPeriod = message.data[0]; // Set duration of relay 1 cycle in ms
    slice.relayHeater2.relayPeriod = message.data[1]; // Set duration of relay 2 cycle in ms
    SLICE_DEBUG_PRINT(F("Slice: Relay Period Change Command Received. Period1: "));
    SLICE_DEBUG_PRINT(slice.relayHeater1.relayPeriod);
    SLICE_DEBUG_PRINT(F(", Period2: "));
    SLICE_DEBUG_PRINTLN(slice.relayHeater2.relayPeriod);
    break;

  case 5:
    // CommandType 5: Change Thermocouple Select Command
    slice.relayHeater1.thermocoupleSelect = message.data[0]; // Select which thermocouple pairs with relay 1
    slice.relayHeater2.thermocoupleSelect = message.data[1]; // Select which thermocouple pairs with relay 2
    SLICE_DEBUG_PRINT(F("Slice: Thermocouple Select Command Received. ThermoSelect1: "));
    SLICE_DEBUG_PRINT(slice.relayHeater1.thermocoupleSelect);
    SLICE_DEBUG_PRINT(F(", ThermoSelect2: "));
    SLICE_DEBUG_PRINTLN(slice.relayHeater2.thermocoupleSelect);
    break;

  case 6:
    // CommandType 6: Write to the relays directly (write mode only)
    if (currentMode == WRITE)
    {
      // assume data0 is relay 1 and data1 is relay 2 and both are 0-100% duty cycle
      double input1 = message.data[0]; // Extract relay input
      double input2 = message.data[1]; // Extract relay input

      // limit it from 0 to 100
      input1 = (input1 < 0) ? 0 : (input1 > 100) ? 100
                                                 : input1;
      input2 = (input2 < 0) ? 0 : (input2 > 100) ? 100
                                                 : input2;
      // cast the input to an integer
      int newOnTime1 = (int)input1;
      int newOnTime2 = (int)input2;

      // scale from 0 to 100 to 0 to rPeriod
      newOnTime1 = map(newOnTime1, 0, 100, 0, slice.relayHeater1.relayPeriod);
      newOnTime2 = map(newOnTime2, 0, 100, 0, slice.relayHeater2.relayPeriod);
      slice.relayHeater1.relayOnTime = newOnTime1;
      slice.relayHeater2.relayOnTime = newOnTime2;
      SLICE_DEBUG_PRINT(F("Slice: Relay 1 input updated to: "));
      SLICE_DEBUG_PRINT(input1);
      SLICE_DEBUG_PRINT(F(", Relay 2 input updated to: "));
      SLICE_DEBUG_PRINTLN(input2);
    }
    else
    {
      SLICE_DEBUG_PRINTLN(F("Slice: ERROR: Not in WRITE mode!"));
    }
    break;

  default:
    SLICE_DEBUG_PRINTLN(F("Slice: Unknown Command Type."));
    break;
  }

  SLICE_DEBUG_PRINTLN(F("Slice: Message processing complete."));
}

/**
 * @brief Callback function to handle data requests from the Master.
 *
 * @note This function sends a CRUMBSMessage back to the Master in response to a request.
 */
void handleRequest()
{
  SLICE_DEBUG_PRINTLN(F("Slice: Master requested data, sending response..."));

  // Prepare response message
  CRUMBSMessage responseMessage;
  responseMessage.typeID = 1;      /**< SLICE type ID */
  responseMessage.commandType = 0; /**< CommandType 0 for status response */

  // Populate data fields with example data
  responseMessage.data[0] = slice.temperature1;                     /**< Thermocouple 1 temperature */
  responseMessage.data[1] = slice.temperature2;                     /**< Thermocouple 2 temperature */
  responseMessage.data[2] = slice.relayHeater1.setpointTemperature; /**< Heater 1 setpoint */
  responseMessage.data[3] = slice.relayHeater2.setpointTemperature; /**< Heater 2 setpoint */
  responseMessage.data[4] = slice.relayHeater1.relayOnTime;         /**< Relay 1 on time */
  responseMessage.data[5] = slice.relayHeater2.relayOnTime;         /**< Relay 2 on time */

  responseMessage.errorFlags = 0; /**< No errors */

  uint8_t buffer[CRUMBS_MESSAGE_SIZE];
  size_t encodedSize = crumbsSlice.encodeMessage(responseMessage, buffer, sizeof(buffer));

  if (encodedSize == 0)
  {
    SLICE_DEBUG_PRINTLN(F("Slice: Failed to encode response message."));
    return;
  }

  // Send the encoded message back to the Master
  Wire.write(buffer, encodedSize);
  SLICE_DEBUG_PRINTLN(F("Slice: Response message sent."));
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

  if (!slice.eStop && currentMode == CONTROL)
  {
    // ensure the PID is on
    relay1PID.SetMode(AUTOMATIC);
    relay2PID.SetMode(AUTOMATIC);

    // set the PID tunings
    relay1PID.SetTunings(slice.relayHeater1.Kp, slice.relayHeater1.Ki, slice.relayHeater1.Kd);
    relay2PID.SetTunings(slice.relayHeater2.Kp, slice.relayHeater2.Ki, slice.relayHeater2.Kd);

    // assign thermocouple readings to relay inputs
    switch (slice.relayHeater1.thermocoupleSelect)
    {
    case 1: // thermocouple 1 to relay 1
      slice.relayHeater1.inputTemperature = slice.temperature1;
      break;
    case 2: // thermocouple 2 to relay 1
      slice.relayHeater1.inputTemperature = slice.temperature2;
      break;
    }
    if (isnan(slice.relayHeater1.inputTemperature))
      slice.relayHeater1.relayOnTime = 0;
    else
      relay1PID.Compute();

    switch (slice.relayHeater2.thermocoupleSelect)
    {
    case 1: // thermocouple 1 to relay 2
      slice.relayHeater2.inputTemperature = slice.temperature1;
      break;
    case 2: // thermocouple 2 to relay 2
      slice.relayHeater2.inputTemperature = slice.temperature2;
      break;
    }
    if (isnan(slice.relayHeater2.inputTemperature))
      slice.relayHeater2.relayOnTime = 0;
    else
      relay2PID.Compute();

    // actuate the relays
    actuateRelays();
  }
  else if (!slice.eStop && currentMode == WRITE)
  {
    // ensure the PID is off
    relay1PID.SetMode(MANUAL);
    relay2PID.SetMode(MANUAL);

    // actuate the relays
    actuateRelays();
  }
  else
  {
    // turn off the relays
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);
    SLICE_DEBUG_PRINTLN(F("ERROR, UNKNOWN STATE!"));
  }
}

void actuateRelays()
{

  // turn off relays if setpoint is zero
  if (slice.relayHeater1.setpointTemperature == 0 && currentMode == CONTROL)
    slice.relayHeater1.relayOnTime = 0;
  if (slice.relayHeater2.setpointTemperature == 0 && currentMode == CONTROL)
    slice.relayHeater2.relayOnTime = 0;

  // Relay 1
  if (millis() - timing.relay1Start > slice.relayHeater1.relayPeriod)
  { // time to shift the Relay Window
    timing.relay1Start += slice.relayHeater1.relayPeriod;
  }
  if ((int)(slice.relayHeater1.relayOnTime) > millis() - timing.relay1Start)
    digitalWrite(RELAY1, HIGH);
  else
    digitalWrite(RELAY1, LOW);

  // Relay 2
  if (millis() - timing.relay2Start > slice.relayHeater2.relayPeriod)
  { // time to shift the Relay Window
    timing.relay2Start += slice.relayHeater2.relayPeriod;
  }
  if ((int)(slice.relayHeater2.relayOnTime) > millis() - timing.relay2Start)
    digitalWrite(RELAY2, HIGH);
  else
    digitalWrite(RELAY2, LOW);
}

void printSerialOutput()
{
  if (millis() - timing.lastSerialPrint >= SERIAL_UPDATE_TIME_MS)
  {
    SLICE_PRINT(F("Mode: "));
    SLICE_PRINT(currentMode == CONTROL ? "CONTROL" : "WRITE");
    SLICE_PRINT(F(", T1:"));
    SLICE_PRINT(slice.temperature1);
    SLICE_PRINT(F(", T2:"));
    SLICE_PRINT(slice.temperature2);

    SLICE_PRINT(F(", PID1:"));
    SLICE_PRINT(slice.relayHeater1.Kp);
    SLICE_PRINT(F(","));
    SLICE_PRINT(slice.relayHeater1.Ki);
    SLICE_PRINT(F(","));
    SLICE_PRINT(slice.relayHeater1.Kd);

    SLICE_PRINT(F(", Relay1Input:"));
    SLICE_PRINT(slice.relayHeater1.inputTemperature);
    SLICE_PRINT(F(", Setpoint1:"));
    SLICE_PRINT(slice.relayHeater1.setpointTemperature);
    SLICE_PRINT(F(",onTime1:"));
    SLICE_PRINT((int)slice.relayHeater1.relayOnTime);
    SLICE_PRINT(F(", rPeriod1:"));
    SLICE_PRINT(slice.relayHeater1.relayPeriod);

    SLICE_PRINT(F(", PID2:"));
    SLICE_PRINT(slice.relayHeater2.Kp);
    SLICE_PRINT(F(","));
    SLICE_PRINT(slice.relayHeater2.Ki);
    SLICE_PRINT(F(","));
    SLICE_PRINT(slice.relayHeater2.Kd);

    SLICE_PRINT(F(", Relay2Input:"));
    SLICE_PRINT(slice.relayHeater2.inputTemperature);
    SLICE_PRINT(F(", Setpoint2:"));
    SLICE_PRINT(slice.relayHeater2.setpointTemperature);
    SLICE_PRINT(F(", onTime2:"));
    SLICE_PRINT((int)slice.relayHeater2.relayOnTime);
    SLICE_PRINT(F(", rPeriod2:"));
    SLICE_PRINT(slice.relayHeater2.relayPeriod);
    SLICE_PRINT(F(", Thermo Select Relay 1:"));
    SLICE_PRINT(slice.relayHeater1.thermocoupleSelect);
    SLICE_PRINT(F(", Thermo Select Relay 2:"));
    SLICE_PRINT(slice.relayHeater2.thermocoupleSelect);
    SLICE_PRINT(F(", ESTOP:"));
    SLICE_PRINTLN(slice.eStop);

    timing.lastSerialPrint = millis();
  }
}

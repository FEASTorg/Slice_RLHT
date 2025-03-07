#include <Wire.h>
#include "max6675.h"
#include <FastLED.h>
#include <PID_v1.h>

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

struct RelayHeaters
{
  double heatSetpoint_1 = 0; // set desired temperature for heater 1 (if using relay as heater actuator)
  double relay1Input;        // input to relay 1 taken from selected thermocouple
  double rOnTime_1;          // time relay is on in ms (0-period)
  int rPeriod_1 = 2000;      // duration of relay 1 cycle in ms (1000ms = 1s)
  double Kp_1 = 0;           // PID proportional gain for heater 1
  double Ki_1 = 0;           // PID integral gain for heater 1
  double Kd_1 = 0;           // PID derivative gain for heater 1
  double heatSetpoint_2 = 0;
  double relay2Input;
  double rOnTime_2;
  int rPeriod_2 = 2000;
  double Kp_2 = 0;
  double Ki_2 = 0;
  double Kd_2 = 0;
  double thermo1; // thermocouple 1 measurement
  double thermo2;
  char thermoSelect[2] = {1, 2}; // select which thermocouples pair with relays {relay1, relay2}
  bool eStop = false;            // emergency stop flag
};

RelayHeaters RLHT_curr, RLHT_prev;

struct Timing
{
  long lastThermoRead;
  long lastSerialPrint;
  unsigned long relay1Start;
  unsigned long relay2Start;
} timing = {0};

volatile bool estopTriggered = false;

// create object for the LED
CRGB led;

// initialize the Thermocouples
MAX6675 thermocouple1(TC_CLK, TC_CS1, TC_DATA);
MAX6675 thermocouple2(TC_CLK, TC_CS2, TC_DATA);

// Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID relay1PID(&(RLHT_curr.relay1Input), &(RLHT_curr.rOnTime_1), &(RLHT_curr.heatSetpoint_1), RLHT_curr.Kp_1, RLHT_curr.Ki_1, RLHT_curr.Kd_1, DIRECT);
PID relay2PID(&(RLHT_curr.relay2Input), &(RLHT_curr.rOnTime_2), &(RLHT_curr.heatSetpoint_2), RLHT_curr.Kp_2, RLHT_curr.Ki_2, RLHT_curr.Kd_2, DIRECT);

enum Mode
{
  CONTROL,
  WRITE
} currentMode = CONTROL;

// CRUMBS start
CRUMBS crumbsSlice(false, I2C_ADR); // Slave mode, I2C address 0x08

void setup()
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

  // initialize the relay pins
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  // tell the PID to range between 0 and the full window size
  relay1PID.SetOutputLimits(0, RLHT_curr.rPeriod_1);
  relay2PID.SetOutputLimits(0, RLHT_curr.rPeriod_2);

  // turn the PID on
  relay1PID.SetMode(AUTOMATIC);
  relay2PID.SetMode(AUTOMATIC);

  // initialize the relay start times
  timing.relay1Start = millis();
  timing.relay2Start = millis();

  // small delay before entering loop
  delay(1000);
}

void loop()
{

  // Poll the estop flag and process if triggered.
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

  // read the thermocouples
  measureThermocouples();

  // handle serial commands
  // handleSerialInput();

  // print output to serial
  printSerialOutput();

  if (!RLHT_curr.eStop && currentMode == CONTROL)
  {
    // ensure the PID is on
    relay1PID.SetMode(AUTOMATIC);
    relay2PID.SetMode(AUTOMATIC);

    // set the PID tunings
    relay1PID.SetTunings(RLHT_curr.Kp_1, RLHT_curr.Ki_1, RLHT_curr.Kd_1);
    relay2PID.SetTunings(RLHT_curr.Kp_2, RLHT_curr.Ki_2, RLHT_curr.Kd_2);

    // assign thermocouple readings to relay inputs
    switch (RLHT_curr.thermoSelect[0])
    {
    case 1: // thermocouple 1 to relay 1
      RLHT_curr.relay1Input = RLHT_curr.thermo1;
      break;
    case 2: // thermocouple 2 to relay 1
      RLHT_curr.relay1Input = RLHT_curr.thermo2;
      break;
    }
    if (isnan(RLHT_curr.relay1Input))
      RLHT_curr.rOnTime_1 = 0;
    else
      relay1PID.Compute();

    switch (RLHT_curr.thermoSelect[1])
    {
    case 1: // thermocouple 1 to relay 2
      RLHT_curr.relay2Input = RLHT_curr.thermo1;
      break;
    case 2: // thermocouple 2 to relay 2
      RLHT_curr.relay2Input = RLHT_curr.thermo2;
      break;
    }
    if (isnan(RLHT_curr.relay2Input))
      RLHT_curr.rOnTime_2 = 0;
    else
      relay2PID.Compute();

    // actuate the relays
    actuateRelays();
  }
  else if (!RLHT_curr.eStop && currentMode == WRITE)
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

void estopISR()
{
  estopTriggered = true;
}

void processEStop()
{
  // Read the current state of the estop pin if needed for debouncing/validation
  if (digitalRead(ESTOP) == HIGH)
  {

    // Save current states if necessary
    RLHT_prev.heatSetpoint_1 = RLHT_curr.heatSetpoint_1;
    RLHT_prev.heatSetpoint_2 = RLHT_curr.heatSetpoint_2;
    RLHT_prev.rOnTime_1 = RLHT_curr.rOnTime_1;
    RLHT_prev.rOnTime_2 = RLHT_curr.rOnTime_2;

    // Disable PID and turn off relays safely
    RLHT_curr.heatSetpoint_1 = 0;
    RLHT_curr.heatSetpoint_2 = 0;
    RLHT_curr.rOnTime_1 = 0;
    RLHT_curr.rOnTime_2 = 0;
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    // Set the emergency stop flag in the system
    RLHT_curr.eStop = true;
    SLICE_DEBUG_PRINTLN("ESTOP PRESSED!");
  }
  else
  {
    // If the button/switch is released, restore or handle recovery
    led = CRGB::Green;
    FastLED.show();

    // Optionally restore previous settings or require a manual reset
    RLHT_curr.eStop = false;
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
    RLHT_curr.heatSetpoint_1 = message.data[0]; // Set desired temperature for heater 1
    RLHT_curr.heatSetpoint_2 = message.data[1]; // Set desired temperature for heater 2
    SLICE_DEBUG_PRINT(F("Slice: Setpoint Change Command Received. Setpoint1: "));
    SLICE_DEBUG_PRINT(RLHT_curr.heatSetpoint_1);
    SLICE_DEBUG_PRINT(F(", Setpoint2: "));
    SLICE_DEBUG_PRINTLN(RLHT_curr.heatSetpoint_2);
    break;

  case 3:
    // CommandType 3: Change PID Tuning Command
    RLHT_curr.Kp_1 = message.data[0]; // Set PID proportional gain for heater 1
    RLHT_curr.Ki_1 = message.data[1]; // Set PID integral gain for heater 1
    RLHT_curr.Kd_1 = message.data[2]; // Set PID derivative gain for heater 1
    RLHT_curr.Kp_2 = message.data[3]; // Set PID proportional gain for heater 2
    RLHT_curr.Ki_2 = message.data[4]; // Set PID integral gain for heater 2
    RLHT_curr.Kd_2 = message.data[5]; // Set PID derivative gain for heater 2
    SLICE_DEBUG_PRINT(F("Slice: PID Tuning Change Command Received. Kp1: "));
    SLICE_DEBUG_PRINT(RLHT_curr.Kp_1);
    SLICE_DEBUG_PRINT(F(", Ki1: "));
    SLICE_DEBUG_PRINT(RLHT_curr.Ki_1);
    SLICE_DEBUG_PRINT(F(", Kd1: "));
    SLICE_DEBUG_PRINT(RLHT_curr.Kd_1);
    SLICE_DEBUG_PRINT(F(", Kp2: "));
    SLICE_DEBUG_PRINT(RLHT_curr.Kp_2);
    SLICE_DEBUG_PRINT(F(", Ki2: "));
    SLICE_DEBUG_PRINT(RLHT_curr.Ki_2);
    SLICE_DEBUG_PRINT(F(", Kd2: "));
    SLICE_DEBUG_PRINTLN(RLHT_curr.Kd_2);
    break;

  case 4:
    // CommandType 4: Change Relay Period Command
    RLHT_curr.rPeriod_1 = message.data[0]; // Set duration of relay 1 cycle in ms
    RLHT_curr.rPeriod_2 = message.data[1]; // Set duration of relay 2 cycle in ms
    SLICE_DEBUG_PRINT(F("Slice: Relay Period Change Command Received. Period1: "));
    SLICE_DEBUG_PRINT(RLHT_curr.rPeriod_1);
    SLICE_DEBUG_PRINT(F(", Period2: "));
    SLICE_DEBUG_PRINTLN(RLHT_curr.rPeriod_2);
    break;

  case 5:
    // CommandType 5: Change Thermocouple Select Command
    RLHT_curr.thermoSelect[0] = message.data[0]; // Select which thermocouple pairs with relay 1
    RLHT_curr.thermoSelect[1] = message.data[1]; // Select which thermocouple pairs with relay 2
    SLICE_DEBUG_PRINT(F("Slice: Thermocouple Select Command Received. ThermoSelect1: "));
    SLICE_DEBUG_PRINT(RLHT_curr.thermoSelect[0]);
    SLICE_DEBUG_PRINT(F(", ThermoSelect2: "));
    SLICE_DEBUG_PRINTLN(RLHT_curr.thermoSelect[1]);
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
      newOnTime1 = map(newOnTime1, 0, 100, 0, RLHT_curr.rPeriod_1);
      newOnTime2 = map(newOnTime2, 0, 100, 0, RLHT_curr.rPeriod_2);
      RLHT_curr.rOnTime_1 = newOnTime1;
      RLHT_curr.rOnTime_2 = newOnTime2;
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
  responseMessage.data[0] = 42.0f; /**< Example data0 */
  responseMessage.data[1] = 1.0f;  /**< Example data1 */
  responseMessage.data[2] = 2.0f;  /**< Example data2 */
  responseMessage.data[3] = 3.0f;  /**< Example data3 */
  responseMessage.data[4] = 4.0f;  /**< Example data4 */
  responseMessage.data[5] = 5.0f;  /**< Example data5 */

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
    RLHT_curr.thermo1 = thermocouple1.readCelsius();
    RLHT_curr.thermo2 = thermocouple2.readCelsius();
    timing.lastThermoRead = millis();
  }
}

void actuateRelays()
{

  // turn off relays if setpoint is zero
  if (RLHT_curr.heatSetpoint_1 == 0 && currentMode == CONTROL)
    RLHT_curr.rOnTime_1 = 0;
  if (RLHT_curr.heatSetpoint_2 == 0 && currentMode == CONTROL)
    RLHT_curr.rOnTime_2 = 0;

  // Relay 1
  if (millis() - timing.relay1Start > RLHT_curr.rPeriod_1)
  { // time to shift the Relay Window
    timing.relay1Start += RLHT_curr.rPeriod_1;
  }
  if ((int)(RLHT_curr.rOnTime_1) > millis() - timing.relay1Start)
    digitalWrite(RELAY1, HIGH);
  else
    digitalWrite(RELAY1, LOW);

  // Relay 2
  if (millis() - timing.relay2Start > RLHT_curr.rPeriod_2)
  { // time to shift the Relay Window
    timing.relay2Start += RLHT_curr.rPeriod_2;
  }
  if ((int)(RLHT_curr.rOnTime_2) > millis() - timing.relay2Start)
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
    SLICE_PRINT(RLHT_curr.thermo1);
    SLICE_PRINT(F(", T2:"));
    SLICE_PRINT(RLHT_curr.thermo2);

    SLICE_PRINT(F(", PID1:"));
    SLICE_PRINT(RLHT_curr.Kp_1);
    SLICE_PRINT(F(","));
    SLICE_PRINT(RLHT_curr.Ki_1);
    SLICE_PRINT(F(","));
    SLICE_PRINT(RLHT_curr.Kd_1);

    SLICE_PRINT(F(", Relay1Input:"));
    SLICE_PRINT(RLHT_curr.relay1Input);
    SLICE_PRINT(F(", Setpoint1:"));
    SLICE_PRINT(RLHT_curr.heatSetpoint_1);
    SLICE_PRINT(F(",onTime1:"));
    SLICE_PRINT((int)RLHT_curr.rOnTime_1);
    SLICE_PRINT(F(", rPeriod1:"));
    SLICE_PRINT(RLHT_curr.rPeriod_1);

    SLICE_PRINT(F(", PID2:"));
    SLICE_PRINT(RLHT_curr.Kp_2);
    SLICE_PRINT(F(","));
    SLICE_PRINT(RLHT_curr.Ki_2);
    SLICE_PRINT(F(","));
    SLICE_PRINT(RLHT_curr.Kd_2);

    SLICE_PRINT(F(", Relay2Input:"));
    SLICE_PRINT(RLHT_curr.relay2Input);
    SLICE_PRINT(F(", Setpoint2:"));
    SLICE_PRINT(RLHT_curr.heatSetpoint_2);
    SLICE_PRINT(F(", onTime2:"));
    SLICE_PRINT((int)RLHT_curr.rOnTime_2);
    SLICE_PRINT(F(", rPeriod2:"));
    SLICE_PRINT(RLHT_curr.rPeriod_2);
    SLICE_PRINT(F(", Thermo Select Relay 1:"));
    SLICE_PRINT(RLHT_curr.thermoSelect[0]);
    SLICE_PRINT(F(", Thermo Select Relay 2:"));
    SLICE_PRINT(RLHT_curr.thermoSelect[1]);
    SLICE_PRINT(F(", ESTOP:"));
    SLICE_PRINTLN(RLHT_curr.eStop);

    timing.lastSerialPrint = millis();
  }
}

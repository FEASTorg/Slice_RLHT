#include <Wire.h>
#include "Adafruit_MAX31855.h"
#include <FastLED.h>
#include <PID_v1.h>

#define I2C_ADR 10

#define ESTOP   2
#define LED_PIN 5
#define TC_CS1     10
#define TC_CS2     11
#define TC_DATA    12
#define TC_CLK     13
#define RELAY1  6
#define RELAY2  7

#define THERMO_UPDATE_TIME_MS   300
#define SERIAL_UPDATE_TIME_MS   1000

long lastThermoRead = 0;
long lastSerialPrint = 0;

bool E_STOP = false;

CRGB led;
int hue = 0;

struct RLHT_t
{
  double heatSetpoint_1 = 0;   // set desired temperature for heater 1 (if using relay as heater actuator)
  double relay1Input;  // input to relay 1 taken from selected thermocouple
  double rOnTime_1;           // time relay is on in ms (0-period)
  int rPeriod_1 = 2000;          // duration of relay 1 cycle in ms (1000ms = 1s)
  double Kp_1 = 0;             // PID proportional gain for heater 1
  double Ki_1 = 0;             // PID integral gain for heater 1
  double Kd_1 = 0;             // PID derivative gain for heater 1 
  double heatSetpoint_2 = 0;
  double relay2Input;
  double rOnTime_2;
  int rPeriod_2 = 2000;
  double Kp_2 = 0;
  double Ki_2 = 0;
  double Kd_2 = 0;
  double thermo1;   // thermocouple 1 measurement
  double thermo2;
  char thermoSelect[2];   // select which thermocouples pair with relays {relay1, relay2}
} RLHT, RLHT_old;

union FLOATUNION_t //Define a float that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
};

// initialize the Thermocouples
Adafruit_MAX31855 CH1(TC_CLK, TC_CS1, TC_DATA);
Adafruit_MAX31855 CH2(TC_CLK, TC_CS2, TC_DATA);

//Specify the links and initial tuning parameters
PID relay1PID(&(RLHT.relay1Input), &(RLHT.rOnTime_1), &(RLHT.heatSetpoint_1), RLHT.Kp_1, RLHT.Ki_1, RLHT.Kd_1, DIRECT);
PID relay2PID(&(RLHT.relay2Input), &(RLHT.rOnTime_2), &(RLHT.heatSetpoint_2), RLHT.Kp_2, RLHT.Ki_2, RLHT.Kd_2, DIRECT);

unsigned long relay1StartTime;
unsigned long relay2StartTime;

void estop()
{
  if(digitalRead(ESTOP) == HIGH)    // when set to HIGH state
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

    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    E_STOP = true;
    Serial.println("ESTOP PRESSED!");
  }
  else    // when ESTOP state is LOW
  {
    led = CRGB::Black;
    FastLED.show();

    // reassign old states
    RLHT_old.heatSetpoint_1 = RLHT_old.heatSetpoint_1;
    RLHT_old.heatSetpoint_2 = RLHT_old.heatSetpoint_2;
    RLHT_old.rOnTime_1 = RLHT_old.rOnTime_1;
    RLHT_old.rOnTime_2 = RLHT_old.rOnTime_2;

    E_STOP = false;

    Serial.println("ESTOP RELEASED!");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);

  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estop, CHANGE);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  Wire.begin(I2C_ADR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  //tell the PID to range between 0 and the full window size
  relay1PID.SetOutputLimits(0, RLHT.rPeriod_1);
  relay2PID.SetOutputLimits(0, RLHT.rPeriod_2);

  //turn the PID on
  relay1PID.SetMode(AUTOMATIC);
  relay2PID.SetMode(AUTOMATIC);

  relay1StartTime = millis();
  relay2StartTime = millis();

  Serial.println("MAX31855  CH1");
  // wait for MAX chip to stabilize
  delay(500);
  if (!CH1.begin()) {
    Serial.println("ERROR CH1.");
    while (1) delay(10);
  }
  Serial.println("DONE CH1.");

  Serial.println("MAX31855  CH2");
  // wait for MAX chip to stabilize
  delay(500);
  if (!CH2.begin()) {
    Serial.println("ERROR CH2.");
    while (1) delay(10);
  }
  Serial.println("DONE CH2.");


  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  measureThermocouples();
  if(!E_STOP)
  {
    setPIDTunings();
    // assign thermocouple readings to relay inputs
    switch(RLHT.thermoSelect[0]){
      case 1:   // thermocouple 1 to relay 1 
        RLHT.relay1Input = RLHT.thermo1;
        break;
      case 2:   // thermocouple 2 to relay 1
        RLHT.relay1Input = RLHT.thermo2;
        break;
    }
    if(isnan(RLHT.relay1Input))
      RLHT.rOnTime_1 = 0;
    else
      relay1PID.Compute();

    switch(RLHT.thermoSelect[1]){
      case 1:   // thermocouple 1 to relay 2 
        RLHT.relay2Input = RLHT.thermo1;
        break;
      case 2:   // thermocouple 2 to relay 2
        RLHT.relay2Input = RLHT.thermo2;
        break;
    }
    if(isnan(RLHT.relay2Input))
      RLHT.rOnTime_2 = 0;
    else
      relay2PID.Compute();

    actuateRelays();
    printOutput();
  }
}

void setPIDTunings()
{
  relay1PID.SetTunings(RLHT.Kp_1, RLHT.Ki_1, RLHT.Kd_1);
  relay2PID.SetTunings(RLHT.Kp_2, RLHT.Ki_2, RLHT.Kd_2);
}

void printOutput()
{
  if(millis() - lastSerialPrint >= SERIAL_UPDATE_TIME_MS)
  {
    Serial.print("T1: ");
    Serial.print(RLHT.thermo1);
    Serial.print("\tT2: ");
    Serial.print(RLHT.thermo2);

    Serial.print("\tPID1: ");
    Serial.print(RLHT.Kp_1);
    Serial.print(",");
    Serial.print(RLHT.Ki_1);
    Serial.print(",");
    Serial.print(RLHT.Kd_1);

    Serial.print("\tRelay1Input: ");
    Serial.print(RLHT.relay1Input);
    Serial.print("\tSetpoint: ");
    Serial.print(RLHT.heatSetpoint_1);
    Serial.print("\tonTime2: ");
    Serial.print((int)(RLHT.rOnTime_1));

    Serial.print("\tPID2: ");
    Serial.print(RLHT.Kp_2);
    Serial.print(",");
    Serial.print(RLHT.Ki_2);
    Serial.print(",");
    Serial.print(RLHT.Kd_2);

    Serial.print("\tRelay2Input: ");
    Serial.print(RLHT.relay2Input);
    Serial.print("\tSetpoint: ");
    Serial.print(RLHT.heatSetpoint_2);
    Serial.print("\tonTime2: ");
    Serial.println((int)(RLHT.rOnTime_2));

    lastSerialPrint = millis();
  }
}

void measureThermocouples()
{
  if(millis() - lastThermoRead >= THERMO_UPDATE_TIME_MS)
  {
    RLHT.thermo1 = CH1.readCelsius();
   if (isnan(RLHT.thermo1)) {
     Serial.println("Thermocouple fault(s) detected!");
     uint8_t e = CH1.readError();
     if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
     if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
     if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
   } 

    RLHT.thermo2 = CH2.readCelsius();
   if (isnan(RLHT.thermo2)) {
     Serial.println("Thermocouple fault(s) detected!");
     uint8_t e = CH2.readError();
     if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
     if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
     if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
   } 

    lastThermoRead = millis();
  }
}

void actuateRelays()
{
  if(RLHT.heatSetpoint_1 == 0)
    RLHT.rOnTime_1 = 0;
  if(RLHT.heatSetpoint_2 == 0)
    RLHT.rOnTime_2 = 0;
  // Relay 1
  if (millis() - relay1StartTime > RLHT.rPeriod_1)
  { //time to shift the Relay Window
    relay1StartTime += RLHT.rPeriod_1;
  }
  if ((int)(RLHT.rOnTime_1) > millis() - relay1StartTime) digitalWrite(RELAY1, HIGH);
  else digitalWrite(RELAY1, LOW);

  // Relay 2
  if (millis() - relay2StartTime > RLHT.rPeriod_2)
  { //time to shift the Relay Window
    relay2StartTime += RLHT.rPeriod_2;
  }
  if ((int)(RLHT.rOnTime_2) > millis() - relay2StartTime) digitalWrite(RELAY2, HIGH);
  else digitalWrite(RELAY2, LOW);
}

void requestEvent()
{
  FLOATUNION_t t1;
  FLOATUNION_t t2;

  if(isnan(RLHT.thermo1))
    t1.number = 0;
  else
    t1.number = RLHT.thermo1;

  if(isnan(RLHT.thermo2))
    t2.number = 0;
  else
    t2.number = RLHT.thermo2;

  for (int i = 0; i < 4; i++) Wire.write(t1.bytes[i]);
  for (int i = 0; i < 4; i++) Wire.write(t2.bytes[i]);
}

/* Command layout
  Byte 1:     T (want temperature reading)
              H (change heater setpoint)
              P (change PID tuning)
  Byte 2:     1 (first heater/thermocouple)
              2 (second heater/thermocouple)
  Byte 3-6:   Kp or heatSetpoint
  Byte 7-10:  Ki
  Byte 11-14: Kd
*/

void receiveEvent(int howMany)
{
  byte in_char;
  char in_TC_DATA[20];

  led = CRGB::Green;
  FastLED.show();

  //Serial.println("Receiving: ");
  int i=0;
  while(Wire.available())
  {
    in_char = Wire.read();
    in_TC_DATA[i] = in_char;
    Serial.print((byte)in_TC_DATA[i]);
    Serial.print(",");
    i++;
  }
  Serial.println();

  setParametersRLHT(in_TC_DATA);

  led = CRGB::Black;
  FastLED.show();
}

/*
RLHT Command Format:
  H,1,setpoint,2,0:   temperature setpoint between relay 1 and thermo 2. Enable reverse response = 0 (false)
  P,Kp,Ki,Kd:       PID tuning
*/

void setParametersRLHT(char *in_TC_DATA)
{
  FLOATUNION_t float1;
  FLOATUNION_t float2;
  FLOATUNION_t float3;

  switch(in_TC_DATA[0])
  {
    case 'H':
      for(int i=0; i<4; i++) float1.bytes[i] = in_TC_DATA[i+2];
      if(in_TC_DATA[1] == 1){
        RLHT.heatSetpoint_1 = float1.number;
        RLHT.thermoSelect[0] = in_TC_DATA[6];
        relay1PID.SetControllerDirection(in_TC_DATA[7]);   // Direct = 0, Reverse = 1
      }
      if(in_TC_DATA[1] == 2){
        RLHT.heatSetpoint_2 = float1.number;
        RLHT.thermoSelect[1] = in_TC_DATA[6];
        relay2PID.SetControllerDirection(in_TC_DATA[7]);   // Direct = 0, Reverse = 1
      }
      break;
    case 'P':
      for(int i=0; i<4; i++)  // populate variables for PID tuning
      {
        float1.bytes[i] = in_TC_DATA[i+2];
        float2.bytes[i] = in_TC_DATA[i+6];
        float3.bytes[i] = in_TC_DATA[i+10];
      }
      if(in_TC_DATA[1] == 1)
      {
        RLHT.Kp_1 = (double)float1.number;
        RLHT.Ki_1 = (double)float2.number;
        RLHT.Kd_1 = (double)float3.number;
      }
      if(in_TC_DATA[1] == 2)
      {
        RLHT.Kp_2 = (double)float1.number;
        RLHT.Ki_2 = (double)float2.number;
        RLHT.Kd_2 = (double)float3.number;
      }
      break;
  }
}
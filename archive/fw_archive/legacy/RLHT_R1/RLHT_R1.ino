#include <Wire.h>
#include "max6675.h"

#define PWM 6

#define SS1 10 // 10
#define SS2 11 // 11
#define DATA 12
#define CLK 13

#define TMSTR2 A0
#define TMSTR1 A1

#define I2C_ADR 13 // Set this however you want

#define MAX_DUTY_CYCLE 100 // to prevent PID control from overheating coils (increase for slower heating systems)

// initialize the Thermocouples
MAX6675 CH1(CLK, SS1, DATA);
MAX6675 CH2(CLK, SS2, DATA);

typedef union // Define a float that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t RX;

byte commandSelect[3];
byte tempSelect = 0;  // select thermocouple to measure
float dutySelect = 0; // select duty cycle for heater

int onTime;
int offTime;

int period = 1000; // duty cycle period in milliseconds
bool heaterOn = false;

long curTime = 0;

long lastUpdate = 0;
long updateInterval = 300; // in milliseconds

int blinkPeriod = 1000;
long lastBlinkTime = 0;
bool blinkOn = false;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(PWM, OUTPUT);

  Wire.begin(I2C_ADR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // wait for MAX chip to stabilize
  delay(500);
  curTime = millis();
  onTime = period * dutySelect / 100;
  offTime = period - onTime;

  Serial.print("On: ");
  Serial.print(onTime);
  Serial.print(" Off: ");
  Serial.println(offTime);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (millis() - lastBlinkTime >= blinkPeriod)
  {
    lastBlinkTime = millis();
    if (blinkOn)
    {
      digitalWrite(LED_BUILTIN, LOW);
      blinkOn = false;
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
      blinkOn = true;
    }
  }

  if (dutySelect != 0)
  { // only run if duty cycle is nonzero
    onTime = period * dutySelect / 100;
    offTime = period - onTime;
    if (!heaterOn && millis() - curTime >= offTime)
    { // wait for off cycle to finish
      curTime = millis();
      digitalWrite(PWM, HIGH); // turn on
      Serial.println(" -> ON");
      heaterOn = true;
    }
    else if (heaterOn && millis() - curTime >= onTime)
    { // wait for on cycle to finish
      curTime = millis();
      digitalWrite(PWM, LOW); // turn off
      Serial.println(" -> OFF");
      heaterOn = false;
    }
  }
  else if (heaterOn)
  {
    heaterOn = false;
    digitalWrite(PWM, LOW); // turn off
  }

  if (millis() - lastUpdate >= updateInterval)
  {
    lastUpdate = millis();
    Serial.print("Temp1: ");
    Serial.print(CH1.readCelsius());
    Serial.print(" Temp2: ");
    Serial.print(CH2.readCelsius());
    if (heaterOn)
      Serial.println(" -> ON");
    else
      Serial.println(" -> OFF");
  }
}

void requestEvent()
{
  switch (commandSelect[0])
  {
  case 'H':
    Wire.write(int(dutySelect));
    break;
  case 'T':
    switch (tempSelect)
    {
    case 1:
      RX.number = CH1.readCelsius(); // reference a pointer instead
      break;
    case 2:
      RX.number = CH2.readCelsius();
      break;
    default:
      RX.number = 0;
    }
    if (!isnan(RX.number))
    {
      for (int i = 0; i <= 3; i++)
      {
        Wire.write(RX.bytes[i]);
        Serial.print(RX.bytes[i]);
        Serial.print(", ");
      }
      Serial.println("-> Float: ");
      Serial.println(RX.number);
    }
    else
    {
      for (int i = 0; i <= 3; i++)
      {
        Wire.write(0);
        Serial.print(0);
        Serial.print(", ");
      }
      Serial.println();
    }
    break;
  }

  commandSelect[1] = 0;
}

void receiveEvent(int howMany)
{
  int i = 0;
  while (Wire.available())
  {
    commandSelect[i] = Wire.read();
    Serial.print(commandSelect[i]);
    Serial.print(", ");
    i++;
  }
  Serial.println();
  switch (commandSelect[0])
  {
  case 'T':
    tempSelect = commandSelect[1]; // wants temperature readings
    break;
  case 'H':
    dutySelect = commandSelect[1]; // wants to change duty cycle of heater
    tempSelect = 0;
    break;
  }
  if (dutySelect > MAX_DUTY_CYCLE)
  {
    dutySelect = MAX_DUTY_CYCLE;
  }
}

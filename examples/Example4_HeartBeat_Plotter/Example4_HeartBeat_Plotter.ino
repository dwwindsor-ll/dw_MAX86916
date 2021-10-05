/*
  MAX86916 PPG sensor
  By: LLL @ Designworks
  Date: October 2nd, 2021
  Outputs all IR/Red/Green/Blue values.
  Hardware Connections (Breakoutboard to Arduino using HeartRate10 Click board):
  -3.3V = 3.3V
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  The MAX86916 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
  Shows the user's heart beat on Arduino's serial plotter
  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.
*/

#include <Wire.h>
#include "dw_MAX86916.h"

MAX86916 ppgSensor;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!ppgSensor.begin()) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX86916 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  int range = 0; //Options: 0=x1 to 3=x4
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = IR only, 2 = Red + IR, 3 = Red + IR + Green+ Blue
  int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 420; //Options: 70, 120, 220, 420
  int adcRange = 4096; //Options: 4096, 8192, 16384, 32768

  ppgSensor.setup(range, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //Arduino plotter auto-scales annoyingly. To get around this, pre-populate
  //the plotter with 500 of an average reading from the sensor

  //Take an average of IR readings at power up
  const byte avgAmount = 64;
  long baseValue = 0;
  for (byte x = 0 ; x < avgAmount ; x++)
  {
    baseValue += ppgSensor.getIR(); //Read the IR value
  }
  baseValue /= avgAmount;

  //Pre-populate the plotter so that the Y scale is close to IR values
  for (int x = 0 ; x < 500 ; x++)
    Serial.println(baseValue);
}

void loop()
{
  Serial.println(ppgSensor.getIR()); //Send raw data to plotter
}
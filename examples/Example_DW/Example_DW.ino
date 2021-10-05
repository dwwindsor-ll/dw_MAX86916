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
  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "dw_MAX86916.h"
#include "heartRate.h"

MAX86916 ppgSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

int16_t IR_AC_Current = 0;
int16_t IR_Average;
int32_t ir_reg = 0;

int16_t R_AC_Current = 0;
int16_t R_Average;
int32_t r_reg = 0;

int16_t G_AC_Current = 0;
int16_t G_Average;
int32_t g_reg = 0;

int16_t B_AC_Current = 0;
int16_t B_Average;
int32_t b_reg = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("MAX86916 Basic Readings Example");

  // Initialize sensor
  if (ppgSensor.begin() == false)
  {
    Serial.println("MAX86916 was not found. Please check wiring/power. ");
    while (1);
  }
  byte range = 1;
  byte powerLevel = 0x1F;
  byte sampleAverage = 8;
  byte ledMode = 3;
  int sampleRate = 1600;
  int pulseWidth = 70;
  int adcRange = 16384;
  ppgSensor.setup(range, powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor. Use 6.4mA for LED drive
}

void loop()
{
  int val0 = ppgSensor.getIR();
  int val1 = ppgSensor.getRed();
  int val2 = ppgSensor.getGreen();
  int val3 = ppgSensor.getBlue();

  //check for heartrate BPM
  if (checkForBeat(val0) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  int val4 = beatAvg;

  int val5 = IR_AC_Current;
  IR_Average = averageDCEstimator(&ir_reg, val0);
  IR_AC_Current = lowPassFIRFilter(val0 - IR_Average);

  int val6 = R_AC_Current;
  R_Average = averageDCEstimator(&r_reg, val1);
  R_AC_Current = lowPassFIRFilter(val1 - R_Average);

  int val7 = G_AC_Current;
  G_Average = averageDCEstimator(&g_reg, val2);
  G_AC_Current = lowPassFIRFilter(val2 - G_Average);

  int val8 = B_AC_Current;
  B_Average = averageDCEstimator(&b_reg, val3);
  B_AC_Current = lowPassFIRFilter(val3 - B_Average);

  Serial.print(" IR:");
  Serial.print(val0);
  Serial.print(" R:");
  Serial.print(val1);
  Serial.print(" G:");
  Serial.print(val2);
  Serial.print(" B:");
  Serial.print(val3);
  Serial.print(" BPM:");
  Serial.print(val4);
  Serial.print(" FIR:");
  Serial.print(val5);
  Serial.print(" FR:");
  Serial.print(val6);
  Serial.print(" FG:");
  Serial.print(val7);
  Serial.print(" FB:");
  Serial.print(val8);
  Serial.println();
}


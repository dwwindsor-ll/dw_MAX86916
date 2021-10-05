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

MAX86916 ppgSensor;

#define debug Serial

void setup()
{
  debug.begin(115200);
  debug.println("MAX86916 Basic Readings Example");

  // Initialize sensor
  if (ppgSensor.begin() == false)
  {
    debug.println("MAX86916 was not found. Please check wiring/power. ");
    while (1);
  }

  ppgSensor.setup(); //Configure sensor. Use 6.4mA for LED drive
}

void loop()
{
  debug.print("IR:");
  debug.print(ppgSensor.getIR());
  debug.print(" R:");
  debug.print(ppgSensor.getRed());
  debug.print(" G:");
  debug.print(ppgSensor.getGreen());
  debug.print(" B:");
  debug.print(ppgSensor.getBlue());

  debug.println();
}
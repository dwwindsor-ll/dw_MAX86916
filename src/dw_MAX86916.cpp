/****************************************************************************
**Glue logic for Heart Rate 10 Click board from Mikroe
**By LLL
*****************************************************************************/



#include <Arduino.h>
#include <Wire.h>

#include "dw_MAX86916.h"

// Status Registers
static const uint8_t MAX86916_INTSTAT1 =		0x00;
static const uint8_t MAX86916_INTENABLE1 =		0x02;

// FIFO Registers
static const uint8_t MAX86916_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX86916_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX86916_FIFOREADPTR = 	0x06;
static const uint8_t MAX86916_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX86916_FIFOCONFIG = 		0x08;
static const uint8_t MAX86916_MODECONFIG = 		0x09;
static const uint8_t MAX86916_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX86916_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX86916_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX86916_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX86916_LED4_PULSEAMP = 	0x0F;
static const uint8_t MAX86916_LED_RANGE = 		0x11;
static const uint8_t MAX86916_LED_PROX_AMP = 	0x12;
static const uint8_t MAX86916_LED_SEQREG1 = 	0x13;
static const uint8_t MAX86916_LED_SEQREG2 = 	0x14;

//Cross talk DAC
static const uint8_t MAX86916_CROSSTALK1 = 		0x26;
static const uint8_t MAX86916_CROSSTALK2 = 		0x27;
static const uint8_t MAX86916_CROSSTALK3 = 		0x28;
static const uint8_t MAX86916_CROSSTALK4 = 		0x29;

// Proximity Function Registers
static const uint8_t MAX86916_PROXINTTHRESH = 	0x30;

// LED Connectivity Test
static const uint8_t MAX86916_COMPARENABLE = 		0x31;
static const uint8_t MAX86916_COMPARSTATUS = 		0x32;

// Part ID Registers
static const uint8_t MAX86916_REVISIONID = 		0xFE;
static const uint8_t MAX86916_PARTID = 			0xFF;    // Should always be 0x2B. Identical to MAX86916.

// MAX86916 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX86916_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX86916_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX86916_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX86916_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX86916_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX86916_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX86916_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX86916_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX86916_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX86916_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX86916_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX86916_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX86916_SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t MAX86916_SAMPLEAVG_1 = 	  0x00;
static const uint8_t MAX86916_SAMPLEAVG_2 = 	  0x20;
static const uint8_t MAX86916_SAMPLEAVG_4 = 	  0x40;
static const uint8_t MAX86916_SAMPLEAVG_8 = 	  0x60;
static const uint8_t MAX86916_SAMPLEAVG_16 = 	  0x80;
static const uint8_t MAX86916_SAMPLEAVG_32 = 	  0xA0;

static const uint8_t MAX86916_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX86916_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX86916_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX86916_A_FULL_MASK = 	  0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX86916_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX86916_SHUTDOWN = 		    0x80;
static const uint8_t MAX86916_WAKEUP = 			    0x00;

static const uint8_t MAX86916_RESET_MASK = 		  0xBF;
static const uint8_t MAX86916_RESET = 			    0x40;

static const uint8_t MAX86916_MODE_MASK = 		  0xFC;
static const uint8_t MAX86916_MODE_DISABLED = 	0x00;
static const uint8_t MAX86916_MODE_IRONLY = 	  0x01;
static const uint8_t MAX86916_MODE_REDIRONLY = 	0x02;
static const uint8_t MAX86916_MODE_FLEXLED = 	  0x03;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX86916_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX86916_ADCRANGE_4096 = 	0x00;
static const uint8_t MAX86916_ADCRANGE_8192 = 	0x20;
static const uint8_t MAX86916_ADCRANGE_16384 = 	0x40;
static const uint8_t MAX86916_ADCRANGE_32768 = 	0x60;

static const uint8_t MAX86916_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX86916_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX86916_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX86916_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX86916_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX86916_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX86916_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX86916_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX86916_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX86916_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX86916_PULSEWIDTH_70 = 	0x00;
static const uint8_t MAX86916_PULSEWIDTH_120 = 	0x01;
static const uint8_t MAX86916_PULSEWIDTH_220 = 	0x02;
static const uint8_t MAX86916_PULSEWIDTH_420 = 	0x03;

//LED Range
static const uint8_t MAX86916_LED1_RGE_MASK =   0xFC;
static const uint8_t MAX86916_LED2_RGE_MASK =   0xF3;
static const uint8_t MAX86916_LED3_RGE_MASK =   0xCF;
static const uint8_t MAX86916_LED4_RGE_MASK =   0x3F;
static const uint8_t MAX86916_LEDx_RGE_X1 = 	  0x00;
static const uint8_t MAX86916_LEDx_RGE_X2 = 	  0x01;
static const uint8_t MAX86916_LEDx_RGE_X3 = 	  0x02;
static const uint8_t MAX86916_LEDx_RGE_X4 = 	  0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX86916_SLOT1_MASK = 		  0xF0;
static const uint8_t MAX86916_SLOT2_MASK = 		  0x0F;
static const uint8_t MAX86916_SLOT3_MASK = 		  0xF0;
static const uint8_t MAX86916_SLOT4_MASK = 		  0x0F;
   
static const uint8_t SLOT_NONE = 				        0x00;
static const uint8_t SLOT_IR_LED = 				      0x01;
static const uint8_t SLOT_RED_LED = 			      0x02;
static const uint8_t SLOT_GREEN_LED = 			    0x03;
static const uint8_t SLOT_BLUE_LED = 			      0x04;
static const uint8_t SLOT_IR_PILOT =			      0x05;
static const uint8_t SLOT_RED_PILOT = 			    0x06;
static const uint8_t SLOT_GREEN_PILOT = 		    0x07;
static const uint8_t SLOT_BLUE_PILOT = 			    0x08;

static const uint8_t MAX86916_EXPECTEDPARTID =  0x2B;

MAX86916::MAX86916() {  
  // Constructor
}

boolean MAX86916::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr) {

  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  // Step 1: Initial Communication and Verification
  // Check that a MAX86916 is connected
  if (readPartID() != MAX86916_EXPECTEDPARTID) {
    // Error -- Part ID read from MAX86916 does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
    return false;
  }

  // Populate revision ID
  readRevisionID();
  
  return true;
}

//
// Configuration
//

//Begin Interrupt configuration
uint8_t MAX86916::getINT1(void) {
  return (readRegister8(_i2caddr, MAX86916_INTSTAT1));
}

void MAX86916::enableAFULL(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_A_FULL_MASK, MAX86916_INT_A_FULL_ENABLE);
}
void MAX86916::disableAFULL(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_A_FULL_MASK, MAX86916_INT_A_FULL_DISABLE);
}

void MAX86916::enableDATARDY(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_DATA_RDY_MASK, MAX86916_INT_DATA_RDY_ENABLE);
}
void MAX86916::disableDATARDY(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_DATA_RDY_MASK, MAX86916_INT_DATA_RDY_DISABLE);
}

void MAX86916::enableALCOVF(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_ALC_OVF_MASK, MAX86916_INT_ALC_OVF_ENABLE);
}
void MAX86916::disableALCOVF(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_ALC_OVF_MASK, MAX86916_INT_ALC_OVF_DISABLE);
}

void MAX86916::enablePROXINT(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_PROX_INT_MASK, MAX86916_INT_PROX_INT_ENABLE);
}
void MAX86916::disablePROXINT(void) {
  bitMask(MAX86916_INTENABLE1, MAX86916_INT_PROX_INT_MASK, MAX86916_INT_PROX_INT_DISABLE);
}

//End Interrupt configuration

void MAX86916::softReset(void) {
  bitMask(MAX86916_MODECONFIG, MAX86916_RESET_MASK, MAX86916_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX86916_MODECONFIG);
    if ((response & MAX86916_RESET) == 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
}

void MAX86916::shutDown(void) {
  // Put IC into low power mode (datasheet)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX86916_MODECONFIG, MAX86916_SHUTDOWN_MASK, MAX86916_SHUTDOWN);
}

void MAX86916::wakeUp(void) {
  // Pull IC out of low power mode (datasheet)
  bitMask(MAX86916_MODECONFIG, MAX86916_SHUTDOWN_MASK, MAX86916_WAKEUP);
}

void MAX86916::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- IR only, RED+IR only, or FLEX.
  // See datasheet
  bitMask(MAX86916_MODECONFIG, MAX86916_MODE_MASK, mode);
}

void MAX86916::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX86916_ADCRANGE_4096, _8192, _16384, _32768
  bitMask(MAX86916_PARTICLECONFIG, MAX86916_ADCRANGE_MASK, adcRange);
}

void MAX86916::setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX86916_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX86916_PARTICLECONFIG, MAX86916_SAMPLERATE_MASK, sampleRate);
}

void MAX86916::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX86916_PULSEWIDTH_70, _120, _220, _420
  bitMask(MAX86916_PARTICLECONFIG, MAX86916_PULSEWIDTH_MASK, pulseWidth);
}

// See datasheet, page 30
void MAX86916::setLEDRange(uint8_t rangeMask, uint8_t range) {
  // rangeMask: one of MAX86916_LEDx_RGE_MASK
  // range: one of MAX86916_LEDx_RGE_X1, _X2, _X3, _X4
  if(rangeMask      == MAX86916_LED1_RGE_MASK) bitMask(MAX86916_LED_RANGE, rangeMask, range);
  else if(rangeMask == MAX86916_LED2_RGE_MASK) bitMask(MAX86916_LED_RANGE, rangeMask, range << 2);
  else if(rangeMask == MAX86916_LED3_RGE_MASK) bitMask(MAX86916_LED_RANGE, rangeMask, range << 4);
  else if(rangeMask == MAX86916_LED4_RGE_MASK) bitMask(MAX86916_LED_RANGE, rangeMask, range << 6);
  else bitMask(MAX86916_LED_RANGE, MAX86916_LED1_RGE_MASK, range);
}

void MAX86916::setAllLEDRange(uint8_t range){
  // range: one of MAX86916_LEDx_RGE_X1, _X2, _X3, _X4
  setLEDRange(MAX86916_LED1_RGE_MASK, range);
  setLEDRange(MAX86916_LED2_RGE_MASK, range);
  setLEDRange(MAX86916_LED3_RGE_MASK, range);
  setLEDRange(MAX86916_LED4_RGE_MASK, range);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0xFF = 50.4mA @ LEDX_RGE = 0x0, 0xFF = 201.6mA @ LEDX_RGE = 0x3
void MAX86916::setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX86916_LED1_PULSEAMP, amplitude);
}

void MAX86916::setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX86916_LED2_PULSEAMP, amplitude);
}

void MAX86916::setPulseAmplitudeGreen(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX86916_LED3_PULSEAMP, amplitude);
}

void MAX86916::setPulseAmplitudeBlue(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX86916_LED4_PULSEAMP, amplitude);
}

void MAX86916::setPulseAmplitudeProximity(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX86916_LED_PROX_AMP, amplitude);
}

void MAX86916::setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  writeRegister8(_i2caddr, MAX86916_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX86916::enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX86916_LED_SEQREG1, MAX86916_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX86916_LED_SEQREG1, MAX86916_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX86916_LED_SEQREG2, MAX86916_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX86916_LED_SEQREG2, MAX86916_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void MAX86916::disableSlots(void) {
  writeRegister8(_i2caddr, MAX86916_LED_SEQREG1, 0);
  writeRegister8(_i2caddr, MAX86916_LED_SEQREG2, 0);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX86916::setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX86916_FIFOCONFIG, MAX86916_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX86916::clearFIFO(void) {
  writeRegister8(_i2caddr, MAX86916_FIFOWRITEPTR, 0);
  writeRegister8(_i2caddr, MAX86916_FIFOOVERFLOW, 0);
  writeRegister8(_i2caddr, MAX86916_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void MAX86916::enableFIFORollover(void) {
  bitMask(MAX86916_FIFOCONFIG, MAX86916_ROLLOVER_MASK, MAX86916_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX86916::disableFIFORollover(void) {
  bitMask(MAX86916_FIFOCONFIG, MAX86916_ROLLOVER_MASK, MAX86916_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX86916::setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX86916_FIFOCONFIG, MAX86916_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX86916::getWritePointer(void) {
  return (readRegister8(_i2caddr, MAX86916_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t MAX86916::getReadPointer(void) {
  return (readRegister8(_i2caddr, MAX86916_FIFOREADPTR));
}

// Set the PROX_INT_THRESHold
void MAX86916::setPROXINTTHRESH(uint8_t val) {
  writeRegister8(_i2caddr, MAX86916_PROXINTTHRESH, val);
}

//
// Device ID and Revision
//
uint8_t MAX86916::readPartID() {
  return readRegister8(_i2caddr, MAX86916_PARTID);
}

void MAX86916::readRevisionID() {
  revisionID = readRegister8(_i2caddr, MAX86916_REVISIONID);
}

uint8_t MAX86916::getRevisionID() {
  return revisionID;
}

//Setup the sensor
//The MAX86916 has many settings. By default we select:
// Sample Average = 4
// Mode = 3(FlexLED)
// Sample rate = 400
// Pulse Width = 420
// ADC Range = 16384 (31.25pA per LSB)
//Use the default setup if you are just getting started with the MAX86916 sensor
void MAX86916::setup(byte range, byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) setFIFOAverage(MAX86916_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) setFIFOAverage(MAX86916_SAMPLEAVG_2);
  else if (sampleAverage == 4) setFIFOAverage(MAX86916_SAMPLEAVG_4);
  else if (sampleAverage == 8) setFIFOAverage(MAX86916_SAMPLEAVG_8);
  else if (sampleAverage == 16) setFIFOAverage(MAX86916_SAMPLEAVG_16);
  else if (sampleAverage == 32) setFIFOAverage(MAX86916_SAMPLEAVG_32);
  else setFIFOAverage(MAX86916_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX86916_MODE_FLEXLED); //Watch all three LED channels
  else if (ledMode == 2) setLEDMode(MAX86916_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX86916_MODE_IRONLY); //IR only
  if (ledMode < 3) activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  else if(ledMode == 3) activeLEDs = 4;
  else activeLEDs = ledMode;
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 8192) setADCRange(MAX86916_ADCRANGE_4096); //7.81pA per LSB
  else if(adcRange < 16384) setADCRange(MAX86916_ADCRANGE_8192); //15.63pA per LSB
  else if(adcRange < 32768) setADCRange(MAX86916_ADCRANGE_16384); //31.25pA per LSB
  else if(adcRange == 32768) setADCRange(MAX86916_ADCRANGE_32768); //62.5pA per LSB
  else setADCRange(MAX86916_ADCRANGE_4096);

  if (sampleRate < 100) setSampleRate(MAX86916_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX86916_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX86916_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX86916_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX86916_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX86916_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX86916_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX86916_SAMPLERATE_3200);
  else setSampleRate(MAX86916_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 70us and 0.4mA it's about 2 inches
  //At 420us and 0.4mA it's about 6 inches
  if (pulseWidth < 120) setPulseWidth(MAX86916_PULSEWIDTH_70); //Page 30, Gets us 19 bit resolution
  else if (pulseWidth < 220) setPulseWidth(MAX86916_PULSEWIDTH_120); //19 bit resolution
  else if (pulseWidth < 420) setPulseWidth(MAX86916_PULSEWIDTH_220); //19 bit resolution
  else if (pulseWidth == 420) setPulseWidth(MAX86916_PULSEWIDTH_420); //19 bit resolution
  else setPulseWidth(MAX86916_PULSEWIDTH_70);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is range=0 powerLevel=0x1F which gets us 6.4mA
  //range      = 0
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setAllLEDRange(range);
  setPulseAmplitudeIR(powerLevel);
  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeGreen(powerLevel);
  setPulseAmplitudeBlue(powerLevel);
  setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Flex-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_IR_LED);
  if (ledMode > 1) enableSlot(2, SLOT_RED_LED);
  if (ledMode > 2){
    enableSlot(3, SLOT_GREEN_LED);
    enableSlot(4, SLOT_BLUE_LED);
  }
  //enableSlot(1, SLOT_IR_PILOT);
  //enableSlot(2, SLOT_RED_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //enableSlot(4, SLOT_BLUE_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

//
// Data Collection
//

//Tell caller how many samples are available
uint8_t MAX86916::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent IR value
uint32_t MAX86916::getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent red value
uint32_t MAX86916::getRed(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t MAX86916::getGreen(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.green[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Blue value
uint32_t MAX86916::getBlue(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.blue[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the next IR value in the FIFO
uint32_t MAX86916::getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

//Report the next Red value in the FIFO
uint32_t MAX86916::getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

//Report the next Green value in the FIFO
uint32_t MAX86916::getFIFOGreen(void)
{
  return (sense.green[sense.tail]);
}

//Report the next Blue value in the FIFO
uint32_t MAX86916::getFIFOBlue(void)
{
  return (sense.blue[sense.tail]);
}

//Advance the tail
void MAX86916::nextSample(void)
{
  if(available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
  }
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX86916::check(void)
{
  //Read register FIFO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    _i2cPort->beginTransmission(MAX86916_ADDRESS);
    _i2cPort->write(MAX86916_FIFODATA);
    _i2cPort->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (IR+Red * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 12 (IR+Red+GREEN+BLUE) = 8 left over. We want to request 24.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      _i2cPort->requestFrom(MAX86916_ADDRESS, toGet);
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - IR
        temp[3] = 0;
        temp[2] = _i2cPort->read();
        temp[1] = _i2cPort->read();
        temp[0] = _i2cPort->read();

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
    
    tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.IR[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - Red
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits
          
      sense.red[sense.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = tempLong;

          //Burst read three more bytes - Blue
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.blue[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool MAX86916::safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();
  
  while(1)
  {
  if(millis() - markTime > maxTimeToCheck) return(false);

  if(check() == true) //We found new data!
    return(true);

  delay(1);
  }
}

//Given a register, read it, mask it, and then set the thing
void MAX86916::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

//
// Low-level I2C Communication
//
uint8_t MAX86916::readRegister8(uint8_t address, uint8_t reg) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->endTransmission(false);

  _i2cPort->requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (_i2cPort->available())
  {
    return(_i2cPort->read());
  }

  return (0); //Fail

}

void MAX86916::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->write(value);
  _i2cPort->endTransmission();
}
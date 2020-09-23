#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include "globals.h"

#include "BLEPowerCSC.cpp" //TODO can this be swapped out for an .h?

BLEPowerCSC *bluetooth = new BLEPowerCSC();

//dviceConencted and oldDeviceConnected are defined in blepowercsc

void setup()
{
  Serial.begin(115200);
  bluetooth->initialize();
}

// modify to your liking
uint16_t maxPower = 500; // in W
uint16_t maxCad = 200; // in rpm
uint32_t loopDelay = 500; // time between BLE notifications and potentiometer detection, min 3ms

// initialze variables
uint16_t powerReading = maxPower;
uint16_t cadenceReading = maxCad;
uint64_t cumulativeRevolutions = 0; // represents the total number of times a crank rotates
uint64_t lastCET = 0;        
/*
  lastCET: The 'crank event time' is a free-running-count of 1/1024 second units and it 
  represents the time when the crank revolution was detected by the crank rotation sensor. Since
  several crank events can occur between transmissions, only the Last Crank Event Time value is
  transmitted. This value is used in combination with the Cumulative Crank Revolutions value to
  enable the Client to calculate cadence. The Last Crank Event Time value rolls over every 64 seconds.
*/

bool deviceConnected = false;
bool oldDeviceConnected = false;

void loop()
{
  // read power and cadence potentiometers
  uint16_t powVal = analogRead(34);
  uint16_t cadVal = analogRead(39);
  powerReading = powVal * maxPower/4096;
  cadenceReading = cadVal * maxCad/4096;

  // print some debug stuff
  Serial.print(powVal);
  Serial.print(" ");
  Serial.print(powerReading);
  Serial.print("W - ");
  Serial.print(cadVal);
  Serial.print(" ");
  Serial.print(cadenceReading);
  Serial.println("rpm ");

  // calculate time for a revolution in ms
  int cadTime = ((float)60/(float)cadenceReading)*1000;
  // revs in this time frame
  int revs = loopDelay/cadTime;
  // add to total revs
  cumulativeRevolutions+=revs;
  // set the time when the revolution occured
  lastCET+= cadTime * revs * 1000 / 1024;

  // notify changed value
  if (deviceConnected)
  {
    bluetooth->sendPower(powerReading); // the power send function.
    bluetooth->sendCSC(lastCET, cumulativeRevolutions); // the cadence send function 
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    bluetooth->startBroadcast();
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  delay(loopDelay); // the minimum is 3ms according to official docs
}
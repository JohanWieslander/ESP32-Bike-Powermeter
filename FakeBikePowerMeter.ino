#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define CYCLING_POWER_SERVICE_UUID "00001818-0000-1000-8000-00805F9B34FB"
#define POWER_CHARACTERISTIC_UUID "00002A63-0000-1000-8000-00805F9B34FB"
#define SENSORPOS_CHARACTERISTIC_UUID "00002A5D-0000-1000-8000-00805F9B34FB"
#define POWERFEATURE_CHARACTERISTIC_UUID "00002A65-0000-1000-8000-00805F9B34FB"

#define CSC_SERVICE_UUID "00001816-0000-1000-8000-00805F9B34FB"
#define CSC_MEASUREMENT_CHARACTERISTIC_UUID "00002A5B-0000-1000-8000-00805F9B34FB"
#define CSC_FEATURE_CHARACTERISTIC_UUID "00002A5C-0000-1000-8000-00805F9B34FB"
#define powerFlags 0b0000000000000000;
#define CSCFlags 0b10;

#define LED_PIN 13 // Red LED on huzzah32
#define PWR_PIN 34 // A2 on huzzah32
#define CAD_PIN 39 // A3 on huzzah32

// modify to your liking
uint16_t maxPower = 500; // in W
uint16_t maxCad = 200; // in rpm
uint32_t loopDelay = 500; // time between BLE notifications and potentiometer detection, min 3ms

//BLE
BLEServer *pServer = NULL;
//Power Characteristics
BLECharacteristic *pCharacteristicPower = NULL; //the power reading itself
BLECharacteristic *pCharacteristicSensorPos = NULL;
BLECharacteristic *pCharacteristicPowerFeature = NULL;
//CSC ONES
BLECharacteristic *pCharacteristicCSC = NULL; //the cadence reading
BLECharacteristic *pCharacteristicCSCFeature = NULL;
//Connection status
bool deviceConnected = false;
bool oldDeviceConnected = false;
// initialze variables
uint32_t powerTxValue = 0; //the BLE-Compliant, flagged, ready to transmit power value
uint64_t CSCTxValue = 0;   //the BLE-Compliant, flagged, ready to transmit CSC value
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

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer)
    {
        deviceConnected=true;
        digitalWrite(LED_PIN, HIGH);
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected=false;
        digitalWrite(LED_PIN, LOW);
    }
};

void sendPower(int16_t powerReading)
{
    powerTxValue = (powerReading << 16) | powerFlags;
    pCharacteristicPower->setValue((uint8_t *)&powerTxValue, 4);
    pCharacteristicPower->notify();
}

void sendCSC(uint64_t lastCET, uint64_t cumulativeRevolutions)
{
    CSCTxValue = (lastCET << 24) | (cumulativeRevolutions << 8) | CSCFlags;
    pCharacteristicCSC->setValue((uint8_t *)&CSCTxValue, 5);
    pCharacteristicCSC->notify();
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  // Create the BLE Device
  BLEDevice::init("ESP32FakePowerMeter"); // weirdly enough names with spaces do not seem to work

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(CYCLING_POWER_SERVICE_UUID);

  //CSC SERVICE
  BLEService *CSCService = pServer->createService(CSC_SERVICE_UUID);

  // Create the needed BLE Characteristics
  pCharacteristicPower = pService->createCharacteristic(
      POWER_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristicSensorPos = pService->createCharacteristic(
      SENSORPOS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ);

  pCharacteristicPowerFeature = pService->createCharacteristic(
      POWERFEATURE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ);

  //CSC CHARACTERISTICS
  pCharacteristicCSC = CSCService->createCharacteristic(
      CSC_MEASUREMENT_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristicCSCFeature = CSCService->createCharacteristic(
      CSC_FEATURE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ);

  //Create BLE Descriptors
  pCharacteristicPower->addDescriptor(new BLE2902());
  pCharacteristicCSC->addDescriptor(new BLE2902());

  //Start the services
  pService->start();
  CSCService->start();

  byte posvalue = 6; // right crank
  pCharacteristicSensorPos->setValue((uint8_t *)&posvalue, 1);

  //Feature Setting
  uint32_t powerFeature = 0b0; //just 32 old zeroes
  pCharacteristicPowerFeature->setValue((uint8_t *)&powerFeature, 4);
  uint16_t CSCFeature = 0b010; //only crank rpms supported
  pCharacteristicCSCFeature->setValue((uint8_t *)&CSCFeature, 2);

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(CYCLING_POWER_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
}

void loop()
{
  // read power and cadence potentiometers
  uint16_t powVal = analogRead(PWR_PIN);
  uint16_t cadVal = analogRead(CAD_PIN);
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
    sendPower(powerReading); // the power send function.
    sendCSC(lastCET, cumulativeRevolutions); // the cadence send function 
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
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
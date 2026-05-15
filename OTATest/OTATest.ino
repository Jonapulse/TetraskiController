#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "NimBLEDevice.h" //CHANGE: replaced ArduinoBLE.h with NimBLEDevice.h

//Serial output for development/debugging. TURN OFF FOR TETRASKI USE
#define COMMS 1
#define SENSOR_COUNT 2 // Set to 2 or 4 depending on configuration

/************ WiFi OTA Stuff **************************************************/
const char* ssid = "TetraOTA";
const char* password = "tetra2034";
bool OTAUpdateEnable = 0;


/************ BLE Sensor Stuff ************************************************/
const char* targetLocalName  = "ANR Corp M40"; // Match any device with the name for Muscle Sense Model M40

const char* battServiceUUID   = "180f";
const char* battCharUUID      = "2A19";
const char* AutoIOServiceUUID = "1815";
const char* AnalogCharUUID    = "2A58";
const char* DigitalCharUUID   = "2A56";

//CHANGE: SensorBLE struct updated to use NimBLE pointer types
struct SensorBLE {
  NimBLEClient*               client         = nullptr;
  NimBLERemoteService*        batteryService = nullptr;
  NimBLERemoteService*        ioService      = nullptr;
  NimBLERemoteCharacteristic* batteryChar    = nullptr;
  NimBLERemoteCharacteristic* analogChar     = nullptr;
  NimBLERemoteCharacteristic* digitalChar    = nullptr;
};

SensorBLE sensors[SENSOR_COUNT];  // Array to hold connected sensors
int connectedSensorCount = 0;

#define RECONNECT_FREQ 10000

//Sensitivities
const uint8_t sensitivityLevels[3] = {20, 50, 80};
uint16_t sensorThresholds[SENSOR_COUNT];

//Directions
int sensorOutputs[4] = {1, 2, 3, 4}; //1 - left, 2 - right, FOR SENSOR_COUNT > 2 3 - wedge in, 4 - wedge out
int idle       = 0;

uint16_t sensorAverages[SENSOR_COUNT]; 
const int SIZE_OF_AVE = 200;

#define BUFFER_SIZE 20 //data transmission @ 10 Hz for 2 sec
uint16_t sensorBuffers[SENSOR_COUNT][BUFFER_SIZE];

void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  //CHANGE: replaced BLE.begin() with NimBLEDevice::init()
  //NOTE: esp32 3.3.7 breaks ArduinoBLE. NimBLE does not have this issue.
  NimBLEDevice::init("");

  // Connect sensors
  while(1) { //Enter connection loop

    //attempt to connect again until connection established
    if (connectSensors()) {
      Serial.print('$');  // Signal successful connection to ski
      break;
    }

    //check for incoming byte from ski to trigger WiFi OTA
    if(Serial.available()) {
      if(Serial.read()=='w') {
        enterWifiOTA(); //signal from ski to enter wifi OTA
      }
    }
  }

  //perform initial calibration
  calibrateThreshold();
}



// --------------------------------------------------
// loop - 
// read/respond to serial input
// 'w' - enter wifi pairing
// '5' - calibrate sensors
// 'c'/'f' - confirmation bytes for control change and calibration
// '0,1,2'/'6,7,8'/'i,j,k'/'l,m,n' - sensitivity for sensors 0,1,2,3 
// '3,4'/'g,h' - set 'left/right' sensors to standard or inverted controls
// --------------------------------------------------
void loop() {

  //check for incoming comms from TetraSki
  if(Serial.available()) {

    //read incoming byte from TetraSki
    char incomingByte = Serial.read();

    switch(incomingByte) {
      //Signal from ski to enter wifi pairing
      case 'w':
        //CHANGE: replaced BLE.stopAdvertise()/BLE.connected()/BLE.disconnect() with NimBLE client disconnects and deinit
        for (int i = 0; i < SENSOR_COUNT; i++) {
          if (sensors[i].client && sensors[i].client->isConnected()) {
            sensors[i].client->disconnect();
          }
        }
        NimBLEDevice::deinit(true);
        delay(1000); //delay to allow disconnect
        enterWifiOTA();

      //Signal from ski to change sensitivity for  (*currently preset levels) for sensor 0
      case '0':
      case '1':
      case '2':
        setSensitivityBySensor(0, sensitivityLevels[incomingByte-'0']);
        if(COMMS){
          Serial.println();
          Serial.print("\nSensitivity for sensor 0 set to ");
          Serial.print(sensitivityLevels[incomingByte-'0']);
          Serial.print(" with a threshold of ");
          Serial.println(sensorThresholds[0]);
        }
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      //Signal from ski for standard direction controls
      case '3':
        sensorOutputs[0] = 1; // 1 - left
        sensorOutputs[1] = 2; // 2 - right
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      //Signal from ski for inverted direction controls
      case '4':
        sensorOutputs[0] = 2; // 2 - right
        sensorOutputs[1] = 1; // 1 - left
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      case '5':
        Serial.print('c'); //send confirmation byte to TetraSki
        calibrateThreshold();
        Serial.print('f'); //send confirmation byte to TetraSki
        break;

      //Signal from ski to change sensitivity for  (*currently preset levels) for sensor[1]
      case '6':
      case '7':
      case '8':
        setSensitivityBySensor(1, sensitivityLevels[incomingByte-'6']);
        if(COMMS){
          Serial.println();
          Serial.print("\nSensitivity for sensor 1 set to ");
          Serial.print(sensitivityLevels[incomingByte-'6']);
          Serial.print(" with a threshold of ");
          Serial.println(sensorThresholds[1]);
        }

        Serial.print('c');  //send confirmation byte to TetraSki
        break;

#if SENSOR_COUNT == 4
      //Signal from ski for standard wedge direction controls
      case 'g':
        sensorOutputs[2] = 3; // 3 - wedge in
        sensorOutputs[3] = 4; // 4 - wedge out
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      //Signal from ski for inverted wedge direction controls
      case 'h':
        sensorOutputs[2] = 4; // 4 - wedge in
        sensorOutputs[3] = 3; // 3 - wedge out
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      //Signal from ski to change sensitivity for  (*currently preset levels) for sensor[2]
      case 'i':
      case 'j':
      case 'k':
        setSensitivityBySensor(2, sensitivityLevels[incomingByte-'i']);
        if(COMMS){
          Serial.println();
          Serial.print("\nSensitivity for sensor 2 set to ");
          Serial.print(sensitivityLevels[incomingByte-'i']);
          Serial.print(" with a threshold of ");
          Serial.println(sensorThresholds[2]);
        }

        Serial.print('c');  //send confirmation byte to TetraSki
        break;

      //Signal from ski to change sensitivity for  (*currently preset levels) for sensor[3]
      case 'l':
      case 'm':
      case 'n':
        setSensitivityBySensor(3, sensitivityLevels[incomingByte-'l']);
        if(COMMS){
          Serial.println();
          Serial.print("\nSensitivity for sensor 3 set to ");
          Serial.print(sensitivityLevels[incomingByte-'l']);
          Serial.print(" with a threshold of ");
          Serial.println(sensorThresholds[3]);
        }

        Serial.print('c');  //send confirmation byte to TetraSki
        break;
#endif
    }
  }

  uint16_t sensorValues[SENSOR_COUNT];

  for(int i = 0; i < SENSOR_COUNT; i += 2) //Looping for 2 or 4 sensors
  {
    //CHANGE: replaced analogChar.readValue() with NimBLE readValue() returning std::string, cast to uint16_t
    std::string rawA = sensors[i].analogChar->readValue();
    std::string rawB = sensors[i + 1].analogChar->readValue();

    if (rawA.length() >= 2 && rawB.length() >= 2) {
      sensorValues[i]     = *(uint16_t*)rawA.data();
      sensorValues[i + 1] = *(uint16_t*)rawB.data();

      //update buffer
      updateBuffer(sensorBuffers[i], sensorValues[i]);
      updateBuffer(sensorBuffers[i + 1], sensorValues[i + 1]);

      static int fillCount = 0;
      if (fillCount < BUFFER_SIZE) fillCount++;

      //calculate changes in signals
      float slopeA = computeSlope(sensorBuffers[i]);
      float slopeB = computeSlope(sensorBuffers[i + 1]);

      bool sensATriggered = sensorValues[i] > sensorThresholds[i];
      bool sensBTriggered = sensorValues[i + 1] > sensorThresholds[i + 1];

      //turning logic
      if (sensATriggered && !sensBTriggered) {
        Serial.print(sensorOutputs[i]);
      }
      else if (!sensATriggered && sensBTriggered) {
        Serial.print(sensorOutputs[i + 1]);
      }
      else if (sensATriggered && sensBTriggered) {
        if (slopeA > 0.1 && slopeB < 0.1){
          Serial.print(sensorOutputs[i]);
        }
        else if (slopeA < 0.1 && slopeB > 0.1) {
          Serial.print(sensorOutputs[i + 1]);
        }
      }
      else {
        Serial.print(idle);
      }
    }
  }
  
}


// --------------------------------------------------
// Connect to N sensors matching target local name
// --------------------------------------------------
//CHANGE: connectSensors rewritten to use NimBLE scanning and client API
bool connectSensors() {
  connectedSensorCount = 0;

  if(COMMS) Serial.println("Scanning for sensors...");

  NimBLEScan* pScan = NimBLEDevice::getScan(); //CHANGE: get NimBLE scan instance
  pScan->setActiveScan(true); //CHANGE: active scan returns more device info including name

  long scanStart = millis();

  while (millis() - scanStart < RECONNECT_FREQ && connectedSensorCount < SENSOR_COUNT) {

    //CHANGE: blocking scan for 1 second per iteration, then check results
    NimBLEScanResults results = pScan->getResults(1000, false);

    for (int i = 0; i < results.getCount(); i++) {
      const NimBLEAdvertisedDevice* device = results.getDevice(i);
      std::string name = device->getName();

      if(COMMS) {
        Serial.print("Found: ");
        Serial.print(device->getAddress().toString().c_str());
        Serial.print(" | Name: ");
        Serial.println(name.c_str());
      }

      if (name == targetLocalName) {
        if(COMMS) {
          Serial.print("Target found: ");
          Serial.println(device->getAddress().toString().c_str());
        }

        SensorBLE &sensor = sensors[connectedSensorCount];

        sensor.client = NimBLEDevice::createClient(); //CHANGE: create NimBLE client
        if (!sensor.client) {
          if(COMMS) Serial.println("Client creation failed, skipping");
          continue;
        }

        if (!sensor.client->connect(device)) {
          if(COMMS) Serial.println("Connection failed, skipping");
          NimBLEDevice::deleteClient(sensor.client); //CHANGE: clean up failed client
          sensor.client = nullptr;
          continue;
        }

        //CHANGE: NimBLE discovers attributes automatically on connect, no separate call needed

        sensor.batteryService = sensor.client->getService(battServiceUUID); //CHANGE: getService replaces peripheral.service()
        if (!sensor.batteryService) {
          if(COMMS) Serial.println("Battery service not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.batteryChar = sensor.batteryService->getCharacteristic(battCharUUID); //CHANGE: getCharacteristic replaces .characteristic()
        if (!sensor.batteryChar || !sensor.batteryChar->canRead()) {
          if(COMMS) Serial.println("Battery characteristic not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.ioService = sensor.client->getService(AutoIOServiceUUID);
        if (!sensor.ioService) {
          if(COMMS) Serial.println("IO service not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.analogChar = sensor.ioService->getCharacteristic(AnalogCharUUID);
        if (!sensor.analogChar || !sensor.analogChar->canRead()) {
          if(COMMS) Serial.println("Analog characteristic not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.digitalChar = sensor.ioService->getCharacteristic(DigitalCharUUID);
        if (!sensor.digitalChar || !sensor.digitalChar->canWrite()) {
          if(COMMS) Serial.println("Digital characteristic not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        if(COMMS) {
          Serial.print("Sensor ");
          Serial.print(connectedSensorCount);
          Serial.print(" connected: ");
          Serial.println(device->getAddress().toString().c_str());
        }

        connectedSensorCount++;
      }
    }
  }

  if (connectedSensorCount == SENSOR_COUNT) {
    sortSensors();
    if(COMMS) Serial.println("All sensors connected!");
    return true;
  }

  if(COMMS) {
    Serial.print("Timeout. Connected ");
    Serial.print(connectedSensorCount);
    Serial.print(" of ");
    Serial.println(SENSOR_COUNT);
  }
  return false;
}

// --------------------------------------------------
// Sort Sensors - ensures Muscle Sensors detected in 
// arbitrary order will maintain "identity" on reset, so
// 'turn left' is not reassigned on reset (unless sensors change)
// --------------------------------------------------
//CHANGE: sortSensors updated to use NimBLE address API
void sortSensors()
{
  for (int i = 0; i < SENSOR_COUNT - 1; i++) {
    for (int j = i + 1; j < SENSOR_COUNT; j++) {
      if (String(sensors[i].client->getPeerAddress().toString().c_str()) >
          String(sensors[j].client->getPeerAddress().toString().c_str())) {
          SensorBLE temp = sensors[i];
          sensors[i] = sensors[j];
          sensors[j] = temp;
      }
    }
  }
}

// --------------------------------------------------
// Buffer calculation fxns
// --------------------------------------------------
void updateBuffer(uint16_t *buffer, uint16_t value) {
  //Shift left
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
  //Insert newest at the end
  buffer[BUFFER_SIZE - 1] = value;
}

float computeAverageDerivative(uint16_t *buffer) {
  int32_t sum = 0;
  for (int i = 1; i < BUFFER_SIZE; i++) {
    sum += (int32_t)buffer[i] - (int32_t)buffer[i - 1];
  }
  return (float)sum / (BUFFER_SIZE - 1);
}

float computeSlope(uint16_t *buffer) {
  const int N = BUFFER_SIZE;
  float sumX  = 0, sumY  = 0, sumXY = 0, sumXX = 0;
  for (int i = 0; i < N; i++) {
    sumX  += i;
    sumY  += buffer[i];
    sumXY += i * buffer[i];
    sumXX += i * i;
  }
  float denominator = N * sumXX - sumX * sumX;
  if (denominator == 0) return 0;
  return (N * sumXY - sumX * sumY) / denominator;
}

//TODO: Update to individual thresholds
// --------------------------------------------------
// Set Sensitivity by sensor
// --------------------------------------------------
void setSensitivityBySensor(uint16_t sensor, uint16_t value)
{
  sensorThresholds[sensor] = (sensorAverages[sensor] / SIZE_OF_AVE) + value;
}

// --------------------------------------------------
// Calibrate sensors to baseline
// --------------------------------------------------
//CHANGE: writeValue() and readValue() updated to NimBLE API throughout
void calibrateThreshold() {

  uint8_t orangeLED = 12;
  uint8_t greenLED  = 5;

  sensors[0].digitalChar->writeValue(&orangeLED, 1); //CHANGE: NimBLE writeValue takes pointer and length
  sensors[1].digitalChar->writeValue(&orangeLED, 1);
#if SENSOR_COUNT == 4
  sensors[2].digitalChar->writeValue(&orangeLED, 1);
  sensors[3].digitalChar->writeValue(&orangeLED, 1);
#endif

  if(COMMS) Serial.println("Starting Calibration");

  sensorAverages[0] = 0;  // Reset averages before accumulating
  sensorAverages[1] = 0;
#if SENSOR_COUNT == 4
  sensorAverages[2] = 0;
  sensorAverages[3] = 0;
#endif

  //CHANGE: readValue() returns std::string, cast raw bytes to uint16_t
  for (int i = 0; i < SIZE_OF_AVE; i++) {
    std::string v0 = sensors[0].analogChar->readValue();
    std::string v1 = sensors[1].analogChar->readValue();
    if (v0.length() >= 2) sensorAverages[0] += *(uint16_t*)v0.data();
    if (v1.length() >= 2) sensorAverages[1] += *(uint16_t*)v1.data();
#if SENSOR_COUNT == 4
    std::string v2 = sensors[2].analogChar->readValue();
    std::string v3 = sensors[3].analogChar->readValue();
    if (v2.length() >= 2) sensorAverages[2] += *(uint16_t*)v2.data();
    if (v3.length() >= 2) sensorAverages[3] += *(uint16_t*)v3.data();
#endif
  }

  setSensitivityBySensor(0, sensitivityLevels[1]);
  setSensitivityBySensor(1, sensitivityLevels[1]);
  sensors[0].digitalChar->writeValue(&greenLED, 1); //CHANGE: NimBLE writeValue takes pointer and length
  sensors[1].digitalChar->writeValue(&greenLED, 1);
#if SENSOR_COUNT == 4
  setSensitivityBySensor(2, sensitivityLevels[1]);
  setSensitivityBySensor(3, sensitivityLevels[1]);
  sensors[2].digitalChar->writeValue(&greenLED, 1);
  sensors[3].digitalChar->writeValue(&greenLED, 1);
#endif
}


// --------------------------------------------------
// Enter Wifi OTA
// --------------------------------------------------
void enterWifiOTA() {

  NimBLEDevice::deinit(true); //CHANGE: replaced BLE.end() with NimBLEDevice::deinit()
  delay(1000);

  if(COMMS) Serial.println("Entering WiFi OTA");

  WiFi.begin(ssid, password);
  delay(1000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if(COMMS) Serial.println("Connecting to WiFi...");
  }
  if(COMMS) Serial.println("Connected to WiFi");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { //U_SPIFFS
      type = "filesystem";
    }
    if(COMMS) Serial.println("Start updating " + type);
    Serial.print('#');//Send signal to ski that wifi update is starting
  });
  // ... (other OTA callbacks)

  ArduinoOTA.begin();
  ArduinoOTA.setPassword("test");
  if(COMMS) Serial.println("Ready");
  if(COMMS) Serial.print("IP address: ");
  if(COMMS) Serial.println(WiFi.localIP());

  Serial.print('@'); //Send signal to TetraSki that Wifi is connected

  delay(1000);

  while(1)
    ArduinoOTA.handle();
}
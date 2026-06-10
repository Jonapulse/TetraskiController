#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "NimBLEDevice.h"

//Serial output for development/debugging. TURN OFF FOR TETRASKI USE
#define COMMS 1
#define DEFAULT_SENSOR_COUNT 4  
#define MAX_SENSOR_COUNT 4

/************ WiFi OTA Stuff **************************************************/
const char* ssid = "TetraOTA";
const char* password = "tetra2034";
bool OTAUpdateEnable = 0;

/************ BLE Sensor Stuff ************************************************/
const char* targetLocalName = "ANR Corp M40";  // Match any device with the name for Muscle Sense Model M40

const char* battServiceUUID = "180f";
const char* battCharUUID = "2A19";
const char* AutoIOServiceUUID = "1815";
const char* AnalogCharUUID = "2A58";
const char* DigitalCharUUID = "2A56";

struct SensorBLE {
  NimBLEClient* client = nullptr;
  NimBLERemoteService* batteryService = nullptr;
  NimBLERemoteService* ioService = nullptr;
  NimBLERemoteCharacteristic* batteryChar = nullptr;
  NimBLERemoteCharacteristic* analogChar = nullptr;
  NimBLERemoteCharacteristic* digitalChar = nullptr;
};

SensorBLE sensors[MAX_SENSOR_COUNT];
int targetSensorCount = MAX_SENSOR_COUNT;
int connectedSensorCount = 0;

volatile uint16_t latestAnalogValues[MAX_SENSOR_COUNT] = { 0 };
volatile bool newValueReady[MAX_SENSOR_COUNT] = { false };

#define RECONNECT_FREQ 10000

//Sensitivities
const uint8_t sensitivityLevels[3] = { 20, 50, 80 };
uint16_t sensorThresholds[MAX_SENSOR_COUNT];

//Directions
int sensorOutputs[MAX_SENSOR_COUNT] = { 1, 2, 3, 4 };  //1 - left, 2 - right, FOR targetSensorCount > 2: 3 - wedge in, 4 - wedge out
int idle = 0;

uint16_t sensorAverages[MAX_SENSOR_COUNT];
const int SIZE_OF_AVE = 200;

#define BUFFER_SIZE 20  //data transmission @ 10 Hz for 2 sec
uint16_t sensorBuffers[MAX_SENSOR_COUNT][BUFFER_SIZE];


// --------------------------------------------------
// We're using this asynchronous NotifyCallback approach to reading data from sensors rather than 
// the sequential sensor.readValue() approach which must wait for each sensor to call/respond in order  
// --------------------------------------------------
// notification callback - called by NimBLE stack when sensor pushes a new value
// pData contains raw bytes, length should be 2 for a uint16_t analog value
// sensorIndex identifies which sensor triggered this callback
void makeNotifyCallback(int sensorIndex) {
  // Returns a lambda capturing sensorIndex, used when subscribing each sensor
  // Called on BLE stack thread - only write to volatiles, no Serial or BLE calls here
}

// actual notification callback function, one per sensor slot
// NimBLE requires a plain function or lambda with this exact signature
void notifyCallback0(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    latestAnalogValues[0] = *(uint16_t*)pData;
    newValueReady[0] = true;
  }
}
void notifyCallback1(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    latestAnalogValues[1] = *(uint16_t*)pData;
    newValueReady[1] = true;
  }
}
void notifyCallback2(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    latestAnalogValues[2] = *(uint16_t*)pData;
    newValueReady[2] = true;
  }
}
void notifyCallback3(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    latestAnalogValues[3] = *(uint16_t*)pData;
    newValueReady[3] = true;
  }
}

typedef void (*NotifyCallback)(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);
NotifyCallback notifyCallbacks[4] = {
  notifyCallback0,
  notifyCallback1,
  notifyCallback2,
  notifyCallback3
};


/************ NimBLE Phone Peripheral Stuff **************************************/

// UUIDs must match App.js constants exactly
#define PHONE_SERVICE_UUID      "12345678-1234-1234-1234-123456789abc"
#define SENSOR_DATA_CHAR_UUID   "12345678-1234-1234-1234-123456789abd"
#define BATTERY_DATA_CHAR_UUID  "12345678-1234-1234-1234-123456789abe"
#define COMMAND_CHAR_UUID       "12345678-1234-1234-1234-123456789abf"

NimBLEServer*         pPhoneServer       = nullptr;
NimBLECharacteristic* pSensorDataChar    = nullptr;  // NOTIFY  — 5 bytes: [dir, t2h, t2l, t3h, t3l]
NimBLECharacteristic* pBatteryDataChar   = nullptr;  // READ    — 2 bytes: [battT2, battT3]
NimBLECharacteristic* pCommandChar       = nullptr;  // WRITE   — 1 byte command
bool phoneConnected = false;

// Forward declaration — handleCommand() is used inside CommandCallbacks::onWrite()
void handleCommand(char cmd);

// NimBLE server callbacks — track phone connect/disconnect
class PhoneServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {  
    phoneConnected = true;
    if (COMMS) Serial.println("Phone connected");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override { 
    phoneConnected = false;
    if (COMMS) Serial.println("Phone disconnected — restarting advertising");
    NimBLEDevice::startAdvertising();  // auto-restart so phone can reconnect
  }
};

// NimBLE characteristic callbacks — handle incoming command writes from phone
class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {  
    std::string val = pChar->getValue();
    if (val.length() > 0) {
      handleCommand((char)val[0]);
    }
  }
};

void setupPhonePeripheral() {
  pPhoneServer = NimBLEDevice::createServer();
  pPhoneServer->setCallbacks(new PhoneServerCallbacks());

  NimBLEService* pService = pPhoneServer->createService(PHONE_SERVICE_UUID);

  // Sensor data: NOTIFY so phone gets pushed updates
  pSensorDataChar = pService->createCharacteristic(
    SENSOR_DATA_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  // Battery: READ only
  pBatteryDataChar = pService->createCharacteristic(
    BATTERY_DATA_CHAR_UUID,
    NIMBLE_PROPERTY::READ
  );

  // Command: WRITE from phone
  pCommandChar = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pCommandChar->setCallbacks(new CommandCallbacks());

  pService->start();

  // Configure and start advertising
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(PHONE_SERVICE_UUID);
  pAdvertising->setName("TetraRadio");
  NimBLEDevice::startAdvertising();

  if (COMMS) Serial.println("BLE advertising as TetraRadio");
}


void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  NimBLEDevice::init("TetraRadio");

  setupPhonePeripheral();

  //TODO: check save for previous targetSensorCount
  targetSensorCount = MAX_SENSOR_COUNT;
  scanAndConnectSensors();
}

void scanAndConnectSensors()
{
  while (1) {  //Enter connection loop

    //attempt to connect again until connection established
    if (connectSensors()) {
      Serial.print('$');  // Signal successful connection to ski
      break;
    }

    //check for incoming byte from ski to trigger WiFi OTA
    if (Serial.available()) {
      if (Serial.read() == 'w') {
        enterWifiOTA();  //signal from ski to enter wifi OTA
      }
    }
  }

  //Check save

  //perform initial calibration
  calibrateThreshold();
}



// --------------------------------------------------
// loop -
// --------------------------------------------------
void loop() {
  //check for incoming comms from TetraSki
  if (Serial.available()) {
    char incomingByte = Serial.read();
    handleCommand(incomingByte); 
  }

  for (int i = 0; i < targetSensorCount; i += 2) {
    if (newValueReady[i] && newValueReady[i + 1]) {
      uint16_t valA = latestAnalogValues[i];
      uint16_t valB = latestAnalogValues[i + 1];
      newValueReady[i] = false;  
      newValueReady[i + 1] = false;

      //update buffer
      updateBuffer(sensorBuffers[i], valA);
      updateBuffer(sensorBuffers[i + 1], valB);

      static int fillCount = 0;
      if (fillCount < BUFFER_SIZE) fillCount++;

      //calculate changes in signals
      float slopeA = computeSlope(sensorBuffers[i]);
      float slopeB = computeSlope(sensorBuffers[i + 1]);

      bool sensATriggered = valA > sensorThresholds[i];
      bool sensBTriggered = valB > sensorThresholds[i + 1];

      //determine current direction output
      int currentDirection = idle;
      if (sensATriggered && !sensBTriggered) {
        currentDirection = sensorOutputs[i];
      } else if (!sensATriggered && sensBTriggered) {
        currentDirection = sensorOutputs[i + 1];
      } else if (sensATriggered && sensBTriggered) {
        if (slopeA > 0.1 && slopeB < 0.1) {
          currentDirection = sensorOutputs[i];
        } else if (slopeA < 0.1 && slopeB > 0.1) {
          currentDirection = sensorOutputs[i + 1];
        }
      }

      //output direction over serial to TetraSki
      Serial.print(currentDirection);

      // Broadcast sensor data to phone if connected.
      // Only broadcasting the first pair (sensors 0+1) for now —
      // expand payload if the app is updated to display all 4 sensors.
      if (phoneConnected && i == 0) {
        uint8_t payload[5];
        payload[0] = (uint8_t)currentDirection;
        payload[1] = (uint8_t)(valA >> 8);
        payload[2] = (uint8_t)(valA & 0xFF);
        payload[3] = (uint8_t)(valB >> 8);
        payload[4] = (uint8_t)(valB & 0xFF);
        pSensorDataChar->setValue(payload, 5);
        pSensorDataChar->notify();

        // Read and broadcast battery levels periodically
        static unsigned long lastBattUpdate = 0;
        if (millis() - lastBattUpdate > 5000) {  // every 5 seconds
          lastBattUpdate = millis();
          uint8_t battA = 0, battB = 0;
          std::string battValA = sensors[0].batteryChar->readValue();
          std::string battValB = sensors[1].batteryChar->readValue();
          if (battValA.length() > 0) battA = (uint8_t)battValA[0];
          if (battValB.length() > 0) battB = (uint8_t)battValB[0];
          uint8_t battPayload[2] = { battA, battB };
          pBatteryDataChar->setValue(battPayload, 2);
        }
      }
    }
  }
}


// --------------------------------------------------
// Shared command handler (serial + BLE phone)
// read/respond to serial input
// 'w' - enter wifi pairing
// '5' - calibrate sensors
// 'c'/'f' - confirmation bytes for control change and calibration
// '0','1','2'/'6','7','8'/'i','j','k'/'l','m','n' - sensitivity for sensors 0,1,2,3
// '3','4'/'g','h' - set 'left','right'/'wedge in','wedge out' sensors to standard or inverted controls
// 'o'/'p' - set sensor count to '2'/'4'.
// --------------------------------------------------
// TODO: Update loop so it can handle multi-char
// Extracted so CommandCallbacks::onWrite() (phone app) and loop() (instructor override control) share one implementation.
void handleCommand(char cmd) {
  switch (cmd) {

    case 'w':
      for (int i = 0; i < targetSensorCount; i++) {
        if (sensors[i].client && sensors[i].client->isConnected()) {
          sensors[i].client->disconnect();
        }
      }
      NimBLEDevice::deinit(true);
      delay(1000);
      enterWifiOTA();
      break;

    case '0':
    case '1':
    case '2':
      setSensitivityBySensor(0, sensitivityLevels[cmd - '0']);
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 0 set to ");
        Serial.println(sensitivityLevels[cmd - '0']);
      }
      Serial.print('c');
      break;

    case '3':
      sensorOutputs[0] = 1;
      sensorOutputs[1] = 2;
      Serial.print('c');
      break;

    case '4':
      sensorOutputs[0] = 2;
      sensorOutputs[1] = 1;
      Serial.print('c');
      break;

    case '5':
      Serial.print('c');
      calibrateThreshold();
      Serial.print('f');
      break;

    case '6':
    case '7':
    case '8':
      setSensitivityBySensor(1, sensitivityLevels[cmd - '6']);
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 1 set to ");
        Serial.println(sensitivityLevels[cmd - '6']);
      }
      Serial.print('c');
      break;

    case 'g':
      sensorOutputs[2] = 3;
      sensorOutputs[3] = 4;
      Serial.print('c');
      break;

    case 'h':
      sensorOutputs[2] = 4;
      sensorOutputs[3] = 3;
      Serial.print('c');
      break;

    case 'i':
    case 'j':
    case 'k':
      setSensitivityBySensor(2, sensitivityLevels[cmd - 'i']);
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 2 set to ");
        Serial.println(sensitivityLevels[cmd - 'i']);
      }
      Serial.print('c');
      break;

    case 'l':
    case 'm':
    case 'n':
      setSensitivityBySensor(3, sensitivityLevels[cmd - 'l']);
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 3 set to ");
        Serial.println(sensitivityLevels[cmd - 'l']);
      }
      Serial.print('c');
      break;

    case 'o':
      targetSensorCount = 2;
      if(connectedSensorCount < targetSensorCount)
        scanAndConnectSensors();
      break;
    case 'p':
      targetSensorCount = 4;
      if(connectedSensorCount < targetSensorCount)
        scanAndConnectSensors();
      break;
  }
}


// --------------------------------------------------
// Connect to N sensors matching target local name
// --------------------------------------------------
bool connectSensors() {
  if (COMMS) Serial.println("Scanning for sensors...");

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);

  long scanStart = millis();

  while (millis() - scanStart < RECONNECT_FREQ && connectedSensorCount < targetSensorCount) {

    NimBLEScanResults results = pScan->getResults(1000, false);

    for (int i = 0; i < results.getCount(); i++) {
      const NimBLEAdvertisedDevice* device = results.getDevice(i);
      std::string name = device->getName();

      if (COMMS) {
        Serial.print("Found: ");
        Serial.print(device->getAddress().toString().c_str());
        Serial.print(" | Name: ");
        Serial.println(name.c_str());
      }

      if (name == targetLocalName) {
        if (COMMS) {
          Serial.print("Target found: ");
          Serial.println(device->getAddress().toString().c_str());
        }

        SensorBLE& sensor = sensors[connectedSensorCount];

        sensor.client = NimBLEDevice::createClient();
        if (!sensor.client) {
          if (COMMS) Serial.println("Client creation failed, skipping");
          continue;
        }

        if (!sensor.client->connect(device)) {
          if (COMMS) Serial.println("Connection failed, skipping");
          NimBLEDevice::deleteClient(sensor.client);
          sensor.client = nullptr;
          continue;
        }

        sensor.batteryService = sensor.client->getService(battServiceUUID);
        if (!sensor.batteryService) {
          if (COMMS) Serial.println("Battery service not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.batteryChar = sensor.batteryService->getCharacteristic(battCharUUID);
        if (!sensor.batteryChar || !sensor.batteryChar->canRead()) {
          if (COMMS) Serial.println("Battery characteristic not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.ioService = sensor.client->getService(AutoIOServiceUUID);
        if (!sensor.ioService) {
          if (COMMS) Serial.println("IO service not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.analogChar = sensor.ioService->getCharacteristic(AnalogCharUUID);
        if (!sensor.analogChar || !sensor.analogChar->canRead()) {
          if (COMMS) Serial.println("Analog characteristic not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        sensor.digitalChar = sensor.ioService->getCharacteristic(DigitalCharUUID);
        if (!sensor.digitalChar || !sensor.digitalChar->canWrite()) {
          if (COMMS) Serial.println("Digital characteristic not found, skipping");
          sensor.client->disconnect();
          continue;
        }

        // notifyCallbacks[] maps sensor index to its callback function
        if (!sensor.analogChar->subscribe(true, notifyCallbacks[connectedSensorCount])) {
          if (COMMS) Serial.println("Notification subscription failed, skipping");
          sensor.client->disconnect();
          continue;
        }

        if (COMMS) {
          Serial.print("Sensor ");
          Serial.print(connectedSensorCount);
          Serial.print(" connected and subscribed: ");
          Serial.println(device->getAddress().toString().c_str());
        }

        connectedSensorCount++;
        
        if(connectedSensorCount == targetSensorCount)
          break;
      }
    }
  }

  if (connectedSensorCount == targetSensorCount) {
    sortSensors();
    if (COMMS) Serial.println("All sensors connected!");
    return true;
  }

  if (COMMS) {
    Serial.print("Timeout. Connected ");
    Serial.print(connectedSensorCount);
    Serial.print(" of ");
    Serial.println(targetSensorCount);
  }
  return false;
}

// --------------------------------------------------
// Sort Sensors - ensures Muscle Sensors detected in
// arbitrary order will maintain "identity" on reset, so
// 'turn left' is not reassigned on reset (unless sensors change)
// --------------------------------------------------
void sortSensors() {
  for (int i = 0; i < targetSensorCount - 1; i++) {
    for (int j = i + 1; j < targetSensorCount; j++) {
      if (String(sensors[i].client->getPeerAddress().toString().c_str()) > String(sensors[j].client->getPeerAddress().toString().c_str())) {
        SensorBLE temp = sensors[i];
        sensors[i] = sensors[j];
        sensors[j] = temp;

        //swap callback assignments to keep them aligned with sensors[] after sort
        NotifyCallback tempCB = notifyCallbacks[i];
        notifyCallbacks[i] = notifyCallbacks[j];
        notifyCallbacks[j] = tempCB;
      }
    }
  }
}

// --------------------------------------------------
// Buffer calculation fxns
// --------------------------------------------------
void updateBuffer(uint16_t* buffer, uint16_t value) {
  //Shift left
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
  //Insert newest at the end
  buffer[BUFFER_SIZE - 1] = value;
}

float computeAverageDerivative(uint16_t* buffer) {
  int32_t sum = 0;
  for (int i = 1; i < BUFFER_SIZE; i++) {
    sum += (int32_t)buffer[i] - (int32_t)buffer[i - 1];
  }
  return (float)sum / (BUFFER_SIZE - 1);
}

float computeSlope(uint16_t* buffer) {
  const int N = BUFFER_SIZE;
  float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  for (int i = 0; i < N; i++) {
    sumX += i;
    sumY += buffer[i];
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
void setSensitivityBySensor(uint16_t sensor, uint16_t value) {
  sensorThresholds[sensor] = (sensorAverages[sensor] / SIZE_OF_AVE) + value;
}

// --------------------------------------------------
// Calibrate sensors to baseline
// --------------------------------------------------
void calibrateThreshold() {

  uint8_t orangeLED = 12;
  uint8_t greenLED = 5;

  sensors[0].digitalChar->writeValue(&orangeLED, 1);
  sensors[1].digitalChar->writeValue(&orangeLED, 1);
  if(targetSensorCount > 2){
    sensors[2].digitalChar->writeValue(&orangeLED, 1);
    sensors[3].digitalChar->writeValue(&orangeLED, 1);
  }

  if (COMMS) Serial.println("Starting Calibration");

  sensorAverages[0] = 0;
  sensorAverages[1] = 0;
  if(targetSensorCount > 2){
    sensorAverages[2] = 0;
    sensorAverages[3] = 0;
  }

  int samplesCollected[targetSensorCount] = { 0 };
  while (true) {
    bool allDone = true;
    for (int i = 0; i < targetSensorCount; i++) {
      if (samplesCollected[i] < SIZE_OF_AVE) {
        allDone = false;
        if (newValueReady[i]) {
          sensorAverages[i] += latestAnalogValues[i];
          newValueReady[i] = false;
          samplesCollected[i]++;
        }
      }
    }
    if (allDone) break;
    delay(5);  //yield to allow BLE stack to deliver notifications
  }

  setSensitivityBySensor(0, sensitivityLevels[1]);
  setSensitivityBySensor(1, sensitivityLevels[1]);
  sensors[0].digitalChar->writeValue(&greenLED, 1);
  sensors[1].digitalChar->writeValue(&greenLED, 1);
  if(targetSensorCount > 2)
  {
    setSensitivityBySensor(2, sensitivityLevels[1]);
    setSensitivityBySensor(3, sensitivityLevels[1]);
    sensors[2].digitalChar->writeValue(&greenLED, 1);
    sensors[3].digitalChar->writeValue(&greenLED, 1);
  }
  ///absljsbdlkajsbdkl
}


// --------------------------------------------------
// Enter Wifi OTA
// --------------------------------------------------
void enterWifiOTA() {

  NimBLEDevice::deinit(true);
  delay(1000);

  if (COMMS) Serial.println("Entering WiFi OTA");

  WiFi.begin(ssid, password);
  delay(1000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (COMMS) Serial.println("Connecting to WiFi...");
  }
  if (COMMS) Serial.println("Connected to WiFi");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  //U_SPIFFS
      type = "filesystem";
    }
    if (COMMS) Serial.println("Start updating " + type);
    Serial.print('#');  //Send signal to ski that wifi update is starting
  });
  // ... (other OTA callbacks)

  ArduinoOTA.begin();
  ArduinoOTA.setPassword("test");
  if (COMMS) Serial.println("Ready");
  if (COMMS) Serial.print("IP address: ");
  if (COMMS) Serial.println(WiFi.localIP());

  Serial.print('@');  //Send signal to TetraSki that Wifi is connected

  delay(1000);

  while (1)
    ArduinoOTA.handle();
}

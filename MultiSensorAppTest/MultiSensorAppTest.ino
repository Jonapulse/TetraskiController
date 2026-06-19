#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "NimBLEDevice.h"
#include <Preferences.h>  

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

volatile bool sensorDisconnectFlagged = false;

#define RECONNECT_FREQ 10000

//Sensitivities
const uint8_t sensitivityLevels[3] = { 20, 50, 80 };
uint16_t sensorThresholds[MAX_SENSOR_COUNT];
uint8_t sensitivityIndices[MAX_SENSOR_COUNT] = { 1, 1, 1, 1 };  

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

class SensorClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    connectedSensorCount--;
    sensorDisconnectFlagged = true;
    if (COMMS) {
      Serial.print("Sensor disconnected: ");
      Serial.println(pClient->getPeerAddress().toString().c_str());
    }
  }
};
SensorClientCallbacks sensorClientCallbacks;


/************ NimBLE Phone Peripheral Stuff **************************************/

// UUIDs must match App.js constants exactly
#define PHONE_SERVICE_UUID      "12345678-1234-1234-1234-123456789abc"
#define SENSOR_DATA_CHAR_UUID   "12345678-1234-1234-1234-123456789abd"
#define BATTERY_DATA_CHAR_UUID  "12345678-1234-1234-1234-123456789abe"
#define COMMAND_CHAR_UUID       "12345678-1234-1234-1234-123456789abf"
#define CONFIG_CHAR_UUID        "12345678-1234-1234-1234-123456789ac0"  

NimBLEServer*         pPhoneServer       = nullptr;
NimBLECharacteristic* pSensorDataChar    = nullptr;  // NOTIFY  — 5 bytes: [dir, t2h, t2l, t3h, t3l]
NimBLECharacteristic* pBatteryDataChar   = nullptr;  // READ    — 2 bytes: [battT2, battT3]
NimBLECharacteristic* pCommandChar       = nullptr;  // WRITE   — 1 byte command
NimBLECharacteristic* pConfigChar        = nullptr;  // NOTIFY — 6 bytes: [sensorCount, inversionFlags, sens0, sens1, sens2, sens3]
bool phoneConnected = false;

// Forward declaration — handleCommand() is used inside CommandCallbacks::onWrite()
void handleCommand(char cmd);
void scanAndConnectSensors(bool isReconnect = false); //Arduino's pre-compiler was failing to auto-generate these when arguments were added
bool connectSensors(bool isReconnect = false);

// NimBLE server callbacks — track phone connect/disconnect
class PhoneServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {  
    phoneConnected = true;
    if (COMMS) Serial.println("Phone connected");

    // Send current settings to phone so app displays correct state on connect.
    // Byte layout: [sensorCount, inversionFlags, sens0, sens1, sens2, sens3]
    // inversionFlags bit 0 = pair 0 inverted (sensorOutputs[0]==2)
    //                bit 1 = pair 1 inverted (sensorOutputs[2]==4)
    uint8_t invFlags = 0;
    if (sensorOutputs[0] == 2) invFlags |= 0x01;
    if (sensorOutputs[2] == 4) invFlags |= 0x02;
    uint8_t configPayload[6] = {
      (uint8_t)targetSensorCount,
      invFlags,
      sensitivityIndices[0],
      sensitivityIndices[1],
      sensitivityIndices[2],
      sensitivityIndices[3]
    };
    pConfigChar->setValue(configPayload, 6);
    pConfigChar->notify();
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

  pConfigChar = pService->createCharacteristic(
    CONFIG_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  pService->start();

  // Configure and start advertising
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(PHONE_SERVICE_UUID);
  pAdvertising->setName("TetraRadio");
  NimBLEDevice::startAdvertising();

  if (COMMS) Serial.println("BLE advertising as TetraRadio");
}


/************ Persistent Save/Restore *****************************************/
// Stored in NVS under namespace "tetra" using the Preferences library.
// Keys:
//   "sensorCount"     - int,    targetSensorCount
//   "out0".."out3"    - int,    sensorOutputs[i]  (encodes per-pair inversion)
//   "thresh0".."th3"  - ushort, sensorThresholds[i]
//   "ave0".."ave3"    - ushort, sensorAverages[i]
//   "mac0".."mac3"    - String, MAC address of sensor[i] at time of save
//
// Slots 2–3 are never overwritten while in 2-sensor mode, so their data
// is preserved in case the user switches back to 4-sensor mode.

Preferences prefs;

void saveSettings() {
  prefs.begin("tetra", false);  // false = read/write

  prefs.putInt("sensorCount", targetSensorCount);

  for (int i = 0; i < targetSensorCount; i++) {
    char key[8];

    snprintf(key, sizeof(key), "out%d", i);
    prefs.putInt(key, sensorOutputs[i]);

    snprintf(key, sizeof(key), "thresh%d", i);
    prefs.putUShort(key, sensorThresholds[i]);

    snprintf(key, sizeof(key), "sensIdx%d", i);  
    prefs.putUChar(key, sensitivityIndices[i]);

    snprintf(key, sizeof(key), "ave%d", i);
    prefs.putUShort(key, sensorAverages[i]);

    snprintf(key, sizeof(key), "mac%d", i);
    if (sensors[i].client) {
      prefs.putString(key, sensors[i].client->getPeerAddress().toString().c_str());
    }
  }
  // Slots beyond targetSensorCount are intentionally left untouched.

  prefs.end();
  if (COMMS) Serial.println("Settings saved.");
}

// Returns true if every sensor slot relevant to the current targetSensorCount
// matches a saved MAC. On true, restores thresholds, averages, and outputs
// for those slots so calibration can be skipped.
// NOTE: Save stores data for up to 4 sensors. For 2 sensor setting, higher sensor 
// data is not saved or read but remains present.
bool loadAndMatchSettings() {
  prefs.begin("tetra", true);  // true = read-only

  // Restore sensor count and per-pair inversion for all 4 slots regardless of
  // current mode, so outputs are always consistent with last save.
  int savedCount = prefs.getInt("sensorCount", -1);
  if (savedCount == -1) {
    // No save exists yet
    prefs.end();
    if (COMMS) Serial.println("No save found — calibrating fresh.");
    return false;
  }

  // Restore sensorOutputs for all 4 slots (safe to always do this)
  for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
    char key[8];
    snprintf(key, sizeof(key), "out%d", i);
    sensorOutputs[i] = prefs.getInt(key, sensorOutputs[i]);
  }

  // Check MACs for slots relevant to current mode only
  bool match = true;
  for (int i = 0; i < targetSensorCount; i++) {
    char key[8];
    snprintf(key, sizeof(key), "mac%d", i);
    String savedMAC = prefs.getString(key, "");
    String currentMAC = sensors[i].client
      ? String(sensors[i].client->getPeerAddress().toString().c_str())
      : String("");

    if (savedMAC == "" || savedMAC != currentMAC) {
      if (COMMS) {
        Serial.print("MAC mismatch on slot ");
        Serial.print(i);
        Serial.print(": saved=");
        Serial.print(savedMAC);
        Serial.print(" current=");
        Serial.println(currentMAC);
      }
      match = false;
      break;
    }
  }

  if (match) {
    // Restore thresholds and averages for current mode's slots
    for (int i = 0; i < targetSensorCount; i++) {
      char key[8];
      snprintf(key, sizeof(key), "thresh%d", i);
      sensorThresholds[i] = prefs.getUShort(key, 0);
      snprintf(key, sizeof(key), "sensIdx%d", i);  
      sensitivityIndices[i] = prefs.getUChar(key, 1);
      snprintf(key, sizeof(key), "ave%d", i);
      sensorAverages[i] = prefs.getUShort(key, 0);
    }
    if (COMMS) Serial.println("Sensors match save — skipping calibration.");
  } else {
    if (COMMS) Serial.println("Sensors do not match save — calibrating fresh.");
  }

  prefs.end();
  return match;
}


void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  NimBLEDevice::init("TetraRadio");

  setupPhonePeripheral();

  prefs.begin("tetra", true);
  targetSensorCount = prefs.getInt("sensorCount", MAX_SENSOR_COUNT);
  prefs.end();

  scanAndConnectSensors();
}

void scanAndConnectSensors(bool isReconnect)
{
  while (1) {  //Enter connection loop

    //attempt to connect again until connection established
    if (connectSensors(isReconnect)) {
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

  //check if connected sensors match save; skip calibration if they do
  if (!loadAndMatchSettings()) {
    calibrateThreshold();
    saveSettings();  // save fresh calibration
  }

  sensorDisconnectFlagged = false;  // clear after successful reconnect, not before — avoids missing a drop that occurs during the reconnect attempt
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

  readAndPrintSensors();

  // Checked here so reconnect happens from loop() rather than the BLE stack thread.
  if (sensorDisconnectFlagged) {
    if (COMMS) Serial.println("Sensor drop detected — reconnecting...");
    scanAndConnectSensors(true);  // isReconnect=true: skips sort, uses saved MAC slots
  }
}

// --------------------------------------------------
// readAndPrintSensors -
// Reads sensor pair (or pairs for 4 sensor setting) and prints output
// For convenience, also periodically (set to 5s) checks battery charge and updates 
// sensor LEDs.
// 
// NOTE: slope calculation was added to give expected output in the common experience where
// the user has activated one sensor and changing to the other sensor. Commonly, both sensors
// will be active during this transition, but measuring the slope will let us respond with the
// sensor the user is moving towards MUCH more quickly than waiting for the sensor they are 
// moving away from to fully deactivate.
// --------------------------------------------------
void readAndPrintSensors()
{
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
      // Payload layout (10 bytes):
      //   [0]     direction pair 0   (0=idle, 1=left, 2=right)
      //   [1-2]   valA (sensor 0) big-endian
      //   [3-4]   valB (sensor 1) big-endian
      //   [5]     direction pair 1   (0=idle, 3=wedge in, 4=wedge out) — 0 in 2-sensor mode
      //   [6-7]   valA (sensor 2) big-endian — 0 in 2-sensor mode
      //   [8-9]   valB (sensor 3) big-endian — 0 in 2-sensor mode
      if (phoneConnected && i == 0) {
        // Preserve whatever pair 1 last wrote into bytes 5-9 instead of
        // zeroing them — otherwise every pair-0 update stomps pair-1 data.
        uint8_t payload[10] = { 0 };
        std::string existing = pSensorDataChar->getValue();
        if (existing.length() == 10) {
          memcpy(payload + 5, existing.data() + 5, 5);
        }
        payload[0] = (uint8_t)currentDirection;
        payload[1] = (uint8_t)(valA >> 8);
        payload[2] = (uint8_t)(valA & 0xFF);
        payload[3] = (uint8_t)(valB >> 8);
        payload[4] = (uint8_t)(valB & 0xFF);
        pSensorDataChar->setValue(payload, 10);
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

          //update sensor LED color based on battery level after each periodic read
          // >66% -> 6 (green-green), >33% -> 7 (green-red), else -> 11 (red-red)
          uint8_t battLevels[2] = { battA, battB };
          for (int s = 0; s < 2; s++) {
            uint8_t ledCode;
            if (battLevels[s] > 66)     ledCode = 6;
            else if (battLevels[s] > 33) ledCode = 7;
            else                         ledCode = 11;
            sensors[s].digitalChar->writeValue(&ledCode, 1);
          }
        }
      }
      if (phoneConnected && i == 2) {
        // Patch pair 1 data into bytes 5-9 and notify directly — wedge-only
        // updates must not wait for the next pair-0 cycle to reach the phone.
        std::string current = pSensorDataChar->getValue();
        if (current.length() == 10) {
          uint8_t payload[10];
          memcpy(payload, current.data(), 10);
          payload[5] = (uint8_t)currentDirection;
          payload[6] = (uint8_t)(valA >> 8);
          payload[7] = (uint8_t)(valA & 0xFF);
          payload[8] = (uint8_t)(valB >> 8);
          payload[9] = (uint8_t)(valB & 0xFF);
          pSensorDataChar->setValue(payload, 10);
          pSensorDataChar->notify();
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
      sensitivityIndices[0] = cmd - '0';  
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 0 set to ");
        Serial.println(sensitivityLevels[cmd - '0']);
      }
      Serial.print('c');
      saveSettings();  
      break;

    case '3':
      sensorOutputs[0] = 1;
      sensorOutputs[1] = 2;
      Serial.print('c');
      saveSettings(); 
      break;

    case '4':
      sensorOutputs[0] = 2;
      sensorOutputs[1] = 1;
      Serial.print('c');
      saveSettings(); 
      break;

    case '5':
      Serial.print('c');
      calibrateThreshold();
      saveSettings();
      Serial.print('f');
      break;

    case '6':
    case '7':
    case '8':
      setSensitivityBySensor(1, sensitivityLevels[cmd - '6']);
      sensitivityIndices[1] = cmd - '6'; 
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 1 set to ");
        Serial.println(sensitivityLevels[cmd - '6']);
      }
      Serial.print('c');
      saveSettings();  
      break;

    case 'g':
      sensorOutputs[2] = 3;
      sensorOutputs[3] = 4;
      Serial.print('c');
      saveSettings();  
      break;

    case 'h':
      sensorOutputs[2] = 4;
      sensorOutputs[3] = 3;
      Serial.print('c');
      saveSettings();  
      break;

    case 'i':
    case 'j':
    case 'k':
      setSensitivityBySensor(2, sensitivityLevels[cmd - 'i']);
      sensitivityIndices[2] = cmd - 'i';  
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 2 set to ");
        Serial.println(sensitivityLevels[cmd - 'i']);
      }
      Serial.print('c');
      saveSettings();
      break;

    case 'l':
    case 'm':
    case 'n':
      setSensitivityBySensor(3, sensitivityLevels[cmd - 'l']);
      sensitivityIndices[3] = cmd - 'l'; 
      if (COMMS) {
        Serial.print("\nSensitivity for sensor 3 set to ");
        Serial.println(sensitivityLevels[cmd - 'l']);
      }
      Serial.print('c');
      saveSettings();  
      break;

    case 'o':
      targetSensorCount = 2;
      if(connectedSensorCount > 2)
        connectedSensorCount = 2;
      saveSettings(); 
      break;

    case 'p':
      targetSensorCount = 4;
      saveSettings(); 
      if(connectedSensorCount < targetSensorCount)
        scanAndConnectSensors();
      break;
  }
}

// --------------------------------------------------
// Connect to N sensors matching target local name
// isReconnect: true when recovering from a drop — skips sortSensors() since
// sensors are placed directly into their saved MAC slots.
// --------------------------------------------------
bool connectSensors(bool isReconnect) {
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

        String foundMAC = String(device->getAddress().toString().c_str());

        // Skip sensors that are already connected — avoids duplicate clients on reconnect
        bool alreadyConnected = false;
        for (int s = 0; s < targetSensorCount; s++) {
          if (sensors[s].client && sensors[s].client->isConnected() &&
              String(sensors[s].client->getPeerAddress().toString().c_str()) == foundMAC) {
            if (COMMS) Serial.println("Already connected, skipping");
            alreadyConnected = true;
            break;
          }
        }
        if (alreadyConnected) continue;

        // Place sensor into its saved MAC slot if one matches, otherwise use next open slot.
        int targetSlot = connectedSensorCount;
        for (int s = 0; s < targetSensorCount; s++) {
          if (!sensors[s].client || !sensors[s].client->isConnected()) {
            // Check if saved MAC for this slot matches
            prefs.begin("tetra", true);
            char key[8];
            snprintf(key, sizeof(key), "mac%d", s);
            String savedMAC = prefs.getString(key, "");
            prefs.end();
            if (savedMAC == foundMAC) {
              targetSlot = s;
              if (COMMS) {
                Serial.print("Matched to saved slot ");
                Serial.println(s);
              }
              break;
            }
          }
        }

        SensorBLE& sensor = sensors[targetSlot];

        sensor.client = NimBLEDevice::createClient();
        if (!sensor.client) {
          if (COMMS) Serial.println("Client creation failed, skipping");
          continue;
        }

        sensor.client->setClientCallbacks(&sensorClientCallbacks, false);

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
        if (!sensor.analogChar->subscribe(true, notifyCallbacks[targetSlot])) {
          if (COMMS) Serial.println("Notification subscription failed, skipping");
          sensor.client->disconnect();
          continue;
        }

        if (COMMS) {
          Serial.print("Sensor ");
          Serial.print(targetSlot);
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
    // Only sort on initial connection. On reconnect, sensors are placed directly
    // into their saved MAC slots in connectSensors(), so sort would scramble callbacks.
    if (!isReconnect) {
      sortSensors();
    }
    //if (COMMS) debugPrintSensors();
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

// void debugPrintSensors()
// {
//   Serial.print("sensors for...");
//   Serial.print(targetSensorCount);
//   for(int i = 0; i < targetSensorCount; i++)
//     Serial.print(sensors[i].client->getPeerAddress().toString().c_str());
// }

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

  for (int i = 0; i < targetSensorCount; i++) {
    sensors[i].digitalChar->writeValue(&orangeLED, 1);
    sensorAverages[i] = 0;
  }

  if (COMMS) Serial.println("Starting Calibration");

  int samplesCollected[MAX_SENSOR_COUNT] = { 0 };
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

  for (int i = 0; i < targetSensorCount; i++) {
    setSensitivityBySensor(i, sensitivityLevels[1]);
  }
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

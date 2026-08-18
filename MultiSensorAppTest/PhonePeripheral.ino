// PhonePeripheral.ino
// -------------------------------------------------------------------------
// Everything related to the radio's NimBLE PERIPHERAL role: the phone
// (running App.js) connects to this device as "TetraRadio" to view live
// sensor data/battery and send config commands.
//
// The radio's CENTRAL role (scanning/connecting to the muscle sensors)
// lives in MultiSensorAppTest.ino instead — see connectSensors(),
// scanAndConnectSensors(), and the notify callbacks there.
//
// handleCommand() ALSO lives in MultiSensorAppTest.ino, not here — it's the
// shared command handler for both serial (TetraSki) and phone commands, and
// serial commands must keep working even with no phone connected. This file
// only wraps the BLE-specific plumbing that calls it.
//
// TO DISABLE PHONE SUPPORT ENTIRELY: set ENABLE_PHONE_PERIPHERAL to 0 in
// MultiSensorAppTest.ino and remove/comment out this tab. The main file
// guards its own phone-related lines behind that same flag, so this tab is
// the only place with unguarded references to the globals defined below.
// -------------------------------------------------------------------------


/************ NimBLE Phone Peripheral Stuff **************************************/

// UUIDs must match App.js constants exactly
#define PHONE_SERVICE_UUID      "12345678-1234-1234-1234-123456789abc"
#define SENSOR_DATA_CHAR_UUID   "12345678-1234-1234-1234-123456789abd"
#define BATTERY_DATA_CHAR_UUID  "12345678-1234-1234-1234-123456789abe"
#define COMMAND_CHAR_UUID       "12345678-1234-1234-1234-123456789abf"
#define CONFIG_CHAR_UUID        "12345678-1234-1234-1234-123456789ac0"  

NimBLEServer*         pPhoneServer       = nullptr;
NimBLECharacteristic* pSensorDataChar    = nullptr;  // NOTIFY  — 5 bytes: [dir, t2h, t2l, t3h, t3l]
NimBLECharacteristic* pBatteryDataChar   = nullptr;  // READ    — 4 bytes: [batt0, batt1, batt2, batt3]
NimBLECharacteristic* pCommandChar       = nullptr;  // WRITE   — 1 byte command
NimBLECharacteristic* pConfigChar        = nullptr;  // NOTIFY — 6 bytes: [sensorCount, inversionFlags, sens0, sens1, sens2, sens3]
bool phoneConnected = false;

// Send current settings to phone so app displays correct state on connect.
// Byte layout: [sensorCount, inversionFlags, sens0, sens1, sens2, sens3]
// inversionFlags bit 0 = pair 0 inverted (sensorOutputs[0]==2)
//                bit 1 = pair 1 inverted (sensorOutputs[2]==4)
// NOTE: must only be called after the phone has subscribed to pConfigChar
// (i.e. from onSubscribe), not from onConnect — notify() called before the
// client writes the CCCD has no subscriber to deliver to and is silently dropped.
void sendConfigToPhone() {
  uint8_t invFlags = 0;
  if (sensorOutputs[0] == 2) invFlags |= 0x01;
  if (sensorOutputs[2] == 4) invFlags |= 0x02;
  uint8_t configPayload[6] = {
    (uint8_t)targetSensorCount,
    invFlags,
    sensitivityValues[0],
    sensitivityValues[1],
    sensitivityValues[2],
    sensitivityValues[3]
  };
  pConfigChar->setValue(configPayload, 6);
  pConfigChar->notify();
}

// NimBLE server callbacks — track phone connect/disconnect
class PhoneServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {  
    phoneConnected = true;
    if (COMMS) Serial.println("Phone connected");
    // Config packet is sent from ConfigCallbacks::onSubscribe() instead of here —
    // see note on sendConfigToPhone().
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override { 
    phoneConnected = false;
    if (COMMS) Serial.println("Phone disconnected — restarting advertising");
    NimBLEDevice::startAdvertising();  // auto-restart so phone can reconnect
  }
};

// commandQueue is defined in MultiSensorAppTest.ino.
extern QueueHandle_t commandQueue;

// NimBLE characteristic callbacks — handle incoming command writes from phone.
// Must never call handleCommand() directly: onWrite() runs on the NimBLE host
// task, and blocking commands (calibration) would deadlock the BLE stack.
// Enqueue non-blockingly instead and let loop() drain it on the main task.
class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {  
    std::string val = pChar->getValue();
    for (size_t i = 0; i < val.length(); i++) {
      char c = val[i];
      xQueueSend(commandQueue, &c, 0);
    }
  }
};

// NimBLE characteristic callbacks for the config characteristic — fires once
// the phone actually enables notifications (writes the CCCD), which is the
// earliest point we're guaranteed the notify() below will be delivered.
class ConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    if (subValue == 0) return;  // client unsubscribed — nothing to send
    if (COMMS) Serial.println("Phone subscribed to config — sending initial settings");
    sendConfigToPhone();
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

  // Config: NOTIFY — pushed once when phone subscribes, to sync app display state
  pConfigChar = pService->createCharacteristic(
    CONFIG_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );
  pConfigChar->setCallbacks(new ConfigCallbacks());

  pService->start();

  // Configure and start advertising
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(PHONE_SERVICE_UUID);
  pAdvertising->setName("TetraRadio");
  NimBLEDevice::startAdvertising();

  if (COMMS) Serial.println("BLE advertising as TetraRadio");
}

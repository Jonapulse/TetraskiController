#include "NimBLEDevice.h"

NimBLERemoteService* sensors[4];
int sensorCount = 0;

void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  NimBLEDevice::init("");

  NimBLEScan *pScan = NimBLEDevice::getScan();
  NimBLEScanResults results = pScan->getResults(10 * 1000);

  NimBLEUUID serviceUuid("ABCD");

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice *device = results.getDevice(i);

    Serial.print("Found: ");
    Serial.print(device->getAddress().toString().c_str());
    Serial.print(" | Name: ");
    Serial.println(device->getName().c_str());

    if (device->getName() == "ANR Corp M40") {
      NimBLEClient *pClient = NimBLEDevice::createClient();
      Serial.print("Attempting with name match...");

      if (!pClient) {  // Make sure the client was created
        break;
      }

      if (pClient->connect(&device)) {
        NimBLERemoteService *pService = pClient->getService(serviceUuid);
        Serial.print("Connect seems to have worked");

        if (pService != nullptr) {
          NimBLERemoteCharacteristic *pCharacteristic = pService->getCharacteristic("1234");

          if (pCharacteristic != nullptr) {
            std::string value = pCharacteristic->readValue();
            // print or do whatever you need with the value
          }
        }

        sensors[sensorCount] = pService; 
        sensorCount++;
      } else {
        // failed to connect
        Serial.print("Connect failed");
      }

      NimBLEDevice::deleteClient(pClient);
    }
  }
  Serial.print("Setup complete");
  Serial.print("Sensors: ");
  Serial.print(sensorCount);
}

void loop() {
  //Debug comms through Serial Monitor
  // if(Serial.available()) {

  //   //read incoming byte from TetraSki
  //   char incomingByte = Serial.read();
  //   if(incomingByte = 't')
  //   {
  //     for(int i = 0; i < sensorCount; i++)
  //     {
  //       Serial.print(i);
  //     }
  //   }
  // }
}

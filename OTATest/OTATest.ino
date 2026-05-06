#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoBLE.h>

//Serial output for development/debugging. TURN OFF FOR TETRASKI USE
#define COMMS 1

/************ WiFi OTA Stuff **************************************************/
const char* ssid = "TetraOTA";
const char* password = "tetra2034";
bool OTAUpdateEnable = 0;


/************ BLE Sensor Stuff ************************************************/
const char* targetLocalName  = "ANR Corp M40"; // Match any device with the name for Muscle Sense Model M40
const int   targetSensorCount = 2;                 // How many sensors to connect to

const char* battServiceUUID   = "180f";
const char* battCharUUID      = "2A19";
const char* AutoIOServiceUUID = "1815";
const char* AnalogCharUUID    = "2A58";
const char* DigitalCharUUID   = "2A56";

struct SensorBLE {
  BLEDevice peripheral;
  BLEService batteryService;
  BLEService ioService;
  BLECharacteristic batteryChar;
  BLECharacteristic analogChar;
  BLECharacteristic digitalChar;
};

SensorBLE sensors[2];  // Array to hold connected sensors
int connectedSensorCount = 0;

#define RECONNECT_FREQ 10000

const uint8_t sensitivityLevels[3] = {20, 50, 80};
uint16_t t2Threshold = 0;
uint16_t t3Threshold = 0;

int direction1 = 1;
int direction2 = 2;
int idle       = 0;

uint16_t t2Ave = 0;
uint16_t t3Ave = 0;
const int sizeOfAve = 200;

#define BUFFER_SIZE 20 //data transmission @ 10 Hz for 2 sec
uint16_t t2Buffer[BUFFER_SIZE];
uint16_t t3Buffer[BUFFER_SIZE];
uint16_t bufferIndex  = 0;
bool     bufferFilled = false;
bool     invertDir = false;


void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);

  if (!BLE.begin()) {
    //NOTE: esp32 3.3.7 breaks BLE. 3.3.6 by Espressif works - as of 2/18/26
    Serial.println("BLE INIT FAILED");
    while(1);
  }

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


void loop() {

  //check for incoming comms from TetraSki
  if(Serial.available()) {

    //read incoming byte from TetraSki
    char incomingByte = Serial.read();

    switch(incomingByte) {
      //Signal from ski to enter wifi pairing
      case 'w':
        BLE.stopAdvertise(); //Stop advertising first
        if (BLE.connected()) { //disconnect BLE if connected
          BLE.disconnect();
        }
        delay(1000); //delay to allow disconnect
        enterWifiOTA();

      //Signal from ski to change sensitivity for  (*currently preset levels) for sensor 0
      case '0':
      case '1':
      case '2':
        //setBothSensitivitiesSensitivity(sensitivityLevels[incomingByte-'0'], sensitivityLevels[incomingByte-'0']);
        setSensitivityBySensor(0, sensitivityLevels[incomingByte-'0']);
        if(COMMS){
          Serial.println();
          Serial.print("\nSensitivity for sensor 0 set to ");
          Serial.print(sensitivityLevels[incomingByte-'0']);
          Serial.print(" with a threshold of ");
          Serial.println(t2Threshold);
        }
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      //Signal from ski for inverting direction
      case '3':
        direction1 = 1;
        direction2 = 2;
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      //Signal from ski for inverting direction
      case '4':
        direction1 = 2;
        direction2 = 1;
        Serial.print('c'); //send confirmation byte to TetraSki
        break;

      case '5':
        Serial.print('c'); //send confirmation byte to TetraSki
        calibrateThreshold();
        Serial.print('f'); //send confirmation byte to TetraSki
        break;

      //Signal from ski to change sensitivity for  (*currently preset levels) for sensor 1 (2nd sensor)
      case '6':
      case '7':
      case '8':
        setSensitivityBySensor(1, sensitivityLevels[incomingByte-'6']);
        if(COMMS){
          Serial.println();
          Serial.print("\nSensitivity for sensor 1 set to ");
          Serial.print(sensitivityLevels[incomingByte-'6']);
          Serial.print(" with a threshold of ");
          Serial.println(t3Threshold);
        }

        Serial.print('c');  //send confirmation byte to TetraSki
        break;

    }
  }

  uint16_t sensorValueT2;
  uint16_t sensorValueT3;

  if (sensors[0].analogChar.readValue(sensorValueT2) && sensors[1].analogChar.readValue(sensorValueT3)) {

    //update buffer
    updateBuffer(t2Buffer, sensorValueT2);
    updateBuffer(t3Buffer, sensorValueT3);
    static int fillCount = 0;
    if (fillCount < BUFFER_SIZE) fillCount++;
    if (fillCount == BUFFER_SIZE) bufferFilled = true;

    //calculate changes in signals
    float slope2 = computeSlope(t2Buffer);
    float slope3 = computeSlope(t3Buffer);

    //turning logic
    if (sensorValueT2 > t2Threshold && sensorValueT3 < t3Threshold) {
      Serial.print(direction1);
    }
    else if (sensorValueT2 < t2Threshold && sensorValueT3 > t3Threshold) {
      Serial.print(direction2);
    }
    else if (sensorValueT2 > t2Threshold && sensorValueT3 > t3Threshold) {
      if (slope2 > 0.1 && slope3 < 0.1){
        Serial.print(direction1);
      }
      else if (slope2 < 0.1 && slope3 > 0.1) {
        Serial.print(direction2);
      }
    }
    else {
      Serial.print(idle);
    }
  }
}


// --------------------------------------------------
// Connect to N sensors matching target local name
// --------------------------------------------------
bool connectSensors() {
  connectedSensorCount = 0;

  if(COMMS) Serial.println("Scanning for sensors...");
  BLE.scan();

  long scanStart = millis();

  while (millis() - scanStart < RECONNECT_FREQ && connectedSensorCount < targetSensorCount) {

    BLEDevice peripheral = BLE.available();

    if (peripheral) {
      String name = peripheral.localName();
      if(COMMS) {
        Serial.print("Found: ");
        Serial.print(peripheral.address());
        Serial.print(" | Name: ");
        Serial.println(name);
      }

      if (name == targetLocalName) {
        if(COMMS) {
          Serial.print("Target found: ");
          Serial.println(peripheral.address());
        }

        BLE.stopScan();

        SensorBLE &sensor = sensors[connectedSensorCount];
        sensor.peripheral = peripheral;

        if (!sensor.peripheral.connect()) {
          if(COMMS) Serial.println("Connection failed, skipping"); 
          BLE.scan(); 
          continue;   
        }

        if (!sensor.peripheral.discoverAttributes()) {
          if(COMMS) Serial.println("Attribute discovery failed, skipping"); 
          sensor.peripheral.disconnect(); 
          BLE.scan(); 
          continue; 
        }

        sensor.batteryService = sensor.peripheral.service(battServiceUUID);
        if (!sensor.batteryService) {
          if(COMMS) Serial.println("Battery service not found, skipping"); 
          sensor.peripheral.disconnect(); 
          BLE.scan(); 
          continue;   
        }

        sensor.batteryChar = sensor.batteryService.characteristic(battCharUUID);
        if (!sensor.batteryChar || !sensor.batteryChar.canRead()) {
          if(COMMS) Serial.println("Battery characteristic not found, skipping"); 
          sensor.peripheral.disconnect(); 
          BLE.scan(); 
          continue;   
        }

        sensor.ioService = sensor.peripheral.service(AutoIOServiceUUID);
        if (!sensor.ioService) {
          if(COMMS) Serial.println("IO service not found, skipping"); 
          sensor.peripheral.disconnect(); 
          BLE.scan(); 
          continue;   
        }

        sensor.analogChar = sensor.ioService.characteristic(AnalogCharUUID);
        if (!sensor.analogChar || !sensor.analogChar.canRead()) {
          if(COMMS) Serial.println("Analog characteristic not found, skipping"); 
          sensor.peripheral.disconnect(); 
          BLE.scan(); 
          continue;   
        }

        sensor.digitalChar = sensor.ioService.characteristic(DigitalCharUUID);
        if (!sensor.digitalChar || !sensor.digitalChar.canWrite()) {
          if(COMMS) Serial.println("Digital characteristic not found, skipping"); 
          sensor.peripheral.disconnect(); 
          BLE.scan(); 
          continue; 
        }

        connectedSensorCount++;
        BLE.scan();  // Resume scanning for next sensor
      }
    }
  }

  BLE.stopScan();

  if (connectedSensorCount == targetSensorCount) {
    if(COMMS) Serial.println("All sensors connected!");
    return true;
  }

  if(COMMS) {
    Serial.print("Timeout. Connected ");
    Serial.print(connectedSensorCount);
    Serial.print(" of ");
    Serial.println(targetSensorCount);
  }
  return false;
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


// --------------------------------------------------
// Set Sensitivity for both sensors
// --------------------------------------------------
void setBothSensitivities(uint16_t t2value, uint16_t t3value) {
  t2Threshold = (t2Ave / sizeOfAve) + t2value;
  t3Threshold = (t3Ave / sizeOfAve) + t3value;
}

// --------------------------------------------------
// Set Sensitivity by sensor
// --------------------------------------------------
void setSensitivityBySensor(uint16_t sensor, uint16_t value)
{
  switch(sensor){
    case(0):
      t2Threshold = (t2Ave / sizeOfAve) + value;
      break;
    case(1):
      t3Threshold = (t3Ave / sizeOfAve) + value;
      break;
  }
}


// --------------------------------------------------
// Calibrate sensors to baseline
// --------------------------------------------------
void calibrateThreshold() {

  uint8_t orangeLED = 12;
  uint8_t greenLED  = 5;

  sensors[0].digitalChar.writeValue(orangeLED);
  sensors[1].digitalChar.writeValue(orangeLED);

  if(COMMS) Serial.println("Starting Calibration");

  t2Ave = 0;  // Reset averages before accumulating
  t3Ave = 0;

  uint16_t valT2, valT3;
  for (int i = 0; i < sizeOfAve; i++) {
    sensors[0].analogChar.readValue(valT2);
    sensors[1].analogChar.readValue(valT3);
    t2Ave += valT2;
    t3Ave += valT3;
  }

  setBothSensitivities(sensitivityLevels[1], sensitivityLevels[1]);

  sensors[0].digitalChar.writeValue(greenLED);
  sensors[1].digitalChar.writeValue(greenLED);
}


// --------------------------------------------------
// Enter Wifi OTA
// --------------------------------------------------
void enterWifiOTA() {

  BLE.end();
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
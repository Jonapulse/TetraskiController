#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoBLE.h>

//Serial output for development/debugging. TURN OFF FOR TETRASKI USE
#define COMMS 1
#define SENSOR_COUNT 4 // Set to 2 or 4 depending on configuration

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

struct SensorBLE {
  BLEDevice peripheral;
  BLEService batteryService;
  BLEService ioService;
  BLECharacteristic batteryChar;
  BLECharacteristic analogChar;
  BLECharacteristic digitalChar;
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

        //Signal from ski to change sensitivity for  (*currently preset levels) for sensor[2]
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
    if (sensors[i].analogChar.readValue(sensorValues[i]) && sensors[i + 1].analogChar.readValue(sensorValues[i + 1])) {

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
bool connectSensors() {
  connectedSensorCount = 0;

  if(COMMS) Serial.println("Scanning for sensors...");
  BLE.scan();

  long scanStart = millis();

  while (millis() - scanStart < RECONNECT_FREQ && connectedSensorCount < SENSOR_COUNT) {

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
void sortSensors()
{
  for (int i = 0; i < SENSOR_COUNT - 1; i++) {
    for (int j = i + 1; j < SENSOR_COUNT; j++) {
      if (String(sensors[i].peripheral.address()) > String(sensors[j].peripheral.address())) {
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
void calibrateThreshold() {

  uint8_t orangeLED = 12;
  uint8_t greenLED  = 5;

  sensors[0].digitalChar.writeValue(orangeLED);
  sensors[1].digitalChar.writeValue(orangeLED);
#if SENSOR_COUNT == 4
  sensors[2].digitalChar.writeValue(orangeLED);
  sensors[3].digitalChar.writeValue(orangeLED);
#endif

  if(COMMS) Serial.println("Starting Calibration");

  sensorAverages[0] = 0;  // Reset averages before accumulating
  sensorAverages[1] = 0;
#if SENSOR_COUNT == 4
  sensorAverages[2] = 0;
  sensorAverages[3] = 0;
#endif

  uint16_t valT2, valT3, valT4, valT5;
  for (int i = 0; i < SIZE_OF_AVE; i++) {
    sensors[0].analogChar.readValue(valT2);
    sensors[1].analogChar.readValue(valT3);
    sensorAverages[0] += valT2;
    sensorAverages[1] += valT3;
#if SENSOR_COUNT == 4
    sensors[2].analogChar.readValue(valT4);
    sensors[3].analogChar.readValue(valT5);
    sensorAverages[2] += valT4;
    sensorAverages[3] += valT5;
#endif
  }

  setSensitivityBySensor(0, sensitivityLevels[1]);
  setSensitivityBySensor(1, sensitivityLevels[1]);
  sensors[0].digitalChar.writeValue(greenLED);
  sensors[1].digitalChar.writeValue(greenLED);
#if SENSOR_COUNT == 4
  setSensitivityBySensor(2, sensitivityLevels[1]);
  setSensitivityBySensor(3, sensitivityLevels[1]);
  sensors[2].digitalChar.writeValue(greenLED);
  sensors[3].digitalChar.writeValue(greenLED);
#endif
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
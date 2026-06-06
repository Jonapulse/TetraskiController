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
//MAC address for TS029 Sensors
//const char* targetMAC_T2 = "68:23:b0:b6:b3:e9";
//const char* targetMAC_T3 = "68:23:b0:b6:c8:44";

//MAC address for Ross Test sensors
const char* targetMAC_T2 = "84:72:93:a5:02:8e";
const char* targetMAC_T3 = "90:7b:c6:8e:a1:17";

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

SensorBLE sensorT2;
SensorBLE sensorT3;

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


/************ BLE Phone Peripheral Stuff **************************************/
// Custom service the phone connects to.
// Using a randomly generated 128-bit UUID — replace with your own if desired.
BLEService phoneService("12345678-1234-1234-1234-123456789abc");

// Sensor data out: direction byte + raw T2 + raw T3 packed into 5 bytes
// [direction(1)] [t2_high(1)] [t2_low(1)] [t3_high(1)] [t3_low(1)]
// Properties: NOTIFY so phone receives updates without polling
BLECharacteristic sensorDataChar("12345678-1234-1234-1234-123456789abd", BLENotify, 5);

// Battery levels out: [t2_batt(1)] [t3_batt(1)]
// Properties: READ
BLECharacteristic batteryDataChar("12345678-1234-1234-1234-123456789abe", BLERead, 2);

// Command in: receives single command bytes from phone matching the existing serial protocol
// '0','1','2' = sensitivity | '3','4' = direction | '5' = recalibrate
// Properties: WRITE
BLECharacteristic commandChar("12345678-1234-1234-1234-123456789abf", BLEWrite, 1);

bool phoneConnected = false;



void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);


  BLE.begin();

  // Set up the peripheral role (advertising to phone) before scanning for sensors.
  // ArduinoBLE supports simultaneous central + peripheral on the Nano 33 IoT / Nano 33 BLE.
  BLE.setLocalName("TetraRadio");
  BLE.setAdvertisedService(phoneService);
  phoneService.addCharacteristic(sensorDataChar);
  phoneService.addCharacteristic(batteryDataChar);
  phoneService.addCharacteristic(commandChar);
  BLE.addService(phoneService);
  BLE.advertise();
  if(COMMS) Serial.println("BLE advertising as TetraRadio");


  // Connect sensors
  while(1) {    //Enter connection loop

    //Poll the peripheral role so the phone can see us during sensor connection
    BLE.poll();

    //attempt to connect again until connection established
    if(connectSensor(targetMAC_T3, sensorT3) && connectSensor(targetMAC_T2, sensorT2)) {
      Serial.print('$');  //Send signal to ski signaling successful connection
      break;
    }

    //check for incoming byte from ski to trigger WiFi OTA
    if(Serial.available()) {
      if(Serial.read()=='w') {
        enterWifiOTA();  //signal from ski to enter wifi OTA
      }
    }
  }
  

  //perform initial calibration
  calibrateThreshold();
  
}




void loop() {

  // Poll BLE peripheral role — required to process phone connect/disconnect/write events
  BLE.poll();

  // Track phone connection state
  BLEDevice phone = BLE.central();
  if (phone) {
    if (!phoneConnected) {
      phoneConnected = true;
      if(COMMS) { Serial.print("Phone connected: "); Serial.println(phone.address()); }
    }

    // Handle incoming command from phone (mirrors the serial command protocol)
    if (commandChar.written()) {
      uint8_t cmd;
      commandChar.readValue(cmd);
      handleCommand((char)cmd);
    }

  } else {
    if (phoneConnected) {
      phoneConnected = false;
      if(COMMS) Serial.println("Phone disconnected");
    }
  }


  //check for incoming comms from TetraSki
  if(Serial.available()) {

    //read incoming byte from TetraSki
    char incomingByte = Serial.read();
    handleCommand(incomingByte);
  }


  uint16_t sensorValueT2;
  uint16_t sensorValueT3;

  if (sensorT2.analogChar.readValue(sensorValueT2) && sensorT3.analogChar.readValue(sensorValueT3)) {

    //update buffer
    updateBuffer(t2Buffer, sensorValueT2);
    updateBuffer(t3Buffer, sensorValueT3);
    static int fillCount = 0;
    if (fillCount < BUFFER_SIZE) fillCount++;
    if (fillCount == BUFFER_SIZE) bufferFilled = true;

    //calculate changes in signals
    float slope2 = computeSlope(t2Buffer);
    float slope3 = computeSlope(t3Buffer);

    //determine direction
    int currentDirection = idle;
    if (sensorValueT2 > t2Threshold && sensorValueT3 < t3Threshold) {
      currentDirection = direction1;
    }
    else if (sensorValueT2 < t2Threshold && sensorValueT3 > t3Threshold) {
      currentDirection = direction2;
    }
    else if (sensorValueT2 > t2Threshold && sensorValueT3 > t3Threshold) {
      if (slope2 > 0.1 && slope3 < 0.1){
        currentDirection = direction1;
      }
      else if (slope2 < 0.1 && slope3 > 0.1) { 
        currentDirection = direction2;
      }
    }

    //output direction over serial to TetraSki
    Serial.print(currentDirection);

    //broadcast sensor data to phone if connected
    if (phoneConnected) {
      uint8_t payload[5];
      payload[0] = (uint8_t)currentDirection;
      payload[1] = (uint8_t)(sensorValueT2 >> 8);
      payload[2] = (uint8_t)(sensorValueT2 & 0xFF);
      payload[3] = (uint8_t)(sensorValueT3 >> 8);
      payload[4] = (uint8_t)(sensorValueT3 & 0xFF);
      sensorDataChar.writeValue(payload, 5);

      // Read and broadcast battery levels periodically
      static unsigned long lastBattUpdate = 0;
      if (millis() - lastBattUpdate > 5000) {  // every 5 seconds
        lastBattUpdate = millis();
        uint8_t battT2 = 0, battT3 = 0;
        sensorT2.batteryChar.readValue(battT2);
        sensorT3.batteryChar.readValue(battT3);
        uint8_t battPayload[2] = {battT2, battT3};
        batteryDataChar.writeValue(battPayload, 2);
      }
    }
  }

  
}


// --------------------------------------------------
// Shared command handler (serial + BLE phone)
// --------------------------------------------------
void handleCommand(char cmd) {
  switch(cmd) {

    //Signal to enter wifi pairing
    case 'w':
      BLE.stopAdvertise();
      if (BLE.connected()) {
        BLE.disconnect();
      }
      delay(1000);
      enterWifiOTA();
      break;

    //Signal to change sensitivity (*currently preset levels)
    case '0':
    case '1':
    case '2':
      setSensitivity(sensitivityLevels[cmd-'0'], sensitivityLevels[cmd-'0']);
      if(COMMS) Serial.print("Sensitivity set to");
      if(COMMS) Serial.println(sensitivityLevels[cmd-'0']);
      Serial.print('c');  //send confirmation byte to TetraSki
      break;

    //Signal for normal direction
    case '3':
      direction1 = 1;
      direction2 = 2;
      Serial.print('c');
      break;

    //Signal for inverted direction
    case '4':
      direction1 = 2;
      direction2 = 1; 
      Serial.print('c');
      break;

    //Signal to reinitialize calibration
    case '5':
      Serial.print('c');
      calibrateThreshold();
      Serial.print('f');
      break;               
  }
}




// --------------------------------------------------
// Buffer calculation fxns
// --------------------------------------------------
void updateBuffer(uint16_t *buffer, uint16_t value) {
  // Shift left
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
  // Insert newest at the end
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
  float sumX  = 0;
  float sumY  = 0;
  float sumXY = 0;
  float sumXX = 0;
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
// Connect + discover sensors
// --------------------------------------------------
bool connectSensor(const char* targetMAC, SensorBLE &sensor) {
  BLE.scan();

  long connectionTimeout=millis();
  while (millis()-connectionTimeout < RECONNECT_FREQ) {

    BLE.poll(); // keep peripheral role alive during scan

    sensor.peripheral = BLE.available();

    if (sensor.peripheral && sensor.peripheral.address() == targetMAC) {
      BLE.stopScan();

      if (!sensor.peripheral.connect()) return false;
      if (!sensor.peripheral.discoverAttributes()) return false;

      sensor.batteryService = sensor.peripheral.service(battServiceUUID);
      if (!sensor.batteryService) return false;

      sensor.batteryChar = sensor.batteryService.characteristic(battCharUUID);
      if (!sensor.batteryChar || !sensor.batteryChar.canRead()) return false;

      sensor.ioService = sensor.peripheral.service(AutoIOServiceUUID);
      if (!sensor.ioService) return false;

      sensor.analogChar = sensor.ioService.characteristic(AnalogCharUUID);
      if (!sensor.analogChar || !sensor.analogChar.canRead()) return false;

      sensor.digitalChar = sensor.ioService.characteristic(DigitalCharUUID);
      if (!sensor.digitalChar || !sensor.digitalChar.canWrite()) return false;

      Serial.print(sensor.peripheral.address());
      if(COMMS) Serial.println(" Connected");
      return true;
    }
  }
  if(COMMS) Serial.println("Connection Timeout");
  return false;
}



// --------------------------------------------------
// Set Sensitivity for each sensor
// --------------------------------------------------
void setSensitivity(uint16_t t2value, uint16_t t3value) {
  t2Threshold = (t2Ave / sizeOfAve) + t2value;
  t3Threshold = (t3Ave / sizeOfAve) + t3value;
}


// --------------------------------------------------
// Calibrate sensors to baseline
// --------------------------------------------------
void calibrateThreshold() {

  uint8_t orangeLED = 12;
  uint8_t greenLED  = 5;

  sensorT2.digitalChar.writeValue(orangeLED);
  sensorT3.digitalChar.writeValue(orangeLED);

  if(COMMS) Serial.println("Starting Calibration");
  

  // Baseline averaging
  uint16_t valT2, valT3;

  for (int i = 0; i < sizeOfAve; i++) {
    sensorT2.analogChar.readValue(valT2);
    sensorT3.analogChar.readValue(valT3);
    t2Ave += valT2;
    t3Ave += valT3;
  }

  //set default sensitivity for both sensors
  setSensitivity(sensitivityLevels[0], sensitivityLevels[0]);

  sensorT2.digitalChar.writeValue(greenLED);
  sensorT3.digitalChar.writeValue(greenLED);
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
    } else { // U_SPIFFS
          type = "filesystem";
    }
    if(COMMS) Serial.println("Start updating " + type);
    Serial.print('#'); //Send signal to ski that wifi update is starting
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

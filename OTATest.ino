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
const char* targetMAC_T2 = "68:23:b0:b6:31:61";
const char* targetMAC_T3 = "68:23:b0:b7:18:e5";

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



void setup() {

  Serial.begin(57600);

  //power status LED
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);


  BLE.begin();

  // Connect sensors
  while(1) {    //Enter connection loop

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

  //check for incoming comms from TetraSki
  if(Serial.available()) {

    //read incoming byte from TetraSki
    char incomingByte = Serial.read();


    switch(incomingByte) {

      //Signal from ski to enter wifi pairing
      case 'w':
        BLE.stopAdvertise(); // Stop advertising first
        if (BLE.connected()) { //disconnect BLE if connected
          BLE.disconnect();
        }
        delay(1000); //delay to allow disconnect
        enterWifiOTA();

      //Signal from ski to change sensitivity (*currently preset levels)
      case '0':
      case '1':
      case '2':
        setSensitivity(sensitivityLevels[incomingByte-'0'], sensitivityLevels[incomingByte-'0']);
        if(COMMS) Serial.print("Sensitivity set to");
        if(COMMS) Serial.println(sensitivityLevels[incomingByte-'0']);

        Serial.print('c');  //send confirmation byte to TetraSki
        break;

      //FUTURE PER-SENSOR SENSITIVITY CONFIGURATION FOR TETRASKI UI
      // case '6':
      // case '7':
      // case '8':
      //   setSensitivity(sensitivityLevels[incomingByte-'0'], sensitivityLevels[incomingByte-'0']);
      //   if(COMMS) Serial.print("Sensitivity set to");
      //   if(COMMS) Serial.println(sensitivityLevels[incomingByte-'0']);

      //   Serial.print('c');  //send confirmation byte to TetraSki
      //   break;

      //Signal from ski for inverting direction
      case '3':
        direction1 = 1;
        direction2 = 2;
        Serial.print('c');  //send confirmation byte to TetraSki
        break;

      //Signal from ski for inverting direction
      case '4':
        direction1 = 2;
        direction2 = 1; 
        Serial.print('c');  //send confirmation byte to TetraSki        
        break;

      //Signal from ski to reinitialize calibration
      case '5':
        Serial.print('c');  //send confirmation byte to TetraSki  
        calibrateThreshold();
        Serial.print('f');  //send confirmation byte to TetraSki  
        break;               
    }
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
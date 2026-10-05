// SkiVersionTest.ino
// Teensy 4.0 bench sketch for the ski firmware update pathway.
//   Step 1: version query ('V' -> [lengthByte][ASCII version]).
//   Step 2: enter/exit update mode handshake.
//
// Onboard LED patterns:
//   1 s blink ............ normal mode, sketch alive
//   3 quick flashes ...... answered a version query
//   fast 5 Hz blink ...... in update mode
//   after leaving update mode:
//     2 quick flashes .... clean exit, nothing unexpected arrived in update mode
//     1 long (1 s) flash . exit OK, but stray bytes arrived in update mode
//                          (the radio did not fully stop its control output)
//     5 quick flashes .... idle timeout, the radio never sent EXIT
//
// Wiring: Serial2 (RX2 = pin 7, TX2 = pin 8) is the link to the radio
// controller. Baud matches the radio's Serial.begin(57600).

#define SKI_CMD_VERSION_QUERY  0x56
#define SKI_CMD_ENTER_UPDATE   0x01
#define SKI_CMD_EXIT_UPDATE    0x05
#define SKI_ACK                0x06
#define SKI_FIRMWARE_VERSION   "0.1.0-bench"

#define SKI_UPDATE_IDLE_TIMEOUT_MS 10000  // no bytes this long in update mode -> leave it

#define LED_PIN LED_BUILTIN
#define HEARTBEAT_INTERVAL_MS        1000
#define UPDATE_HEARTBEAT_INTERVAL_MS 100

unsigned long lastHeartbeatMs = 0;
bool heartbeatState = false;

bool updateMode = false;
unsigned long lastUpdateByteMs = 0;
uint32_t strayBytes = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial2.begin(57600);
}

void loop() {
  unsigned long interval = updateMode ? UPDATE_HEARTBEAT_INTERVAL_MS : HEARTBEAT_INTERVAL_MS;
  if (millis() - lastHeartbeatMs > interval) {
    lastHeartbeatMs = millis();
    heartbeatState = !heartbeatState;
    digitalWrite(LED_PIN, heartbeatState);
  }

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (updateMode) handleUpdateModeByte(b);
    else handleNormalByte(b);
  }

  if (updateMode && millis() - lastUpdateByteMs > SKI_UPDATE_IDLE_TIMEOUT_MS) {
    leaveUpdateMode(true);
  }
}

void handleNormalByte(uint8_t b) {
  if (b == SKI_CMD_VERSION_QUERY) {
    sendVersionResponse();
    flashConfirmation(3);
  } else if (b == SKI_CMD_ENTER_UPDATE) {
    updateMode = true;
    strayBytes = 0;
    lastUpdateByteMs = millis();
    Serial2.write(SKI_ACK);
  } else if (b == SKI_CMD_EXIT_UPDATE) {
    Serial2.write(SKI_ACK);  // already out of update mode, a repeated EXIT still gets an ACK
  }
  // Anything else is ski control traffic, ignored by this bench sketch.
}

void handleUpdateModeByte(uint8_t b) {
  lastUpdateByteMs = millis();
  if (b == SKI_CMD_ENTER_UPDATE) {
    Serial2.write(SKI_ACK);  // repeated ENTER, the first ACK may have been lost
  } else if (b == SKI_CMD_EXIT_UPDATE) {
    Serial2.write(SKI_ACK);  // ACK first, the indicator flashes below block for a while
    leaveUpdateMode(false);
  } else {
    strayBytes++;
  }
}

void leaveUpdateMode(bool timedOut) {
  updateMode = false;
  digitalWrite(LED_PIN, LOW);
  if (timedOut) {
    flashConfirmation(5);
  } else if (strayBytes > 0) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    lastHeartbeatMs = millis();
  } else {
    flashConfirmation(2);
  }
}

void sendVersionResponse() {
  uint8_t len = strlen(SKI_FIRMWARE_VERSION);
  Serial2.write(len);
  Serial2.write((const uint8_t*)SKI_FIRMWARE_VERSION, len);
}

void flashConfirmation(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  lastHeartbeatMs = millis();
}

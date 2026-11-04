// OTAService.ino
// BLE OTA firmware update receiver for TetraRadio (ESP32 / NimBLE)
//
// Provides a dedicated GATT service for receiving firmware images over BLE,
// verifying them, and applying them via the ESP32 Update library.
//
// The same chunk path also carries a ski controller image. That one is
// written to LittleFS instead of the OTA partition, and a separate command
// has the radio relay it to the ski (see SkiUpdateService.ino).

#include <NimBLEDevice.h>
#include <Update.h>
#include "esp32/rom/crc.h"   // crc32_le

// ---- UUIDs (placeholders — generate real ones before use) ----
#define OTA_SERVICE_UUID       "c076ed50-9e91-4566-9023-3cb1b9173244"
#define OTA_CONTROL_CHAR_UUID  "ed3c98b9-0a71-45e0-9b14-b89d3316549d"
#define OTA_DATA_CHAR_UUID     "ee191e3a-95c6-4bf1-92b8-1980c9e7b8e9"
#define OTA_STATUS_CHAR_UUID   "5c8b02e7-5520-468f-bcba-fcc5093da1c9"
#define OTA_VERSION_CHAR_UUID  "c82f2a3c-f48c-4cfa-b447-0277467898e4"
#define OTA_SKI_VERSION_CHAR_UUID "f937d204-2a79-423d-bddf-e8530aafe87b"

// ---- Tunables (most speedup rn on App.js end)
#define OTA_CHUNK_PAYLOAD_MAX  512
#define OTA_QUEUE_LEN          10      // backpressure depth before we flag an error
#define OTA_IDLE_TIMEOUT_MS    15000   // ms with no activity before auto-abort
#define OTA_SKI_VERSION_MAX    32      // longest ski version string a START may carry

// ---- State machine ----
enum OTAState {
  OTA_ST_IDLE,
  OTA_ST_RECEIVING,
  OTA_ST_VERIFYING,
  OTA_ST_APPLYING,
  OTA_ST_ERROR
};

static volatile OTAState otaState = OTA_ST_IDLE;
static volatile bool otaStartPending = false;  // START received on the BLE host task, Update.begin() not run yet
static volatile bool otaTargetSki = false;     // this transfer is a ski image for LittleFS, not radio firmware
static volatile bool otaSkiAbortPending = false;  // ABORT of a ski transfer, handled on the main task which owns the file
static char otaSkiVersion[OTA_SKI_VERSION_MAX + 1] = "";

struct OTAChunk {
  uint16_t seq;
  uint8_t  data[OTA_CHUNK_PAYLOAD_MAX];
  size_t   len;
};

static QueueHandle_t otaQueue = nullptr;

static uint32_t otaExpectedSize      = 0;
static uint32_t otaExpectedCRC       = 0;
static uint32_t otaBytesWritten      = 0;
static uint32_t otaRunningCRC        = 0;
static uint16_t otaExpectedSeq       = 0;
static unsigned long otaLastActivityMs = 0;

static NimBLECharacteristic* pOTAStatusChar = nullptr;
static NimBLECharacteristic* pSkiVersionChar = nullptr;

// ---- Status codes sent over the OTA Status characteristic ----
enum OTAStatusCode : uint8_t {
  OTA_STATUS_READY     = 0x01, // ack for START
  OTA_STATUS_PROGRESS  = 0x02, // followed by 4-byte bytesWritten (uint32 LE)
  OTA_STATUS_DONE       = 0x03, // update applied, about to reboot
  OTA_STATUS_SKI_STORED = 0x04, // ski image written to LittleFS and CRC verified, no reboot
  OTA_STATUS_SKI_RELAY_PROGRESS = 0x10, // followed by phase(1), bytesSent(uint32 LE), bytesTotal(uint32 LE)
  OTA_STATUS_SKI_RELAY_OK       = 0x11, // followed by the ski's new version (ASCII)
  OTA_STATUS_SKI_RELAY_FAIL     = 0x12, // followed by reason(1), skiNak(1), known(1), the ski's old version (ASCII)
  OTA_STATUS_ERR_SIZE   = 0xE0, // requested size too large for OTA partition
  OTA_STATUS_ERR_SEQ    = 0xE1, // out-of-order / dropped chunk
  OTA_STATUS_ERR_CRC    = 0xE2, // CRC mismatch after full transfer
  OTA_STATUS_ERR_WRITE  = 0xE3, // Update.write() failed, or malformed/oversized chunk
  OTA_STATUS_ERR_STATE  = 0xE4, // command not valid in current state
  OTA_STATUS_ERR_BEGIN  = 0xE5  // Update.begin() failed
};

static void sendOTAStatus(uint8_t code) {
  if (!pOTAStatusChar) return;
  pOTAStatusChar->setValue(&code, 1);
  pOTAStatusChar->notify();
}

static void sendOTAProgress() {
  if (!pOTAStatusChar) return;
  uint8_t payload[5];
  payload[0] = OTA_STATUS_PROGRESS;
  memcpy(&payload[1], &otaBytesWritten, 4);
  pOTAStatusChar->setValue(payload, 5);
  pOTAStatusChar->notify();
}

// ---- Ski relay reports, called from SkiUpdateService.ino ----

void sendSkiRelayProgress(uint8_t phase, uint32_t bytesSent, uint32_t bytesTotal) {
  if (!pOTAStatusChar) return;
  uint8_t payload[10];
  payload[0] = OTA_STATUS_SKI_RELAY_PROGRESS;
  payload[1] = phase;
  memcpy(&payload[2], &bytesSent, 4);
  memcpy(&payload[6], &bytesTotal, 4);
  pOTAStatusChar->setValue(payload, 10);
  pOTAStatusChar->notify();
}

void sendSkiRelayOk(const char* newVersion) {
  if (!pOTAStatusChar) return;
  uint8_t payload[1 + OTA_SKI_VERSION_MAX];
  size_t len = strnlen(newVersion, OTA_SKI_VERSION_MAX);
  payload[0] = OTA_STATUS_SKI_RELAY_OK;
  memcpy(&payload[1], newVersion, len);
  pOTAStatusChar->setValue(payload, 1 + len);
  pOTAStatusChar->notify();
}

// reason: a SkiUpdateResult value. skiNak: the ski's last NAK code, 0 if none.
// known: true if the ski is known to be running oldVersion, false if there is
// no telling what it runs now.
void sendSkiRelayFail(uint8_t reason, uint8_t skiNak, bool known, const char* oldVersion) {
  if (!pOTAStatusChar) return;
  uint8_t payload[4 + OTA_SKI_VERSION_MAX];
  size_t len = strnlen(oldVersion, OTA_SKI_VERSION_MAX);
  payload[0] = OTA_STATUS_SKI_RELAY_FAIL;
  payload[1] = reason;
  payload[2] = skiNak;
  payload[3] = known ? 1 : 0;
  memcpy(&payload[4], oldVersion, len);
  pOTAStatusChar->setValue(payload, 4 + len);
  pOTAStatusChar->notify();
}

// Cached ski version for the phone to read, empty while unknown.
void setSkiVersionChar(const char* version) {
  if (!pSkiVersionChar) return;
  pSkiVersionChar->setValue((const uint8_t*)version, strlen(version));
}

static void otaReset() {
  otaState = OTA_ST_IDLE;
  otaStartPending = false;
  otaTargetSki = false;
  otaExpectedSize = 0;
  otaExpectedCRC = 0;
  otaBytesWritten = 0;
  otaRunningCRC = 0;
  otaExpectedSeq = 0;
  // drain any stale queued chunks left over from an aborted transfer
  if (otaQueue) {
    OTAChunk stale;
    while (xQueueReceive(otaQueue, &stale, 0) == pdTRUE) {}
  }
}

// --------------------------------------------------
// Control characteristic: handles START / ABORT / END, and the ski relay command
// --------------------------------------------------
class OTAControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    std::string value = pChar->getValue();
    if (value.empty()) return;

    uint8_t cmd = value[0];

    switch (cmd) {

      // 'K' is the ski image START: 'K' + size(4 LE) + crc32(4 LE) + version (ASCII, 1 to 32 chars).
      // A different letter instead of extra bytes on 'S', so older firmware
      // rejects it and cannot mistake a ski image for its own.
      case 'K':
      case 'S': { // START: 'S' + size(4 LE) + crc32(4 LE)  -> 9 bytes total
        if (otaState != OTA_ST_IDLE || otaStartPending || skiRelayBusy()) {
          sendOTAStatus(OTA_STATUS_ERR_STATE);
          return;
        }
        if (value.size() < 9) {
          sendOTAStatus(OTA_STATUS_ERR_STATE); // malformed START
          return;
        }
        if (cmd == 'K') {
          size_t verLen = value.size() - 9;
          if (verLen == 0 || verLen > OTA_SKI_VERSION_MAX) {
            sendOTAStatus(OTA_STATUS_ERR_STATE);
            return;
          }
          memcpy(otaSkiVersion, value.data() + 9, verLen);
          otaSkiVersion[verLen] = '\0';
        }
        otaTargetSki = (cmd == 'K');

        memcpy(&otaExpectedSize, value.data() + 1, 4);
        memcpy(&otaExpectedCRC,  value.data() + 5, 4);

        if (COMMS) {
          Serial.print("OTA: START received, size=");
          Serial.print(otaExpectedSize);
          Serial.print(" crc=0x");
          Serial.print(otaExpectedCRC, HEX);
          Serial.print(" connMTU=");
          Serial.println(connInfo.getMTU());
        }

        // Update.begin(), the RECEIVING transition, and READY are handled by
        // processOTAQueue() on the main task. loop() may be mid scan pass or
        // calibration, and READY tells the app to start sending chunks, so it
        // must not go out until loop() is free to drain the queue.
        otaStartPending = true;
        break;
      }

      case 'A': { // ABORT
        if (COMMS) Serial.println("OTA: ABORT received");
        if (otaTargetSki) {
          otaSkiAbortPending = true;
          break;
        }
        if (otaState != OTA_ST_IDLE) {
          Update.abort();
        }
        otaReset();
        break;
      }

      case 'E': { // END: all chunks sent, verify + apply
        if (otaState != OTA_ST_RECEIVING) {
          sendOTAStatus(OTA_STATUS_ERR_STATE);
          return;
        }
        if (COMMS) Serial.println("OTA: END received, moving to VERIFYING");
        otaState = OTA_ST_VERIFYING;
        // Verification runs in processOTAQueue() once the queue has fully
        // drained, so otaBytesWritten reflects every chunk queued before
        // this END command arrived.
        break;
      }

      case 'R': { // RELAY: send the stored ski image to the ski. Progress and the result arrive as status notifications.
        if (otaBusy() || skiRelayBusy()) {
          sendOTAStatus(OTA_STATUS_ERR_STATE);
          return;
        }
        skiRelayRequest();
        break;
      }

      default:
        sendOTAStatus(OTA_STATUS_ERR_STATE);
        break;
    }
  }
};

// --------------------------------------------------
// Data characteristic: queues chunks, never touches flash here.
// Runs on the NimBLE host task — must stay fast, same reasoning as
// why handleCommand() got moved off this task via commandQueue.
// --------------------------------------------------
class OTADataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    if (otaState != OTA_ST_RECEIVING) return; // silently drop stray data outside a transfer

    std::string value = pChar->getValue();
    if (value.size() < 3) return; // need at least 2-byte seq header + 1 data byte

    OTAChunk chunk;
    chunk.seq = (uint8_t)value[0] | ((uint8_t)value[1] << 8); // LE
    chunk.len = value.size() - 2;

    if (chunk.len > OTA_CHUNK_PAYLOAD_MAX) {
      // Shouldn't happen given fixed conservative chunk sizing on the app
      // side, but never buffer-overflow if something unexpected arrives.
      if (COMMS) {
        Serial.print("OTA: oversized chunk received (");
        Serial.print(chunk.len);
        Serial.print(" bytes, raw value.size()=");
        Serial.print(value.size());
        Serial.println("), aborting");
      }
      otaState = OTA_ST_ERROR;
      sendOTAStatus(OTA_STATUS_ERR_WRITE);
      return;
    }

    memcpy(chunk.data, value.data() + 2, chunk.len);

    // Queue full means the app is outpacing flash writes. Fixed-delay
    // pacing on the app side should prevent this — fail loud rather than
    // block the NimBLE host task waiting for space.
    if (xQueueSend(otaQueue, &chunk, 0) != pdTRUE) {
      if (COMMS) Serial.println("OTA: queue full, chunk dropped, aborting");
      otaState = OTA_ST_ERROR;
      sendOTAStatus(OTA_STATUS_ERR_WRITE);
    }
  }
};

// --------------------------------------------------
// Public setup: call once from your existing BLE server setup
// --------------------------------------------------
void setupOTAService(NimBLEServer* pServer) {
  otaQueue = xQueueCreate(OTA_QUEUE_LEN, sizeof(OTAChunk));

  NimBLEService* pOTAService = pServer->createService(OTA_SERVICE_UUID);

  NimBLECharacteristic* pControlChar = pOTAService->createCharacteristic(
    OTA_CONTROL_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pControlChar->setCallbacks(new OTAControlCallbacks());

  NimBLECharacteristic* pDataChar = pOTAService->createCharacteristic(
    OTA_DATA_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE_NR
  );
  pDataChar->setCallbacks(new OTADataCallbacks());

  pOTAStatusChar = pOTAService->createCharacteristic(
    OTA_STATUS_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  // Static value, set once — read by the app before/after a transfer to
  // compare against a release manifest and to confirm a reboot actually
  // landed on the expected new version. FIRMWARE_VERSION is #defined in
  // the main sketch (TetraEMGControl.ino).
  NimBLECharacteristic* pVersionChar = pOTAService->createCharacteristic(
    OTA_VERSION_CHAR_UUID,
    NIMBLE_PROPERTY::READ
  );
  pVersionChar->setValue((uint8_t*)FIRMWARE_VERSION, strlen(FIRMWARE_VERSION));

  // The ski's version as last reported to the radio, kept current by
  // SkiUpdateService.ino. Empty until the ski has answered.
  pSkiVersionChar = pOTAService->createCharacteristic(
    OTA_SKI_VERSION_CHAR_UUID,
    NIMBLE_PROPERTY::READ
  );
  pSkiVersionChar->setValue((const uint8_t*)"", 0);
}

// --------------------------------------------------
// Public: true from START until the transfer finishes, fails, or is aborted.
// The main sketch uses this to stay out of the way (no sensor scanning, no
// blocking commands) so processOTAQueue() runs every loop() iteration.
// --------------------------------------------------
bool otaBusy() {
  return otaStartPending ||
         otaState == OTA_ST_RECEIVING ||
         otaState == OTA_ST_VERIFYING ||
         otaState == OTA_ST_APPLYING;
}

// --------------------------------------------------
// Public: call once per loop() to drain queued chunks and drive the
// state machine forward. Mirrors the existing commandQueue drain pattern.
// --------------------------------------------------
void processOTAQueue() {
  OTAChunk chunk;

  if (otaSkiAbortPending) {
    otaSkiAbortPending = false;
    skiStoreAbort();
    otaReset();
  }

  if (otaStartPending) {
    otaStartPending = false;

    if (otaTargetSki) {
      int rc = skiStoreBegin(otaExpectedSize);
      if (rc != 0) {
        sendOTAStatus(rc == -2 ? OTA_STATUS_ERR_SIZE : OTA_STATUS_ERR_BEGIN);
        otaReset();
        return;
      }
    } else if (!Update.begin(otaExpectedSize)) {
      if (COMMS) {
        Serial.print("OTA: Update.begin() failed: ");
        Serial.println(Update.errorString());
      }
      sendOTAStatus(OTA_STATUS_ERR_BEGIN); // covers oversize-for-partition too
      otaReset();
      return;
    }

    otaBytesWritten = 0;
    otaRunningCRC = 0;
    otaExpectedSeq = 0;
    otaLastActivityMs = millis();
    otaState = OTA_ST_RECEIVING;

    sendOTAStatus(OTA_STATUS_READY);
  }

  while (xQueueReceive(otaQueue, &chunk, 0) == pdTRUE) {

    // Chunks queued before END still have to be written, so VERIFYING keeps draining
    if (otaState != OTA_ST_RECEIVING && otaState != OTA_ST_VERIFYING) break; // aborted/errored mid-drain

    if (chunk.seq != otaExpectedSeq) {
      if (COMMS) {
        Serial.print("OTA: seq mismatch, expected ");
        Serial.print(otaExpectedSeq);
        Serial.print(" got ");
        Serial.println(chunk.seq);
      }
      otaState = OTA_ST_ERROR;
      sendOTAStatus(OTA_STATUS_ERR_SEQ);
      break;
    }

    bool written = otaTargetSki ? skiStoreWrite(chunk.data, chunk.len)
                                : Update.write(chunk.data, chunk.len) == chunk.len;
    if (!written) {
      if (COMMS && !otaTargetSki) {
        Serial.print("OTA: Update.write() failed: ");
        Serial.println(Update.errorString());
      }
      otaState = OTA_ST_ERROR;
      sendOTAStatus(OTA_STATUS_ERR_WRITE);
      break;
    }

    otaRunningCRC = crc32_le(otaRunningCRC, chunk.data, chunk.len);
    otaBytesWritten += chunk.len;
    otaExpectedSeq++;
    otaLastActivityMs = millis();

    // Throttle progress notifications so we don't flood the BLE stack
    if ((otaExpectedSeq % 20) == 0) {
      sendOTAProgress();
    }
  }

  // Verify + apply once the queue is empty and END has been received
  if (otaState == OTA_ST_VERIFYING && uxQueueMessagesWaiting(otaQueue) == 0) {

    if (otaBytesWritten != otaExpectedSize || otaRunningCRC != otaExpectedCRC) {
      if (COMMS) {
        Serial.print("OTA: CRC/size mismatch. bytesWritten=");
        Serial.print(otaBytesWritten);
        Serial.print(" expectedSize=");
        Serial.print(otaExpectedSize);
        Serial.print(" runningCRC=0x");
        Serial.print(otaRunningCRC, HEX);
        Serial.print(" expectedCRC=0x");
        Serial.println(otaExpectedCRC, HEX);
      }
      otaState = OTA_ST_ERROR;
      sendOTAStatus(OTA_STATUS_ERR_CRC);
      if (otaTargetSki) skiStoreAbort();
      else Update.abort();
      return;
    }

    if (otaTargetSki) {
      // The running CRC above covers what arrived over BLE. skiStoreFinish()
      // reads the file back, so this also covers what actually landed in flash.
      bool stored = skiStoreFinish(otaExpectedSize, otaExpectedCRC, otaSkiVersion);
      sendOTAStatus(stored ? OTA_STATUS_SKI_STORED : OTA_STATUS_ERR_CRC);
      otaReset();
      return;
    }

    if (COMMS) Serial.println("OTA: CRC/size verified, calling Update.end()");
    otaState = OTA_ST_APPLYING;

    if (!Update.end(true)) {
      if (COMMS) {
        Serial.print("OTA: Update.end() failed: ");
        Serial.println(Update.errorString());
      }
      otaState = OTA_ST_ERROR;
      sendOTAStatus(OTA_STATUS_ERR_WRITE);
      return;
    }

    if (COMMS) Serial.println("OTA: Update.end() succeeded, rebooting");
    sendOTAStatus(OTA_STATUS_DONE);
    sendIdleToSki();  // no direction output while the radio reboots
    Serial.flush();
    delay(200); // let the notify flush before reboot
    ESP.restart();
  }

  // Idle timeout: reset a stuck RECEIVING/ERROR state if the app vanished
  if ((otaState == OTA_ST_RECEIVING || otaState == OTA_ST_ERROR) &&
      (millis() - otaLastActivityMs > OTA_IDLE_TIMEOUT_MS)) {
    if (COMMS) Serial.println("OTA: idle timeout, resetting state");
    if (otaTargetSki) skiStoreAbort();
    else if (otaState == OTA_ST_RECEIVING) Update.abort();
    otaReset();
  }
}

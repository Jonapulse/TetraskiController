// OTAService.ino
// BLE OTA firmware update receiver for TetraRadio (ESP32 / NimBLE)
//
// Provides a dedicated GATT service for receiving firmware images over BLE,
// verifying them, and applying them via the ESP32 Update library.

#include <NimBLEDevice.h>
#include <Update.h>
#include "esp32/rom/crc.h"   // crc32_le

// ---- UUIDs (placeholders — generate real ones before use) ----
#define OTA_SERVICE_UUID       "c076ed50-9e91-4566-9023-3cb1b9173244"
#define OTA_CONTROL_CHAR_UUID  "ed3c98b9-0a71-45e0-9b14-b89d3316549d"
#define OTA_DATA_CHAR_UUID     "ee191e3a-95c6-4bf1-92b8-1980c9e7b8e9"
#define OTA_STATUS_CHAR_UUID   "5c8b02e7-5520-468f-bcba-fcc5093da1c9"
#define OTA_VERSION_CHAR_UUID  "c82f2a3c-f48c-4cfa-b447-0277467898e4"

// ---- Tunables (most speedup rn on App.js end)
#define OTA_CHUNK_PAYLOAD_MAX  512
#define OTA_QUEUE_LEN          10      // backpressure depth before we flag an error
#define OTA_IDLE_TIMEOUT_MS    15000   // ms with no activity before auto-abort

// ---- State machine ----
enum OTAState {
  OTA_ST_IDLE,
  OTA_ST_RECEIVING,
  OTA_ST_VERIFYING,
  OTA_ST_APPLYING,
  OTA_ST_ERROR
};

static volatile OTAState otaState = OTA_ST_IDLE;

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

// ---- Status codes sent over the OTA Status characteristic ----
enum OTAStatusCode : uint8_t {
  OTA_STATUS_READY     = 0x01, // ack for START
  OTA_STATUS_PROGRESS  = 0x02, // followed by 4-byte bytesWritten (uint32 LE)
  OTA_STATUS_DONE       = 0x03, // update applied, about to reboot
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

static void otaReset() {
  otaState = OTA_ST_IDLE;
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
// Control characteristic: handles START / ABORT / END
// --------------------------------------------------
class OTAControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    std::string value = pChar->getValue();
    if (value.empty()) return;

    uint8_t cmd = value[0];

    switch (cmd) {

      case 'S': { // START: 'S' + size(4 LE) + crc32(4 LE)  -> 9 bytes total
        if (otaState != OTA_ST_IDLE) {
          sendOTAStatus(OTA_STATUS_ERR_STATE);
          return;
        }
        if (value.size() < 9) {
          sendOTAStatus(OTA_STATUS_ERR_STATE); // malformed START
          return;
        }

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

        if (!Update.begin(otaExpectedSize)) {
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
        break;
      }

      case 'A': { // ABORT
        if (COMMS) Serial.println("OTA: ABORT received");
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

  pOTAService->start();
}

// --------------------------------------------------
// Public: call once per loop() to drain queued chunks and drive the
// state machine forward. Mirrors the existing commandQueue drain pattern.
// --------------------------------------------------
void processOTAQueue() {
  OTAChunk chunk;

  while (xQueueReceive(otaQueue, &chunk, 0) == pdTRUE) {

    if (otaState != OTA_ST_RECEIVING) break; // aborted/errored mid-drain

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

    if (Update.write(chunk.data, chunk.len) != chunk.len) {
      if (COMMS) {
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
      Update.abort();
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
    delay(200); // let the notify flush before reboot
    ESP.restart();
  }

  // Idle timeout: reset a stuck RECEIVING/ERROR state if the app vanished
  if ((otaState == OTA_ST_RECEIVING || otaState == OTA_ST_ERROR) &&
      (millis() - otaLastActivityMs > OTA_IDLE_TIMEOUT_MS)) {
    if (COMMS) Serial.println("OTA: idle timeout, resetting state");
    if (otaState == OTA_ST_RECEIVING) Update.abort();
    otaReset();
  }
}

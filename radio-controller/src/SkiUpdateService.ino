// SkiUpdateService.ino
// -------------------------------------------------------------------------
// Ski controller firmware update pathway (radio side).
//   Step 1: version query round trip over the Serial link to the ski.
//   Step 2: enter/exit update mode handshake.
//   Step 3: send an image stored in LittleFS to the ski, which buffers and
//           verifies it. The bench test then discards it (no commit yet).
//
// Raw bytes (outside update-mode frames):
//   0x56 'V'  version query, ski replies [lengthByte][ASCII chars]
//   0x01      enter update mode, ski replies SKI_ACK
//   0x05      exit update mode, ski replies SKI_ACK
// Update-mode frames: see the header of SkiVersionTest.ino for the format.
//
// While skiLinkPaused is true (defined in TetraEMGControl.ino), nothing may
// be printed on Serial: the ski would count it as stray traffic.
// -------------------------------------------------------------------------

#include <LittleFS.h>
#include "esp32/rom/crc.h"

#define SKI_CMD_VERSION_QUERY    0x56
#define SKI_CMD_ENTER_UPDATE     0x01
#define SKI_CMD_EXIT_UPDATE      0x05
#define SKI_ACK                  0x06

#define SKI_VERSION_TIMEOUT_MS   500
#define SKI_VERSION_MAX_LEN      32

#define SKI_HANDSHAKE_TIMEOUT_MS 300
#define SKI_HANDSHAKE_RETRIES    3

#define SKI_FW_PATH              "/ski_fw.bin"

#define SKI_SOF                  0xA5
#define SKI_MAX_PAYLOAD          255
#define SKI_FT_START             0x10
#define SKI_FT_CHUNK             0x11
#define SKI_FT_END               0x12
#define SKI_FT_ABORT             0x13
#define SKI_RS_ACK               0x06
#define SKI_RS_NAK               0x15
#define SKI_ERR_FRAME_CRC        0x01

#define SKI_CHUNK_DATA_BYTES     128    // multiple of 4
#define SKI_FRAME_RETRIES        8
#define SKI_RETRY_GAP_MS         30     // line quiet longer than the ski's inter-byte timeout, so it drops a half-parsed frame
#define SKI_CHUNK_TIMEOUT_MS     300
#define SKI_START_TIMEOUT_MS     3000   // ski scans flash for its buffer
#define SKI_END_TIMEOUT_MS       3000   // ski reads the image back and checks CRC32
#define SKI_ABORT_TIMEOUT_MS     10000  // ski erases the buffer

#define SKI_BENCH_FIRST_RUN_MS   20000
#define SKI_BENCH_CYCLE_MS       120000

enum SkiXferResult : uint8_t {
  SKI_XFER_OK = 0,
  SKI_XFER_NO_FS,
  SKI_XFER_NO_FILE,
  SKI_XFER_START_FAILED,
  SKI_XFER_CHUNK_FAILED,
  SKI_XFER_END_FAILED
};

static const char* const skiXferNames[] = {
  "OK", "NO_FS", "NO_FILE", "START_FAILED", "CHUNK_FAILED", "END_FAILED"
};

static uint8_t  skiLastNak = 0;      // last NAK code the ski sent, 0 if none
static uint32_t skiXferBytes = 0;    // image bytes the ski has acknowledged
static uint32_t skiRetryCount = 0;   // frames that had to be sent again, a measure of line quality

// Blocks (with timeout) waiting for the ski controller's version reply.
// Response framing: [lengthByte][ASCII chars].
// Returns the version string, or "" if no valid response arrived in time.
// Bench only: blocks the calling task for up to ~1 second.
String querySkiVersion() {
  while (Serial.available()) Serial.read();

  Serial.write(SKI_CMD_VERSION_QUERY);

  unsigned long start = millis();
  while (!Serial.available()) {
    if (millis() - start > SKI_VERSION_TIMEOUT_MS) {
      if (COMMS) Serial.println("Ski version query: timed out waiting for length byte");
      return "";
    }
  }

  uint8_t len = Serial.read();
  if (len == 0 || len > SKI_VERSION_MAX_LEN) {
    if (COMMS) Serial.println("Ski version query: invalid length byte");
    return "";
  }

  char buf[SKI_VERSION_MAX_LEN + 1];
  uint8_t received = 0;
  start = millis();
  while (received < len) {
    if (Serial.available()) {
      buf[received++] = Serial.read();
    } else if (millis() - start > SKI_VERSION_TIMEOUT_MS) {
      if (COMMS) Serial.println("Ski version query: timed out mid-string");
      return "";
    }
  }
  buf[received] = '\0';

  if (COMMS) {
    Serial.print("Ski version query: got \"");
    Serial.print(buf);
    Serial.println("\"");
  }
  return String(buf);
}

// Bench call site: version check every 5 seconds while the link is in normal mode.
void checkSkiVersionPeriodic() {
  if (skiLinkPaused) return;

  static unsigned long lastCheckMs = 0;
  if (millis() - lastCheckMs < 5000) return;
  lastCheckMs = millis();

  String skiVer = querySkiVersion();
  if (skiVer.length() > 0) {
    Serial.print("Ski controller version: ");
    Serial.println(skiVer);
  } else {
    Serial.println("Ski controller: no response");
  }
}

// Discards other bytes until an ACK arrives or the timeout passes.
static bool waitForSkiAck(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Serial.available() && Serial.read() == SKI_ACK) return true;
  }
  return false;
}

// Stops all control output to the ski, then asks it to enter update mode.
// Control output stays stopped on success. On failure it resumes, after a
// best-effort EXIT in case the ski entered update mode and only its ACK was lost.
bool enterSkiUpdateMode() {
  skiLinkPaused = true;
  Serial.flush();  // let bytes already queued for the ski finish going out

  for (int attempt = 0; attempt < SKI_HANDSHAKE_RETRIES; attempt++) {
    while (Serial.available()) Serial.read();
    Serial.write(SKI_CMD_ENTER_UPDATE);
    if (waitForSkiAck(SKI_HANDSHAKE_TIMEOUT_MS)) return true;
  }

  Serial.write(SKI_CMD_EXIT_UPDATE);
  skiLinkPaused = false;
  return false;
}

// Asks the ski to leave update mode, then resumes control output whether or
// not the ACK arrived (the ski also leaves update mode on its own idle timeout).
bool exitSkiUpdateMode() {
  bool acked = false;
  for (int attempt = 0; attempt < SKI_HANDSHAKE_RETRIES && !acked; attempt++) {
    while (Serial.available()) Serial.read();
    Serial.write(SKI_CMD_EXIT_UPDATE);
    acked = waitForSkiAck(SKI_HANDSHAKE_TIMEOUT_MS);
  }
  skiLinkPaused = false;
  return acked;
}

// CRC-16/CCITT-FALSE, same as the ski side
static uint16_t skiCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int k = 0; k < 8; k++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Sends one frame and waits for the ski's 6-byte reply. Returns true if a
// reply with a good CRC arrived, with its status and argument filled in.
static bool skiTransact(uint8_t type, const uint8_t* payload, uint8_t len,
                        uint8_t* status, uint16_t* arg, unsigned long timeoutMs) {
  uint8_t frame[3 + SKI_MAX_PAYLOAD + 2];
  frame[0] = SKI_SOF;
  frame[1] = type;
  frame[2] = len;
  if (len > 0) memcpy(frame + 3, payload, len);
  uint16_t crc = skiCrc16(frame + 1, 2 + len);
  frame[3 + len] = crc & 0xFF;
  frame[4 + len] = crc >> 8;

  while (Serial.available()) Serial.read();
  Serial.write(frame, 5 + len);

  uint8_t reply[6];
  uint8_t got = 0;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (!Serial.available()) {
      delay(1);
      continue;
    }
    uint8_t b = Serial.read();
    if (got == 0 && b != SKI_SOF) continue;  // hunt for the start of a reply
    reply[got++] = b;
    if (got == 6) {
      uint16_t rc = skiCrc16(reply + 1, 3);
      if (rc == (uint16_t)(reply[4] | (reply[5] << 8))) {
        *status = reply[1];
        *arg = reply[2] | (reply[3] << 8);
        return true;
      }
      got = 0;  // corrupted reply, keep hunting until the timeout
    }
  }
  return false;
}

// Sends a frame until the ski ACKs it. Retries on a timeout or a frame-CRC
// NAK, any other NAK is final and its code is left in skiLastNak.
static bool skiRequest(uint8_t type, const uint8_t* payload, uint8_t len, unsigned long timeoutMs) {
  for (int attempt = 0; attempt < SKI_FRAME_RETRIES; attempt++) {
    uint8_t status = 0;
    uint16_t arg = 0;
    if (attempt > 0) {
      skiRetryCount++;
      delay(SKI_RETRY_GAP_MS);
    }
    if (!skiTransact(type, payload, len, &status, &arg, timeoutMs)) continue;
    if (status == SKI_RS_ACK) return true;
    skiLastNak = (uint8_t)arg;
    if (arg != SKI_ERR_FRAME_CRC) return false;
  }
  return false;
}

// Sends the image at path to the ski, which buffers it and verifies it.
// Must be called with the ski already in update mode. Prints nothing.
// On any failure the caller should send ABORT so the ski frees its buffer.
uint8_t skiTransferImage(const char* path) {
  skiLastNak = 0;
  skiXferBytes = 0;
  skiRetryCount = 0;

  if (!LittleFS.begin(false)) return SKI_XFER_NO_FS;
  File f = LittleFS.open(path, "r");
  if (!f || f.size() == 0) return SKI_XFER_NO_FILE;
  uint32_t size = f.size();

  uint32_t crc = 0;
  uint8_t buf[SKI_CHUNK_DATA_BYTES];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    crc = crc32_le(crc, buf, n);
  }
  f.seek(0);

  uint8_t startPayload[8];
  memcpy(startPayload, &size, 4);
  memcpy(startPayload + 4, &crc, 4);
  if (!skiRequest(SKI_FT_START, startPayload, 8, SKI_START_TIMEOUT_MS)) {
    f.close();
    return SKI_XFER_START_FAILED;
  }

  uint16_t seq = 0;
  uint32_t sent = 0;
  uint8_t payload[2 + SKI_CHUNK_DATA_BYTES];
  while (sent < size) {
    size_t n = f.read(payload + 2, SKI_CHUNK_DATA_BYTES);
    if (n == 0) break;
    payload[0] = seq & 0xFF;
    payload[1] = seq >> 8;
    if (!skiRequest(SKI_FT_CHUNK, payload, 2 + n, SKI_CHUNK_TIMEOUT_MS)) {
      f.close();
      return SKI_XFER_CHUNK_FAILED;
    }
    sent += n;
    skiXferBytes = sent;
    seq++;
  }
  f.close();
  if (sent != size) return SKI_XFER_CHUNK_FAILED;

  if (!skiRequest(SKI_FT_END, nullptr, 0, SKI_END_TIMEOUT_MS)) return SKI_XFER_END_FAILED;
  return SKI_XFER_OK;
}

// Bench test: every SKI_BENCH_CYCLE_MS (first run after SKI_BENCH_FIRST_RUN_MS)
// enter update mode, send /ski_fw.bin, have the ski verify it, tell the ski to
// discard it, and exit. Blocks for the whole transfer, which the real update
// path will not do. Results are printed only after control output has resumed.
void checkSkiUpdateModeBench() {
  static bool firstRun = true;
  static unsigned long lastRunMs = 0;
  if (skiLinkPaused) return;
  if (millis() - lastRunMs < (firstRun ? SKI_BENCH_FIRST_RUN_MS : SKI_BENCH_CYCLE_MS)) return;
  firstRun = false;

  unsigned long startMs = millis();
  bool entered = enterSkiUpdateMode();
  uint8_t result = SKI_XFER_START_FAILED;
  bool discarded = false;
  bool exited = false;
  if (entered) {
    result = skiTransferImage(SKI_FW_PATH);
    discarded = skiRequest(SKI_FT_ABORT, nullptr, 0, SKI_ABORT_TIMEOUT_MS);
    exited = exitSkiUpdateMode();
  }
  lastRunMs = millis();

  Serial.printf("Ski transfer bench: enter=%s transfer=%s ski_nak=0x%02X bytes=%u retries=%u discard=%s exit=%s time=%lums\n",
                entered ? "ok" : "FAILED", skiXferNames[result], skiLastNak,
                (unsigned)skiXferBytes, (unsigned)skiRetryCount, discarded ? "ok" : "FAILED",
                exited ? "ok" : "FAILED", lastRunMs - startMs);
}

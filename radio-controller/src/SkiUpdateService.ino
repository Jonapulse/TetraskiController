// SkiUpdateService.ino
// -------------------------------------------------------------------------
// Ski controller firmware update pathway (radio side).
//   Step 1: version query round trip over the Serial link to the ski.
//   Step 2: enter/exit update mode handshake.
//   Step 3: send an image stored in LittleFS to the ski, which buffers and
//           verifies it. The bench test then discards it.
//   Step 4: with SKI_BENCH_COMMIT set to 1, compare versions and, if the
//           stored image is newer, send it, COMMIT it, and watch the ski come
//           back on the new version. Once per boot, never retried.
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
#define SKI_FW_VER_PATH          "/ski_fw.ver"   // one line of text, e.g. 0.2.0-bench

// 0 = the step 3 bench (send, verify, discard, every 2 minutes). Nothing is
//     ever written to the ski's program flash.
// 1 = the real update, once per boot. This REWRITES the ski's program flash.
// Can also be set with build_flags = -DSKI_BENCH_COMMIT=1
#ifndef SKI_BENCH_COMMIT
#define SKI_BENCH_COMMIT 0
#endif

#define SKI_SOF                  0xA5
#define SKI_MAX_PAYLOAD          255
#define SKI_FT_START             0x10
#define SKI_FT_CHUNK             0x11
#define SKI_FT_END               0x12
#define SKI_FT_ABORT             0x13
#define SKI_FT_COMMIT            0x14
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
#define SKI_COMMIT_TIMEOUT_MS    3000   // the ski ACKs before it starts rewriting flash
#define SKI_REBOOT_WAIT_MS       15000  // how long to wait for the ski to answer after COMMIT

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
static uint32_t skiImageSize = 0;    // size and CRC32 of the image last sent, needed for COMMIT
static uint32_t skiImageCrc = 0;

enum SkiUpdateResult : uint8_t {
  SKI_UPD_UPDATED = 0,        // the ski now reports the image's version
  SKI_UPD_UP_TO_DATE,         // the ski already runs this version or newer
  SKI_UPD_NO_SKI_VERSION,     // the ski never answered the version query, nothing attempted
  SKI_UPD_NO_IMAGE_VERSION,   // /ski_fw.ver missing or unreadable, nothing attempted
  SKI_UPD_ENTER_FAILED,
  SKI_UPD_TRANSFER_FAILED,    // the ski still runs its old firmware
  SKI_UPD_COMMIT_REJECTED,    // the ski refused to commit, still runs its old firmware
  SKI_UPD_NO_RESPONSE,        // COMMIT went out, but the ski never answered afterwards
  SKI_UPD_VERSION_MISMATCH    // the ski answers, but not with the image's version
};

static const char* const skiUpdateNames[] = {
  "UPDATED", "UP_TO_DATE", "NO_SKI_VERSION", "NO_IMAGE_VERSION", "ENTER_FAILED",
  "TRANSFER_FAILED", "COMMIT_REJECTED", "NO_RESPONSE", "VERSION_MISMATCH"
};

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
uint8_t skiTransferImage(const char* path) {   // returns a SkiXferResult value
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

  skiImageSize = size;
  skiImageCrc = crc;

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

// Reads the one-line image version from /ski_fw.ver. False if it is missing or empty.
static bool skiReadImageVersion(String* out) {
  if (!LittleFS.begin(false)) return false;
  File f = LittleFS.open(SKI_FW_VER_PATH, "r");
  if (!f) return false;
  char buf[33];
  size_t n = f.read((uint8_t*)buf, sizeof(buf) - 1);
  f.close();
  while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' || buf[n - 1] == ' ')) n--;
  buf[n] = '\0';
  *out = String(buf);
  return n > 0;
}

// Compares the leading MAJOR.MINOR.PATCH numbers of two version strings.
// Positive if a is newer than b. The text after the numbers ("-bench") is
// ignored, so one flipped bit in it cannot cause a false mismatch.
static int skiCompareVersions(const String& a, const String& b) {
  int va[3] = {0, 0, 0};
  int vb[3] = {0, 0, 0};
  sscanf(a.c_str(), "%d.%d.%d", &va[0], &va[1], &va[2]);
  sscanf(b.c_str(), "%d.%d.%d", &vb[0], &vb[1], &vb[2]);
  for (int i = 0; i < 3; i++) {
    if (va[i] != vb[i]) return va[i] > vb[i] ? 1 : -1;
  }
  return 0;
}

// The whole update, once: compare versions, send the image, COMMIT it, then
// watch for the ski to come back. Returns a SkiUpdateResult. Blocks for the
// whole thing, and prints nothing (the caller prints once control output has
// resumed). After COMMIT only version queries are ever sent to the ski.
static uint8_t skiRunUpdate(String* oldVer, String* imgVer, String* newVer, unsigned long* bootMs) {
  *bootMs = 0;

  for (int i = 0; i < 3 && oldVer->length() == 0; i++) *oldVer = querySkiVersion();
  if (oldVer->length() == 0) return SKI_UPD_NO_SKI_VERSION;
  if (!skiReadImageVersion(imgVer)) return SKI_UPD_NO_IMAGE_VERSION;
  if (skiCompareVersions(*imgVer, *oldVer) <= 0) return SKI_UPD_UP_TO_DATE;

  if (!enterSkiUpdateMode()) return SKI_UPD_ENTER_FAILED;

  if (skiTransferImage(SKI_FW_PATH) != SKI_XFER_OK) {
    skiRequest(SKI_FT_ABORT, nullptr, 0, SKI_ABORT_TIMEOUT_MS);
    exitSkiUpdateMode();
    return SKI_UPD_TRANSFER_FAILED;
  }

  // One attempt, no retries. If the ski refuses it is still running its old
  // firmware. If the ACK is lost the ski may be rewriting its flash anyway,
  // so that case falls through to the same wait as an ACK.
  uint8_t commitPayload[8];
  memcpy(commitPayload, &skiImageSize, 4);
  memcpy(commitPayload + 4, &skiImageCrc, 4);
  uint8_t status = 0;
  uint16_t arg = 0;
  if (skiTransact(SKI_FT_COMMIT, commitPayload, 8, &status, &arg, SKI_COMMIT_TIMEOUT_MS) &&
      status != SKI_RS_ACK) {
    skiLastNak = (uint8_t)arg;
    skiRequest(SKI_FT_ABORT, nullptr, 0, SKI_ABORT_TIMEOUT_MS);
    exitSkiUpdateMode();
    return SKI_UPD_COMMIT_REJECTED;
  }

  unsigned long start = millis();
  while (millis() - start < SKI_REBOOT_WAIT_MS) {
    *newVer = querySkiVersion();
    if (newVer->length() > 0) {
      *bootMs = millis() - start;
      break;
    }
  }
  if (newVer->length() == 0) return SKI_UPD_NO_RESPONSE;

  // A bit error in the reply must not look like a failed update, so ask again before giving up
  for (int i = 0; i < 2 && skiCompareVersions(*newVer, *imgVer) != 0; i++) {
    String again = querySkiVersion();
    if (again.length() > 0) *newVer = again;
  }
  return skiCompareVersions(*newVer, *imgVer) == 0 ? SKI_UPD_UPDATED : SKI_UPD_VERSION_MISMATCH;
}

// Bench call site, see SKI_BENCH_COMMIT. Called from loop() in TetraEMGControl.ino.
void checkSkiUpdateModeBench() {
#if SKI_BENCH_COMMIT
  static bool done = false;
  if (done || skiLinkPaused || millis() < SKI_BENCH_FIRST_RUN_MS) return;
  done = true;

  String oldVer, imgVer, newVer;
  unsigned long bootMs = 0;
  unsigned long startMs = millis();
  uint8_t result = skiRunUpdate(&oldVer, &imgVer, &newVer, &bootMs);
  skiLinkPaused = false;

  Serial.printf("Ski update: result=%s old=%s image=%s new=%s ski_nak=0x%02X boot_ms=%lu retries=%u time=%lums\n",
                skiUpdateNames[result],
                oldVer.length() ? oldVer.c_str() : "unknown",
                imgVer.length() ? imgVer.c_str() : "unknown",
                newVer.length() ? newVer.c_str() : "none",
                skiLastNak, bootMs, (unsigned)skiRetryCount, millis() - startMs);
#else
  // Step 3 bench: every SKI_BENCH_CYCLE_MS (first run after SKI_BENCH_FIRST_RUN_MS)
  // enter update mode, send /ski_fw.bin, have the ski verify it, tell the ski to
  // discard it, and exit. Blocks for the whole transfer, which the real update
  // path will not do. Results are printed only after control output has resumed.
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
#endif
}

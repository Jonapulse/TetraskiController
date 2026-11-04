// SkiUpdateService.ino
// -------------------------------------------------------------------------
// Ski controller firmware update pathway (radio side).
//   - Version query: asks the ski for its version until it answers, and keeps
//     the answer cached for the phone to read.
//   - Image store: the phone sends a ski image over the BLE OTA chunk path
//     (see OTAService.ino), it is written to LittleFS and read back to check it.
//   - Relay: on the phone's command, send the stored image to the ski, COMMIT
//     it, and watch the ski come back on the new version. One attempt, never
//     retried.
//
// Nothing here blocks. skiLinkService() is called every loop() and moves one
// small step each time, so sensors, BLE and the phone keep being serviced.
//
// Raw bytes (outside update-mode frames):
//   0x56 'V'  version query, ski replies 'V' [major][minor][patch], each 1-255
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
#define SKI_VERSION_REPLY_LEN    4      // 'V' + major, minor, patch
#define SKI_VERSION_POLL_MS      5000   // how often to ask while the ski's version is unknown

#define SKI_HANDSHAKE_TIMEOUT_MS 300
#define SKI_HANDSHAKE_RETRIES    3

#define SKI_FW_PATH              "/ski_fw.bin"
#define SKI_FW_VER_PATH          "/ski_fw.ver"   // one line of text, e.g. 0.2.0-bench
#define SKI_FS_MARGIN_BYTES      8192   // LittleFS needs spare blocks for its own bookkeeping

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
#define SKI_REBOOT_WAIT_MS       30000  // how long to wait for the ski to answer after COMMIT (flash rewrite + its 10 s startUpSeq())

#define SKI_BATTERY_MAX_DIGITS   3      // ski battery report: '%' then 0-100 as ASCII
#define SKI_BATTERY_WAIT_MS      5      // the ski writes the digits back to back, about 0.2 ms apart on the wire

#define SKI_PREPARE_STEP_BYTES   1024   // image bytes checked per loop() before the relay starts
#define SKI_PROGRESS_EVERY       16     // chunks between progress reports to the phone

// Sent to the phone as the reason byte of a failed relay, App.js has the same list.
enum SkiUpdateResult : uint8_t {
  SKI_UPD_UPDATED = 0,          // the ski now reports the image's version
  SKI_UPD_UP_TO_DATE,           // the ski already runs this version or newer
  SKI_UPD_NO_SKI_VERSION,       // the ski never answered the version query, nothing attempted
  SKI_UPD_NO_IMAGE_VERSION,     // /ski_fw.ver missing or unreadable, nothing attempted
  SKI_UPD_ENTER_FAILED,
  SKI_UPD_TRANSFER_FAILED,      // the ski still runs its old firmware
  SKI_UPD_COMMIT_REJECTED,      // the ski refused to commit, still runs its old firmware
  SKI_UPD_NO_RESPONSE,          // COMMIT went out, but the ski never answered afterwards
  SKI_UPD_VERSION_MISMATCH,     // the ski answers, but not with the image's version
  SKI_UPD_NO_IMAGE,             // /ski_fw.bin missing or empty, nothing attempted
  SKI_UPD_VERSION_NOT_IN_IMAGE  // the stored image does not contain its own version string, nothing attempted
};

// Sent to the phone with each progress report, App.js has the same list.
enum SkiRelayPhase : uint8_t {
  SKI_PHASE_PREPARE = 1,
  SKI_PHASE_ENTER,
  SKI_PHASE_SEND,
  SKI_PHASE_VERIFY,
  SKI_PHASE_COMMIT,
  SKI_PHASE_REBOOT
};

// ---- One exchange with the ski at a time: send something, collect the reply ----
enum SkiOpKind : uint8_t { SKI_OP_NONE, SKI_OP_VERSION, SKI_OP_RAW, SKI_OP_FRAME };
enum SkiOpResult : uint8_t { SKI_OP_PENDING, SKI_OP_OK, SKI_OP_NAK, SKI_OP_TIMEOUT };

static uint8_t  skiOpKind = SKI_OP_NONE;
static uint8_t  skiOpBuf[3 + SKI_MAX_PAYLOAD + 2];   // the bytes being sent, kept for retries
static uint16_t skiOpLen = 0;
static unsigned long skiOpTimeoutMs = 0;
static unsigned long skiOpGapMs = 0;      // quiet time before a retry
static uint8_t  skiOpSendsLeft = 0;       // retries still allowed
static bool     skiOpInGap = false;
static unsigned long skiOpMarkMs = 0;     // when the current send (or gap) started
static uint8_t  skiOpRx[SKI_VERSION_MAX_LEN + 1];
static uint8_t  skiOpRxGot = 0;
static String   skiOpVersion;             // result of the last version query that was answered

static uint8_t  skiLastNak = 0;           // last NAK code the ski sent, 0 if none

// ---- Relay state ----
enum SkiRelayState : uint8_t {
  SKI_RLY_IDLE,
  SKI_RLY_PREPARE,   // CRC32 of the stored image, and look for its version string in it
  SKI_RLY_ENTER,
  SKI_RLY_START,
  SKI_RLY_CHUNK,
  SKI_RLY_END,
  SKI_RLY_COMMIT,
  SKI_RLY_REBOOT,    // version queries until the ski answers
  SKI_RLY_CONFIRM,   // ask again if the answer was not the image's version
  SKI_RLY_ABORT,
  SKI_RLY_EXIT
};

static uint8_t  skiRelayState = SKI_RLY_IDLE;
static volatile bool skiRelayRequested = false;   // set on the BLE host task, picked up by loop()
static File     skiRelayFile;
static uint32_t skiImageSize = 0;    // size and CRC32 of the image being relayed, needed for COMMIT
static uint32_t skiImageCrc = 0;
static uint32_t skiRelayBytes = 0;   // image bytes checked (PREPARE) or acknowledged by the ski (CHUNK)
static uint16_t skiRelaySeq = 0;
static uint8_t  skiRelayChunkLen = 0;
static uint8_t  skiRelayFailReason = 0;   // reported once ABORT and EXIT are done
static uint8_t  skiRelayConfirmsLeft = 0;
static unsigned long skiRelayMarkMs = 0;
static String   skiRelayImgVer;
static String   skiRelayNewVer;
static bool     skiPrepFound = false;
static size_t   skiPrepKeep = 0;     // tail of the previous block, so a version string split across two blocks is still found
static uint8_t  skiPrepBuf[SKI_VERSION_MAX_LEN + SKI_PREPARE_STEP_BYTES];

static String   skiVersion;          // cached ski version, "" while unknown
static File     skiStoreFile;        // image being received from the phone

static void skiSetVersion(const String& version) {
  skiVersion = version;
  setSkiVersionChar(version.c_str());
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

static void skiOpSend() {
  while (Serial.available()) Serial.read();
  Serial.write(skiOpBuf, skiOpLen);
  skiOpRxGot = 0;
  skiOpInGap = false;
  skiOpMarkMs = millis();
}

// sends = how many times the bytes in skiOpBuf may go out in total
static void skiOpBegin(uint8_t kind, uint16_t len, unsigned long timeoutMs, uint8_t sends, unsigned long gapMs) {
  skiOpKind = kind;
  skiOpLen = len;
  skiOpTimeoutMs = timeoutMs;
  skiOpSendsLeft = sends - 1;
  skiOpGapMs = gapMs;
  skiOpSend();
}

// Version reply framing: 'V' [major][minor][patch], each byte 1-255
static void skiStartVersionQuery() {
  skiOpBuf[0] = SKI_CMD_VERSION_QUERY;
  skiOpBegin(SKI_OP_VERSION, 1, SKI_VERSION_TIMEOUT_MS, 1, 0);
}

// ENTER or EXIT. Other bytes are discarded until the ski's ACK arrives.
static void skiStartRaw(uint8_t cmd) {
  skiOpBuf[0] = cmd;
  skiOpBegin(SKI_OP_RAW, 1, SKI_HANDSHAKE_TIMEOUT_MS, SKI_HANDSHAKE_RETRIES, 0);
}

// One update-mode frame, answered by the ski's 6-byte reply.
static void skiStartFrame(uint8_t type, const uint8_t* payload, uint8_t len, unsigned long timeoutMs, uint8_t sends) {
  skiOpBuf[0] = SKI_SOF;
  skiOpBuf[1] = type;
  skiOpBuf[2] = len;
  if (len > 0) memcpy(skiOpBuf + 3, payload, len);
  uint16_t crc = skiCrc16(skiOpBuf + 1, 2 + len);
  skiOpBuf[3 + len] = crc & 0xFF;
  skiOpBuf[4 + len] = crc >> 8;
  skiOpBegin(SKI_OP_FRAME, 5 + len, timeoutMs, sends, SKI_RETRY_GAP_MS);
}

static uint8_t skiOpFinish(uint8_t result) {
  skiOpKind = SKI_OP_NONE;
  return result;
}

// Sends again if a retry is left, otherwise the exchange ends with the given result.
static uint8_t skiOpRetry(uint8_t resultIfSpent) {
  if (skiOpSendsLeft == 0) return skiOpFinish(resultIfSpent);
  skiOpSendsLeft--;
  if (skiOpGapMs == 0) {
    skiOpSend();
  } else {
    skiOpInGap = true;
    skiOpMarkMs = millis();
  }
  return SKI_OP_PENDING;
}

// Moves the current exchange along without waiting. Returns a SkiOpResult value.
// A frame is retried on a timeout or a frame-CRC NAK, any other NAK is final
// and its code is left in skiLastNak.
static uint8_t skiOpPoll() {
  if (skiOpInGap) {
    if (millis() - skiOpMarkMs >= skiOpGapMs) skiOpSend();
    return SKI_OP_PENDING;
  }

  // Bytes are read before the timeout is checked: a reply that arrived while
  // loop() was busy elsewhere is sitting in the UART buffer and still counts.
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (skiOpKind == SKI_OP_VERSION) {
      if (skiOpRxGot == 0 && b == '%') { consumeSkiBattery(); continue; }  // battery report sent just before the reply
      if (skiOpRxGot == 0 && b != SKI_CMD_VERSION_QUERY) return skiOpFinish(SKI_OP_TIMEOUT);
      if (skiOpRxGot > 0 && b == 0) return skiOpFinish(SKI_OP_TIMEOUT);  // 0 is never a valid version number
      skiOpRx[skiOpRxGot++] = b;
      if (skiOpRxGot == SKI_VERSION_REPLY_LEN) {
        // Kept as MAJOR.MINOR.PATCH text, the form the phone and the image's .ver file use
        char text[12];
        snprintf(text, sizeof(text), "%u.%u.%u", skiOpRx[1], skiOpRx[2], skiOpRx[3]);
        skiOpVersion = String(text);
        return skiOpFinish(SKI_OP_OK);
      }

    } else if (skiOpKind == SKI_OP_RAW) {
      if (b == SKI_ACK) return skiOpFinish(SKI_OP_OK);

    } else {
      if (skiOpRxGot == 0 && b != SKI_SOF) continue;  // hunt for the start of a reply
      skiOpRx[skiOpRxGot++] = b;
      if (skiOpRxGot < 6) continue;
      skiOpRxGot = 0;
      uint16_t rc = skiCrc16(skiOpRx + 1, 3);
      if (rc != (uint16_t)(skiOpRx[4] | (skiOpRx[5] << 8))) continue;  // corrupted reply, keep hunting until the timeout
      if (skiOpRx[1] == SKI_RS_ACK) return skiOpFinish(SKI_OP_OK);
      skiLastNak = skiOpRx[2];
      if (skiLastNak != SKI_ERR_FRAME_CRC) return skiOpFinish(SKI_OP_NAK);
      return skiOpRetry(SKI_OP_NAK);
    }
  }

  if (millis() - skiOpMarkMs > skiOpTimeoutMs) return skiOpRetry(SKI_OP_TIMEOUT);
  return SKI_OP_PENDING;
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

// --------------------------------------------------
// Image store: called from processOTAQueue() in OTAService.ino, on the main task
// --------------------------------------------------

// Returns 0 on success, -1 if the filesystem cannot be used, -2 if the image does not fit.
int skiStoreBegin(uint32_t size) {
  if (!LittleFS.begin(true)) return -1;  // true = format a partition that has never held a filesystem

  // Removed before the new image is written, not overwritten: LittleFS keeps
  // the old file's blocks until the new one is closed, and two images do not fit.
  LittleFS.remove(SKI_FW_PATH);
  LittleFS.remove(SKI_FW_VER_PATH);

  if (LittleFS.totalBytes() - LittleFS.usedBytes() < size + SKI_FS_MARGIN_BYTES) return -2;

  skiStoreFile = LittleFS.open(SKI_FW_PATH, "w");
  return skiStoreFile ? 0 : -1;
}

bool skiStoreWrite(const uint8_t* data, size_t len) {
  return skiStoreFile && skiStoreFile.write(data, len) == len;
}

// Drops a partly received image, so a relay can never pick it up.
void skiStoreAbort() {
  if (skiStoreFile) skiStoreFile.close();
  LittleFS.remove(SKI_FW_PATH);
  LittleFS.remove(SKI_FW_VER_PATH);
}

// Closes the image, reads it back from flash to check its size and CRC32, and
// only then writes the version file next to it.
bool skiStoreFinish(uint32_t size, uint32_t expectedCrc, const char* version) {
  skiStoreFile.close();

  File f = LittleFS.open(SKI_FW_PATH, "r");
  bool ok = f && f.size() == size;
  uint32_t crc = 0;
  while (ok && f.available()) {
    size_t n = f.read(skiPrepBuf, sizeof(skiPrepBuf));
    if (n == 0) break;
    crc = crc32_le(crc, skiPrepBuf, n);
  }
  if (f) f.close();
  ok = ok && crc == expectedCrc;

  if (ok) {
    File v = LittleFS.open(SKI_FW_VER_PATH, "w");
    ok = v && v.print(version) == strlen(version);
    if (v) v.close();
  }
  if (!ok) skiStoreAbort();
  return ok;
}

// --------------------------------------------------
// Relay
// --------------------------------------------------

// Ends the relay, resumes control output, and reports to the phone.
static void skiRelayFinish(uint8_t result) {
  if (skiRelayFile) skiRelayFile.close();
  skiRelayState = SKI_RLY_IDLE;
  skiLinkPaused = false;

  if (result == SKI_UPD_UPDATED) {
    skiSetVersion(skiRelayNewVer);
    sendSkiRelayOk(skiRelayNewVer.c_str());
    return;
  }

  // The ski answered after COMMIT, so what it runs is known, it is just not the image
  if (result == SKI_UPD_VERSION_MISMATCH) skiSetVersion(skiRelayNewVer);

  // After COMMIT with no answer there is no telling what the ski is running.
  // Every other failure happens before its program flash is touched.
  bool known = skiVersion.length() > 0 && result != SKI_UPD_NO_RESPONSE;
  sendSkiRelayFail(result, skiLastNak, known, skiVersion.c_str());

  if (result == SKI_UPD_NO_RESPONSE) skiSetVersion("");  // back to asking until it answers
}

// Has the ski discard its buffer and leave update mode, then reports the failure.
static void skiRelayAbort(uint8_t reason) {
  skiRelayFailReason = reason;
  skiStartFrame(SKI_FT_ABORT, nullptr, 0, SKI_ABORT_TIMEOUT_MS, SKI_FRAME_RETRIES);
  skiRelayState = SKI_RLY_ABORT;
}

static void skiRelaySendChunk() {
  uint8_t payload[2 + SKI_CHUNK_DATA_BYTES];
  size_t n = skiRelayFile.read(payload + 2, SKI_CHUNK_DATA_BYTES);
  if (n == 0) {
    skiRelayAbort(SKI_UPD_TRANSFER_FAILED);
    return;
  }
  payload[0] = skiRelaySeq & 0xFF;
  payload[1] = skiRelaySeq >> 8;
  skiRelayChunkLen = n;
  skiStartFrame(SKI_FT_CHUNK, payload, 2 + n, SKI_CHUNK_TIMEOUT_MS, SKI_FRAME_RETRIES);
  skiRelayState = SKI_RLY_CHUNK;
}

// The checks that need no traffic to the ski. Control output keeps running
// until they have all passed.
static void skiRelayBegin() {
  skiLastNak = 0;
  skiRelayNewVer = "";
  skiRelayImgVer = "";

  if (skiVersion.length() == 0) { skiRelayFinish(SKI_UPD_NO_SKI_VERSION); return; }
  if (!skiReadImageVersion(&skiRelayImgVer)) { skiRelayFinish(SKI_UPD_NO_IMAGE_VERSION); return; }
  if (skiCompareVersions(skiRelayImgVer, skiVersion) <= 0) { skiRelayFinish(SKI_UPD_UP_TO_DATE); return; }

  skiRelayFile = LittleFS.open(SKI_FW_PATH, "r");
  if (!skiRelayFile || skiRelayFile.size() == 0) { skiRelayFinish(SKI_UPD_NO_IMAGE); return; }

  skiImageSize = skiRelayFile.size();
  skiImageCrc = 0;
  skiRelayBytes = 0;
  skiPrepFound = false;
  skiPrepKeep = 0;
  skiRelayState = SKI_RLY_PREPARE;
  sendSkiRelayProgress(SKI_PHASE_PREPARE, 0, skiImageSize);
}

// After COMMIT: the ski has answered a version query. Done if it is the
// image's version. A bit error in the reply must not look like a failed
// update, so ask again before giving up.
static void skiRelayCheckNewVersion() {
  if (skiCompareVersions(skiRelayNewVer, skiRelayImgVer) == 0) {
    skiRelayFinish(SKI_UPD_UPDATED);
  } else if (skiRelayConfirmsLeft > 0) {
    skiRelayConfirmsLeft--;
    skiStartVersionQuery();
    skiRelayState = SKI_RLY_CONFIRM;
  } else {
    skiRelayFinish(SKI_UPD_VERSION_MISMATCH);
  }
}

// One step of the relay per call.
static void skiRelayStep() {
  if (skiRelayState == SKI_RLY_PREPARE) {
    size_t verLen = skiRelayImgVer.length();
    size_t n = skiRelayFile.read(skiPrepBuf + skiPrepKeep, SKI_PREPARE_STEP_BYTES);
    if (n > 0) {
      skiImageCrc = crc32_le(skiImageCrc, skiPrepBuf + skiPrepKeep, n);
      skiRelayBytes += n;
      size_t have = skiPrepKeep + n;
      if (!skiPrepFound && memmem(skiPrepBuf, have, skiRelayImgVer.c_str(), verLen) != nullptr) skiPrepFound = true;
      skiPrepKeep = have < verLen - 1 ? have : verLen - 1;
      memmove(skiPrepBuf, skiPrepBuf + have - skiPrepKeep, skiPrepKeep);
      return;
    }
    if (skiRelayBytes != skiImageSize) { skiRelayFinish(SKI_UPD_NO_IMAGE); return; }
    if (!skiPrepFound) { skiRelayFinish(SKI_UPD_VERSION_NOT_IN_IMAGE); return; }

    // Stops all control output to the ski, then asks it to enter update mode
    skiRelayFile.seek(0);
    sendIdleToSki();  // the last direction sent must not stay latched on the ski while it waits for ENTER
    skiLinkPaused = true;
    Serial.flush();  // let bytes already queued for the ski finish going out
    skiStartRaw(SKI_CMD_ENTER_UPDATE);
    skiRelayState = SKI_RLY_ENTER;
    sendSkiRelayProgress(SKI_PHASE_ENTER, 0, skiImageSize);
    return;
  }

  uint8_t r = skiOpPoll();
  if (r == SKI_OP_PENDING) return;

  switch (skiRelayState) {

    case SKI_RLY_ENTER: {
      if (r != SKI_OP_OK) {
        // Best-effort EXIT in case the ski entered update mode and only its ACK was lost
        Serial.write(SKI_CMD_EXIT_UPDATE);
        skiRelayFinish(SKI_UPD_ENTER_FAILED);
        return;
      }
      uint8_t startPayload[8];
      memcpy(startPayload, &skiImageSize, 4);
      memcpy(startPayload + 4, &skiImageCrc, 4);
      skiStartFrame(SKI_FT_START, startPayload, 8, SKI_START_TIMEOUT_MS, SKI_FRAME_RETRIES);
      skiRelayState = SKI_RLY_START;
      break;
    }

    case SKI_RLY_START:
      if (r != SKI_OP_OK) { skiRelayAbort(SKI_UPD_TRANSFER_FAILED); return; }
      skiRelayBytes = 0;
      skiRelaySeq = 0;
      sendSkiRelayProgress(SKI_PHASE_SEND, 0, skiImageSize);
      skiRelaySendChunk();
      break;

    case SKI_RLY_CHUNK:
      if (r != SKI_OP_OK) { skiRelayAbort(SKI_UPD_TRANSFER_FAILED); return; }
      skiRelayBytes += skiRelayChunkLen;
      skiRelaySeq++;
      if ((skiRelaySeq % SKI_PROGRESS_EVERY) == 0) sendSkiRelayProgress(SKI_PHASE_SEND, skiRelayBytes, skiImageSize);
      if (skiRelayBytes < skiImageSize) {
        skiRelaySendChunk();
      } else {
        skiStartFrame(SKI_FT_END, nullptr, 0, SKI_END_TIMEOUT_MS, SKI_FRAME_RETRIES);
        skiRelayState = SKI_RLY_END;
        sendSkiRelayProgress(SKI_PHASE_VERIFY, skiRelayBytes, skiImageSize);
      }
      break;

    case SKI_RLY_END: {
      if (r != SKI_OP_OK) { skiRelayAbort(SKI_UPD_TRANSFER_FAILED); return; }
      // One attempt, no retries. If the ski refuses it is still running its old
      // firmware. If the ACK is lost the ski may be rewriting its flash anyway,
      // so that case falls through to the same wait as an ACK.
      uint8_t commitPayload[8];
      memcpy(commitPayload, &skiImageSize, 4);
      memcpy(commitPayload + 4, &skiImageCrc, 4);
      skiStartFrame(SKI_FT_COMMIT, commitPayload, 8, SKI_COMMIT_TIMEOUT_MS, 1);
      skiRelayState = SKI_RLY_COMMIT;
      sendSkiRelayProgress(SKI_PHASE_COMMIT, skiRelayBytes, skiImageSize);
      break;
    }

    case SKI_RLY_COMMIT:
      if (r == SKI_OP_NAK) { skiRelayAbort(SKI_UPD_COMMIT_REJECTED); return; }
      // After COMMIT only version queries are ever sent to the ski
      skiRelayMarkMs = millis();
      skiRelayConfirmsLeft = 2;
      skiStartVersionQuery();
      skiRelayState = SKI_RLY_REBOOT;
      sendSkiRelayProgress(SKI_PHASE_REBOOT, skiRelayBytes, skiImageSize);
      break;

    case SKI_RLY_REBOOT:
      if (r == SKI_OP_OK) {
        skiRelayNewVer = skiOpVersion;
        skiRelayCheckNewVersion();
      } else if (millis() - skiRelayMarkMs < SKI_REBOOT_WAIT_MS) {
        skiStartVersionQuery();
      } else {
        skiRelayFinish(SKI_UPD_NO_RESPONSE);
      }
      break;

    case SKI_RLY_CONFIRM:
      if (r == SKI_OP_OK) skiRelayNewVer = skiOpVersion;
      skiRelayCheckNewVersion();
      break;

    case SKI_RLY_ABORT:
      skiStartRaw(SKI_CMD_EXIT_UPDATE);
      skiRelayState = SKI_RLY_EXIT;
      break;

    case SKI_RLY_EXIT:
      // Control output resumes whether or not the ACK arrived (the ski also
      // leaves update mode on its own idle timeout)
      skiRelayFinish(skiRelayFailReason);
      break;
  }
}

// --------------------------------------------------
// Public
// --------------------------------------------------

// The ski sends its battery as '%' followed by the percentage in 1 to 3 ASCII
// digits (main.cpp updateBatteryStats()). Call right after reading the '%'.
// The digits are read here so they never reach handleCommand(), where '3',
// '4' and '5' invert controls and calibrate. Digits arriving more than
// SKI_BATTERY_WAIT_MS after the '%' are left alone. The value is not kept yet.
void consumeSkiBattery() {
  uint8_t digits = 0;
  unsigned long start = millis();
  while (digits < SKI_BATTERY_MAX_DIGITS && millis() - start <= SKI_BATTERY_WAIT_MS) {
    int c = Serial.peek();
    if (c < 0) continue;              // next digit still on the wire
    if (c < '0' || c > '9') break;    // not part of the battery report
    Serial.read();
    digits++;
  }
}

// Asks for a relay of the stored image. Safe to call from the BLE host task.
void skiRelayRequest() {
  skiRelayRequested = true;
}

// True from the phone's relay command until the result has been reported.
bool skiRelayBusy() {
  return skiRelayRequested || skiRelayState != SKI_RLY_IDLE;
}

// True while this file is waiting on bytes from the ski. loop() must leave
// Serial alone then, or it would read a reply as a control command.
bool skiLinkBusy() {
  return skiOpKind != SKI_OP_NONE || skiRelayState != SKI_RLY_IDLE;
}

// Called every loop().
void skiLinkService() {
  if (skiRelayState != SKI_RLY_IDLE) {
    skiRelayStep();
    return;
  }

  if (skiOpKind == SKI_OP_VERSION) {
    if (skiOpPoll() == SKI_OP_OK) skiSetVersion(skiOpVersion);
    return;
  }

  if (skiRelayRequested) {
    skiRelayRequested = false;
    skiRelayBegin();
    return;
  }

  // Until the ski has answered once, ask every few seconds. Runs with or
  // without sensors connected.
  static unsigned long lastQueryMs = 0;
  if (skiVersion.length() == 0 && !otaBusy() && millis() - lastQueryMs >= SKI_VERSION_POLL_MS) {
    lastQueryMs = millis();
    skiStartVersionQuery();
  }
}

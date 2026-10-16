// SkiUpdate.cpp
// Ski side of the firmware update relayed by the radio controller. Ported
// from the bench sketch SkiVersionTest.ino (TetraskiController/ski-controller),
// without its LED patterns: LED_BUILTIN is pin 13, JOY_UNIT_POWER on this board.
//
// The version query ('V' -> 'V' [major][minor][patch]) stays in
// processBTCommands() in main.cpp.
//
// Update-mode wire format (everything after ENTER until EXIT):
//   radio to ski:  [0xA5][type][len][payload, len bytes][crc16 lo][crc16 hi]
//   ski to radio:  [0xA5][status][arg lo][arg hi][crc16 lo][crc16 hi]
//   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over every byte
//   between the 0xA5 and the crc bytes.
//   Frame types: START 0x10 (size u32 LE, crc32 u32 LE)
//                CHUNK 0x11 (seq u16 LE, then image bytes)
//                END   0x12 (verify what was received)
//                ABORT 0x13 (discard the buffer)
//                COMMIT 0x14 (size u32 LE, crc32 u32 LE, must match the verified image)
//   Status: ACK 0x06 (arg = seq for CHUNK, else 0), NAK 0x15 (arg = error code)
// Raw ENTER (0x01) and EXIT (0x05) bytes are accepted only between frames and
// only while no image buffer is open, so a data byte that happens to equal
// 0x05 can never end an update by accident.

#include "SkiUpdate.h"

extern "C" {
  #include "FlashTxx.h"
}

extern const char skiVersionTag[];  // SKI_UPDATE_VERSION_TAG in main.cpp

#define SKI_UPDATE_IDLE_TIMEOUT_MS 10000  // no bytes this long in update mode -> leave it

#define FRAME_SOF          0xA5
#define FRAME_MAX_PAYLOAD  255
#define FRAME_INTERBYTE_TIMEOUT_MS 20     // bytes of one frame arrive back to back, a longer gap means it is broken

#define FT_START  0x10
#define FT_CHUNK  0x11
#define FT_END    0x12
#define FT_ABORT  0x13
#define FT_COMMIT 0x14

#define RS_ACK  0x06
#define RS_NAK  0x15

#define ERR_FRAME_CRC     0x01
#define ERR_STATE         0x02
#define ERR_LENGTH        0x03
#define ERR_SIZE          0x04
#define ERR_NO_BUFFER     0x05
#define ERR_SEQ           0x06
#define ERR_WRITE         0x07
#define ERR_VERIFY_SIZE   0x08
#define ERR_VERIFY_CRC    0x09
#define ERR_WRONG_TARGET  0x0A

static HardwareSerial* skiLink = nullptr;

static bool updateMode = false;
static unsigned long lastUpdateByteMs = 0;
static uint32_t strayBytes = 0;

enum RxState { RX_IDLE, RX_TYPE, RX_LEN, RX_PAYLOAD, RX_CRC_LO, RX_CRC_HI };
static RxState rxState = RX_IDLE;
static uint8_t rxType = 0;
static uint8_t rxLen = 0;
static uint8_t rxIndex = 0;
static uint8_t rxPayload[FRAME_MAX_PAYLOAD];
static uint16_t rxCrc = 0;

static bool bufferActive = false;
static bool imageVerified = false;
static uint32_t bufAddr = 0;
static uint32_t bufSize = 0;
static uint32_t imgSize = 0;
static uint32_t imgCrc = 0;
static uint32_t bytesWritten = 0;
static uint16_t nextSeq = 0;

static uint16_t crc16Update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return crc;
}

// CRC32 of what is actually stored in the buffer, read back from flash
static uint32_t crc32OfBuffer(uint32_t addr, uint32_t len) {
#if defined(__IMXRT1062__)
  arm_dcache_delete((void*)(uintptr_t)addr, (len + 31) & ~31u);  // drop cached copies of flash we just wrote
#endif
  return ~crc32Update(0xFFFFFFFFu, (const uint8_t*)(uintptr_t)addr, len);
}

static uint32_t readU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void sendReply(uint8_t status, uint16_t arg) {
  uint8_t f[6] = { FRAME_SOF, status, (uint8_t)(arg & 0xFF), (uint8_t)(arg >> 8), 0, 0 };
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, f[1]);
  crc = crc16Update(crc, f[2]);
  crc = crc16Update(crc, f[3]);
  f[4] = crc & 0xFF;
  f[5] = crc >> 8;
  skiLink->write(f, 6);
}

// Frees the flash buffer. FlasherX finds the end of the running code by
// scanning for the first erased word, so a buffer left holding partial data
// would shrink the next one. Every failure path has to end up here.
static void releaseBuffer() {
  if (bufferActive) {
    firmware_buffer_free(bufAddr, bufSize);
    bufferActive = false;
  }
  imageVerified = false;
}

static void handleStart() {
  if (rxLen != 8) { sendReply(RS_NAK, ERR_LENGTH); return; }

  releaseBuffer();  // leftover from an earlier attempt, or a repeated START

  uint32_t size = readU32(rxPayload);
  uint32_t crc  = readU32(rxPayload + 4);
  if (size == 0) { sendReply(RS_NAK, ERR_SIZE); return; }

  if (firmware_buffer_init(&bufAddr, &bufSize) == NO_BUFFER_TYPE) {
    sendReply(RS_NAK, ERR_NO_BUFFER);
    return;
  }
  bufferActive = true;
  if (size > bufSize) {
    releaseBuffer();
    sendReply(RS_NAK, ERR_SIZE);
    return;
  }

  imgSize = size;
  imgCrc = crc;
  bytesWritten = 0;
  nextSeq = 0;
  sendReply(RS_ACK, 0);
}

static void handleChunk() {
  if (!bufferActive) { sendReply(RS_NAK, ERR_STATE); return; }
  if (rxLen < 3)     { sendReply(RS_NAK, ERR_LENGTH); return; }

  uint16_t seq = (uint16_t)rxPayload[0] | ((uint16_t)rxPayload[1] << 8);
  uint32_t dataLen = rxLen - 2;

  // The radio never got our ACK and sent this chunk again. Programming a word
  // twice would corrupt it, so acknowledge without writing.
  if (nextSeq > 0 && seq == (uint16_t)(nextSeq - 1)) {
    sendReply(RS_ACK, seq);
    return;
  }
  if (seq != nextSeq) { sendReply(RS_NAK, ERR_SEQ); return; }
  if (bytesWritten + dataLen > imgSize) { sendReply(RS_NAK, ERR_LENGTH); return; }

  bool isLast = (bytesWritten + dataLen == imgSize);
  if (!isLast && (dataLen % 4) != 0) { sendReply(RS_NAK, ERR_LENGTH); return; }

  // flash_write_block() wants 4-byte aligned address and length, so the final
  // partial word is padded with 0xFF (erased flash)
  static char block[FRAME_MAX_PAYLOAD + 4] __attribute__ ((aligned (4)));
  memcpy(block, rxPayload + 2, dataLen);
  uint32_t writeLen = dataLen;
  while (writeLen % 4) block[writeLen++] = (char)0xFF;

  if (flash_write_block(bufAddr + bytesWritten, block, writeLen) != 0) {
    sendReply(RS_NAK, ERR_WRITE);
    return;
  }
  bytesWritten += dataLen;
  nextSeq++;
  sendReply(RS_ACK, seq);
}

static void handleEnd() {
  if (!bufferActive) { sendReply(RS_NAK, ERR_STATE); return; }
  if (bytesWritten != imgSize) { sendReply(RS_NAK, ERR_VERIFY_SIZE); return; }
  if (crc32OfBuffer(bufAddr, imgSize) != imgCrc) { sendReply(RS_NAK, ERR_VERIFY_CRC); return; }
  if (!check_flash_id(bufAddr, imgSize)) { sendReply(RS_NAK, ERR_WRONG_TARGET); return; }

  imageVerified = true;
  sendReply(RS_ACK, 0);
}

// Erasing the buffer takes a while, the radio waits for this reply with a long timeout
static void handleAbort() {
  releaseBuffer();
  sendReply(RS_ACK, 0);
}

// Rewrites program flash from the verified buffer and reboots. Every check
// below has to pass first, because after flash_move() starts there is no
// going back: the old firmware is being overwritten sector by sector.
static void handleCommit() {
  if (rxLen != 8) { sendReply(RS_NAK, ERR_LENGTH); return; }
  if (!bufferActive || !imageVerified) { sendReply(RS_NAK, ERR_STATE); return; }
  if (readU32(rxPayload) != imgSize) { sendReply(RS_NAK, ERR_VERIFY_SIZE); return; }
  if (readU32(rxPayload + 4) != imgCrc) { sendReply(RS_NAK, ERR_VERIFY_CRC); return; }
  if (crc32OfBuffer(bufAddr, imgSize) != imgCrc) { sendReply(RS_NAK, ERR_VERIFY_CRC); return; }  // one last look at what is stored

  Serial.println("Ski update: commit, rewriting flash and rebooting");
  Serial.flush();
  sendReply(RS_ACK, 0);
  skiLink->flush();  // the ACK must be on the wire before the UART goes away
  skiLink->end();    // no UART interrupts while program flash is being rewritten

  flash_move(FLASH_BASE_ADDR, bufAddr, imgSize);  // erases and rewrites program flash, then reboots, does not return
}

static void processFrame() {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, rxType);
  crc = crc16Update(crc, rxLen);
  for (uint8_t i = 0; i < rxLen; i++) crc = crc16Update(crc, rxPayload[i]);
  if (crc != rxCrc) {
    sendReply(RS_NAK, ERR_FRAME_CRC);
    return;
  }

  switch (rxType) {
    case FT_START:  handleStart();  break;
    case FT_CHUNK:  handleChunk();  break;
    case FT_END:    handleEnd();    break;
    case FT_ABORT:  handleAbort();  break;
    case FT_COMMIT: handleCommit(); break;
    default:        sendReply(RS_NAK, ERR_STATE); break;
  }
}

static void leaveUpdateMode(bool timedOut) {
  updateMode = false;
  rxState = RX_IDLE;
  releaseBuffer();  // safety net, normally ABORT has already done this
  if (timedOut) Serial.println("Ski update: left update mode, timed out waiting for the radio");
  else if (strayBytes > 0) { Serial.print("Ski update: left update mode, stray bytes: "); Serial.println(strayBytes); }
  else Serial.println("Ski update: left update mode");
}

static void handleUpdateModeByte(uint8_t b) {
  lastUpdateByteMs = millis();

  switch (rxState) {
    case RX_IDLE:
      if (b == FRAME_SOF) {
        rxState = RX_TYPE;
      } else if (b == SKI_CMD_ENTER_UPDATE && !bufferActive) {
        skiLink->write(SKI_ACK);  // repeated ENTER, the first ACK may have been lost
      } else if (b == SKI_CMD_EXIT_UPDATE && !bufferActive) {
        skiLink->write(SKI_ACK);
        leaveUpdateMode(false);
      } else {
        strayBytes++;
      }
      break;
    case RX_TYPE:
      rxType = b;
      rxState = RX_LEN;
      break;
    case RX_LEN:
      rxLen = b;
      rxIndex = 0;
      rxState = (rxLen > 0) ? RX_PAYLOAD : RX_CRC_LO;
      break;
    case RX_PAYLOAD:
      rxPayload[rxIndex++] = b;
      if (rxIndex >= rxLen) rxState = RX_CRC_LO;
      break;
    case RX_CRC_LO:
      rxCrc = b;
      rxState = RX_CRC_HI;
      break;
    case RX_CRC_HI:
      rxCrc |= (uint16_t)b << 8;
      rxState = RX_IDLE;
      processFrame();
      break;
  }
}

void skiUpdateRun(HardwareSerial& serial) {
  skiLink = &serial;
  updateMode = true;
  strayBytes = 0;
  rxState = RX_IDLE;
  lastUpdateByteMs = millis();
  skiLink->write(SKI_ACK);
  Serial.print("Ski update: entered update mode, running ");
  Serial.println(skiVersionTag);

  while (updateMode) {
    while (updateMode && skiLink->available()) {
      handleUpdateModeByte(skiLink->read());
    }
    if (!updateMode) break;
    if (rxState != RX_IDLE && millis() - lastUpdateByteMs > FRAME_INTERBYTE_TIMEOUT_MS) {
      rxState = RX_IDLE;  // half a frame never finished (a corrupted length byte, say), the radio will resend it
    }
    if (millis() - lastUpdateByteMs > SKI_UPDATE_IDLE_TIMEOUT_MS) {
      leaveUpdateMode(true);
    }
  }
}

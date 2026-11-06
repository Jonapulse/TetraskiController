// BenchTools.ino
// -------------------------------------------------------------------------
// BENCH ONLY. Compiled only with BENCH_TOOLS 1 (TetraEMGControl.ino). To
// remove: set BENCH_TOOLS to 0, or delete this tab and every #if BENCH_TOOLS
// block. Never release a build with BENCH_TOOLS 1.
//
// Bench command characteristic (WRITE), one message per write, up to 64 bytes:
//   [dest][bytes...]
//   dest 'C'  radio command. Bench commands below, anything else goes to
//             handleCommand() byte by byte, like the phone command path.
//   dest 'I'  inject: the bytes are read as if the ski had sent them
//             (battery handling and handleCommand() included).
//   dest 'P'  passthrough: the bytes are written to the ski as they are.
//
// Bench radio commands (dest 'C'):
//   &R<size><ms>  radio sends <size> filler bytes (2 digits, 00 = stop) every
//                 <ms> ms (1 digit, 0 = every loop). Auto-stops after 30 s.
//   &S<size><ms>  same, ski sends to the radio. Relayed to a bench ski build.
//   *<nn>         relayed to the ski: report battery as nn% from now on.
//   #             relayed to the ski: report loop timing, then reset it.
//   !             toggle the 1.0.11 battery fix (on at boot). Off reproduces
//                 1.0.10, where the digits after '%' reach handleCommand().
//
// Bench echo characteristic (NOTIFY): [type][bytes...]
//   'S'  bytes received from the ski, as they arrived (filler bytes excluded)
//   'R'  text from the radio (command results, burst results)
//   'K'  text report from a bench ski build
//
// Filler byte 0x80 is ignored by every radio and ski firmware. Ski text reports
// are sent as 0x81, text with the high bit set on every byte, 0x82, so no byte
// of a report can be read as a command by a radio without this file.
// -------------------------------------------------------------------------

#if BENCH_TOOLS

#define BENCH_CMD_CHAR_UUID   "1c0f52a7-2d68-4220-923b-8ff10aa1432b"
#define BENCH_ECHO_CHAR_UUID  "2c5e88a7-430d-4db0-a2ba-1419ccdd114e"

#define BENCH_MSG_MAX         64
#define BENCH_QUEUE_LEN       8
#define BENCH_INJECT_MAX      128
#define BENCH_ECHO_MAX        180
#define BENCH_ECHO_FLUSH_MS   50
#define BENCH_FILLER_REPORT_MS 1000
#define BENCH_BURST_MAX       99
#define BENCH_BURST_LIMIT_MS  30000
#define BENCH_FILLER          0x80
#define BENCH_TEXT_START      0x81
#define BENCH_TEXT_END        0x82
#define BENCH_TEXT_MAX        120

typedef struct {
  uint8_t len;
  uint8_t data[BENCH_MSG_MAX];
} BenchMsg;

static QueueHandle_t benchQueue = nullptr;
static NimBLECharacteristic* pBenchEchoChar = nullptr;

static uint8_t  benchInject[BENCH_INJECT_MAX];
static uint16_t benchInjectHead = 0;
static uint16_t benchInjectCount = 0;
static bool     benchLastRxInjected = false;

static uint8_t  benchEcho[BENCH_ECHO_MAX];
static uint16_t benchEchoLen = 0;
static unsigned long benchEchoStartMs = 0;
static uint32_t benchEchoDropped = 0;

static uint32_t benchFillerCount = 0;
static unsigned long benchFillerMarkMs = 0;

static bool benchBatteryFix = true;

static uint8_t  benchBurstSize = 0;          // 0 = no burst running
static uint8_t  benchBurstIntervalMs = 0;
static unsigned long benchBurstStartMs = 0;
static unsigned long benchBurstLastMs = 0;
static uint32_t benchBurstMaxWriteUs = 0;
static uint32_t benchBurstCount = 0;
static uint8_t  benchBurstBuf[BENCH_BURST_MAX];

static bool    benchInText = false;
static char    benchText[BENCH_TEXT_MAX + 1];
static uint8_t benchTextLen = 0;

// Runs on the NimBLE host task: only queue the message, loop() handles it.
class BenchCmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    std::string v = pChar->getValue();
    if (v.empty() || !benchQueue) return;
    BenchMsg m;
    m.len = v.size() > BENCH_MSG_MAX ? BENCH_MSG_MAX : v.size();
    memcpy(m.data, v.data(), m.len);
    xQueueSend(benchQueue, &m, 0);
  }
};

void setupBenchService(NimBLEService* svc) {
  benchQueue = xQueueCreate(BENCH_QUEUE_LEN, sizeof(BenchMsg));
  NimBLECharacteristic* cmdChar = svc->createCharacteristic(BENCH_CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  cmdChar->setCallbacks(new BenchCmdCallbacks());
  pBenchEchoChar = svc->createCharacteristic(BENCH_ECHO_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
}

// --------------------------------------------------
// Echo to the app
// --------------------------------------------------

static size_t benchNotifyMax() {
  uint16_t mtu = pPhoneServer->getPeerMTU(phoneConnHandle);
  size_t max = mtu > 3 ? mtu - 3 : 20;
  return max > BENCH_ECHO_MAX + 1 ? BENCH_ECHO_MAX + 1 : max;
}

// Sends one echo notification, split into several if the MTU is small.
static void benchNotify(uint8_t type, const uint8_t* data, size_t len) {
  if (!phoneConnected || !pBenchEchoChar) return;
  size_t room = benchNotifyMax() - 1;
  uint8_t buf[BENCH_ECHO_MAX + 1];
  size_t sent = 0;
  do {
    size_t n = len - sent > room ? room : len - sent;
    buf[0] = type;
    memcpy(buf + 1, data + sent, n);
    pBenchEchoChar->setValue(buf, n + 1);
    pBenchEchoChar->notify();
    sent += n;
  } while (sent < len);
}

// Text to the app. Never to Serial: that is the ski link.
static void benchLog(const char* fmt, ...) {
  char text[BENCH_TEXT_MAX + 1];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);
  if (n < 0) return;
  if (n > BENCH_TEXT_MAX) n = BENCH_TEXT_MAX;
  benchNotify('R', (const uint8_t*)text, n);
}

static void benchFlushEcho() {
  if (benchEchoLen > 0) benchNotify('S', benchEcho, benchEchoLen);
  benchEchoLen = 0;
  if (benchEchoDropped > 0) {
    benchLog("echo: %lu ski bytes not shown (too many at once)", (unsigned long)benchEchoDropped);
    benchEchoDropped = 0;
  }
}

// A byte that really came from the ski. Injected bytes are not echoed.
void benchEchoSki(uint8_t b) {
  if (benchLastRxInjected) return;
  if (benchEchoLen == 0) benchEchoStartMs = millis();
  if (benchEchoLen < BENCH_ECHO_MAX) benchEcho[benchEchoLen++] = b;
  else benchEchoDropped++;
}

// --------------------------------------------------
// Ski-side input: injected bytes first, then the real UART
// --------------------------------------------------

int skiRxAvailable() {
  return benchInjectCount + Serial.available();
}

int skiRxPeek() {
  if (benchInjectCount > 0) return benchInject[benchInjectHead];
  return Serial.peek();
}

int skiRxRead() {
  if (benchInjectCount > 0) {
    uint8_t b = benchInject[benchInjectHead];
    benchInjectHead = (benchInjectHead + 1) % BENCH_INJECT_MAX;
    benchInjectCount--;
    benchLastRxInjected = true;
    return b;
  }
  benchLastRxInjected = false;
  return Serial.read();
}

// Called with every byte loop() reads from the ski side. True if the byte was
// a filler byte or part of a ski text report, which then go no further.
bool benchSkiByte(uint8_t b) {
  if (benchInText) {
    if (b == BENCH_TEXT_END) {
      benchInText = false;
      benchNotify('K', (const uint8_t*)benchText, benchTextLen);
      return true;
    }
    if (b & 0x80) {
      if (benchTextLen < BENCH_TEXT_MAX) benchText[benchTextLen++] = b & 0x7F;
      return true;
    }
    benchInText = false;  // report cut off, this byte is ordinary traffic
  }
  if (b == BENCH_TEXT_START) {
    benchInText = true;
    benchTextLen = 0;
    return true;
  }
  if (b == BENCH_FILLER && !benchLastRxInjected) {
    benchFillerCount++;
    return true;
  }
  benchEchoSki(b);
  return false;
}

bool benchBatteryFixOn() {
  return benchBatteryFix;
}

// --------------------------------------------------
// Commands from the app
// --------------------------------------------------

static bool benchDigits(const uint8_t* p, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (p[i] < '0' || p[i] > '9') return false;
  }
  return true;
}

static bool benchToSki(const uint8_t* p, uint8_t n) {
  if (skiLinkPaused || skiLinkBusy()) {
    benchLog("ski link busy (update or version query), nothing sent");
    return false;
  }
  Serial.write(p, n);
  return true;
}

static void benchBurstStop(const char* why) {
  if (benchBurstSize == 0) return;
  benchLog("radio burst %s: %lu bursts, longest write %lu us", why,
           (unsigned long)benchBurstCount, (unsigned long)benchBurstMaxWriteUs);
  benchBurstSize = 0;
}

static void benchBurstStart(uint8_t size, uint8_t intervalMs) {
  benchBurstStop("stopped");
  if (size == 0) return;
  memset(benchBurstBuf, BENCH_FILLER, sizeof(benchBurstBuf));
  benchBurstSize = size;
  benchBurstIntervalMs = intervalMs;
  benchBurstStartMs = millis();
  benchBurstLastMs = 0;
  benchBurstMaxWriteUs = 0;
  benchBurstCount = 0;
  if (intervalMs == 0) benchLog("radio burst: %u bytes every loop, 30 s", size);
  else benchLog("radio burst: %u bytes every %u ms, 30 s", size, intervalMs);
}

static void benchRadioCommand(const uint8_t* p, uint8_t n) {
  if (p[0] == '&') {
    if (n != 5 || (p[1] != 'R' && p[1] != 'S') || !benchDigits(p + 2, 3)) {
      benchLog("usage: &R or &S, size 00-99, interval 0-9 ms (0 = every loop), e.g. &R093");
      return;
    }
    if (p[1] == 'S') {
      if (benchToSki(p, n)) benchLog("relayed to ski: %.*s", n, (const char*)p);
      return;
    }
    benchBurstStart((p[2] - '0') * 10 + (p[3] - '0'), p[4] - '0');
    return;
  }

  if (p[0] == '*' || p[0] == '#') {
    if (benchToSki(p, n)) benchLog("relayed to ski: %.*s", n, (const char*)p);
    return;
  }

  if (p[0] == '!' && n == 1) {
    benchBatteryFix = !benchBatteryFix;
    benchLog(benchBatteryFix ? "battery fix ON (1.0.11): %%nn is swallowed"
                             : "battery fix OFF (1.0.10): digits after %% reach handleCommand");
    return;
  }

  for (uint8_t i = 0; i < n; i++) handleCommand((char)p[i], CMD_SRC_PHONE);
  benchLog("radio command: %.*s", n, (const char*)p);
}

static void benchHandleMsg(const uint8_t* d, uint8_t len) {
  if (len < 2) return;
  const uint8_t* p = d + 1;
  uint8_t n = len - 1;

  switch (d[0]) {
    case 'C':
      benchRadioCommand(p, n);
      break;

    case 'I': {
      uint8_t taken = 0;
      while (taken < n && benchInjectCount < BENCH_INJECT_MAX) {
        benchInject[(benchInjectHead + benchInjectCount) % BENCH_INJECT_MAX] = p[taken++];
        benchInjectCount++;
      }
      benchLog("injected %u bytes as if from the ski%s", taken, taken < n ? " (buffer full, rest dropped)" : "");
      break;
    }

    case 'P':
      if (benchToSki(p, n)) benchLog("sent %u bytes to the ski", n);
      break;

    default:
      benchLog("unknown bench destination 0x%02X", d[0]);
      break;
  }
}

// --------------------------------------------------
// Called every loop()
// --------------------------------------------------
void benchService() {
  BenchMsg m;
  while (benchQueue && xQueueReceive(benchQueue, &m, 0) == pdTRUE) {
    benchHandleMsg(m.data, m.len);
  }

  if (benchBurstSize > 0) {
    if (millis() - benchBurstStartMs >= BENCH_BURST_LIMIT_MS) {
      benchBurstStop("auto-stopped after 30 s");
    } else if (!skiLinkPaused && !skiLinkBusy()
               && (benchBurstIntervalMs == 0 || millis() - benchBurstLastMs >= benchBurstIntervalMs)) {
      benchBurstLastMs = millis();
      uint32_t t0 = micros();
      Serial.write(benchBurstBuf, benchBurstSize);
      uint32_t dt = micros() - t0;
      if (dt > benchBurstMaxWriteUs) benchBurstMaxWriteUs = dt;
      benchBurstCount++;
    }
  }

  if (millis() - benchFillerMarkMs >= BENCH_FILLER_REPORT_MS) {
    if (benchFillerCount > 0) {
      benchLog("ski filler: %lu bytes received in %lu ms", (unsigned long)benchFillerCount,
               (unsigned long)(millis() - benchFillerMarkMs));
    }
    benchFillerCount = 0;
    benchFillerMarkMs = millis();
  }

  if (benchEchoLen > 0 && (benchEchoLen >= BENCH_ECHO_MAX || millis() - benchEchoStartMs >= BENCH_ECHO_FLUSH_MS)) {
    benchFlushEcho();
  }
}

#endif  // BENCH_TOOLS

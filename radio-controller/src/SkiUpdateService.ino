// SkiUpdateService.ino
// -------------------------------------------------------------------------
// Ski controller firmware update pathway (radio side).
//   Step 1: version query round trip over the Serial link to the ski.
//   Step 2: enter/exit update mode handshake.
//
// Protocol bytes (radio to ski):
//   0x56 'V'  version query, ski replies [lengthByte][ASCII chars]
//   0x01      enter update mode, ski replies SKI_ACK
//   0x05      exit update mode, ski replies SKI_ACK
// Protocol bytes (ski to radio):
//   0x06      ACK
// 0x02, 0x03, 0x04 are reserved for chunk data, commit, and abort.
//
// Control bytes are non-printable so they can never be confused with the
// ASCII ski commands ('0'-'4', 'l','r','u','d','3','4','5','g','h','o','p',
// 'x','z') already on this wire.
//
// While skiLinkPaused is true (defined in TetraEMGControl.ino), nothing may
// be printed on Serial: the ski would count it as stray traffic.
// -------------------------------------------------------------------------

#define SKI_CMD_VERSION_QUERY    0x56
#define SKI_CMD_ENTER_UPDATE     0x01
#define SKI_CMD_EXIT_UPDATE      0x05
#define SKI_ACK                  0x06

#define SKI_VERSION_TIMEOUT_MS   500
#define SKI_VERSION_MAX_LEN      32

#define SKI_HANDSHAKE_TIMEOUT_MS 300
#define SKI_HANDSHAKE_RETRIES    3

#define SKI_BENCH_CYCLE_MS       10000  // idle time between bench enter attempts
#define SKI_BENCH_HOLD_MS        3000   // how long the bench test stays in update mode

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

// Bench test: every SKI_BENCH_CYCLE_MS, enter update mode, hold for
// SKI_BENCH_HOLD_MS without sending anything, then exit. Results are printed
// only after control output has resumed.
void checkSkiUpdateModeBench() {
  static unsigned long lastActionMs = 0;
  unsigned long now = millis();

  if (!skiLinkPaused) {
    if (now - lastActionMs < SKI_BENCH_CYCLE_MS) return;
    lastActionMs = now;
    if (!enterSkiUpdateMode()) {
      Serial.println("Ski update mode: no ACK on enter, control output resumed");
    }
    return;
  }

  if (now - lastActionMs < SKI_BENCH_HOLD_MS) return;
  lastActionMs = now;
  bool ok = exitSkiUpdateMode();
  Serial.println(ok ? "Ski update mode: enter/exit round trip OK"
                    : "Ski update mode: no ACK on exit");
}

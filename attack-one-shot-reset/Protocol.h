// Shared eight-byte application protocol used by A, B, and the attacker C.
// Two-byte fields are encoded in network order (big-endian).
#ifndef CANLAB_PROTOCOL_H
#define CANLAB_PROTOCOL_H
#include <stdint.h>

namespace Lab {
// Standard 11-bit identifiers and fixed payload size.
const uint16_t SENSOR_ID = 0x120;    // owner B
const uint16_t MOTOR_ID = 0x220;     // owner A
const uint16_t ATTACKER_ID = 0x320;  // own ID of node C (distinct from spoofed SENSOR_ID)
const uint8_t VERSION = 1;
const uint8_t DLC = 8;

// Application limits and the false value used by valid spoofing attacks.
const uint16_t STOP_CM = 50;
const uint16_t MAX_CM = 5000;
const uint16_t SPOOF_STOP_CM = 10;   // <= STOP_CM (50) -> forces STOP in A while remaining validSensor()

// Periods and deadlines used by the cooperative main loops.
const uint32_t SAMPLE_MS = 250;
const uint32_t STALE_MS = 1250;
const uint32_t TX_DEADLINE_MS = 100;
const uint32_t SPOOF_INTERVAL_MS = 120;

// Values carried in byte 5 and sequence-classification results.
enum MotorState : uint8_t { STOP = 0, RUN = 1, SENSOR_FAULT = 2 };
enum Order : uint8_t { FIRST, FORWARD, DUPLICATE, OLD };

// Decodes one big-endian 16-bit field.
inline uint16_t u16(const uint8_t *p) {
  return (uint16_t(p[0]) << 8) | p[1];
}

// Encodes one 16-bit value in big-endian order.
inline void put16(uint8_t *p, uint16_t v) {
  p[0] = uint8_t(v >> 8);
  p[1] = uint8_t(v);
}

// Wrap-safe interval check for values returned by millis().
inline bool elapsed(uint32_t now, uint32_t then, uint32_t interval) {
  return uint32_t(now - then) >= interval;
}

// Classifies a 16-bit sequence number, including natural wrap-around.
inline Order order(bool have, uint16_t previous, uint16_t next) {
  if (!have) return FIRST;
  const uint16_t delta = uint16_t(next - previous);
  if (!delta) return DUPLICATE;
  return delta < 0x8000U ? FORWARD : OLD;
}

// Detects Parrot's reserved eight-byte zero Dmessage.
inline bool allZero(const uint8_t *p) {
  for (uint8_t i = 0; i < DLC; ++i) {
    if (p[i]) return false;
  }
  return true;
}

// Generates the repeating distance pattern published by sensor B.
inline uint16_t simulatedDistance(uint16_t seq) {
  const uint16_t cm[8] = {250, 200, 100, 50, 25, 50, 100, 200};
  return cm[seq % 8];
}

// Builds the common VERSION/sequence/value/flags/session payload.
inline void encode(uint8_t *p, uint16_t seq, uint16_t cm,
                   uint8_t flags, uint16_t session) {
  p[0] = VERSION;
  put16(p + 1, seq);
  put16(p + 3, cm);
  p[5] = flags;
  put16(p + 6, session);
}

// Checks the fields that are valid for a SENSOR message from B.
inline bool validSensor(const uint8_t *p) {
  return p[0] == VERSION && u16(p + 3) <= MAX_CM && p[5] <= 1 && u16(p + 6) != 0;
}

// Checks A's echoed status and its distance-dependent motor decision.
inline bool validStatus(const uint8_t *p) {
  if (p[0] != VERSION || u16(p + 3) > MAX_CM || !u16(p + 6)) return false;
  const uint16_t cm = u16(p + 3);
  return p[5] == SENSOR_FAULT ||
         p[5] == (cm <= STOP_CM ? STOP : RUN);
}
}  // namespace Lab
#endif

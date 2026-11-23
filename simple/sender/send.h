#ifndef SEND_H
#define SEND_H
#include "common.h"

const uint8_t DELAY = 

const uint8_t DISTANCES[] = { 50, 150, 100, 200, 250 };
const uint8_t N = sizeof(DISTANCES) / sizeof(DISTANCES[0]);

uint8_t index_ = 0;
unsigned long lastSent = 0;

// ---- Parrot state ----
bool parroting = false;
unsigned long lastSpoofAt = 0;
const unsigned long PARROT_HOLD = 10000; // keep defending 10 s after last forgery
uint32_t spoofs = 0;

void printMessage(uint8_t distance) {
  Serial.print("sensor: distance ");
  Serial.print(distance);
  Serial.println(" cm sent");
  if (distance < BREAK_DISTANCE) Serial.println(" [BREAK]");
  else Serial.println(" [CONTINUE]");
}

// One frame with OUR id reaching our receiver is proof of forgery: a
// normal-mode MCP2515 never delivers our own transmissions to ourselves.
void checkForSpoof() {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return;
  uint8_t len;
  uint8_t d[8];
  if (CAN_OK != CAN.readMsgBuf(&len, d)) return;
  if (CAN.getCanId() == SENSOR_ID) {
    ++spoofs;
    lastSpoofAt = millis();
    if (!parroting) {
      parroting = true;
      Serial.print("FORGERY HEARD -> PARROT begins (spoofs seen: ");
      Serial.print(spoofs);
      Serial.println(")");
    }
  }
}

void loop() {
  checkForSpoof();

  unsigned long now = millis();
  if (parroting && elapsed(now, lastSpoofAt, PARROT_HOLD)) {
    parroting = false;
    Serial.println("bus quiet again -> Parrot ends, normal cadence resumes");
  }

  if (parroting) {
    // Parrot: repeat our own frame back-to-back, leaving C no free slot
    if (CAN.sendMsgBuf(SENSOR_ID, 0, DLC, &DISTANCES[index_]) == CAN_OK) {
      Serial.print("parrot: ");
      Serial.println(DISTANCES[index_]);
    }
    index_ = (index_ + 1) % N;
    delay(2);
    return;
  }

  if (!elapsed(now, lastSent, SEND_PERIOD)) return;
  lastSent = now;

  // ID - RTR - DLC - DATA
  if (CAN.sendMsgBuf(SENSOR_ID, 0, DLC, &DISTANCES[index_]) == CAN_OK) {
    printMessage(DISTANCES[index_]);
  } else {
    Serial.println("SEND FAILED");
  }
  printTECREC();
  index_ = (index_ + 1) % N;
}

#endif
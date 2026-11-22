#ifndef COMMON_H
#define COMMON_H

#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const uint32_t SENSOR_ID = 0x120;       // the identity node C will steal
const uint8_t  DLC = 1;                 // one constant frame length for everyone
const unsigned long SEND_PERIOD = 2000; // B sends every 2 s
const uint8_t  BREAK_DISTANCE = 100;

SPISettings CAN_SPI(10000000, MSBFIRST, SPI_MODE0);

uint8_t regRead(uint8_t addr) {
  SPI.beginTransaction(CAN_SPI);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(0x03);
  SPI.transfer(addr);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void printTECREC() {
  Serial.print("  TEC=");
  Serial.print(regRead(0x1C));
  Serial.print(" REC=");
  Serial.println(regRead(0x1D));
}

// Borrowed from Protocol.h (Lab::elapsed). Unsigned subtraction is wrap-
// safe: even when millis() overflows, now - then still gives the true gap.
bool elapsed(unsigned long now, unsigned long then, unsigned long interval) {
  return now - then >= interval;
}

void setup() {
  Serial.begin(115200);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN init fail, retry...");
    delay(500);
  }
  Serial.println("CAN init ok");
}

#endif
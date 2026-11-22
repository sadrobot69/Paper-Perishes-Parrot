#ifndef COMMON_H
#define COMMON_H

#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const uint8_t BREAK_DISTANCE = 100;

// Read flags and error counters.
// --- direct MCP2515 register read (READ opcode 0x03) ---
SPISettings CAN_SPI(10000000, MSBFIRST, SPI_MODE0);

uint8_t regRead(uint8_t addr){
  SPI.beginTransaction(CAN_SPI);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(0x03);
  SPI.transfer(addr);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void setup(){
  Serial.begin(115200);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN init fail, retry...");
    delay(500);
  }
  Serial.println("CAN init ok (sensor)");
}

void printTECREC(){
    Serial.print("  TEC=");
    Serial.print(regRead(0x1C));
    Serial.print(" REC=");
    Serial.println(regRead(0x1D));
}

void printDecision(uint8_t distance){
    if (distance < BREAK_DISTANCE) {
        Serial.println(" [BREAK]");
        return;
    }
    Serial.println(" [CONTINUE]");
}


#endif 

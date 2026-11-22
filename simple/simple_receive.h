// Minimal receiver example: polls CAN and prints ID, payload, TEC, and REC.
#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"

// Shield wiring and application threshold.
const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const uint8_t BREAK_DISTANCE = 100;

// Last frame and reporting state.
uint32_t id;
uint8_t  len;
uint8_t  distance = 0;

unsigned long lastReport = 0;
const unsigned long IDLE_TIME = 5000;

// Direct register reads use the MCP2515 SPI mode and maximum clock.
SPISettings CAN_SPI(10000000, MSBFIRST, SPI_MODE0);

// Reads one MCP2515 register through the SPI READ command.
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

// Initializes Serial and retries until the CAN controller is available.
void setup() {
    Serial.begin(115200);
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN init fail, retry...");
        delay(500);
    }
    Serial.println("CAN init ok (receiver)");
}

// Interprets the one-byte distance using the example threshold.
void printDecision(uint8_t distance){
    if (distance < BREAK_DISTANCE) {
        Serial.println(" [BREAK]");
        return;
    }
    Serial.println(" [CONTINUE]");
}

// Reads and prints one received CAN frame and its controller diagnostics.
void printMSG(unsigned long *lastReport){
    uint8_t rr = 0;
    char buf[96];
    int n = 0;

    rr = CAN.readMsgBuf(&len, &distance);
    id = CAN.getCanId();
    n = sprintf(buf, "RX [%08lX] dlc=%u ", (unsigned long)id, len);
    n += sprintf(buf + n, "%02X ", distance);
    n += sprintf(buf + n, "T/R=%u/%u rr=%u", regRead(0x1C), regRead(0x1D), rr);
    Serial.println(buf);
    printDecision(distance);
    *lastReport = millis();
}

// Periodically reports controller state while no CAN frame is received.
void printIdle(unsigned long *lastReport, unsigned long now, uint8_t rv){
    *lastReport = now;
    Serial.print("idle: checkReceive="); Serial.print(rv);
    Serial.print(" TEC="); Serial.print(regRead(0x1C));
    Serial.print(" REC="); Serial.println(regRead(0x1D));
}

// Polls CAN continuously and emits an idle report every five seconds.
void loop() {
    unsigned long now = 0;
    uint8_t rv = CAN.checkReceive();
    if (rv == CAN_MSGAVAIL) {
        printMSG(&lastReport);
        return;
    }
    now = millis();
    if (now - lastReport >= IDLE_TIME) {
        printIdle(&lastReport, now, rv);
    }
}

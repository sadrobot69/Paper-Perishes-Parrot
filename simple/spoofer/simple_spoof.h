#ifndef SPOOF_H
#define SPOOF_H

#include "common.h"

const uint32_t SPOOF_ID = 0x120;
// Number of times the spoofed ECU needs to 
// send a message with the same time distance.
uint8_t MIN_EVIDENCE = 5; 
// Tolerance percentage. 
uint8_t TOLERANCE = 5; 

uint32_t id;
uint8_t  len;
uint8_t  distance = 0;

unsigned long lastReport = 0;
const unsigned long IDLE_TIME = 5000;

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

void printMSGSpoof (unsigned *lastReport){
    Serial.println("SPOOF-> ");
    printMSG(lastReport);
}

void printIdle(unsigned long *lastReport, unsigned long now, uint8_t rv){
    *lastReport = now;
    Serial.print("idle: checkReceive="); Serial.print(rv);
    Serial.print(" TEC="); Serial.print(regRead(0x1C));
    Serial.print(" REC="); Serial.println(regRead(0x1D));
}

void printIdleSpoof(unsigned long *lastReport, unsigned long now, uint8_t rv) {
    Serial.println("SPOOF-> ");
    printIdle(lastReport, now, rv);
}

void loop() {
    unsigned long now = 0;
    uint8_t rv = CAN.checkReceive();
    if (rv == CAN_MSGAVAIL) {
        printMSGSpoof(&lastReport);
        return;
    }
    now = millis();
    if (now - lastReport >= IDLE_TIME) {
        printIdle(&lastReport, now, rv);
    }
}

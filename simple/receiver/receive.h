#ifndef RECEIVE_H
#define RECEIVE_H

#include "common.h"

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
    n += sprintf(buf + n, "%d ", (unsigned int) distance);
    n += sprintf(buf + n, "T/R=%u/%u rr=%u", regRead(0x1C), regRead(0x1D), rr);
    Serial.println(buf);
    printDecision(distance);
    *lastReport = millis();
}
void printIdle(unsigned long *lastReport, unsigned long now, uint8_t rv){
    *lastReport = now;
    Serial.print("idle: checkReceive="); Serial.print(rv);
    Serial.print(" TEC="); Serial.print(regRead(0x1C));
    Serial.print(" REC="); Serial.println(regRead(0x1D));
}

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

#endif

// Minimal sensor example: sends one distance byte every two seconds.
#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"

// Shield wiring and application threshold.
const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const uint8_t BREAK_DISTANCE = 100;

// CAN identifier, period, and repeating distance test vector.
const uint32_t ID = 0x120;          // this ECU's fixed identifier
const unsigned long DELAY = 2000;

const uint8_t DISTANCES[] = { 50, 150, 100, 200, 250 };   // cm — edit freely, max 255
const uint8_t N = sizeof(DISTANCES) / sizeof(DISTANCES[0]);

// Current test-vector position and last transmission time.
uint8_t index_ = 0;
unsigned long lastSent = 0;

// Direct register reads use the MCP2515 SPI mode and maximum clock.
SPISettings CAN_SPI(10000000, MSBFIRST, SPI_MODE0);

// Reads one MCP2515 register through the SPI READ command.
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

// Initializes Serial and retries until the CAN controller is available.
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

// Waits until the configured interval has elapsed since the last send.
void wait(unsigned long *lastSent){
    unsigned long now = 0;
    while (true) {
        now = millis();
        if (now - *lastSent >= DELAY){
            *lastSent = now;
            return;
        } 
    }
}

// Prints the MCP2515 transmit and receive error counters.
void printTECREC(){
    Serial.print("  TEC=");
    Serial.print(regRead(0x1C));
    Serial.print(" REC=");
    Serial.println(regRead(0x1D));
}

// Interprets the transmitted distance using the example threshold.
void printDecision(uint8_t distance){
    if (distance < BREAK_DISTANCE) {
        Serial.println(" [BREAK]");
        return;
    }
    Serial.println(" [CONTINUE]");
}

// Prints the distance value sent by the simple sensor example.
void printMessage(uint8_t distance) {
    Serial.print("sensor: distance ");
    Serial.print(distance);
    Serial.println(" cm sent");
    printDecision(distance);
}

// Sends the next distance, reports the result, and advances the test vector.
void loop() {
    wait(&lastSent);
    // ID - RTR - DLC - DATA   (payload = one byte: distance in cm)
    if (CAN.sendMsgBuf(ID, 0, 1, &DISTANCES[index_]) == CAN_OK) {
        printMessage(DISTANCES[index_]);
    } else {
        Serial.println("SEND FAILED");
    }
    printTECREC();
    index_ = (index_ + 1) % N;
}


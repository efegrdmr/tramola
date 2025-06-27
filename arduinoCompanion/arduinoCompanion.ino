#include <SPI.h>
#include <LoRa.h>

#define LORA_BAND    433E6
#define SS_PIN       10
#define RST_PIN      9
#define DIO0_PIN     2

const int   BUFFER_SIZE = 256;
byte        serialBuffer[BUFFER_SIZE];
int         bufferIndex    = 0;
unsigned long lastRxTime   = 0;
const unsigned long TX_TIMEOUT = 200; // ms
unsigned long lastHealthCheck = 0;
const unsigned long HEALTH_CHECK_INTERVAL = 30000; // 30 seconds
bool transmissionFailed = false;

void setup() {
  Serial.begin(9600);
  
  // Wait a moment for Serial to connect
  delay(1000);
  
  initLoRa();
  
  // ensure we're listening
  LoRa.receive();
  lastRxTime = millis();
  lastHealthCheck = millis();
}

void initLoRa() {
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  
  // Retry LoRa initialization if it fails
  byte retries = 0;
  while (!LoRa.begin(LORA_BAND)) {
    delay(500);
    if (++retries > 5) {
      delay(1000);
      // Reset the Arduino
      asm volatile ("  jmp 0");
    }
  }

  LoRa.setSyncWord(0x32);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
}

bool sendPacket() {

  
  bool success = false;
  byte attempts = 0;
  
  while (!success && attempts < 3) {
    LoRa.beginPacket();
    LoRa.write(serialBuffer, bufferIndex);
    success = LoRa.endPacket(); // Returns 1 if successful
    
    if (!success) {
      transmissionFailed = true;
      delay(100 * attempts); // Increasing backoff
      attempts++;
    }
  }
  
  LoRa.receive();  // go back into RX mode
  bufferIndex = 0;
  return success;
}

void performHealthCheck() {
  // Instead of checking isTransmitting() which is private,
  // we'll use a simple ping test and monitor our transmissionFailed flag
  
  
  if (transmissionFailed) {
    // Reset the LoRa module
    LoRa.sleep();
    delay(100);
    initLoRa();
    LoRa.receive();
    transmissionFailed = false;
  } else {
    // Send a tiny ping packet as a health check
    LoRa.beginPacket();
    LoRa.write('P'); // Ping byte
    bool pingSuccess = LoRa.endPacket();
    
    if (!pingSuccess) {
      LoRa.sleep();
      delay(100);
      initLoRa();
    }
    
    LoRa.receive(); // Go back to receive mode
  }
  
  lastHealthCheck = millis();
}

void loop() {
  // 1) Read from Serial
  while (Serial.available()) {
    byte b = Serial.read();
    lastRxTime = millis();

    // flush on CR, LF or overflow
    if (b == '\n' || b == '\r' || bufferIndex >= BUFFER_SIZE) {
      if (bufferIndex > 0) {
        sendPacket();
      }
    } else {
      serialBuffer[bufferIndex++] = b;
    }
  }

  // enforce a timeout flush if user never sends newline
  if (bufferIndex > 0 && millis() - lastRxTime > TX_TIMEOUT) {
    sendPacket();
  }

  // 2) Receive LoRa packets - No delay before checking for packets
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {

    
    while (LoRa.available()) {
      byte rxByte = LoRa.read();
      // Skip printing the ping byte
      if (packetSize == 1 && rxByte == 'P') {
        
      } else {
        Serial.write(rxByte);
      }
    }
    Serial.println();
  }
  
  // Periodic health check
  if (millis() - lastHealthCheck > HEALTH_CHECK_INTERVAL) {
    performHealthCheck();
  }
  
  // Single small delay to prevent CPU hogging
  delay(10); // Reduced from 70ms to be more responsive
}
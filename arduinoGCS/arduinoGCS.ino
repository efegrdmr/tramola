#include <SPI.h>
#include <LoRa.h>

#define LORA_BAND 433E6 

const int BUFFER_SIZE = 256;
byte serialBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial); // wait for Serial

  if (!LoRa.begin(LORA_BAND)) {
    while (1);
  }

  LoRa.setSyncWord(0x32);           // Valid sync word (1 byte)
  LoRa.setSpreadingFactor(7);       
  LoRa.setSignalBandwidth(125E3);   
  LoRa.setCodingRate4(8);           
}

void loop() {
  // 1) Read bytes from Serial and send when newline or buffer full
  while (Serial.available()) {
    byte b = Serial.read();

    if (b == '\n' || bufferIndex >= BUFFER_SIZE) {
      if (bufferIndex > 0) {
        LoRa.beginPacket();
        LoRa.write(serialBuffer, bufferIndex);
        LoRa.endPacket();
        bufferIndex = 0;
      }
    } else {
      serialBuffer[bufferIndex++] = b;
    }
  }

  delay(10);

  // 2) Receive LoRa packets and write raw bytes to Serial
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    while (LoRa.available()) {
      byte b = LoRa.read();
      Serial.write(b);  // Output raw byte
    }
    Serial.write('\n');
  }

  delay(10);
}

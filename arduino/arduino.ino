#include <SPI.h>
#include <LoRa.h>

const long frequency = 915E6; // Adjust according to your region (e.g., 433E6, 868E6, etc.)
const int csPin    = 10;
const int resetPin = 9;
const int irqPin   = 2;

void setup() {
    Serial.begin(9600);
    while (!Serial);
    LoRa.setPins(csPin, resetPin, irqPin); // Set LoRa module pins
    if (!LoRa.begin(frequency)) {
    while (1);
    }
}

void loop() {
    // Check for data from the computer via Serial
    if (Serial.available() > 0) {
    uint8_t buffer[6];
    int bytesRead = 0;
    while (Serial.available() && bytesRead < sizeof(buffer)) {
        buffer[bytesRead++] = Serial.read();
    }
    if (bytesRead > 0) {
        LoRa.beginPacket();
        LoRa.write(buffer, bytesRead);
        LoRa.endPacket();
    }
    Serial.print("sent");
    }
    // Check for incoming messages from Jetson
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        uint8_t buffer[6];
        int bytesRead = 0;
        // Read all available bytes into buffer
        while (LoRa.available() && bytesRead < sizeof(buffer)) {
            buffer[bytesRead++] = LoRa.read();
        }
        // Forward the raw binary data to the computer
        Serial.write(buffer, bytesRead); // Use write() instead of print() for binary data
    }
}

#include <SoftwareSerial.h>
#include "EBYTE.h"

//—— LoRa module control pins ——
#define PIN_M0    4
#define PIN_M1    5
#define PIN_AUX   6

//—— UART on D2 (TX) / D3 (RX) ——
SoftwareSerial loraSerial(2, 3);

// Subclass EBYTE so we can expose the protected read methods
class ExposedEBYTE : public EBYTE {
public:
  ExposedEBYTE(Stream* serial, uint8_t m0, uint8_t m1, uint8_t aux)
    : EBYTE(serial, m0, m1, aux) {}
  bool  readParams()   { return ReadParameters(); }
  void  printParams()  { PrintParameters(); }
};

ExposedEBYTE Transceiver(&loraSerial, PIN_M0, PIN_M1, PIN_AUX);

void setup() {
  // 1) configure M0/M1/AUX
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT);

  // 2) force CONFIG mode
  digitalWrite(PIN_M0, HIGH);
  digitalWrite(PIN_M1, HIGH);
  delay(50);

  // 3) start serials and init the library
  Serial.begin(9600);
  loraSerial.begin(9600);
  Transceiver.init();
  delay(100);

  // 5) now set *your* new parameters:
  Transceiver.SetUARTBaudRate(0x03);    // 0x03 = 9600 bps
  Transceiver.SetAirDataRate(0x02);     // 0x02 = 2.4 kbps
  Transceiver.SetChannel(23);           // channel 23
  Transceiver.SetTransmitPower(OPT_TP10);// highest power your header defines
  Transceiver.SetAddress(0x0001);       // network address

  // 6) save to flash
  Transceiver.SaveParameters(PERMANENT);
  delay(100);

  // 7) return to transparent (UART) mode
  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, LOW);
  delay(50);

}

void loop() {
  // — Serial → LoRa
  if (Serial.available()) {
    String out = Serial.readStringUntil('\n');
    loraSerial.println(out);
  }
  // — LoRa → Serial
  if (loraSerial.available()) {
    String in = loraSerial.readStringUntil('\n');
    Serial.println(in);
  }
}

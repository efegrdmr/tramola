#include "EBYTE.h" // Kütüphaneyi dahil ediyoruz

//—— LoRa modül kontrol pinleri (Blue Pill için) ——
// Bu pinleri kendi kartınızdaki boş pinlere göre değiştirebilirsiniz.
#define PIN_M0    PA4
#define PIN_M1    PA5
#define PIN_AUX   PA6

// LoRa için donanım seriyel portu Serial2 kullanılıyor (PA3 = RX2, PA2 = TX2)
#define LoRaSerial Serial2

// Korumalı (protected) okuma metotlarını ortaya çıkarmak için EBYTE sınıfından yeni bir sınıf türetiyoruz.
class ExposedEBYTE : public EBYTE {
public:
  ExposedEBYTE(Stream* serial, uint8_t m0, uint8_t m1, uint8_t aux)
    : EBYTE(serial, m0, m1, aux) {}
  bool  readParams()   { return ReadParameters(); }
  void  printParams()  { PrintParameters(); }
};

ExposedEBYTE Transceiver(&LoRaSerial, PIN_M0, PIN_M1, PIN_AUX);

void setup() {
  // 1) M0/M1/AUX pinlerinin modunu ayarla
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT);

  // Hata ayıklama (debug) için USB-Seriyel bağlantısını başlat
  Serial.begin(9600);
  while(!Serial) { }  // USB-Seriyel bağlantısı için bekle


  // 2) Programlama (CONFIG) moduna zorla (M0=HIGH, M1=HIGH)
  digitalWrite(PIN_M0, HIGH);
  digitalWrite(PIN_M1, HIGH);
  // Kütüphanenin EBYTE.h dosyasında bu gecikme PIN_RECOVER olarak tanımlanmıştır.
  delay(PIN_RECOVER); 

  // 3) LoRa seriyel portunu başlat ve kütüphaneyi ilklendir
  LoRaSerial.begin(9600); // Programlama modu için kütüphane 9600 bps baud hızı bekler
  if (!Transceiver.init()) {
    while(1);
  }
  delay(100);


  // 5) BÜTÜN parametreleri ayarla (EBYTE.h dosyanızdaki tanımlamalara göre)

  // Adres Ayarı (0-65535, her iki modül de aynı adreste olmalı)
  Transceiver.SetAddress(0x0001);

  // Haberleşme Kanalı (0-31, her iki modülde de aynı olmalı)
  // Frekans = 410 + Kanal (MHz). Kanal 23 -> 433 MHz
  Transceiver.SetChannel(23); 
  
  // UART Ayarları
  Transceiver.SetUARTBaudRate(UDR_9600);    // UART Hızı: 9600 bps
  Transceiver.SetParityBit(PB_8N1);         // Parity: 8N1 (8 data bits, no parity, 1 stop bit)
  
  // Hava Veri Hızı (Air Data Rate)
  // Düşük hız daha uzun menzil ama daha yavaş iletim demektir.
  Transceiver.SetAirDataRate(ADR_2400);     // 2.4 kbps

  // Çalışma Modu Ayarı (Normal çalışma için)
  // Bu kütüphanede SetMode fonksiyonu bulunmuyor, modlar M0 ve M1 pinleri ile fiziksel olarak ayarlanır.
  // Normal moda (M0=LOW, M1=LOW) setup sonunda geçeceğiz.

  // İletim Modu (Transparent / Fixed)
  // Transparent modda modül seriyel kablo gibi davranır. [2, 3] Genellikle değeri 0'dır.
  Transceiver.SetTransmissionMode(0); // 0: Transparent, 1: Fixed

  // İletim Gücü (100mW'lık bir modül için en yüksek güç)
  // Kullandığınız modüle göre (100mW, 500mW, 1W) doğru sabiti seçin.
  Transceiver.SetTransmitPower(OPT_TP20);   // 20dBm (100mW)

  // İleri Hata Düzeltme (FEC - Forward Error Correction)
  // Parazitli ortamlarda veri bütünlüğünü artırır.
  Transceiver.SetFECMode(OPT_FECENABLE);      // FEC'i aç

  // IO Sürücü Modu (AUX pini ve UART pinleri için)
  // Push-pull + Pull-up en yaygın kullanımdır.
  Transceiver.SetPullupMode(OPT_IOPUSHPULL);  // TX/RX pinleri için Push-pull ve dahili pull-up

  // WOR (Wake-on-Radio) Zamanlaması
  // Normal modda kullanılmaz, düşük güç modları içindir.
  Transceiver.SetWORTIming(OPT_WAKEUP2000);   // WOR periyodu 2000ms

  // 6) Ayarları modülün hafızasına kalıcı olarak kaydet
  Transceiver.SaveParameters(PERMANENT);
  delay(100); // Kayıt işleminin tamamlanması için bekleme

  // 7) Normal çalışma moduna geri dön
  // M0 ve M1 pinleri LOW yapılarak şeffaf UART (transparent) moda geçilir.
  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, LOW);
  delay(PIN_RECOVER);

}

void loop() {
  // USB-Seriyel'den gelen veriyi LoRa ile gönder
  if (Serial.available()) {
    String out = Serial.readStringUntil('\n');
    Serial.println(out);
    LoRaSerial.println(out);
  }
  
  // LoRa'dan gelen veriyi USB-Seriyel'e yazdır
  if (LoRaSerial.available()) {
    String in = LoRaSerial.readStringUntil('\n');
    Serial.println(in);
  }
}
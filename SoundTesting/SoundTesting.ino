#include <DFRobotDFPlayerMini.h>

HardwareSerial& mp3Serial = Serial2;   // UART2 on ESP32
DFRobotDFPlayerMini dfp;

// DFPlayer wiring to ESP32:
//   DFPlayer TX -> ESP32 IO16 (RX2)
//   DFPlayer RX -> ESP32 IO17 (TX2)
//   GND common, VCC 5V to DFPlayer

void setup() {
  // Start DFPlayer serial (default 9600 baud)
  mp3Serial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

  if (!dfp.begin(mp3Serial)) {
    // Init failed; stay here (optional: add retries)
    while (1) { delay(1000); }
  }

  dfp.volume(25);  // 0..30
  dfp.play(001);     // play track 001 (e.g., /MP3/001.mp3)
}

void loop() {
}

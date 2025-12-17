#include <SPI.h>
#include <Adafruit_MCP23X17.h>

// ==== EDIT THIS to your actual CS pin ====
#define CS_EXP1 25

// GPB0 maps to logical pin 8 in Adafruit_MCP23X17
#define EXP_LED_PIN 1

Adafruit_MCP23X17 mcp;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Bring up SPI on default ESP32 VSPI pins: SCK=18, MISO=19, MOSI=23
  SPI.begin();  

  // Init expander on SPI with our CS pin
  if (!mcp.begin_SPI(CS_EXP1)) {
    Serial.println("MCP23S17 init FAILED. Check CS wiring, power, and RESET.");
    while (1) delay(10);
  }
  Serial.println("MCP23S17 init OK.");

  // Set GPB0 as output
  mcp.pinMode(EXP_LED_PIN, OUTPUT);

  // Start with LED OFF (assuming active-HIGH wiring)
  mcp.digitalWrite(EXP_LED_PIN, LOW);
  Serial.println("Ready. Blinking GPB0...");
}

void loop() {
  // Toggle every 500 ms
  mcp.digitalWrite(EXP_LED_PIN, HIGH); // ON if active-HIGH wiring
  delay(500);
  mcp.digitalWrite(EXP_LED_PIN, LOW);  // OFF
  delay(500);
}

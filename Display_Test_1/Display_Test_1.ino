#include <MD_Parola.h>
#include <MD_MAX72XX.h>
#include <SPI.h>

// --- Hardware setup ---
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES   4        // <-- change to your number of 8x8 modules
#define DATA_PIN      23       // DIN
#define CLK_PIN       18       // CLK
#define CS_PIN        4        // CS

MD_Parola P(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

void setup() {
  P.begin();
  P.setIntensity(3);          // 0..15 brightness
  P.displayClear();

  // If your display is physically upside-down or mirrored, try one (or both):
  // P.setZoneEffect(0, true, PA_FLIP_UD);
  // P.setZoneEffect(0, true, PA_FLIP_LR);

  // Print a centered READY message (no scroll)
  P.displayText((char*)"READY", PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
}

void loop() {
  // Parola needs this call even for static text
  P.displayAnimate();
}

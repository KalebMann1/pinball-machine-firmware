#pragma once
#include <Arduino.h>

// ========================= Public Types =========================
enum Sym : uint8_t { SEVEN, BAR, STAR, CHERRY, BELL, BLANK, SYM_COUNT };

struct Outcome {
  uint8_t sym[3];   // top-to-bottom reels landed symbols
  int     payout;   // points awarded for result
};

// Optional: switch between skill-stop and predetermined
enum SlotMode : uint8_t { SLOT_MODE_SKILL_STOP = 0, SLOT_MODE_PREDETERMINED = 1 };

// ========================= Public API =========================
// One-time init
void Slot_init();

// Start a spin (reels run indefinitely in skill-stop; in predetermined
// mode the outcome is picked now, and button presses just shorten spin)
void Slot_startSpin();

// Call every loop
void Slot_tick();

// Is the mini-game active (spinning or showing result)?
bool Slot_isBusy();

// Read last result (valid after Slot_isBusy() becomes false)
const Outcome& Slot_getOutcome();

// Player (or drop-target) stops a specific reel [0..2]
void Slot_stopReel(uint8_t reelIndex);

// Optional controls
void Slot_setMode(SlotMode m);         // switch between skill / predetermined
void Slot_setIntensity(uint8_t v);     // LED brightness (0..15)

// ========================= Configuration =========================
// You can override any of these BEFORE including this header.

#ifndef SLOT_BTN0_PIN
#define SLOT_BTN0_PIN 25
#endif
#ifndef SLOT_BTN1_PIN
#define SLOT_BTN1_PIN 26
#endif
#ifndef SLOT_BTN2_PIN
#define SLOT_BTN2_PIN 27
#endif

// MAX7219 chain (MD_MAX72XX hardware type)
#ifndef SLOT_HW_TYPE
  // Common FC16 8x8 modules. Try MD_MAX72XX::PAROLA_HW if mirrored/rotated.
  #define SLOT_HW_TYPE MD_MAX72XX::FC16_HW
#endif

#ifndef SLOT_MAX_DEVICES
#define SLOT_MAX_DEVICES 4   // 4 x 8x8 = 32x8 display
#endif

#ifndef SLOT_CS_PIN
#define SLOT_CS_PIN 5        // ESP32 GPIO for MAX7219 CS/LOAD
#endif

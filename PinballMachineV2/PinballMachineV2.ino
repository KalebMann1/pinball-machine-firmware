/***** Senior Design – Pinball Prototype (Dual-Core, Event-Driven, Polled IOE) *****
  Changes from V1:
    - Replaced GPIO ISR SwitchManager with polled MCP23S17 SwitchManager (IOE1 + IOE2).
    - Switch events delivered via FreeRTOS queue; GameManager consumes them.
    - Polling task on Core 0; Game + Sound + Display + Roulette on Core 1.
  Notes:
    - Mapping section near top: bind StartSlot/Add100/Reel1/2/3 to SW IDs.
    - IOE1 inputs: ACTIVE-HIGH   (pressed = HIGH, no internal pullups)
    - IOE2 GPB1..7: ACTIVE-LOW  (pressed = LOW, uses INPUT_PULLUP)
*******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MCP23X17.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <esp_system.h>
#include "esp32-hal-ledc.h"



// If for some reason the core headers didn't pull these in, declare them ourselves.


/* ====================== I/O EXPANDER PINS & MAPPING ====================== */
// HSPI bus
static const uint8_t PIN_SCK  = 32;
static const uint8_t PIN_MISO = 13;
static const uint8_t PIN_MOSI = 15;

// Chip selects
static const uint8_t CS_IOE1 = 33;   // inputs (active-HIGH)  -> GPA0..7, GPB0..7
static const uint8_t CS_IOE2 = 25;   // inputs (GPB active-LOW) + (optional) outputs on GPAx
static const uint8_t CS_IOE3 = 26;   // outputs-only (LEDs etc.) 
// IOE2: GPB3 used as Drop Target Set coil output
static const uint8_t IOE2_GPB3_DROP_SET = 11;   // GPB3

// --- Switch ID space (0..23) ---
// 0..7:  IOE1: GPA0..GPA7 (active-HIGH)
// 8..15: IOE1: GPB0..GPB7 (active-HIGH)
// 17..23: IOE2: GPB1..GPB7 (active-LOW)
// Shared HSPI bus (matches your working IOE1 test)
SPIClass spiBus(HSPI);
Adafruit_MCP23X17 ioe1;   // "IOE1" – your bumpers/rollovers chip
Adafruit_MCP23X17 ioe2;   // "IOE2" – drop targets + coil chip

// Edge tracking for simple polling
bool lastIOE1[16] = {0};  // for IOE1 pins 0..15 (active-HIGH)
bool lastIOE2[16] = {0};  // for IOE2 pins 0..15 (active-LOW on inputs)

// Logical switch ID space
enum SwitchId : uint8_t {
  // IOE1 GPA0..7
  SW_IOE1_A0 = 0,
  SW_IOE1_A1,
  SW_IOE1_A2,
  SW_IOE1_A3,
  SW_IOE1_A4,
  SW_IOE1_A5,
  SW_IOE1_A6,
  SW_IOE1_A7,

  // IOE1 GPB0..7
  SW_IOE1_B0,
  SW_IOE1_B1,
  SW_IOE1_B2,
  SW_IOE1_B3,
  SW_IOE1_B4,
  SW_IOE1_B5,
  SW_IOE1_B6,
  SW_IOE1_B7,

  // IOE2 GPB0..7 (we’re only *using* B1..B7, but we still define all 8 IDs)
  SW_IOE2_B0,
  SW_IOE2_B1,
  SW_IOE2_B2,
  SW_IOE2_B3,
  SW_IOE2_B4,
  SW_IOE2_B5,
  SW_IOE2_B6,
  SW_IOE2_B7,

  SW_COUNT  // total = 24
};


// IOE1 (ACTIVE-HIGH)
#define SW_BUMPER1         SW_IOE1_A0  // Add 100, bumper sound
#define SW_BUMPER2         SW_IOE1_A1  // Add 100, bumper sound
#define SW_BUMPER3         SW_IOE1_A2  // Add 100, bumper sound
#define SW_BUMPER4         SW_IOE1_A3  // Add 100, bumper sound

#define SW_STANDUP_TOP     SW_IOE1_A4  // Roulette GREEN bet
#define SW_STANDUP_MID     SW_IOE1_A5  // Roulette RED bet
#define SW_STANDUP_BOT     SW_IOE1_A6  // Roulette WHITE bet

#define SW_SPINNER         SW_IOE1_A7  // Spinner (frequency-based later)

#define SW_SLING_R         SW_IOE1_B0  // (stats / future use)
#define SW_SLING_L         SW_IOE1_B1  // (stats / future use)
// SW_IOE1_B2, B3, B4 = Nothing for now
#define SW_RLV_DRAIN       SW_IOE1_B3   // NEW: ball drain switch (GPB3)

#define SW_ROLLOVER_L      SW_IOE1_B5  // Rollover left
#define SW_ROLLOVER_M      SW_IOE1_B6  // Rollover middle
#define SW_ROLLOVER_R      SW_IOE1_B7  // Rollover right

// IOE2 (ACTIVE-LOW) – GPB0..7
#define SW_FLIPPER_R       SW_IOE2_B1  // stats / future use
#define SW_FLIPPER_L       SW_IOE2_B2  // stats / future use

// SW_IOE2_B4 = nothing
#define SW_DROP_L          SW_IOE2_B5  // Controls Left reel
#define SW_DROP_M          SW_IOE2_B6  // Controls Middle reel
#define SW_DROP_R          SW_IOE2_B7  // Controls Right reel

/* ============================= SOUND MANAGER ============================= */
static const int DF_TX = 17; // ESP32 TX2 → DF RX
static const int DF_RX = 16; // ESP32 RX2 ← DF TX
static const int DF_BUSY = -1;

HardwareSerial mp3Serial(2);
DFRobotDFPlayerMini dfp;

class SoundManager {
public:
  bool begin(uint8_t volume = 24) {
    mp3Serial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);
    Serial.println("Initializing DFPlayer...");
    if (!dfp.begin(mp3Serial)) {
      Serial.println("❌ DFPlayer init failed.");
      return false;
    }
    dfp.setTimeOut(500);
    dfp.outputDevice(DFPLAYER_DEVICE_SD);
    dfp.volume(constrain(volume, 0, 30));
    dfp.EQ(DFPLAYER_EQ_NORMAL);
    Serial.println("✅ DFPlayer ready.");
    return true;
  }
  void play(uint16_t i) { dfp.playMp3Folder(i); }
  void stop() { dfp.stop(); }
  bool isPlaying() { return DF_BUSY >= 0 ? (digitalRead(DF_BUSY) == LOW) : (dfp.readState() == 1); }

  void playGameStart()   { play(1); }
  void playBumperHit()   { play(2); }
  void playDropTarget()  { play(3); }
  void playSlotStart()   { play(4); }
  void playRouletteWin() { play(5); }
  void playGameOver()    { play(6); }
};
SoundManager gSound;

/* ========================== SWITCH EVENTS & QUEUE ========================= */
struct SwitchEvent { uint8_t id; bool pressed; uint32_t t_ms; };
QueueHandle_t gSwitchQ;



/* ========================= POLLED SWITCH MANAGER ========================= */



class DropTargetCoilManager {
public:
  void begin(Adafruit_MCP23X17* ioe2) {
    _ioe2 = ioe2;
    if (!_ioe2) return;
    _ioe2->pinMode(IOE2_GPB3_DROP_SET, OUTPUT);
    _ioe2->digitalWrite(IOE2_GPB3_DROP_SET, LOW); // coil off
    _active = false;
    _offTime = 0;
  }

  // Request a pulse of the given duration (default 400 ms)
  void pulse(uint32_t durationMs = 400) {
    if (!_ioe2) return;
    if (_active) return;              // don't stack pulses
    _active  = true;
    _offTime = millis() + durationMs;
    _ioe2->digitalWrite(IOE2_GPB3_DROP_SET, HIGH);  // energize coil
  }

  // Call frequently (e.g., from gameTask) to turn it off when time’s up
  void update(uint32_t now) {
    if (!_active) return;
    if ((int32_t)(now - _offTime) >= 0) {
      _ioe2->digitalWrite(IOE2_GPB3_DROP_SET, LOW); // de-energize coil
      _active = false;
    }
  }

private:
  Adafruit_MCP23X17* _ioe2{nullptr};
  bool     _active{false};
  uint32_t _offTime{0};
};

DropTargetCoilManager gDropCoil;

// ============================== LED MANAGER (IOE3) ===============================
static const uint8_t LED_ON  = HIGH;
static const uint8_t LED_OFF = LOW;

// MCP23S17 pin numbers for IOE3 (0..7 = GPA0..7, 8..15 = GPB0..7)
static const uint8_t LED_BALL1      = 0;   // GPA0
static const uint8_t LED_BALL2      = 1;   // GPA1
static const uint8_t LED_BALL3      = 2;   // GPA2
static const uint8_t LED_SLOT_READY = 3;   // GPA3 (RSTAT LED)

static const uint8_t LED_ROLL_L     = 8;   // GPB0
static const uint8_t LED_ROLL_M     = 9;   // GPB1
static const uint8_t LED_ROLL_R     = 10;  // GPB2

static const uint8_t LED_ON_STATUS  = 12;  // GPB4
static const uint8_t LED_STAND_BOT  = 13;  // GPB5 – white
static const uint8_t LED_STAND_MID  = 14;  // GPB6 – red
static const uint8_t LED_STAND_TOP  = 15;  // GPB7 – green

class LedManager {
public:

  bool begin(SPIClass* spi, uint8_t cs, uint32_t spiHz = 100000) {
    _spi = spi;
    _cs = cs;
    if (!_spi) return false;

    if (!_ioe3.begin_SPI(_cs, _spi, spiHz)) {
      return false;
    }

    // Configure all used pins as outputs and default OFF
    const uint8_t pins[] = {
      LED_BALL1, LED_BALL2, LED_BALL3,
      LED_SLOT_READY,
      LED_ROLL_L, LED_ROLL_M, LED_ROLL_R,
      LED_ON_STATUS,
      LED_STAND_BOT, LED_STAND_MID, LED_STAND_TOP
    };
    for (uint8_t i = 0; i < sizeof(pins); ++i) {
      _ioe3.pinMode(pins[i], OUTPUT);
      _ioe3.digitalWrite(pins[i], LED_OFF);
    }

    // Machine alive LED
    setOnStatus(true);
    // Default: 3 balls lit
    setBallCount(3);
    return true;
  }

  void startRolloverFlash() {
      // Start with all three on
      setRolloverState(true, true, true);
      _rolloverFlashActive = true;
      _rolloverFlashOn = true;
      uint32_t now = millis();
      _rolloverFlashEnd = now + _rolloverFlashDuration;
      _rolloverLastToggle = now;
    }
  // Called from game task periodically (for future animations)
  void update(uint32_t now) {
  // Rollover flash effect
  if (_rolloverFlashActive) {
    if ((int32_t)(now - _rolloverFlashEnd) >= 0) {
      // Done flashing: turn all off and stop
      _rolloverFlashActive = false;
      setRolloverState(false, false, false);
    } else if ((int32_t)(now - _rolloverLastToggle) >= (int32_t)_rolloverFlashInterval) {
      _rolloverLastToggle = now;
      _rolloverFlashOn = !_rolloverFlashOn;
      setRolloverState(_rolloverFlashOn, _rolloverFlashOn, _rolloverFlashOn);
    }
  }

  // (room for other animations later)
}

  // ---- Simple API for GameManager ----
  void setOnStatus(bool on) {
    _ioe3.digitalWrite(LED_ON_STATUS, on ? LED_ON : LED_OFF);
  }

  void setBallCount(uint8_t balls) {
    if (balls > 3) balls = 3;
    _ioe3.digitalWrite(LED_BALL1, (balls >= 1) ? LED_ON : LED_OFF);
    _ioe3.digitalWrite(LED_BALL2, (balls >= 2) ? LED_ON : LED_OFF);
    _ioe3.digitalWrite(LED_BALL3, (balls >= 3) ? LED_ON : LED_OFF);
  }

  void setRolloverState(bool left, bool mid, bool right) {
    _ioe3.digitalWrite(LED_ROLL_L, left  ? LED_ON : LED_OFF);
    _ioe3.digitalWrite(LED_ROLL_M, mid   ? LED_ON : LED_OFF);
    _ioe3.digitalWrite(LED_ROLL_R, right ? LED_ON : LED_OFF);
  }

  void setSlotReady(bool ready) {
    _ioe3.digitalWrite(LED_SLOT_READY, ready ? LED_ON : LED_OFF);
  }

  // When roulette is armed at score threshold (5000, 10000, ...)
  void setRouletteReady(bool ready) {
    if (!ready) {
      clearStandups();
      return;
    }
    // All three lit = “pick a color”
    _ioe3.digitalWrite(LED_STAND_BOT, LED_ON);
    _ioe3.digitalWrite(LED_STAND_MID, LED_ON);
    _ioe3.digitalWrite(LED_STAND_TOP, LED_ON);
  }

  // betCode matches GameManager::RouletteBet numeric values:
  // 0 = BET_NONE, 1 = BET_GREEN, 2 = BET_RED, 3 = BET_WHITE
  void setRouletteBet(uint8_t betCode) {
    clearStandups();
    switch (betCode) {
      case 1: // BET_GREEN
        _ioe3.digitalWrite(LED_STAND_TOP, LED_ON);
        break;
      case 2: // BET_RED
        _ioe3.digitalWrite(LED_STAND_MID, LED_ON);
        break;
      case 3: // BET_WHITE
        _ioe3.digitalWrite(LED_STAND_BOT, LED_ON);
        break;
      default:
        // none
        break;
    }
  }

private:
    // Rollover flash effect
  bool     _rolloverFlashActive{false};
  bool     _rolloverFlashOn{false};
  uint32_t _rolloverFlashEnd{0};
  uint32_t _rolloverLastToggle{0};
  const uint32_t _rolloverFlashDuration{600}; // total ms
  const uint32_t _rolloverFlashInterval{120}; // blink speed

  SPIClass* _spi{nullptr};
    uint8_t   _cs{0};
  Adafruit_MCP23X17 _ioe3;

  void clearStandups() {
    _ioe3.digitalWrite(LED_STAND_BOT, LED_OFF);
    _ioe3.digitalWrite(LED_STAND_MID, LED_OFF);
    _ioe3.digitalWrite(LED_STAND_TOP, LED_OFF);
  }
};



LedManager          gLeds;


/*** DisplayManager ***/
#define HARDWARE_TYPE   MD_MAX72XX::FC16_HW
#define DEVICES  4
#define CS_SCORE 19
#define PIN_CLK_SCORE 18
#define PIN_MOSI_SCORE 23
#define CS_GAME  4

class DisplayManager {
public:
  DisplayManager()
  : scoreDisp(HARDWARE_TYPE, PIN_MOSI_SCORE, PIN_CLK_SCORE, CS_SCORE, DEVICES),
    gameDisp(HARDWARE_TYPE, PIN_MOSI_SCORE, PIN_CLK_SCORE, CS_GAME,  DEVICES) {}

  void begin() {
    scoreDisp.begin(); 
    scoreDisp.setIntensity(6);
    gameDisp.begin();  
    gameDisp.setIntensity(7);
    scoreDisp.displayText("0",     PA_RIGHT, 0, 0, PA_PRINT, PA_NO_EFFECT);
    gameDisp.displayText("READY", PA_LEFT,  0, 0, PA_PRINT, PA_NO_EFFECT);
  }
  void tick() { scoreDisp.displayAnimate(); gameDisp.displayAnimate(); }
  void showScore(uint32_t s) {
    static char buf[16]; snprintf(buf, sizeof(buf), "%lu", (unsigned long)s);
    scoreDisp.displayText(buf, PA_RIGHT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  }
  void showStatus(const char* msg) {
    gameDisp.displayText(msg, PA_LEFT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  }
  MD_Parola& gamePanel() { return gameDisp; }
  MD_MAX72XX* gameGfx()  { return gameDisp.getGraphicObject(); }
private:
  MD_Parola scoreDisp;
  MD_Parola gameDisp;
};

/*** Slot Machine (unchanged logic; APIs used by GameManager) ***/
struct Reel;  // fwd
#define SLOT_STANDALONE 1   // 0 = integrated (no internal buttons), 1 = self-test with buttons/autospin



// ---- Parola + zones ----
// Slot manager no longer owns the panel; it gets one from DisplayManager
static MD_Parola* P = nullptr;
static bool g_inited = false;

// call this from setup() to give Slot manager the CS=5 panel
void Slot_attachDisplay(MD_Parola* panel) { P = panel; g_inited = false; }

// helper now uses the borrowed panel
static MD_MAX72XX* gfx() { return P ? P->getGraphicObject() : nullptr; }



static const uint8_t ZONES = 3;
static uint8_t Z_START[ZONES];
static uint8_t Z_END  [ZONES];

// ---- symbols & geometry ----
enum Sym : uint8_t { SEVEN, BAR, STAR, CHERRY, BELL, BLANK, SYM_COUNT };
static const uint8_t SYMBOL_COLS=5, SPACER_COLS=1;
static const uint8_t SYMBOL_W = SYMBOL_COLS + SPACER_COLS;
static const uint8_t SYMBOL_ROWS=7, SPACER_ROWS=1;
static const uint8_t SYMBOL_H = SYMBOL_ROWS + SPACER_ROWS;   // 8 rows tall incl spacer
static const uint8_t SYM_SEVEN[5]  = {0x7F,0x02,0x04,0x08,0x10};
static const uint8_t SYM_BAR[5]    = {0x7F,0x41,0x41,0x41,0x7F};
static const uint8_t SYM_STAR[5]   = {0x14,0x08,0x3E,0x08,0x14};
static const uint8_t SYM_CHERRY[5] = {0x0C,0x1E,0x1E,0x0C,0x00};
static const uint8_t SYM_BELL[5]   = {0x1C,0x22,0x22,0x3E,0x08};
static const uint8_t SYM_BLANK[5]  = {0x00,0x00,0x00,0x00,0x00};
static const uint8_t* SYM_BITMAPS[SYM_COUNT] = {
  SYM_SEVEN, SYM_BAR, SYM_STAR, SYM_CHERRY, SYM_BELL, SYM_BLANK
};
static int payout3[SYM_COUNT] = { 1000, 300, 150, 80, 50, 0 };

// ---- types ----
#define STRIP_LEN 8
struct Reel {
  uint8_t  zone, zoneWidth;
  uint8_t  strip[STRIP_LEN];
  uint8_t  stripLen;
  int32_t  offset;               // row offset into vertical strip
  int8_t   dir;                  // +1 up, -1 down
  uint16_t stepInterval, minInterval, maxInterval;
  uint32_t lastStep;
  bool     spinning;
  bool     armed;
  int32_t  stopOffset;
};
struct Outcome { uint8_t sym[3]; int payout; };
enum SlotMode  : uint8_t { SLOT_MODE_SKILL_STOP = 0, SLOT_MODE_PREDETERMINED = 1 };
enum SlotState : uint8_t { S_IDLE, S_SPINNING, S_SHOW };

// ---- globals ----
static Reel     R[3];
static Outcome  g_outcome;
static SlotMode g_mode = SLOT_MODE_PREDETERMINED;
static SlotState slotState = S_IDLE;
static uint32_t  stateTs = 0;


// ---- rendering helpers ----
static inline void   drawZoneColumn(uint8_t z, uint8_t xIn, uint8_t colBits){ gfx()->setColumn(Z_START[z]+xIn, colBits); }
static inline uint8_t glyphCol(uint8_t s, uint8_t colIn){ return (colIn<SYMBOL_COLS)? SYM_BITMAPS[s][colIn] : 0x00; }
static inline int32_t stripRows(const Reel& r){ return (int32_t)r.stripLen * SYMBOL_H; }
static inline uint8_t centerStartX(uint8_t w){ return (w - SYMBOL_COLS) / 2; }

static void renderReelVertical(const Reel& r){
  int32_t totalRows = stripRows(r); if (totalRows<=0) return;
  int32_t base = r.offset % totalRows; if (base<0) base += totalRows;
  uint8_t xStart = centerStartX(r.zoneWidth);

  for(uint8_t x=0; x<r.zoneWidth; x++){
    if (x < xStart || x >= xStart+SYMBOL_COLS){ drawZoneColumn(r.zone,x,0x00); continue; }
    uint8_t colInSym = x - xStart, outCol = 0;
    for(uint8_t y=0; y<8; y++){
      int32_t rowIdx = (base + y) % totalRows;
      uint8_t symIdx = rowIdx / SYMBOL_H;
      uint8_t rowIn  = rowIdx % SYMBOL_H;
      uint8_t bit=0;
      if (rowIn < SYMBOL_ROWS){
        uint8_t colBits = glyphCol(r.strip[symIdx], colInSym);
        bit = (colBits >> rowIn) & 0x01;
      }
      outCol |= (bit << y);
    }
    drawZoneColumn(r.zone, x, outCol);
  }
}

static int findSymbolIndexInStrip(const Reel& r, uint8_t sym){
  for(int i=0;i<r.stripLen;i++) if (r.strip[i]==sym) return i;
  return 0;
}

static int32_t computeStopOffset(const Reel& r, uint8_t targetSym, uint16_t extraRevs){
  int32_t total = stripRows(r);
  int32_t curBase = r.offset % total; if (curBase<0) curBase += total;
  int32_t desiredBase = (int32_t)findSymbolIndexInStrip(r, targetSym) * SYMBOL_H;
  if (r.dir > 0){ int32_t adv = (desiredBase - curBase + total) % total;  return r.offset + adv + (int32_t)extraRevs * total; }
  else          { int32_t back= (curBase - desiredBase + total) % total;  return r.offset - (int32_t)(back + (int32_t)extraRevs * total); }
}

static int32_t computeStopOffsetBoundarySoon(const Reel& r, uint16_t minMoreRows){
  int32_t total = stripRows(r);
  int32_t curBase = r.offset % total; if (curBase<0) curBase += total;
  int32_t mod = curBase % SYMBOL_H, delta;
  if (r.dir > 0){ delta = (mod==0?SYMBOL_H:(SYMBOL_H-mod)); if (delta<(int32_t)minMoreRows) delta += ((minMoreRows-delta+SYMBOL_H-1)/SYMBOL_H)*SYMBOL_H; return r.offset + delta; }
  else          { delta = (mod==0?SYMBOL_H:mod);             if (delta<(int32_t)minMoreRows) delta += ((minMoreRows-delta+SYMBOL_H-1)/SYMBOL_H)*SYMBOL_H; return r.offset - delta; }
}

static uint8_t landedSymbol(const Reel& r){
  int32_t total = stripRows(r);
  int32_t base = r.offset % total; if (base<0) base += total;
  int idx = base / SYMBOL_H; return r.strip[idx % r.stripLen];
}

static bool stepAndMaybeStop(Reel& r){
  if (!r.spinning) return false;
  uint32_t now = millis();
  if (now - r.lastStep < r.stepInterval) return false;
  r.lastStep = now;

  if (!r.armed){
    if (r.stepInterval > r.minInterval) r.stepInterval -= 1;
    r.offset += r.dir;
    renderReelVertical(r);
    return false;
  }

  int32_t remain = (r.dir>0) ? (r.stopOffset - r.offset) : (r.offset - r.stopOffset);
  if (remain < 64 && r.stepInterval < r.maxInterval) r.stepInterval += 2;
  else if (r.stepInterval > r.minInterval) r.stepInterval -= 1;

  r.offset += r.dir;
  renderReelVertical(r);

  if ((r.dir>0 && r.offset >= r.stopOffset) || (r.dir<0 && r.offset <= r.stopOffset)){
    r.offset = r.stopOffset;
    renderReelVertical(r);
    r.spinning = false;
    return true;
  }
  return false;
}

// ---- FX ----
static uint8_t g_baseIntensity = 3;
struct ResultFX { bool active=false, won=false; uint32_t started=0, last=0; uint16_t duration=1200, interval=60; uint8_t phase=0; } fx;
static inline void drawGutter(uint8_t col, bool on){ gfx()->setColumn(col, on?0xFF:0x00); }
static void FX_start(bool won){
  fx.active=true; fx.won=won; fx.started=millis(); fx.last=0; fx.phase=0;
  fx.duration = won ? 1200 : 600; fx.interval = 60;
}
static void FX_tick(){
  if (!fx.active) return;
  uint32_t now = millis();
  if (now - fx.last < fx.interval) return;
  fx.last = now;
  for(int i=0;i<3;i++) renderReelVertical(R[i]);
  if (fx.won){
    static const uint8_t seq[4] = {3,8,12,8};
    P->setIntensity(seq[fx.phase & 3]);
    bool on = (fx.phase & 1);
    drawGutter(10,on); drawGutter(21,on);
    for (int k=0;k<4;k++){ gfx()->setPoint(random(0,32), random(0,8), true); }
  } else {
    P->setIntensity(1);
    bool on = (fx.phase & 1);
    drawGutter(10,on); drawGutter(21,on);
  }
  fx.phase++;
  if (now - fx.started >= fx.duration){
    fx.active=false;
    P->setIntensity(g_baseIntensity);
    drawGutter(10,false); drawGutter(21,false);
    for(int i=0;i<3;i++) renderReelVertical(R[i]);
  }
}

// ---- outcome helpers ----
static void chooseOutcomeDeterministic(){ g_outcome.sym[0]=BAR; g_outcome.sym[1]=STAR; g_outcome.sym[2]=SEVEN; g_outcome.payout=0; }
static void evaluatePayoutFromLanded(){
  uint8_t a=landedSymbol(R[0]), b=landedSymbol(R[1]), c=landedSymbol(R[2]);
  g_outcome.sym[0]=a; g_outcome.sym[1]=b; g_outcome.sym[2]=c;
  g_outcome.payout = (a==b && b==c) ? payout3[a] : 0;
}

// ---- public-ish slot API for GameManager ----
static inline void Slot_setMode(uint8_t m){ (void)m; /* keep default for now */ }
static inline void Slot_setIntensity(uint8_t v){ if (!g_inited) return; g_baseIntensity = min<uint8_t>(v,15); P->setIntensity(g_baseIntensity); }

static inline bool Slot_isSpinning() { return slotState == S_SPINNING; }
static inline bool Slot_isShowing()  { return slotState == S_SHOW; }
static inline bool Slot_isIdle()     { return slotState == S_IDLE && !fx.active; }
static inline int  Slot_payout()     { return g_outcome.payout; }

static void Slot_init(){
  if (!g_inited){
    g_baseIntensity=3; P->setIntensity(g_baseIntensity);
    P->displayClear();
    g_inited=true;

    // zones (0..9), (11..20), (22..31) — gutters at 10, 21
    P->setZone(0,0,9);    Z_START[0]=0;  Z_END[0]=9;
    P->setZone(1,11,20);  Z_START[1]=11; Z_END[1]=20;
    P->setZone(2,22,31);  Z_START[2]=22; Z_END[2]=31;

    auto initReel = [](Reel& r,uint8_t z,uint16_t step,uint16_t minI,uint16_t maxI){
      r.zone=z; r.zoneWidth=10; r.stripLen=8;
      r.offset=0; r.dir=-1; r.stepInterval=step; r.minInterval=minI; r.maxInterval=maxI; r.lastStep=0;
      r.spinning=false; r.armed=false; r.stopOffset=0;
    };
    initReel(R[0],0,44,26,90);
    initReel(R[1],1,40,24,88);
    initReel(R[2],2,36,22,86);

    uint8_t s0[]={BAR,STAR,CHERRY,BLANK,BELL,SEVEN,CHERRY,BAR}; for(uint8_t i=0;i<8;i++) R[0].strip[i]=s0[i];
    uint8_t s1[]={BLANK,CHERRY,BAR,BELL,STAR,SEVEN,BLANK,STAR}; for(uint8_t i=0;i<8;i++) R[1].strip[i]=s1[i];
    uint8_t s2[]={STAR,BLANK,BELL,SEVEN,CHERRY,BAR,BELL,STAR};  for(uint8_t i=0;i<8;i++) R[2].strip[i]=s2[i];

    for(int i=0;i<3;i++) renderReelVertical(R[i]);

#if SLOT_STANDALONE
    // Standalone mode would attach its own buttons here, but we leave it off in integration
    // and let SwitchManager own inputs.
#endif
    randomSeed((uint32_t)micros());
  } else {
    P->displayClear();
    for(int i=0;i<3;i++) renderReelVertical(R[i]);
  }
  slotState = S_IDLE;
  g_outcome = Outcome{{BLANK,BLANK,BLANK},0};
}

static void Slot_startSpin(){
  chooseOutcomeDeterministic();  // for predetermined mode
  for(int i=0;i<3;i++){
    R[i].spinning=true; R[i].armed=false;
    R[i].stepInterval=(uint16_t)(i==0?40:i==1?36:32);
  }
  slotState=S_SPINNING; stateTs=millis();
}

static void Slot_stopReel(uint8_t i){
  if (i>2) return;
  Reel& r = R[i];
  if (!r.spinning || r.armed) return;
  if (g_mode==SLOT_MODE_SKILL_STOP){
    r.stopOffset = computeStopOffsetBoundarySoon(r, 12);
  }else{
    r.stopOffset = computeStopOffset(r, g_outcome.sym[i], 0);
    int32_t remain = (r.dir>0)?(r.stopOffset-r.offset):(r.offset-r.stopOffset);
    if (remain<8) r.stopOffset = computeStopOffset(r, g_outcome.sym[i], 1);
  }
  r.armed=true;
}

static void Slot_tick(){
  if (slotState==S_SPINNING){
    stepAndMaybeStop(R[0]);
    stepAndMaybeStop(R[1]);
    stepAndMaybeStop(R[2]);

    if (!R[0].spinning && !R[1].spinning && !R[2].spinning){
      if (g_mode==SLOT_MODE_SKILL_STOP) evaluatePayoutFromLanded();
      else g_outcome.payout = (g_outcome.sym[0]==g_outcome.sym[1] && g_outcome.sym[1]==g_outcome.sym[2]) ? payout3[g_outcome.sym[0]] : 0;
      FX_start(g_outcome.payout>0);
      slotState=S_SHOW; stateTs=millis();
    }
  } else if (slotState==S_SHOW){
    FX_tick();
    if (!fx.active) slotState = S_IDLE;
  }
}

/*** RouletteManager ***/
class RouletteManager {
public:
  const int RED[3] = {14,12,13};
  const int WHT[3] = {19,4,21};
  const int GRN    = 22;
  void begin() {
    ORDER[0]=RED[0]; ORDER[1]=WHT[0]; ORDER[2]=RED[1];
    ORDER[3]=WHT[1]; ORDER[4]=RED[2]; ORDER[5]=WHT[2]; ORDER[6]=GRN;
    for (int p: RED) pinMode(p, OUTPUT);
    for (int p: WHT) pinMode(p, OUTPUT);
    pinMode(GRN, OUTPUT);
    allOff();
    randomSeed((uint32_t)esp_random());
    _state = IDLE;
  }
  bool startSpin() {
    if (_state != IDLE) return false;
    _target = random(0, 7);
    int laps = random(3, 6);
    int extra = random(0, 7);
    _stepsTotal = laps * 7 + extra + _target;
    _dFast = 40; _dSlow = 240;
    _idx = 0; _step = 0; _nextAt = millis(); _winner = -1;
    _state = SPINNING;
    return true;
  }
  void tick() {
    const uint32_t now = millis();
    switch (_state) {
      case IDLE: break;
      case SPINNING:
        if ((int32_t)(now - _nextAt) >= 0) {
          showIndex(_idx);
          float t = (_stepsTotal == 0) ? 1.0f : (float)_step / (float)_stepsTotal;
          unsigned long d = (unsigned long)(_dFast + (float)(_dSlow - _dFast) * (t*t));
          _nextAt = now + d;
          _idx = (_idx + 1) % 7; _step++;
          if (_step > _stepsTotal) {
            _winner = (_idx + 6) % 7;
            _blinkOn = false; _blinkCount = 0; _nextAt = now; _state = BLINKING;
          }
        } break;
      case BLINKING:
        if ((int32_t)(now - _nextAt) >= 0) {
          if (_blinkOn) {
            digitalWrite(ORDER[_winner], LOW);
            _nextAt = now + _blinkOffMs; _blinkOn = false; _blinkCount++;
            if (_blinkCount >= _blinkCycles) { allOff(); _state = IDLE; }
          } else {
            digitalWrite(ORDER[_winner], HIGH);
            _nextAt = now + _blinkOnMs; _blinkOn = true;
          }
        } break;
    }
  }
  bool isIdle() const { return _state == IDLE; }
  int  winner() const { return _winner; }
private:
  int ORDER[7];
  enum State : uint8_t { IDLE, SPINNING, BLINKING } _state{IDLE};
  int _idx{0}, _step{0}, _stepsTotal{0}, _target{0}, _winner{-1};
  unsigned long _nextAt{0}, _dFast{40}, _dSlow{240};
  const int _blinkCycles = 4, _blinkOnMs = 180, _blinkOffMs = 120;
  bool _blinkOn{false}; int _blinkCount{0};
  void allOff(){ for (int i=0;i<7;i++) digitalWrite(ORDER[i], LOW); }
  void showIndex(int i){
    static int prev = -1;
    if (prev >= 0) digitalWrite(ORDER[prev], LOW);
    digitalWrite(ORDER[i%7], HIGH);
    prev = i%7;
  }
};
RouletteManager gRoulette;

/* ============================== GAME MANAGER ============================== */
class GameManager {
public:
  enum State { READY, PLAYING, SLOT_ACTIVE, GAME_OVER };

   void begin(DisplayManager* dm) {
    _d = dm; _state = READY; 
    _score = 0; 
    _awaitPayout = false;
    _d->showScore(_score);
    _d->showStatus("READY");
    // Initialize LED state
    gLeds.setOnStatus(true);
    gLeds.setBallCount(_ballsRemaining);           
    gLeds.setSlotReady(false);
    gLeds.setRouletteReady(false);
    gLeds.setRouletteBet(0);         // BET_NONE

    Serial.println("READY: StartSlot + Add100 mapped via IOE switch IDs.");
  }

  // Consume all pending switch events (call in game task on Core 1)
  void pollSwitchEvents() {
    SwitchEvent e;
    while (xQueueReceive(gSwitchQ, &e, 0) == pdTRUE) {
      if (e.pressed) onPress(e.id);
      else onRelease(e.id);
    }
  }

  void loopTick() {
    // roulette tick
    gRoulette.tick();
      if (_rouletteRunning && gRoulette.isIdle()) {
      int winner = gRoulette.winner(); (void)winner;
      gSound.playRouletteWin();
      _nextRouletteScore += 500;
      _rouletteRunning = false;

      // Clear bet indicators
      _currentBet = BET_NONE;
      gLeds.setRouletteBet(0);
      gLeds.setRouletteReady(false);

      _d->showStatus("PLAY");
    }


    // slot lifecycle
    if (_state == SLOT_ACTIVE) {
      // Slot_tick() must be called frequently
      Slot_tick();

      if (_awaitPayout && Slot_isIdle()) {
        int p = Slot_payout();
        if (p > 0) addScore((uint32_t)p);
        _awaitPayout = false;
        _state = PLAYING;

        _lastSlotEndMs = millis();    // start 30 s cooldown (see section 2)
         gLeds.setSlotReady(false);  // <-- RSTAT LED off after mini-game
        _d->showStatus("READY");      // <-- go back to READY text
        Serial.println("Slot finished. Back to PLAYING.");
      }
    }
  }

private:
  DisplayManager* _d{nullptr};
  State _state{READY};
  uint32_t _score{0};
  bool _awaitPayout{false};
  uint32_t _nextRouletteScore{5000};
  const uint32_t _rouletteStep{5000};
  bool     _rouletteReady{false};    // score threshold hit, waiting for bet
  bool     _rouletteRunning{false};
  uint8_t _ballsRemaining{3};
  enum RouletteBet : uint8_t { BET_NONE, BET_GREEN, BET_RED, BET_WHITE };
  RouletteBet _currentBet{BET_NONE};
  // rollover latch state
  bool _rollL{false}, _rollM{false}, _rollR{false};
  const uint32_t _rolloverBonus = 1000;   // choose a bonus
  // spinner speed detection (3 hits within window → start slot)
  uint32_t _spinTimes[3] = {0,0,0};  // circular buffer
  uint8_t  _spinIdx = 0;
  const uint32_t _spinWindowMs = 700;  // tweak: 3 hits within 700 ms means "fast"
   // slot cooldown: prevent re-start for 30 s after last slot game ends
  uint32_t _lastSlotEndMs{0};
  const uint32_t _slotCooldownMs{30000};  // 30,000 ms = 30 seconds
 
 void onBallDrain() {
  // Ignore drains during slot mini-game or after game over if you want
  if (_state == SLOT_ACTIVE || _state == GAME_OVER) return;

  if (_ballsRemaining > 0) {
    _ballsRemaining--;
    gLeds.setBallCount(_ballsRemaining);
  }

  if (_ballsRemaining == 0) {
    _state = GAME_OVER;

    // Turn off all “mode” indicators
    _rouletteReady   = false;
    _rouletteRunning = false;
    gLeds.setRouletteReady(false);
    gLeds.setRouletteBet(0);
    gLeds.setSlotReady(false);

    _d->showStatus("GAME OVER");
    gSound.playGameOver();
    Serial.println("GAME OVER: out of balls; press reset to start a new game.");
  }
}

  void onSpinnerHit() {
    uint32_t now = millis();

    // Record this hit in a 3-sample ring buffer
    _spinTimes[_spinIdx] = now;
    _spinIdx = (_spinIdx + 1) % 3;

    // Only check once we have 3 non-zero timestamps
    if (_spinTimes[0] == 0 || _spinTimes[1] == 0 || _spinTimes[2] == 0) return;

    // Oldest is the one at _spinIdx (because we just advanced it)
    uint32_t oldest = _spinTimes[_spinIdx];  // wraps 0..2

    // 3 hits occurred within the window
    if ((now - oldest) <= _spinWindowMs && _state != SLOT_ACTIVE) {
      // Enforce 30 s cooldown since last slot end
      if (now - _lastSlotEndMs < _slotCooldownMs) {
        Serial.println("Spinner combo hit but slot is in cooldown.");
        return;
      }

      // Cooldown passed: player earned slot + (later) drop reset.
      startSlot();

     gDropCoil.pulse(400);   // coil on GPB3, 400 ms
     gLeds.setSlotReady(true); // <-- RSTAT LED on during slot mini-game
    }
  }


    void onPress(uint8_t id) {
    Serial.print("Switch press ID = ");
    Serial.println(id);
    if (id == SW_RLV_DRAIN) {
    onBallDrain();
    return;
    }
  // ---------- BUMPERS: add 100 ----------
  if (id == SW_BUMPER1 || id == SW_BUMPER2 ||
      id == SW_BUMPER3 || id == SW_BUMPER4)
  {
    Serial.println("Bumper hit → +100");
    addScore(100);
    return;
  }

  // ---------- STANDUP TARGETS: roulette bets (placeholder) ----------
  if (id == SW_STANDUP_TOP || id == SW_STANDUP_MID || id == SW_STANDUP_BOT) {
  if (_rouletteReady && !_rouletteRunning && _state != SLOT_ACTIVE) {
    // lock in bet color
    if (id == SW_STANDUP_TOP) _currentBet = BET_GREEN;
    if (id == SW_STANDUP_MID) _currentBet = BET_RED;
    if (id == SW_STANDUP_BOT) _currentBet = BET_WHITE;

    _rouletteReady = false;
    _rouletteRunning = true;
     // LEDs: stop "ready" (all three) and show the chosen color
      gLeds.setRouletteReady(false);
      gLeds.setRouletteBet(_currentBet);

    // Show color name on game display
    switch (_currentBet) {
      case BET_GREEN: _d->showStatus("GREEN"); break;
      case BET_RED:   _d->showStatus("RED");   break;
      case BET_WHITE: _d->showStatus("WHITE"); break;
      default: break;
    }
   
    gRoulette.startSpin();        // 🎯 spin only AFTER bet
  }
  return;
}

  // ---------- SPINNER: points + speed detection for slot start ----------
  if (id == SW_SPINNER) {
    addScore(10);       // small award per hit (tweak as desired)
    onSpinnerHit();     // check if fast enough to start slot
    return;
  }

  // ---------- SLINGSHOTS & FLIPPERS: stats / future ----------
  if (id == SW_SLING_L || id == SW_SLING_R ||
      id == SW_FLIPPER_L || id == SW_FLIPPER_R)
  {
    // For now, do nothing or track stats
    return;
  }

  // ---------- ROLLOVERS: latch + bonus when all 3 ----------
  if (id == SW_ROLLOVER_L || id == SW_ROLLOVER_M || id == SW_ROLLOVER_R) {
    if (id == SW_ROLLOVER_L) _rollL = !_rollL;
    if (id == SW_ROLLOVER_M) _rollM = !_rollM;
    if (id == SW_ROLLOVER_R) _rollR = !_rollR;

    Serial.print("Rollovers: L="); Serial.print(_rollL);
    Serial.print(" M="); Serial.print(_rollM);
    Serial.print(" R="); Serial.println(_rollR);

    gLeds.setRolloverState(_rollL, _rollM, _rollR);

    if (_rollL && _rollM && _rollR) {
      addScore(_rolloverBonus);
      gSound.playDropTarget();   // or a special award sound
      gLeds.startRolloverFlash();
      _rollL = _rollM = _rollR = false;
    }
    return;
  }

  // ---------- DROP TARGET L/M/R: stop slot reels ----------
  if (id == SW_DROP_L || id == SW_DROP_M || id == SW_DROP_R) {
    if (_state == SLOT_ACTIVE) {
      if (id == SW_DROP_L) Slot_stopReel(0);
      if (id == SW_DROP_M) Slot_stopReel(1);
      if (id == SW_DROP_R) Slot_stopReel(2);
    }
    return;
  }
}



  void onRelease(uint8_t) {/* no-op for now */ }

  void addScore(uint32_t inc){
    if (_state == GAME_OVER) return;  // no scoring when game is over
  _score += inc;
  Serial.print("Score now = ");
  Serial.println(_score);
  _d->showScore(_score);
  gSound.playBumperHit();

  // When we cross 5000, 10000, 15000, ...: arm roulette, don't spin yet
  if (!_rouletteReady && !_rouletteRunning &&
      _state != SLOT_ACTIVE && _score >= _nextRouletteScore)
  {
    _rouletteReady = true;
    _currentBet = BET_NONE;
    _nextRouletteScore += _rouletteStep;   // move threshold forward

     // LEDs: all three standups lit to say "pick a color"
    gLeds.setRouletteBet(0);              // clear any previous bet color
    gLeds.setRouletteReady(true);
    
    _d->showStatus("BET");                 // prompt player to pick a color
  }
}


  void startSlot(){
    Slot_init();
    Slot_startSpin();
    gSound.playSlotStart();
    _awaitPayout = true;
    _state = SLOT_ACTIVE;
    Serial.println("Slot started. Use Drop L/M/R to stop reels.");
  }
};

/* ============================ GLOBALS & TASKS ============================ */
DisplayManager gDisplay;
GameManager    gGame;

// Sanity LED on IOE2:GPA7 (optional)
static const uint8_t IOE2_GPA7_LED = 7;


static bool gSanityOn = true;
// Map IOE1 pin numbers to logical switch IDs
bool mapIOE1PinToSwitchId(uint8_t pin, SwitchId &out) {
  switch (pin) {
    // GPA0..A3 = bumpers
    case 0: out = SW_IOE1_A0; return true;  // BUMPER1
    case 1: out = SW_IOE1_A1; return true;  // BUMPER2
    case 2: out = SW_IOE1_A2; return true;  // BUMPER3
    case 3: out = SW_IOE1_A3; return true;  // BUMPER4

    // GPA4..A6 = standups
    case 4: out = SW_IOE1_A4; return true;  // STANDUP_TOP
    case 5: out = SW_IOE1_A5; return true;  // STANDUP_MID
    case 6: out = SW_IOE1_A6; return true;  // STANDUP_BOT

    // GPB3 = drain
    case 11: out = SW_IOE1_B3; return true; // RLV_DRAIN

    // GPB5..B7 = rollovers L/M/R
    case 13: out = SW_IOE1_B5; return true; // ROLLOVER_L
    case 14: out = SW_IOE1_B6; return true; // ROLLOVER_M
    case 15: out = SW_IOE1_B7; return true; // ROLLOVER_R

    default: return false; // pins we don’t care about
  }
}

// Map IOE2 GPB pins to SwitchId (active-LOW)
bool mapIOE2PinToSwitchId(uint8_t pin, SwitchId &out) {
  switch (pin) {
    case 13: out = SW_IOE2_B5; return true;  // DROP_L
    case 14: out = SW_IOE2_B6; return true;  // DROP_M
    case 15: out = SW_IOE2_B7; return true;  // DROP_R
    default: return false;
  }
}

void switchPollTask(void*){
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2); // ~500 Hz

  // Same IOE1 pin list as your test
  const uint8_t ioe1Pins[] = { 0,1,2,3,4,5,6, 11, 13,14,15 };

  for(;;){
    uint32_t now = millis();

    // ---- IOE1: ACTIVE-HIGH pins (bumpers, standups, drain, rollovers) ----
    for (uint8_t i = 0; i < sizeof(ioe1Pins); ++i) {
      uint8_t pin = ioe1Pins[i];
      bool nowHigh = ioe1.digitalRead(pin);

      // rising edge: LOW -> HIGH
      if (nowHigh && !lastIOE1[pin]) {
        SwitchId sid;
        if (mapIOE1PinToSwitchId(pin, sid)) {
          SwitchEvent e{ (uint8_t)sid, true, now };
          xQueueSend(gSwitchQ, &e, 0);
        }
      }
      // falling edge: HIGH -> LOW (optional release event)
      if (!nowHigh && lastIOE1[pin]) {
        SwitchId sid;
        if (mapIOE1PinToSwitchId(pin, sid)) {
          SwitchEvent e{ (uint8_t)sid, false, now };
          xQueueSend(gSwitchQ, &e, 0);
        }
      }

      lastIOE1[pin] = nowHigh;
    }

    // ---- IOE2: ACTIVE-LOW pins (flippers, drops, etc.) ----
       // ---- IOE2: ONLY GPB5..B7 as ACTIVE-LOW (DROP_L/M/R) ----
    const uint8_t ioe2Pins[] = { 13, 14, 15 };  // GPB5,6,7
    for (uint8_t i = 0; i < sizeof(ioe2Pins); ++i) {
      uint8_t pin = ioe2Pins[i];

      bool level   = ioe2.digitalRead(pin);
      bool pressed = (level == LOW);  // active-LOW

      if (pressed != lastIOE2[pin]) {
        SwitchId sid;
        if (mapIOE2PinToSwitchId(pin, sid)) {
          SwitchEvent e{ (uint8_t)sid, pressed, now };
          xQueueSend(gSwitchQ, &e, 0);
        }
        lastIOE2[pin] = pressed;
      }
    }


    vTaskDelayUntil(&last, period);
  }
}


void gameTask(void*){
  ioe2.pinMode(IOE2_GPA7_LED, OUTPUT);
  ioe2.digitalWrite(IOE2_GPA7_LED, HIGH);
  gSanityOn = true;

  for(;;){
    gGame.pollSwitchEvents();
    gGame.loopTick();
    gDisplay.tick();
    gLeds.update(millis());
    gDropCoil.update(millis());
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}






/* ================================= SETUP/LOOP ================================ */
void setup() {
  Serial.begin(115200);
  delay(60);

  // Displays + Slot rendering hookup
  gDisplay.begin();
  Slot_attachDisplay(&gDisplay.gamePanel());  // provide panel to Slot manager

  // Switches (IOE1 + IOE2)
    // === IOE1 + IOE2 init (simple style, like your working test) ===
  spiBus.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CS_IOE1);

  if (!ioe1.begin_SPI(CS_IOE1, &spiBus)) {
    Serial.println("IOE1 (MCP23S17) init FAILED");
    while (1) { delay(1000); }
  }
  if (!ioe2.begin_SPI(CS_IOE2, &spiBus)) {
    Serial.println("IOE2 (MCP23S17) init FAILED");
    while (1) { delay(1000); }
  }
  Serial.println("IOE1 + IOE2 init OK");

  // IOE1: configure only the used pins as INPUT (active-high)
  const uint8_t ioe1Pins[] = { 0,1,2,3,4,5,6, 11, 13,14,15 };
  for (uint8_t i = 0; i < sizeof(ioe1Pins); ++i) {
    uint8_t pin = ioe1Pins[i];
    ioe1.pinMode(pin, INPUT);
  }

  // IOE2: GPB1..B7 as INPUT_PULLUP (active-LOW), skip GPB3 (coil)
  const uint8_t ioe2Pins[] = { 13, 14, 15 };
  for (uint8_t i = 0; i < sizeof(ioe2Pins); ++i) {
    uint8_t pin = ioe2Pins[i];
    ioe2.pinMode(pin, INPUT_PULLUP);
  }


  if (!gLeds.begin(&spiBus, CS_IOE3, 100000)) {
    Serial.println("IOE3 (LED manager) init FAILED");
    while (1) { delay(100); }
  }

  gDropCoil.begin(&ioe2);

  gGame.begin(&gDisplay); 
  // Sound + Roulette
  gSound.begin(13);
  gRoulette.begin();

  // Queue + tasks
  gSwitchQ = xQueueCreate(64, sizeof(SwitchEvent));
  xTaskCreatePinnedToCore(switchPollTask, "swPoll", 4096, nullptr, 3, nullptr, 0); // Core 0
  xTaskCreatePinnedToCore(gameTask,       "game",   8192, nullptr, 4, nullptr, 1); // Core 1

  Serial.println("System up: polling on Core0, game on Core1.");

 

  // ====== PWM on IO27: 400 Hz, 80% duty (core v3 API) =======
const uint8_t PWM_PIN      = 27;
const uint32_t PWM_FREQ_HZ = 400;
const uint8_t PWM_RES_BITS = 10;

// Attach LEDC to this pin with desired frequency and resolution
bool ok = ledcAttach(PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
if (!ok) {
  Serial.println("LEDC attach failed on GPIO27");
}

// Compute 80% duty for chosen resolution
uint32_t maxVal   = (1u << PWM_RES_BITS) - 1u;   // e.g. 1023 for 10-bit
uint32_t duty80pc = (maxVal * 8u) / 10u;         // exact 80%

// Set duty on that pin
ledcWrite(PWM_PIN, duty80pc);
// ==========================================================

}

void loop() {
  // Empty — everything runs in tasks.
}

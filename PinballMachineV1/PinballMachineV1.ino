/***** Senior Design – Pinball Prototype (Single File, Integrated) *****
  Managers:
    - SwitchManager: interrupt edges for IO26/27/25/33/32
    - Slot Machine Manager: reels + MD_Parola rendering (no buttons here)
    - GameManager: reads edges, updates score, controls slot start/stop
  Notes:
    - Score display: for now, shown via Serial. Swap in your real display later.
************************************************************************/
// --- Forward decls to keep Arduino's auto-prototyper happy ---
struct Reel;                 // tell the compiler Reel exists

// ================== INCLUDES ==================
#include <Arduino.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <esp_system.h>  // for esp_random() on ESP32

// ================== PIN MAP ===================
// Game integration mapping
const uint8_t PIN_START_SLOT = 26;  // IO26 -> start slot mode
const uint8_t PIN_ADD_100    = 27;  // IO27 -> +100 only
const uint8_t PIN_REEL1      = 25;  // IO25 -> stop reel 1
const uint8_t PIN_REEL2      = 33;  // IO33 -> stop reel 2
const uint8_t PIN_REEL3      = 32;  // IO32 -> stop reel 3
// Shared SPI pins (ESP32 defaults)
constexpr uint8_t PIN_CLK  = 18;
constexpr uint8_t PIN_MOSI = 23;
//DF player stuff
static const int DF_TX = 17;   // ESP32 TX2 → DF RX (1kΩ resistor recommended)
static const int DF_RX = 16;   // ESP32 RX2 ← DF TX (optional)
static const int DF_BUSY = -1; // set to actual GPIO if using BUSY pin, else -1

HardwareSerial mp3Serial(2);
DFRobotDFPlayerMini dfp;



// ================== SOUND MANAGER ==================
class SoundManager {
public:
  bool begin(uint8_t volume = 24) {
    mp3Serial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);
    Serial.println("Initializing DFPlayer...");
    if (!dfp.begin(mp3Serial)) {
      Serial.println("❌ DFPlayer init failed. Check wiring and SD card (/mp3/0001.mp3).");
      return false;
    }
    dfp.setTimeOut(500);
    dfp.outputDevice(DFPLAYER_DEVICE_SD);
    dfp.volume(constrain(volume, 0, 30));
    dfp.EQ(DFPLAYER_EQ_NORMAL);
    Serial.println("✅ DFPlayer ready.");
    return true;
  }

  void play(uint16_t index) { dfp.playMp3Folder(index); }
  void stop() { dfp.stop(); }

  bool isPlaying() {
    if (DF_BUSY >= 0) return digitalRead(DF_BUSY) == LOW;
    return dfp.readState() == 1;
  }

  // Game event helpers
  void playGameStart()   { play(1); }  // /mp3/0001.mp3
  void playBumperHit()   { play(2); }  // /mp3/0002.mp3
  void playDropTarget()  { play(3); }
  void playSlotStart()   { play(4); }
  void playRouletteWin() { play(5); }
  void playGameOver()    { play(6); }
};

SoundManager gSound;

// ================== SWITCH MANAGER ==================
// (Free-function ISR version; tiny/IRAM-safe)

static volatile bool g_edgeStart=false, g_edgeAdd=false, g_edgeR1=false, g_edgeR2=false, g_edgeR3=false;
static volatile uint32_t g_tStart=0, g_tAdd=0, g_tR1=0, g_tR2=0, g_tR3=0;
static uint16_t g_addCooldownMs = 200;                 // helps avoid long-press repeats on +100
static constexpr uint32_t DEBOUNCE_US = 40000;         // 40 ms

void IRAM_ATTR isr_start() { uint32_t n=micros(); if (n-g_tStart>DEBOUNCE_US){ g_tStart=n; g_edgeStart=true; } }
void IRAM_ATTR isr_add()   { uint32_t n=micros(); uint32_t cool=(uint32_t)g_addCooldownMs*1000UL;
                             if ((n-g_tAdd>DEBOUNCE_US) && (cool==0 || (n-g_tAdd>cool))){ g_tAdd=n; g_edgeAdd=true; } }
void IRAM_ATTR isr_r1()    { uint32_t n=micros(); if (n-g_tR1>DEBOUNCE_US){ g_tR1=n; g_edgeR1=true; } }
void IRAM_ATTR isr_r2()    { uint32_t n=micros(); if (n-g_tR2>DEBOUNCE_US){ g_tR2=n; g_edgeR2=true; } }
void IRAM_ATTR isr_r3()    { uint32_t n=micros(); if (n-g_tR3>DEBOUNCE_US){ g_tR3=n; g_edgeR3=true; } }

class SwitchManager {
public:
  void begin() {
    pinMode(PIN_START_SLOT, INPUT_PULLUP);
    pinMode(PIN_ADD_100,    INPUT_PULLUP);
    pinMode(PIN_REEL1,      INPUT_PULLUP);
    pinMode(PIN_REEL2,      INPUT_PULLUP);
    pinMode(PIN_REEL3,      INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_START_SLOT), isr_start, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ADD_100),    isr_add,   FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_REEL1),      isr_r1,    FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_REEL2),      isr_r2,    FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_REEL3),      isr_r3,    FALLING);
  }

  bool takeStartSlot() { return take(g_edgeStart); }
  bool takeAdd100()    { return take(g_edgeAdd);   }
  bool takeReel1()     { return take(g_edgeR1);    }
  bool takeReel2()     { return take(g_edgeR2);    }
  bool takeReel3()     { return take(g_edgeR3);    }

  void setAdd100CooldownMs(uint16_t ms){ g_addCooldownMs = ms; }

private:
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  bool take(volatile bool& flag) {
    bool v;
    portENTER_CRITICAL(&_mux);
    v = flag; flag = false;
    portEXIT_CRITICAL(&_mux);
    return v;
  }
};

// ================== SLOT MACHINE MANAGER ==================
// Your original implementation, refactored for integration:
//  - No button ISRs inside (SwitchManager owns inputs).
//  - No auto-spin on setup (GameManager decides when to spin).
//  - Helper accessors for GameManager: Slot_startSpin/stopReel/tick,
//    Slot_isIdle(), Slot_isSpinning(), Slot_isShowing(), Slot_payout().

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

// ---------- DisplayManager (Score on CS=15 using Parola; Slot on CS=5) ----------
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

#define HARDWARE_TYPE   MD_MAX72XX::FC16_HW
#define DEVICES  4          // number of 8x8 modules for score display
#define CS_SCORE    15         // CS for score panel
#define PIN_CLK         18         // ESP32 SCLK
#define PIN_MOSI        23         // ESP32 MOSI (DIN)
// ---- hardware config for slot display ----
#define CS_GAME     5

class DisplayManager {
public:
  DisplayManager()
  : scoreDisp(HARDWARE_TYPE, PIN_MOSI, PIN_CLK, CS_SCORE, DEVICES),
    gameDisp (HARDWARE_TYPE, PIN_MOSI, PIN_CLK, CS_GAME,  DEVICES) {}

  void begin() {
    scoreDisp.begin(); scoreDisp.setIntensity(6);
    gameDisp.begin();  gameDisp.setIntensity(7);

    scoreDisp.displayText("0",     PA_RIGHT, 0, 0, PA_PRINT, PA_NO_EFFECT);
    gameDisp .displayText("READY", PA_LEFT,  0, 0, PA_PRINT, PA_NO_EFFECT);
  }

  void tick() { scoreDisp.displayAnimate(); gameDisp.displayAnimate(); }

  void showScore(uint32_t s) {
    static char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s);
    scoreDisp.displayText(buf, PA_RIGHT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  }
  void showStatus(const char* msg) {
    gameDisp.displayText(msg, PA_LEFT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  }

  // --- give Slot manager access to the game panel safely ---
  MD_Parola& gamePanel() { return gameDisp; }
  MD_MAX72XX* gameGfx()  { return gameDisp.getGraphicObject(); }

private:
  MD_Parola scoreDisp; // CS=15
  MD_Parola gameDisp;  // CS=5 (slot/status panel)
};


class RouletteManager {
public:
  // your pins (keep series resistors)
  const int RED[3] = {14,12,13};
  const int WHT[3] = {19,4,21};
  const int GRN    = 22;

  // lifecycle
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

  // start a new spin (returns false if already running)
  bool startSpin() {
    if (_state != IDLE) return false;

    // pick landing spot and path length
    _target = random(0, 7);
    int laps = random(3, 6);
    int extra = random(0, 7);
    _stepsTotal = laps * 7 + extra + _target;

    // timing profile
    _dFast = 40;      // ms at start
    _dSlow = 240;     // ms at end

    _idx = 0;
    _step = 0;
    _nextAt = millis();  // update immediately
    _winner = -1;

    _state = SPINNING;
    return true;
  }

  // call this every loop()
  void tick() {
    const uint32_t now = millis();

    switch (_state) {
      case IDLE: break;

      case SPINNING:
        if ((int32_t)(now - _nextAt) >= 0) {
          showIndex(_idx);
          // schedule next step with quadratic ease-out
          float t = (_stepsTotal == 0) ? 1.0f : (float)_step / (float)_stepsTotal; // 0→1
          unsigned long d = (unsigned long)(_dFast + (float)(_dSlow - _dFast) * (t*t));
          _nextAt = now + d;

          _idx = (_idx + 1) % 7;
          _step++;

          if (_step > _stepsTotal) {
            _winner = (_idx + 6) % 7;  // the LED we just left lit
            _blinkOn = false;
            _blinkCount = 0;
            _nextAt = now;             // blink immediately
            _state = BLINKING;
          }
        }
        break;

      case BLINKING:
        if ((int32_t)(now - _nextAt) >= 0) {
          if (_blinkOn) {
            digitalWrite(ORDER[_winner], LOW);
            _nextAt = now + _blinkOffMs;
            _blinkOn = false;
            _blinkCount++;
            if (_blinkCount >= _blinkCycles) {
              allOff();
              _state = IDLE;
            }
          } else {
            digitalWrite(ORDER[_winner], HIGH);
            _nextAt = now + _blinkOnMs;
            _blinkOn = true;
          }
        }
        break;
    }
  }

  bool isIdle() const { return _state == IDLE; }
  bool isRunning() const { return _state != IDLE; }
  int  winner() const { return _winner; } // -1 until finished

private:
  // pins in display order (edit if your wheel order changes)
  int ORDER[7];

  // state
  enum State : uint8_t { IDLE, SPINNING, BLINKING } _state{IDLE};
  int  _idx{0}, _step{0}, _stepsTotal{0}, _target{0}, _winner{-1};
  unsigned long _nextAt{0}, _dFast{40}, _dSlow{240};

  // blink params
  const int _blinkCycles = 4;
  const int _blinkOnMs   = 180;
  const int _blinkOffMs  = 120;
  bool _blinkOn{false};
  int  _blinkCount{0};

  // helpers
  void allOff(){ for (int i=0;i<7;i++) digitalWrite(ORDER[i], LOW); }
  void showIndex(int i){
    // turn previous off, current on (cheap allOff to avoid ghosting)
    static int prev = -1;
    if (prev >= 0) digitalWrite(ORDER[prev], LOW);
    digitalWrite(ORDER[i%7], HIGH);
    prev = i%7;
  }
};

// Forward declare the global so GameManager can see it
class RouletteManager;
extern RouletteManager gRoulette;

// ================== GAME MANAGER ==================
class GameManager {
public:
  enum State { READY, PLAYING, SLOT_ACTIVE };

  void begin(DisplayManager* dm, SwitchManager* sm) {
    _d = dm; _s = sm; _state = READY; _score = 0; _awaitPayout = false;
    _d->showScore(_score);
    Serial.println("READY: IO26 starts slot, IO27 adds +100.");
  }

  void loop() {

    gRoulette.tick(); 
      if (_rouletteRunning && gRoulette.isIdle()) {
        int winner = gRoulette.winner();     // 0..6 (6 is green with ORDER above)
        (void)winner;                        // hook payouts here if desired
        gSound.playRouletteWin();
        _nextRouletteScore += 500;
        _rouletteRunning = false;
        _d->showStatus("PLAY");
      }
    // Global: add points via IO27
    if (_s->takeAdd100()) addScore(100);

    // READY -> start slot on IO26
    if (_state == READY) { if (_s->takeStartSlot()) { startSlot(); } return;}

    // PLAYING: can start another slot on IO26
    if (_state == PLAYING) { if (_s->takeStartSlot()) { startSlot(); } return;}

    // SLOT_ACTIVE: handle reel stops and slot lifecycle
    if (_state == SLOT_ACTIVE) {
      if (_s->takeReel1()) Slot_stopReel(0);
      if (_s->takeReel2()) Slot_stopReel(1);
      if (_s->takeReel3()) Slot_stopReel(2);

      Slot_tick();

      // When slot fully done (spinning+FX complete), apply payout once
      if (_awaitPayout && Slot_isIdle()) {
        int p = Slot_payout();
        if (p > 0) addScore((uint32_t)p);
        _awaitPayout = false;
        _state = PLAYING;
        Serial.println("Slot finished. Back to PLAYING.");
      }
    }
  }

  private:
  DisplayManager* _d{nullptr};
  SwitchManager*  _s{nullptr};
  State _state{READY};
  uint32_t _score{0};
  bool _awaitPayout{false};
  uint32_t _nextRouletteScore{500};
  bool     _rouletteRunning{false};


  void addScore(uint32_t inc){
  _score += inc;
  _d->showScore(_score);
  gSound.playBumperHit();

  // trigger roulette at 500,1000,1500,... when not in SLOT_ACTIVE
    if (!_rouletteRunning && _state != SLOT_ACTIVE && _score >= _nextRouletteScore) {
      _d->showStatus("ROULETTE");
      gRoulette.startSpin();      // fire and forget
      _rouletteRunning = true;
    }
  }
    void startSlot(){
      Slot_startSpin();
      gSound.playSlotStart();  // play spin sound
      _awaitPayout = true;
      _state = SLOT_ACTIVE;
      Serial.println("Slot started. Use IO25/IO33/IO32 to stop reels.");
    }
};

// ================== GLOBALS + SETUP/LOOP ==================
DisplayManager gDisplay;
SwitchManager  gSwitches;
GameManager    gGame;
RouletteManager gRoulette;


void setup() {
  Serial.begin(115200);
  delay(50);
  gDisplay.begin();
  Slot_attachDisplay(&gDisplay.gamePanel());
  Slot_init();           // initialize slot display & reels (no autospin)
  gSwitches.begin();
  gSound.begin(13);  // Initialize DFPlayer, set volume 0–30
  gGame.begin(&gDisplay, &gSwitches);
  gRoulette.begin();
  Serial.println("System up.");
}

void loop() {
  gGame.loop();
  gDisplay.tick();  
  delay(1);
}

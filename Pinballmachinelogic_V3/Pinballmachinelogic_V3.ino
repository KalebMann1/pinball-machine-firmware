#include <SPI.h>
#include <Adafruit_MCP23X17.h>
#include <MD_Parola.h>
#include <MD_MAX72XX.h>
#include <DFRobotDFPlayerMini.h>

// ================== SCORE + GAME DISPLAYS (MAX7219) ==================
#define HARDWARE_TYPE   MD_MAX72XX::FC16_HW
#define DEVICES         4

// Score display
#define CS_SCORE        19
#define PIN_CLK_SCORE   18
#define PIN_MOSI_SCORE  23

// Game / status display (also used for slot reels)
#define CS_GAME         4
static const uint8_t SLOT_STRIP_LEN = 8;

MD_Parola scoreDisplay = MD_Parola(
  HARDWARE_TYPE,
  PIN_MOSI_SCORE, PIN_CLK_SCORE, CS_SCORE,
  DEVICES
);

MD_Parola gameDisplay = MD_Parola(
  HARDWARE_TYPE,
  PIN_MOSI_SCORE, PIN_CLK_SCORE, CS_GAME,
  DEVICES
);

// Global text buffers
char scoreBuf[12];
char gameBuf[16];

// ================== HSPI + IO EXPANDERS ==================
static const uint8_t PIN_SCK   = 32;  // HSPI SCK
static const uint8_t PIN_MISO  = 13;  // HSPI MISO (SO)
static const uint8_t PIN_MOSI  = 15;  // HSPI MOSI (SI)

// Chip selects
static const uint8_t CS_IOE1   = 33;  // bumpers/standups/drain/rollovers
static const uint8_t CS_IOE2   = 25;  // drop targets + drop-set coil
static const uint8_t CS_IOE3   = 26;  // LEDs (ball count, rollovers, slot-ready)

SPIClass spiBus(HSPI);
Adafruit_MCP23X17 ioe1;
Adafruit_MCP23X17 ioe2;

// IOE1 pins we care about:
// A0–A3 (0–3): bumpers
// A4–A6 (4–6): standups
// B3 (11):    drain
// B5–B7 (13–15): rollovers L/M/R

const uint8_t ioe1Pins[] = {
  0, 1, 2, 3,    // bumpers
  4, 5, 6, 8,     // standups spinner on a7
  11,            // drain
  13, 14, 15     // rollovers L/M/R
};


const uint8_t NUM_IOE1_PINS = sizeof(ioe1Pins) / sizeof(ioe1Pins[0]);

// IOE2 pins as drop target inputs (active-LOW): GPB5,6,7 = 13,14,15
const uint8_t ioe2Pins[] = { 13, 14, 15 };
const uint8_t NUM_IOE2_PINS = sizeof(ioe2Pins) / sizeof(ioe2Pins[0]);

// Track edges / input state
bool lastHighIOE1[16]      = { false };
bool lastPressedIOE2[16]   = { false };

// ================== GAME STATE ==================
long score      = 0;
int  ballCount  = 3;
bool gameOver   = false;

// Rollover latches
bool rollL = false;
bool rollM = false;   // middle board flaky but keep logic
bool rollR = false;

// === Spinner speed detection ===
unsigned long lastSpinnerHitMs = 0;
const unsigned long SPINNER_FAST_THRESHOLD_MS = 120;  // ms between hits = "fast spin"

// ================== IOE3 LED MANAGER ==================
// PORT A (0–7)
static const uint8_t IOE3_LED_BALL1      = 0;  // SW_IOE3_A0 – Ball 1
static const uint8_t IOE3_LED_BALL2      = 1;  // SW_IOE3_A1 – Ball 2
static const uint8_t IOE3_LED_BALL3      = 2;  // SW_IOE3_A2 – Ball 3
static const uint8_t IOE3_LED_SLOT_READY = 3;  // SW_IOE3_A3 – RSTAT

// PORT B (8–15)
static const uint8_t IOE3_LED_ROLL_L     = 8;  // SW_IOE3_B0 – Rollover L
static const uint8_t IOE3_LED_ROLL_M     = 9;  // SW_IOE3_B1 – Rollover M
static const uint8_t IOE3_LED_ROLL_R     = 10; // SW_IOE3_B2 – Rollover R

static const uint8_t IOE3_LED_ON_STATUS  = 12; // SW_IOE3_B4 – On status
static const uint8_t IOE3_LED_ROUL_W     = 13; // SW_IOE3_B5 – Roulette W
static const uint8_t IOE3_LED_ROUL_R     = 14; // SW_IOE3_B6 – Roulette R 
static const uint8_t IOE3_LED_ROUL_G     = 15; // SW_IOE3_B7 – Roulette G 

// ================== SCORE MILESTONES ==================
bool slotReady          = false;


long rouletteCheckpointScore = 0;      // score right after last roulette finishes
const long rouletteStep      = 2000;

bool inSlotMode         = false;  // slot mini-game active?

// ================== ROULETTE STATE ==================
enum RouletteBet : uint8_t { RBET_NONE, RBET_GREEN, RBET_RED, RBET_WHITE };

bool        rouletteArmed    = false;   // roulette "credit" armed, waiting for bet
bool        rouletteSpinning = false;   // roulette PCB currently spinning
RouletteBet rouletteBet      = RBET_NONE;
long        rouletteBetAmount = 0;      // 10% bet amount in points


// Map IOE1 standup pins to roulette colors:
// A4 (pin 4) = GREEN, A5 (pin 5) = RED, A6 (pin 6) = WHITE  :contentReference[oaicite:1]{index=1}
static RouletteBet betFromStandupPin(uint8_t pin)
{
  switch (pin)
  {
    case 4: return RBET_GREEN; // top standup -> green
    case 5: return RBET_RED;   // middle standup -> red
    case 6: return RBET_WHITE; // bottom standup -> white
    default: return RBET_NONE;
  }
}

// ================== DROP-SET COIL PULSE ==================
// IOE2 GPB3 = pin index 11
static const uint8_t IOE2_DROP_SET_PIN = 11;   // GPB3: Drop-set coil

bool dropArmPulseActive        = false;
unsigned long dropArmPulseStartMs = 0;
const unsigned long DROP_ARM_PULSE_MS = 400;


// ================== DFPLAYER / SOUND MANAGER ==================
HardwareSerial& mp3Serial = Serial2;

class SoundManager {
public:
  bool begin() {
    // Start UART to DFPlayer
    mp3Serial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

    // isACK = false → don't wait for ACKs (less blocking)
    if (!_dfp.begin(mp3Serial, false, true)) {   // (stream, isACK, doReset)
      Serial.println("DFPlayer init FAILED");
      return false;
    }
    
    _dfp.setTimeOut(200);

    _dfp.volume(22);         // 0..30
    Serial.println("DFPlayer init OK (ACK disabled via begin)");
    return true;
  }

  // PUBLIC API (same names you already use):
  void playBumperHit()          { enqueue(TRACK_BUMPER_HIT); }
  void playStandupHit()         { enqueue(TRACK_STANDUP_HIT); }
  void playSpinner()            { enqueue(TRACK_SPINNER); }
  void playSlotStart()          { enqueue(TRACK_SLOT_START); }
  void playSlotWin()            { enqueue(TRACK_SLOT_WIN); }
  void playSlotLose()           { enqueue(TRACK_SLOT_LOSE); }
  void playRolloversComplete()  { enqueue(TRACK_ROLLOVER_BONUS); }
  void playBallDrain()          { enqueue(TRACK_DRAIN); }
  void playGameStart()          { enqueue(TRACK_GAME_START); }
  void playGameOver()           { enqueue(TRACK_GAME_OVER); }

  // Call this once per loop()
  void loop() {
    unsigned long now = millis();

    if (_pendingTrack == 0) return;                 // nothing queued
    if (now - _lastPlayMs < MIN_GAP_MS) return;     // rate limit

    int track = _pendingTrack;
    _pendingTrack = 0;        // consume
    _lastPlayMs = now;

   Serial.print("DFPlayer PLAY track ");
    Serial.println(track);

    // For files in /MP3/0001_xxx.mp3 style:
    _dfp.playMp3Folder(track);
        // single actual DFPlayer call
  }

private:
  DFRobotDFPlayerMini _dfp;

  int _pendingTrack = 0;                     // simple 1-slot queue
  unsigned long _lastPlayMs = 0;
  static const unsigned long MIN_GAP_MS = 120;  // ms between commands

  // ---- TRACK MAPPING: match your SD filenames ----
  static const int TRACK_BUMPER_HIT      = 1;
  static const int TRACK_STANDUP_HIT     = 2;
  static const int TRACK_SPINNER         = 3;
  static const int TRACK_SLOT_START      = 4;
  static const int TRACK_SLOT_WIN        = 5;
  static const int TRACK_SLOT_LOSE       = 6;
  static const int TRACK_ROLLOVER_BONUS  = 7;
  static const int TRACK_DRAIN           = 8;
  static const int TRACK_GAME_START      = 9;
  static const int TRACK_GAME_OVER       = 10;

  void enqueue(int track) {
    if (track <= 0) return;
    _pendingTrack = track;   // overwrite older pending sound
  }
};

SoundManager gSound;



// ================== LED Manager ==================
class LedManager {
public:
  bool begin(SPIClass* spi, uint8_t cs, uint32_t spiHz = 100000) {
    _spi = spi;
    _cs  = cs;
    if (!_spi) return false;

    if (!_ioe3.begin_SPI(_cs, _spi, spiHz)) {
      return false;
    }

    // Ball count LEDs
    _ioe3.pinMode(IOE3_LED_BALL1, OUTPUT);
    _ioe3.pinMode(IOE3_LED_BALL2, OUTPUT);
    _ioe3.pinMode(IOE3_LED_BALL3, OUTPUT);

    // Slot-ready (RSTAT)
    _ioe3.pinMode(IOE3_LED_SLOT_READY, OUTPUT);

    // Rollover LEDs
    _ioe3.pinMode(IOE3_LED_ROLL_L, OUTPUT);
    _ioe3.pinMode(IOE3_LED_ROLL_M, OUTPUT);
    _ioe3.pinMode(IOE3_LED_ROLL_R, OUTPUT);

    // ON_STATUS + roulette LEDs
    _ioe3.pinMode(IOE3_LED_ON_STATUS, OUTPUT);
    _ioe3.pinMode(IOE3_LED_ROUL_W, OUTPUT);
    _ioe3.pinMode(IOE3_LED_ROUL_R, OUTPUT);
    _ioe3.pinMode(IOE3_LED_ROUL_G, OUTPUT);

    // Initial LED state
    setBallCount(0);
    setRolloverState(false, false, false);
    setSlotReady(false);
    setOnStatus(false);
    setRouletteReady(false);

    return true;
  }

  void setBallCount(int balls) {
    if (balls < 0) balls = 0;
    if (balls > 3) balls = 3;

    _ioe3.digitalWrite(IOE3_LED_BALL1, (balls >= 1) ? HIGH : LOW);
    _ioe3.digitalWrite(IOE3_LED_BALL2, (balls >= 2) ? HIGH : LOW);
    _ioe3.digitalWrite(IOE3_LED_BALL3, (balls >= 3) ? HIGH : LOW);
  }

  void setRolloverState(bool l, bool m, bool r) {
    _ioe3.digitalWrite(IOE3_LED_ROLL_L, l ? HIGH : LOW);
    _ioe3.digitalWrite(IOE3_LED_ROLL_M, m ? HIGH : LOW);
    _ioe3.digitalWrite(IOE3_LED_ROLL_R, r ? HIGH : LOW);
  }

  void setSlotReady(bool on) {
    _ioe3.digitalWrite(IOE3_LED_SLOT_READY, on ? HIGH : LOW);
  }

  void setOnStatus(bool on) {
    _ioe3.digitalWrite(IOE3_LED_ON_STATUS, on ? HIGH : LOW);
  }

  // "Roulette ready" = all three bet LEDs on
  void setRouletteReady(bool ready) {
    setRouletteAll(ready, ready, ready);
  }

  // Only one color on for the chosen bet:
  void setRouletteBetWhite() { setRouletteAll(true,  false, false); }
  void setRouletteBetRed()   { setRouletteAll(false, true,  false); }
  void setRouletteBetGreen() { setRouletteAll(false, false, true ); }

  // Turn all bet LEDs off
  void clearRouletteBet()    { setRouletteAll(false, false, false); }

private:
  SPIClass* _spi = nullptr;
  Adafruit_MCP23X17 _ioe3;
  uint8_t _cs = 0;

  void setRouletteAll(bool wOn, bool rOn, bool gOn) {
    _ioe3.digitalWrite(IOE3_LED_ROUL_W, wOn ? HIGH : LOW);
    _ioe3.digitalWrite(IOE3_LED_ROUL_R, rOn ? HIGH : LOW);
    _ioe3.digitalWrite(IOE3_LED_ROUL_G, gOn ? HIGH : LOW);
  }
};


LedManager gLeds;

// ================== DISPLAY HELPERS ==================
void showScore() {
  snprintf(scoreBuf, sizeof(scoreBuf), "%ld", score);

  scoreDisplay.displayClear();
  scoreDisplay.displayText(
    scoreBuf,
    PA_LEFT,
    0, 0,
    PA_PRINT, PA_NO_EFFECT
  );
}

void showGameMessage(const char* msg) {
  strncpy(gameBuf, msg, sizeof(gameBuf));
  gameBuf[sizeof(gameBuf)-1] = '\0';

  gameDisplay.displayClear();
  gameDisplay.displayText(
    gameBuf,
    PA_CENTER,
    0, 0,
    PA_PRINT, PA_NO_EFFECT
  );
}

// ================== SLOT ENGINE CLASS ==================
class SlotEngine {
public:
  static void init() {
    if (engineInited) return;
    engineInited = true;

    // Initialize reels
    initReel(R[0], 0, 44, 26, 90);
    initReel(R[1], 1, 40, 24, 88);
    initReel(R[2], 2, 36, 22, 86);

    // Symbol strips
    uint8_t s0[]={BAR,STAR,CHERRY,BLANK,BELL,SEVEN,CHERRY,BAR};
    uint8_t s1[]={BLANK,CHERRY,BAR,BELL,STAR,SEVEN,BLANK,STAR};
    uint8_t s2[]={STAR,BLANK,BELL,SEVEN,CHERRY,BAR,BELL,STAR};
   for (uint8_t i = 0; i < SLOT_STRIP_LEN; i++) R[0].strip[i] = s0[i];
    for (uint8_t i = 0; i < SLOT_STRIP_LEN; i++) R[1].strip[i] = s1[i];
    for (uint8_t i = 0; i < SLOT_STRIP_LEN; i++) R[2].strip[i] = s2[i];
   
    lastPayout = 0;
    state      = S_IDLE;
  }

  static void startSpin() {
    init();
    gameDisplay.displayClear();   // slot now owns the display

    // Start all reels spinning, skill-stop mode only
    for (int i=0;i<3;i++) {
      R[i].spinning      = true;
      R[i].armed         = false;
      R[i].stepInterval  = (uint16_t)(i==0?40 : i==1?36 : 32);
    }
    lastPayout = 0;
    stateTs    = millis();
    state      = S_SPINNING;
  }

  static void stopReel(uint8_t i) {
    if (i>2) return;
    Reel& r = R[i];
    if (!r.spinning || r.armed) return;

    // Skill-stop: stop on next symbol boundary with a small buffer
    r.stopOffset = computeStopOffsetBoundarySoon(r, 12);
    r.armed      = true;
  }

  static void tick() {
    if (state == S_SPINNING) {
      stepAndMaybeStop(R[0]);
      stepAndMaybeStop(R[1]);
      stepAndMaybeStop(R[2]);

      if (!R[0].spinning && !R[1].spinning && !R[2].spinning) {
  // Compute payout from landed symbols
  uint8_t a = landedSymbol(R[0]);
  uint8_t b = landedSymbol(R[1]);
  uint8_t c = landedSymbol(R[2]);

  // 3-of-a-kind → 1000
  // exactly 2-of-a-kind → 500
  // else → 0
  if (a == b && b == c) {
    lastPayout = 1000;
  } else if (a == b || a == c || b == c) {
    lastPayout = 500;
  } else {
    lastPayout = 0;
  }

  state   = S_SHOW;
  stateTs = millis();
}
    }
    else if (state == S_SHOW) {
      if (millis() - stateTs > 1200) state = S_IDLE;
    }
  }

  static bool isBusy() { return state != S_IDLE; }
  static int  getPayout() { return lastPayout; }

private:
  enum Sym : uint8_t { SEVEN, BAR, STAR, CHERRY, BELL, BLANK, SYM_COUNT };

  struct Reel {
  uint8_t  xStart;
  uint8_t  width;
  uint8_t  strip[SLOT_STRIP_LEN];
  uint8_t  stripLen;
  int32_t  offset;
  int8_t   dir;
  uint16_t stepInterval;
  uint16_t minInterval, maxInterval;
  uint32_t lastStep;
  bool     spinning;
  bool     armed;
  int32_t  stopOffset;
};


  enum SlotState : uint8_t { S_IDLE, S_SPINNING, S_SHOW };

  // Static data
  static bool      engineInited;
  static Reel      R[3];
  static SlotState state;
  static uint32_t  stateTs;
  static int       lastPayout;

  // Geometry / bitmaps
  static const uint8_t SYMBOL_COLS = 5, SPACER_COLS = 1;
  static const uint8_t SYMBOL_W    = SYMBOL_COLS + SPACER_COLS;
  static const uint8_t SYMBOL_ROWS = 7, SPACER_ROWS = 1;
  static const uint8_t SYMBOL_H    = SYMBOL_ROWS + SPACER_ROWS;

  static const uint8_t SYM_SEVEN[5];
  static const uint8_t SYM_BAR[5];
  static const uint8_t SYM_STAR[5];
  static const uint8_t SYM_CHERRY[5];
  static const uint8_t SYM_BELL[5];
  static const uint8_t SYM_BLANK[5];

  static const uint8_t* const SYM_BITMAPS[SYM_COUNT];
  static const int payout3[SYM_COUNT];

  static MD_MAX72XX* gfx() { return gameDisplay.getGraphicObject(); }

  static void initReel(Reel& r, uint8_t reelIndex, uint16_t step, uint16_t minI, uint16_t maxI) {
    static const uint8_t REEL_X_START[3] = { 0, 11, 22 };
    static const uint8_t REEL_WIDTH      = 10;
    r.xStart      = REEL_X_START[reelIndex];
    r.width       = REEL_WIDTH;
    r.stripLen = SLOT_STRIP_LEN;
    r.offset      = 0;
    r.dir         = -1;
    r.stepInterval= step;
    r.minInterval = minI;
    r.maxInterval = maxI;
    r.lastStep    = 0;
    r.spinning    = false;
    r.armed       = false;
    r.stopOffset  = 0;
  }

  static uint8_t glyphCol(uint8_t s, uint8_t colIn) {
    if (s >= SYM_COUNT) return 0;
    if (colIn >= SYMBOL_COLS) return 0;
    return SYM_BITMAPS[s][colIn];
  }

  static int32_t stripRows(const Reel& r) {
    return (int32_t)r.stripLen * SYMBOL_H;
  }

  static void renderReelVertical(const Reel& r) {
    MD_MAX72XX* g = gfx();
    if (!g) return;

    int32_t totalRows = stripRows(r);
    if (totalRows <= 0) return;
    int32_t base = r.offset % totalRows;
    if (base < 0) base += totalRows;

    uint8_t xStart = (r.width - SYMBOL_COLS) / 2;

    for (uint8_t x=0; x<r.width; x++) {
      uint8_t outCol = 0;

      if (x >= xStart && x < xStart + SYMBOL_COLS) {
        uint8_t colInSym = x - xStart;
        for (uint8_t y=0; y<8; y++) {
          int32_t rowIdx = (base + y) % totalRows;
          uint8_t symIdx = rowIdx / SYMBOL_H;
          uint8_t rowIn  = rowIdx % SYMBOL_H;
          uint8_t bit    = 0;
          if (rowIn < SYMBOL_ROWS) {
            uint8_t colBits = glyphCol(r.strip[symIdx], colInSym);
            bit = (colBits >> rowIn) & 0x01;
          }
          outCol |= (bit << y);
        }
      }
      g->setColumn(r.xStart + x, outCol);
    }
  }

  static int32_t computeStopOffsetBoundarySoon(const Reel& r, uint16_t minMoreRows) {
    int32_t total   = stripRows(r);
    int32_t curBase = r.offset % total;
    if (curBase < 0) curBase += total;

    int32_t mod = curBase % SYMBOL_H;
    int32_t delta;

    if (r.dir > 0) { // up
      delta = (mod == 0 ? SYMBOL_H : (SYMBOL_H - mod));
      if (delta < (int32_t)minMoreRows) {
        delta += ((minMoreRows - delta + SYMBOL_H - 1) / SYMBOL_H) * SYMBOL_H;
      }
      return r.offset + delta;
    } else {         // down
      delta = (mod == 0 ? SYMBOL_H : mod);
      if (delta < (int32_t)minMoreRows) {
        delta += ((minMoreRows - delta + SYMBOL_H - 1) / SYMBOL_H) * SYMBOL_H;
      }
      return r.offset - delta;
    }
  }

  static uint8_t landedSymbol(const Reel& r) {
    int32_t total = stripRows(r);
    int32_t base  = r.offset % total;
    if (base < 0) base += total;
    int idx = base / SYMBOL_H;
    return r.strip[idx % r.stripLen];
  }

  static bool stepAndMaybeStop(Reel& r) {
    if (!r.spinning) return false;

    uint32_t now = millis();
    if (now - r.lastStep < r.stepInterval) return false;
    r.lastStep = now;

    if (!r.armed) {
      if (r.stepInterval > r.minInterval) r.stepInterval -= 1;
      r.offset += r.dir;
      renderReelVertical(r);
      return false;
    }

    int32_t remain = (r.dir > 0) ? (r.stopOffset - r.offset) : (r.offset - r.stopOffset);
    if (remain < 64 && r.stepInterval < r.maxInterval) r.stepInterval += 2;
    else if (r.stepInterval > r.minInterval) r.stepInterval -= 1;

    r.offset += r.dir;
    renderReelVertical(r);

    if ((r.dir > 0 && r.offset >= r.stopOffset) ||
        (r.dir < 0 && r.offset <= r.stopOffset)) {
      r.offset = r.stopOffset;
      renderReelVertical(r);
      r.spinning = false;
      return true;
    }
    return false;
  }
};

class RouletteManager {
public:
  // IOE2-based: RLED1..RLED7 = IOE2 GPA0..GPA6
  // pass a pointer to ioe2 in begin()
  void begin(Adafruit_MCP23X17* dev) {
    _ioe2 = dev;
    if (!_ioe2) return;

    // Configure GPA0..GPA6 as outputs for RLED1..RLED7
    for (int i = 0; i < 7; i++) {
      _ioe2->pinMode(i, OUTPUT);   // IOE2 GPAi
      _ioe2->digitalWrite(i, LOW);
      ORDER[i] = i;                // index 0..6 -> pin 0..6
    }

    allOff();
    randomSeed((uint32_t)micros());
    _state = IDLE;
  }

  bool startSpin() {
    if (_state != IDLE || !_ioe2) return false;

    _target = random(0, 7);             // 0..6 winner index
    int laps  = random(3, 6);           // number of full laps
    int extra = random(0, 7);
    _stepsTotal = laps * 7 + extra + _target;

    Serial.print("Roulette startSpin: target=");
    Serial.print(_target);
    Serial.print(" totalSteps=");
    Serial.println(_stepsTotal);

    _dFast  = 40;
    _dSlow  = 240;
    _idx    = 0;
    _step   = 0;
    _nextAt = millis();
    _winner = -1;
    _state  = SPINNING;
    return true;
  }

  void tick() {
    if (!_ioe2) return;

    const uint32_t now = millis();

    switch (_state) {
      case IDLE:
        break;

      case SPINNING:
        if ((int32_t)(now - _nextAt) >= 0) {
          showIndex(_idx);

          float t = (_stepsTotal == 0) ? 1.0f
                                       : (float)_step / (float)_stepsTotal;
          unsigned long d =
            (unsigned long)(_dFast + (float)(_dSlow - _dFast) * (t * t));
          _nextAt = now + d;

          _idx  = (_idx + 1) % 7;
          _step++;

          if (_step > _stepsTotal) {
            _winner  = (_idx + 6) % 7;  // last lit index
            _blinkOn = false;
            _blinkCount = 0;
            _nextAt  = now;
            _state   = BLINKING;
          }
        }
        break;

      case BLINKING:
        if ((int32_t)(now - _nextAt) >= 0) {
          if (_blinkOn) {
            _ioe2->digitalWrite(ORDER[_winner], LOW);
            _nextAt   = now + _blinkOffMs;
            _blinkOn  = false;
            _blinkCount++;

            if (_blinkCount >= _blinkCycles) {
              allOff();
              _state = IDLE;
            }
          } else {
            _ioe2->digitalWrite(ORDER[_winner], HIGH);
            _nextAt  = now + _blinkOnMs;
            _blinkOn = true;
          }
        }
        break;
    }
  }

  bool isIdle() const { return _state == IDLE; }
  int  winner() const { return _winner; }

  // Winner color helper: 0–2 white, 3–5 red, 6 green
  const char* winnerColorName() const {
    if (_winner >= 0 && _winner <= 2) return "WHITE";
    if (_winner >= 3 && _winner <= 5) return "RED";
    if (_winner == 6)                 return "GREEN";
    return "UNKNOWN";
  }

private:
  Adafruit_MCP23X17* _ioe2 = nullptr;
  int ORDER[7];  // maps indices 0..6 -> IOE2 pins 0..6

  enum State : uint8_t { IDLE, SPINNING, BLINKING } _state{IDLE};
  int _idx{0}, _step{0}, _stepsTotal{0}, _target{0}, _winner{-1};
  unsigned long _nextAt{0}, _dFast{40}, _dSlow{240};

  const int _blinkCycles = 4;
  const int _blinkOnMs   = 180;
  const int _blinkOffMs  = 120;
  bool _blinkOn{false};
  int  _blinkCount{0};

  void allOff() {
    if (!_ioe2) return;
    for (int i = 0; i < 7; i++) {
      _ioe2->digitalWrite(ORDER[i], LOW);
    }
  }

  void showIndex(int i) {
    if (!_ioe2) return;
    static int prev = -1;
    if (prev >= 0) _ioe2->digitalWrite(ORDER[prev], LOW);
    int idx = i % 7;
    _ioe2->digitalWrite(ORDER[idx], HIGH);
    prev = idx;
  }
};


RouletteManager gRoulette;
// ---- Static member definitions for SlotEngine ----
bool      SlotEngine::engineInited = false;
SlotEngine::Reel      SlotEngine::R[3];
SlotEngine::SlotState SlotEngine::state     = SlotEngine::S_IDLE;
uint32_t SlotEngine::stateTs                = 0;
int      SlotEngine::lastPayout            = 0;

// Bitmaps
const uint8_t SlotEngine::SYM_SEVEN[5]  = {0x7F,0x02,0x04,0x08,0x10};
const uint8_t SlotEngine::SYM_BAR[5]    = {0x7F,0x41,0x41,0x41,0x7F};
const uint8_t SlotEngine::SYM_STAR[5]   = {0x14,0x08,0x3E,0x08,0x14};
const uint8_t SlotEngine::SYM_CHERRY[5] = {0x0C,0x1E,0x1E,0x0C,0x00};
const uint8_t SlotEngine::SYM_BELL[5]   = {0x1C,0x22,0x22,0x3E,0x08};
const uint8_t SlotEngine::SYM_BLANK[5]  = {0x00,0x00,0x00,0x00,0x00};

const uint8_t* const SlotEngine::SYM_BITMAPS[SlotEngine::SYM_COUNT] = {
  SlotEngine::SYM_SEVEN,
  SlotEngine::SYM_BAR,
  SlotEngine::SYM_STAR,
  SlotEngine::SYM_CHERRY,
  SlotEngine::SYM_BELL,
  SlotEngine::SYM_BLANK
};

const int SlotEngine::payout3[SlotEngine::SYM_COUNT] = {
  1000, 300, 150, 80, 50, 0
};

// ---- Simple global wrappers for SlotEngine ----
void Slot_startSpin()                 { SlotEngine::startSpin(); }
void Slot_stopReel(uint8_t reelIndex) { SlotEngine::stopReel(reelIndex); }
void Slot_tick()                      { SlotEngine::tick(); }
bool Slot_isBusy()                    { return SlotEngine::isBusy(); }
int  Slot_getPayout()                 { return SlotEngine::getPayout(); }

// ================== GAME STATE HELPERS ==================
void setGameOver() {
  if (gameOver) return;
  gameOver   = true;
  inSlotMode = false;
  slotReady  = false;

  Serial.println("GAME OVER: ball count reached 0");

  gSound.playGameOver();

  gLeds.setBallCount(0);
  gLeds.setRolloverState(false, false, false);
  gLeds.setSlotReady(false);
  gLeds.setRouletteReady(false);
  gLeds.setOnStatus(false);

  showGameMessage("GG");
}

void onFirstPlay() {
  static bool started = false;
  if (!started) {
    started = true;
    showGameMessage("PLAY");
  }
}

// Score milestones: SLOT auto-starts here
// ================== Score milestones helper ==================
void checkScoreMilestones() {
  if (gameOver) return;

  // Don't stack credits – only arm when nothing is pending
  if (rouletteArmed || rouletteSpinning) return;

  long delta = score - rouletteCheckpointScore;

  // Arm roulette once the player has earned 2000+ points since last spin
  if (delta >= rouletteStep) {
    long thisThreshold = rouletteCheckpointScore + rouletteStep;

    Serial.print("Roulette threshold reached at score >= ");
    Serial.println(thisThreshold);
    Serial.print("Current score = ");
    Serial.println(score);

    rouletteArmed     = true;
    rouletteBet       = RBET_NONE;
    rouletteBetAmount = 0;

    gLeds.setRouletteReady(true);  // all three bet LEDs on
    Serial.println("Roulette ARMED -> bet LEDs ON");
  }
}




// ================== IOE1 HANDLER ==================
void handleIOE1PinRise(uint8_t pin) {
  Serial.print("IOE1 pin P");
  Serial.print(pin);
  Serial.println(" went HIGH");

  // Ignore most inputs if game over
  if (gameOver) {
    if (pin == 11) {
      Serial.println("Drain detected but game is already over.");
    }
    return;
  }

  // ----- BUMPERS: A0..A3 → +100 -----
  if (pin == 0 || pin == 1 || pin == 2 || pin == 3) {
    score += 100;
    Serial.print("Bumper hit, score = ");
    Serial.println(score);
    onFirstPlay();
    showScore();
    checkScoreMilestones();
    gSound.playBumperHit();
    return;
  }

    // ----- STANDUPS: A4..A6 → +200 normally
  // The FIRST hit that locks in a roulette bet gives NO points.
  if (pin == 4 || pin == 5 || pin == 6) {
    bool usedForBet = false;

    // --- Roulette betting logic first ---
    if (rouletteArmed && !rouletteSpinning && rouletteBet == RBET_NONE) {
      RouletteBet bet = betFromStandupPin(pin);
        if (bet != RBET_NONE) {
      usedForBet = true;

      // 10% bet of CURRENT score – taken only now, when standup locks the bet
      long betAmount = score / 10;  // integer 10%
      rouletteBetAmount = betAmount;

      if (betAmount > 0) {
        score -= betAmount;         // remove bet from player's score
        if (score < 0) score = 0;
        Serial.print("Roulette bet amount (10% of score) = ");
        Serial.println(betAmount);
        showScore();
      } else {
        Serial.println("Score too low for roulette bet (10% is 0).");
      }

      rouletteBet      = bet;
      rouletteSpinning = gRoulette.startSpin();  // start roulette PCB animation
      rouletteArmed    = false;                  // credit is now consumed
      gLeds.setRouletteReady(false);             // no longer "ready" for a new bet

      Serial.print("Roulette bet placed on ");
      if (bet == RBET_GREEN) Serial.println("GREEN");
      else if (bet == RBET_RED)  Serial.println("RED");
      else if (bet == RBET_WHITE)Serial.println("WHITE");

      // Update IOE3 LEDs: only chosen color stays ON
      switch (bet) {
        case RBET_GREEN: gLeds.setRouletteBetGreen(); break;
        case RBET_RED:   gLeds.setRouletteBetRed();   break;
        case RBET_WHITE: gLeds.setRouletteBetWhite(); break;
        default: break;
      }
    }


    }

    // --- Scoring only if this hit was NOT used to place the bet ---
    if (!usedForBet) {
      score += 200;
      Serial.print("Standup hit, score = ");
      Serial.println(score);
      onFirstPlay();
      showScore();
      checkScoreMilestones();
      gSound.playStandupHit();
    }

    // Sound still plays regardless (you can move this into the !usedForBet
    // block if you ONLY want sound on scoring hits)
    

    return;
  }



  // ----- SPINNER: A7 (pin 7) -----
  if (pin == 8) {
    unsigned long now = millis();
    unsigned long dt  = now - lastSpinnerHitMs;
    lastSpinnerHitMs  = now;

    // Normal spinner scoring
    score += 100;  // tweak as you like
    Serial.print("Spinner hit, score = ");
    Serial.print(score);
    Serial.print(" (dt = ");
    Serial.print(dt);
    Serial.println(" ms)");
    onFirstPlay();
    showScore();
    gSound.playSpinner();

    // Fast spin → trigger slot (if not already in slot mode)
    bool fastSpin = (dt > 0 && dt <= SPINNER_FAST_THRESHOLD_MS);

    if (fastSpin && !inSlotMode && !Slot_isBusy() && !gameOver) {
      Serial.println("Fast spinner detected -> START SLOT MODE");

      // Light RSTAT, mark slot ready
      slotReady = true;
      gLeds.setSlotReady(true);

      // Pulse drop-set coil for 400 ms (IOE2 GPB3)
      ioe2.digitalWrite(IOE2_DROP_SET_PIN, HIGH);
      dropArmPulseActive   = true;
      dropArmPulseStartMs  = now;

      // Enter slot mode
      inSlotMode = true;
      Slot_startSpin();

      gSound.playSlotStart();
    }

    // Spinner still contributes to score-based roulette milestones
    checkScoreMilestones();
    return;
  }

  // ----- DRAIN: B3 (11) -----
  if (pin == 11) {
    Serial.println("Ball drain detected (IOE1 B3)");

    gSound.playBallDrain();

    if (ballCount > 0) {
      ballCount--;
      Serial.print("Ball count -> ");
      Serial.println(ballCount);

      gLeds.setBallCount(ballCount);
    }

    if (ballCount <= 0) {
      setGameOver();
    }
    return;
  }

  // ----- ROLLOVERS: B5..B7 (13–15) -----
  if (pin == 13 || pin == 14 || pin == 15) {
    if (pin == 13) rollL = true;
    if (pin == 14) rollM = true;
    if (pin == 15) rollR = true;

    Serial.print("Rollovers: L=");
    Serial.print(rollL);
    Serial.print(" M=");
    Serial.print(rollM);
    Serial.print(" R=");
    Serial.println(rollR);

    gLeds.setRolloverState(rollL, rollM, rollR);

    // Now require ALL three: L + M + R
    if (rollL && rollM && rollR) {
      long bonus = 1000;
      score += bonus;
      Serial.print("Rollovers complete (L+M+R)! +");
      Serial.print(bonus);
      Serial.print(" => score = ");
      Serial.println(score);
      gSound.playRolloversComplete();

      rollL = rollM = rollR = false;
      gLeds.setRolloverState(false, false, false);

      onFirstPlay();
      showScore();
      checkScoreMilestones();
    }
    return;
  }

}


// ================== IOE2 HANDLER (DROP TARGETS) ==================
void handleIOE2DropPress(uint8_t pin) {
  Serial.print("IOE2 drop pin P");
  Serial.print(pin);
  Serial.println(" PRESSED (active-LOW)");

  int reelIndex = -1;
  if (pin == 13) reelIndex = 0; // Left
  if (pin == 14) reelIndex = 1; // Middle
  if (pin == 15) reelIndex = 2; // Right

  // In slot mode, drop targets stop their reels ONLY
  if (inSlotMode && reelIndex >= 0) {
    Slot_stopReel(reelIndex);
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== PINBALL + SLOT (auto-start on score thresholds) ===");

  // Init HSPI
  spiBus.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CS_IOE1);

  // IOE1
  if (!ioe1.begin_SPI(CS_IOE1, &spiBus)) {
    Serial.println("IOE1 init FAILED");
    while (1) { delay(1000); }
  }

  // IOE2
  if (!ioe2.begin_SPI(CS_IOE2, &spiBus)) {
    Serial.println("IOE2 init FAILED");
    while (1) { delay(1000); }
  }

  Serial.println("IOE1 + IOE2 init OK");

  // Roulette LEDs (IOE2 GPA0..GPA6 = RLED1..RLED7)
  gRoulette.begin(&ioe2);

  // IOE3 via LED manager
  if (!gLeds.begin(&spiBus, CS_IOE3, 100000)) {
    Serial.println("IOE3 init FAILED");
    while (1) { delay(1000); }
  }
  Serial.println("IOE3 (LEDs) init OK");

    //Sound
  if (!gSound.begin()) {
    Serial.println("Sound system failed to start (DFPlayer)");
    // You *could* continue without sound instead of while(1)
  } else {
    gSound.playGameStart();   // little startup jingle
  }

  // IOE1 pins as INPUT
  for (uint8_t i = 0; i < NUM_IOE1_PINS; i++) {
    uint8_t pin = ioe1Pins[i];
    ioe1.pinMode(pin, INPUT);
  }

  // IOE2 drop target pins as INPUT_PULLUP
  for (uint8_t i = 0; i < NUM_IOE2_PINS; i++) {
    uint8_t pin = ioe2Pins[i];
    ioe2.pinMode(pin, INPUT_PULLUP);
  }

  
  // IOE2 drop-set coil as OUTPUT (idle LOW)
  ioe2.pinMode(IOE2_DROP_SET_PIN, OUTPUT);
  ioe2.digitalWrite(IOE2_DROP_SET_PIN, LOW);

  Serial.println("IOE1 watching: A0–A6, B3, B5–B7");
  Serial.println("IOE2 watching: B5–B7 as drops + drop-set coil GPB3");
  Serial.println("IOE3 driving: ball count, rollovers, RSTAT, ON_STATUS");

  // Displays
  scoreDisplay.begin();
  scoreDisplay.setIntensity(3);
  scoreDisplay.displayClear();

  gameDisplay.begin();
  gameDisplay.setIntensity(4);
  gameDisplay.displayClear();

  scoreDisplay.setZoneEffect(0, true, PA_FLIP_UD);
  scoreDisplay.setZoneEffect(0, true, PA_FLIP_LR);

  gameDisplay.setZoneEffect(0, true, PA_FLIP_UD);
  gameDisplay.setZoneEffect(0, true, PA_FLIP_LR);

  // Initial state
  score      = 0;
  ballCount  = 3;
  gameOver   = false;
  rollL = rollM = rollR = false;
  slotReady  = false;
  inSlotMode = false;
  rouletteCheckpointScore = score;

  Serial.print("Next roulette will arm at score >= ");
  Serial.println(rouletteCheckpointScore + rouletteStep);


  showScore();
  showGameMessage("READY");

  gLeds.setBallCount(ballCount);
  gLeds.setRolloverState(false, false, false);
  gLeds.setSlotReady(false);
  gLeds.setRouletteReady(false);
  gLeds.setOnStatus(true);
}

// ================== LOOP ==================
void loop() {
  // IOE1 polling (bumpers, standups, drain, rollovers)
  for (uint8_t i = 0; i < NUM_IOE1_PINS; i++) {
    uint8_t pin = ioe1Pins[i];
    bool nowHigh = ioe1.digitalRead(pin);

    if (nowHigh && !lastHighIOE1[pin]) {
      handleIOE1PinRise(pin);
    }
    lastHighIOE1[pin] = nowHigh;
  }

  // IOE2 polling (drop targets)
  for (uint8_t i = 0; i < NUM_IOE2_PINS; i++) {
    uint8_t pin = ioe2Pins[i];
    bool level   = ioe2.digitalRead(pin);
    bool pressed = (level == LOW);

    if (pressed != lastPressedIOE2[pin]) {
      if (pressed) {
        handleIOE2DropPress(pin);
      }
      lastPressedIOE2[pin] = pressed;
    }
  }

  // Finish 400ms drop-set coil pulse
  if (dropArmPulseActive && (millis() - dropArmPulseStartMs >= DROP_ARM_PULSE_MS)) {
    ioe2.digitalWrite(IOE2_DROP_SET_PIN, LOW);
    dropArmPulseActive = false;
  }

  // Slot engine + payout
  if (inSlotMode) {
    Slot_tick();

    if (!Slot_isBusy()) {
      int payout = Slot_getPayout();
      if (payout > 0) {
        score += payout;
        Serial.print("Slot payout +");
        Serial.print(payout);
        Serial.print(" => score = ");
        Serial.println(score);
        showScore();
        checkScoreMilestones();

        gSound.playSlotWin();
      } else {
        Serial.println("Slot finished with no payout.");
        gSound.playSlotLose();
      }

      // Clear slot credit until next 2500 milestone
      slotReady = false;
      gLeds.setSlotReady(false);
      inSlotMode = false;

      if (!gameOver) {
        showGameMessage("PLAY");
      }

      gameDisplay.displayClear();
    }
  }
// --- Roulette PCB animation ---
gRoulette.tick();

// Detect end of roulette spin
if (rouletteSpinning && gRoulette.isIdle()) {
  int w = gRoulette.winner();
  Serial.print("Roulette spin finished, winner index = ");
  Serial.print(w);
  Serial.print(" (color = ");
  const char* winName = gRoulette.winnerColorName();
  Serial.print(winName);
  Serial.println(")");

  // Map winner index to a RouletteBet color:
  // 0–2 WHITE, 3–5 RED, 6 GREEN
  RouletteBet winningColor = RBET_NONE;
  if      (w >= 0 && w <= 2) winningColor = RBET_WHITE;
  else if (w >= 3 && w <= 5) winningColor = RBET_RED;
  else if (w == 6)           winningColor = RBET_GREEN;

  if (rouletteBetAmount > 0) {
    if (winningColor == rouletteBet && rouletteBet != RBET_NONE) {
      int mult = (winningColor == RBET_GREEN) ? 7 : 2;  // green=7x, red/white=2x
      long win = rouletteBetAmount * mult;
      score += win;

      Serial.print("Roulette WIN! Bet=");
      Serial.print(rouletteBetAmount);
      Serial.print("  multiplier=");
      Serial.print(mult);
      Serial.print("  -> +");
      Serial.println(win);
    } else {
      // Loss: bet was already removed when the standup was hit
      Serial.print("Roulette LOSS. Lost bet of ");
      Serial.println(rouletteBetAmount);
    }

    rouletteBetAmount = 0;
    if (score < 0) score = 0;
    showScore();
  }

  // After payout/loss, this new score becomes the base for the next +2000
  rouletteCheckpointScore = score;
  Serial.print("Roulette resolved. New checkpoint = ");
  Serial.print(rouletteCheckpointScore);
  Serial.print("  |  Next roulette will arm at score >= ");
  Serial.println(rouletteCheckpointScore + rouletteStep);

  gLeds.clearRouletteBet();
  rouletteSpinning = false;
  rouletteArmed    = false;
  rouletteBet      = RBET_NONE;
}



  // ON_STATUS: blink in slot mode, solid otherwise (until game over)
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  if (!gameOver && inSlotMode) {
    unsigned long now = millis();
    if (now - lastBlink >= 300) {
      blinkState = !blinkState;
      gLeds.setOnStatus(blinkState);
      lastBlink = now;
    }
  } else if (!gameOver) {
    gLeds.setOnStatus(true);
  }

  // Parola animate (gameDisplay disabled during slot so graphics don't fight)
  scoreDisplay.displayAnimate();
  if (!inSlotMode) {
    gameDisplay.displayAnimate();
  }

  gSound.loop();

  delay(10);
}







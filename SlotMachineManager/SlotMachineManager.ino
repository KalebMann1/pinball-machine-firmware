// --------- forward declarations to satisfy Arduino auto-prototype step ----------

struct Reel;
struct Outcome;
enum SlotMode : uint8_t;   // <-- add this line

// --------- includes ----------
#include <MD_Parola.h>
#include <MD_MAX72XX.h>
#include <SPI.h>

// --------- hardware config (edit if needed) ----------
#define SLOT_HW_TYPE   MD_MAX72XX::FC16_HW
#define SLOT_MAX_DEVICES 4
#define SLOT_CS_PIN    5

#define SLOT_BTN0_PIN  25
#define SLOT_BTN1_PIN  26
#define SLOT_BTN2_PIN  27

// --------- display & zones ----------
static MD_Parola P(SLOT_HW_TYPE, SLOT_CS_PIN, SLOT_MAX_DEVICES);
static bool g_inited = false;

static const uint8_t ZONES = 3;
static uint8_t Z_START[ZONES];
static uint8_t Z_END  [ZONES];
static MD_MAX72XX* gfx() { return P.getGraphicObject(); }

// --------- symbols & geometry ----------
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

// --------- types (full definitions) ----------
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

struct Outcome {
  uint8_t sym[3];
  int     payout;
};

enum SlotMode : uint8_t { SLOT_MODE_SKILL_STOP = 0, SLOT_MODE_PREDETERMINED = 1 };
enum SlotState : uint8_t { S_IDLE, S_SPINNING, S_SHOW };

// --------- globals ----------
static Reel     R[3];
static Outcome  g_outcome;
static SlotMode g_mode = SLOT_MODE_SKILL_STOP;
static SlotState slotState = S_IDLE;
static uint32_t  stateTs = 0;

// --------- ISR buttons (ESP32) ----------
volatile bool     g_btnReq[3] = {false,false,false};
volatile uint32_t g_btnUs [3] = {0,0,0};
static const uint32_t DEBOUNCE_US = 25000;
portMUX_TYPE      g_isrMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrBtn0(){ uint32_t n=micros(); if (n-g_btnUs[0]>DEBOUNCE_US){ portENTER_CRITICAL_ISR(&g_isrMux); g_btnUs[0]=n; g_btnReq[0]=true; portEXIT_CRITICAL_ISR(&g_isrMux);} }
void IRAM_ATTR isrBtn1(){ uint32_t n=micros(); if (n-g_btnUs[1]>DEBOUNCE_US){ portENTER_CRITICAL_ISR(&g_isrMux); g_btnUs[1]=n; g_btnReq[1]=true; portEXIT_CRITICAL_ISR(&g_isrMux);} }
void IRAM_ATTR isrBtn2(){ uint32_t n=micros(); if (n-g_btnUs[2]>DEBOUNCE_US){ portENTER_CRITICAL_ISR(&g_isrMux); g_btnUs[2]=n; g_btnReq[2]=true; portEXIT_CRITICAL_ISR(&g_isrMux);} }

// --------- helpers that use Reel (now safe, struct defined above) ----------
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

// --------- result feedback FX ----------
static uint8_t g_baseIntensity = 3;

struct ResultFX {
  bool     active=false;
  bool     won=false;
  uint32_t started=0, last=0;
  uint16_t duration=1200, interval=60;
  uint8_t  phase=0;
} fx;

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

  // base frame
  for(int i=0;i<3;i++) renderReelVertical(R[i]);

  if (fx.won){
    static const uint8_t seq[4] = {3,8,12,8};
    P.setIntensity(seq[fx.phase & 3]);
    bool on = (fx.phase & 1);
    drawGutter(10,on); drawGutter(21,on);
    for (int k=0;k<4;k++){ gfx()->setPoint(random(0,32), random(0,8), true); }
  } else {
    P.setIntensity(1);
    bool on = (fx.phase & 1);
    drawGutter(10,on); drawGutter(21,on);
  }

  fx.phase++;
  if (now - fx.started >= fx.duration){
    fx.active=false;
    P.setIntensity(g_baseIntensity);
    drawGutter(10,false); drawGutter(21,false);
    for(int i=0;i<3;i++) renderReelVertical(R[i]);
  }
}

// --------- outcome helpers ----------
static void chooseOutcomeDeterministic(){ g_outcome.sym[0]=BAR; g_outcome.sym[1]=STAR; g_outcome.sym[2]=SEVEN; g_outcome.payout=0; }
static void evaluatePayoutFromLanded(){
  uint8_t a=landedSymbol(R[0]), b=landedSymbol(R[1]), c=landedSymbol(R[2]);
  g_outcome.sym[0]=a; g_outcome.sym[1]=b; g_outcome.sym[2]=c;
  g_outcome.payout = (a==b && b==c) ? payout3[a] : 0;
}

// --------- public-ish controls (kept here since single .ino) ----------
static void Slot_setMode(SlotMode m){ g_mode = m; }
static void Slot_setIntensity(uint8_t v){ if (!g_inited) return; g_baseIntensity = min<uint8_t>(v,15); P.setIntensity(g_baseIntensity); }

// --------- Arduino setup/loop + API-like functions ----------
static void Slot_init(){
  if (!g_inited){
    P.begin(ZONES);
    g_baseIntensity=3; P.setIntensity(g_baseIntensity);
    P.displayClear();
    g_inited=true;

    // zones (0..9), (11..20), (22..31) — gutters at 10, 21
    P.setZone(0,0,9);    Z_START[0]=0;  Z_END[0]=9;
    P.setZone(1,11,20);  Z_START[1]=11; Z_END[1]=20;
    P.setZone(2,22,31);  Z_START[2]=22; Z_END[2]=31;

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

    // buttons + interrupts
    pinMode(SLOT_BTN0_PIN, INPUT_PULLUP);
    pinMode(SLOT_BTN1_PIN, INPUT_PULLUP);
    pinMode(SLOT_BTN2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SLOT_BTN0_PIN), isrBtn0, FALLING);
    attachInterrupt(digitalPinToInterrupt(SLOT_BTN1_PIN), isrBtn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(SLOT_BTN2_PIN), isrBtn2, FALLING);

    randomSeed((uint32_t)micros());
  } else {
    P.displayClear();
    for(int i=0;i<3;i++) renderReelVertical(R[i]);
  }

  slotState = S_IDLE;
  g_outcome = Outcome{{BLANK,BLANK,BLANK},0};
}

static void Slot_startSpin(){
  chooseOutcomeDeterministic();  // only used in predetermined mode
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
  // consume button requests
  bool req[3];
  portENTER_CRITICAL(&g_isrMux);
  req[0]=g_btnReq[0]; g_btnReq[0]=false;
  req[1]=g_btnReq[1]; g_btnReq[1]=false;
  req[2]=g_btnReq[2]; g_btnReq[2]=false;
  portEXIT_CRITICAL(&g_isrMux);
  if (req[0]) Slot_stopReel(0);
  if (req[1]) Slot_stopReel(1);
  if (req[2]) Slot_stopReel(2);

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

// --------- Arduino entry points ----------
void setup(){
  Slot_init();
  // optional: Slot_setMode(SLOT_MODE_PREDETERMINED);
  // optional: Slot_setIntensity(3);

  // spin once at power-up for testing
  Slot_startSpin();
}

void loop(){
  Slot_tick();
}

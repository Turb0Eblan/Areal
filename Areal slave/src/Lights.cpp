/*
 * ============================================================
 *  LED Strip Slave Controller  —  ESP32
 *  Device ID: green
 *  SK6812 RGBW — 16 блоков по 100 LED (1600 total)
 *  Библиотека: NeoPixelBus (поддерживает RGBW нативно)
 *
 *  Маппинг блок → пин:
 *    Block 1  → Pin 32    Block 9  → Pin 16
 *    Block 2  → Pin 13    Block 10 → Pin 23
 *    Block 3  → Pin 26    Block 11 → Pin 17
 *    Block 4  → Pin 33    Block 12 → Pin 22
 *    Block 5  → Pin 25    Block 13 → Pin  5
 *    Block 6  → Pin 14    Block 14 → Pin 21
 *    Block 7  → Pin  4    Block 15 → Pin 18
 *    Block 8  → Pin 27    Block 16 → Pin 19
 *
 *  Команды:
 *    block <1-16> <mode>             — режим для блока
 *    block all <mode>                — режим для всех
 *    block <1-16> color (R,G,B,W)   — цвет блока
 *    block <1-16> (R,G,B,W)         — сокращённо
 *    <mode>                          — все блоки (совместимость)
 *
 *  Режимы: off | allOn | fade | snake | rain | random |
 *          rainbow | fire | pulse | auto
 *  Цвет:   (255,0,0,0)=красный  (0,0,0,255)=белый
 *
 *  platformio.ini:
 *    lib_deps = makuna/NeoPixelBus @ ^2.7.9
 * ============================================================
 */

#include <Arduino.h>
#include <NeoPixelBus.h>
#include <esp_now.h>
#include <WiFi.h>

#define NUM_BLOCKS      16
#define LEDS_PER_BLOCK  300
#define BRIGHTNESS_SCALE 250  // 0-255

const char* MY_DEVICE_ID = "blue";

const uint8_t BLOCK_PINS[NUM_BLOCKS] = {
  32, 13, 26, 33, 25, 14, 4, 27, 16, 23, 17, 22, 5, 21, 18, 19
};

// ESP32 имеет 8 RMT-каналов — блоки 1-8 через RMT, 9-16 через BitBang
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt0800KbpsMethod> strip0(LEDS_PER_BLOCK, 32);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt1800KbpsMethod> strip1(LEDS_PER_BLOCK, 13);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt2800KbpsMethod> strip2(LEDS_PER_BLOCK, 26);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt3800KbpsMethod> strip3(LEDS_PER_BLOCK, 33);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt4800KbpsMethod> strip4(LEDS_PER_BLOCK, 25);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt5800KbpsMethod> strip5(LEDS_PER_BLOCK, 14);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt6800KbpsMethod> strip6(LEDS_PER_BLOCK,  4);
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt7800KbpsMethod> strip7(LEDS_PER_BLOCK, 27);

NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip8 (LEDS_PER_BLOCK, 16);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip9 (LEDS_PER_BLOCK, 23);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip10(LEDS_PER_BLOCK, 17);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip11(LEDS_PER_BLOCK, 22);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip12(LEDS_PER_BLOCK,  5);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip13(LEDS_PER_BLOCK, 21);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip14(LEDS_PER_BLOCK, 18);
NeoPixelBus<NeoGrbwFeature, NeoEsp32BitBangWs2812xMethod> strip15(LEDS_PER_BLOCK, 19);

// ─── Pixel access ─────────────────────────────────────────────
void setRawPixel(int b, int i, RgbwColor c) {
  switch(b) {
    case  0: strip0.SetPixelColor(i,c);  break; case  1: strip1.SetPixelColor(i,c);  break;
    case  2: strip2.SetPixelColor(i,c);  break; case  3: strip3.SetPixelColor(i,c);  break;
    case  4: strip4.SetPixelColor(i,c);  break; case  5: strip5.SetPixelColor(i,c);  break;
    case  6: strip6.SetPixelColor(i,c);  break; case  7: strip7.SetPixelColor(i,c);  break;
    case  8: strip8.SetPixelColor(i,c);  break; case  9: strip9.SetPixelColor(i,c);  break;
    case 10: strip10.SetPixelColor(i,c); break; case 11: strip11.SetPixelColor(i,c); break;
    case 12: strip12.SetPixelColor(i,c); break; case 13: strip13.SetPixelColor(i,c); break;
    case 14: strip14.SetPixelColor(i,c); break; case 15: strip15.SetPixelColor(i,c); break;
  }
}

RgbwColor getRawPixel(int b, int i) {
  switch(b) {
    case  0: return strip0.GetPixelColor(i);  case  1: return strip1.GetPixelColor(i);
    case  2: return strip2.GetPixelColor(i);  case  3: return strip3.GetPixelColor(i);
    case  4: return strip4.GetPixelColor(i);  case  5: return strip5.GetPixelColor(i);
    case  6: return strip6.GetPixelColor(i);  case  7: return strip7.GetPixelColor(i);
    case  8: return strip8.GetPixelColor(i);  case  9: return strip9.GetPixelColor(i);
    case 10: return strip10.GetPixelColor(i); case 11: return strip11.GetPixelColor(i);
    case 12: return strip12.GetPixelColor(i); case 13: return strip13.GetPixelColor(i);
    case 14: return strip14.GetPixelColor(i); case 15: return strip15.GetPixelColor(i);
  }
  return RgbwColor(0);
}

void setPixel(int b, int i, RgbwColor c) {
  c.R = (uint8_t)((uint16_t)c.R * BRIGHTNESS_SCALE / 255);
  c.G = (uint8_t)((uint16_t)c.G * BRIGHTNESS_SCALE / 255);
  c.B = (uint8_t)((uint16_t)c.B * BRIGHTNESS_SCALE / 255);
  c.W = (uint8_t)((uint16_t)c.W * BRIGHTNESS_SCALE / 255);
  setRawPixel(b, i, c);
}

void setBlockSolid(int b, uint8_t r, uint8_t g, uint8_t bl, uint8_t w) {
  for (int i = 0; i < LEDS_PER_BLOCK; i++) setPixel(b, i, RgbwColor(r, g, bl, w));
}

void fadeBlock(int b, uint8_t amount) {
  for (int i = 0; i < LEDS_PER_BLOCK; i++) {
    RgbwColor c = getRawPixel(b, i);
    c.R = (uint8_t)((uint16_t)c.R * (255 - amount) / 255);
    c.G = (uint8_t)((uint16_t)c.G * (255 - amount) / 255);
    c.B = (uint8_t)((uint16_t)c.B * (255 - amount) / 255);
    c.W = (uint8_t)((uint16_t)c.W * (255 - amount) / 255);
    setRawPixel(b, i, c);
  }
}

void showAll() {
  strip0.Show();  strip1.Show();  strip2.Show();  strip3.Show();
  strip4.Show();  strip5.Show();  strip6.Show();  strip7.Show();
  strip8.Show();  strip9.Show();  strip10.Show(); strip11.Show();
  strip12.Show(); strip13.Show(); strip14.Show(); strip15.Show();
}

void clearAll() {
  for (int b = 0; b < NUM_BLOCKS; b++)
    for (int i = 0; i < LEDS_PER_BLOCK; i++)
      setRawPixel(b, i, RgbwColor(0));
  showAll();
}

// ─── Helpers ──────────────────────────────────────────────────
RgbwColor hsvToRgbw(uint8_t hue, uint8_t sat, uint8_t val) {
  uint8_t r, g, b;
  uint8_t region = hue / 43;
  uint8_t rem = (hue - region * 43) * 6;
  uint8_t p = (val * (255 - sat)) >> 8;
  uint8_t q = (val * (255 - ((sat * rem) >> 8))) >> 8;
  uint8_t t = (val * (255 - ((sat * (255 - rem)) >> 8))) >> 8;
  switch (region) {
    case 0: r=val; g=t;   b=p;   break;
    case 1: r=q;   g=val; b=p;   break;
    case 2: r=p;   g=val; b=t;   break;
    case 3: r=p;   g=q;   b=val; break;
    case 4: r=t;   g=p;   b=val; break;
    default:r=val; g=p;   b=q;   break;
  }
  return RgbwColor(r, g, b, 0);
}

uint8_t beatsin8val(uint8_t bpm, uint8_t low, uint8_t high) {
  uint32_t period = 60000UL / bpm;
  float phase = (float)(millis() % period) / period;
  return low + (uint8_t)((high - low) * (sinf(phase * TWO_PI - HALF_PI) + 1.0f) / 2.0f);
}

RgbwColor heatColor(uint8_t heat) {
  if (heat < 85)       return RgbwColor(heat*3, 0, 0, 0);
  else if (heat < 170) return RgbwColor(255, (heat-85)*3, 0, 0);
  else                 return RgbwColor(255, 255, (heat-170)*3, 0);
}

// ─── Modes ────────────────────────────────────────────────────
enum Mode {
  MODE_OFF, MODE_ALL_ON, MODE_FADE, MODE_SNAKE, MODE_RAIN,
  MODE_RANDOM, MODE_RAINBOW, MODE_FIRE, MODE_PULSE,
  MODE_SOLID_COLOR, MODE_COLOR_FADE, MODE_AUTO_CYCLE
};

struct BlockState {
  Mode  mode       = MODE_PULSE;
  bool  autoCycle  = false;
  uint8_t fadeHue  = 0;
  int   snakePos   = 0;
  uint8_t solidR=0, solidG=0, solidB=0, solidW=255;
  unsigned long modeStart  = 0;
  unsigned long lastRandom = 0;
  int   cycleIndex = 0;
  struct RainDrop { int pos=0; float bri=0; bool active=false; } drops[20];
  uint8_t heat[LEDS_PER_BLOCK];
  // colorFade
  RgbwColor colorFadeColors[8];
  uint8_t   colorFadeCount = 0;
  uint8_t   colorFadeIdx   = 0;
  float     colorFadeBlend = 0.0f;
  float     colorFadeSpeed = 0.01f;   // ~3 сек на переход
};

BlockState blocks[NUM_BLOCKS];

const int   SNAKE_LENGTH          = 50;
const unsigned long UPDATE_DELAY  = 30;
const unsigned long RANDOM_DELAY  = 2000;
const unsigned long MODE_DURATION = 10000;

const Mode AUTO_CYCLE_ORDER[] = {
  MODE_FADE, MODE_RAINBOW, MODE_SNAKE, MODE_RAIN,
  MODE_FIRE, MODE_PULSE, MODE_RANDOM, MODE_SOLID_COLOR
};
const int AUTO_CYCLE_COUNT = 8;
unsigned long lastUpdate = 0;

// ─── Палитра по умолчанию для cfade ───────────────────────────
const RgbwColor DEFAULT_CFADE_PALETTE[5] = {
  RgbwColor( 38,  54,  80, 0),  // Midnight Horizon
  RgbwColor(135, 173, 214, 0),  // Morning Mist
  RgbwColor(214, 184, 150, 0),  // Sandstone Light
  RgbwColor(113,  78,  58, 0),  // Dark Sienna
  RgbwColor( 89, 105,  59, 0),  // Olive Grove
};
const uint8_t DEFAULT_CFADE_COUNT = 5;

void setDefaultCFadePalette(int b) {
  for (uint8_t i = 0; i < DEFAULT_CFADE_COUNT; i++)
    blocks[b].colorFadeColors[i] = DEFAULT_CFADE_PALETTE[i];
  blocks[b].colorFadeCount = DEFAULT_CFADE_COUNT;
}

// ─── Effects ──────────────────────────────────────────────────
static inline uint8_t lerp8(uint8_t a, uint8_t b, float t) {
  return (uint8_t)((int)a + (int)(((int)b - (int)a) * t));
}

void updateColorFade(int b) {
  auto& bs = blocks[b];
  if (bs.colorFadeCount == 0) return;
  if (bs.colorFadeCount == 1) {
    for (int i = 0; i < LEDS_PER_BLOCK; i++) setPixel(b, i, bs.colorFadeColors[0]);
    return;
  }
  RgbwColor from = bs.colorFadeColors[bs.colorFadeIdx];
  RgbwColor to   = bs.colorFadeColors[(bs.colorFadeIdx + 1) % bs.colorFadeCount];
  float t = bs.colorFadeBlend;
  RgbwColor cur(lerp8(from.R, to.R, t), lerp8(from.G, to.G, t),
                lerp8(from.B, to.B, t), lerp8(from.W, to.W, t));
  for (int i = 0; i < LEDS_PER_BLOCK; i++) setPixel(b, i, cur);
  bs.colorFadeBlend += bs.colorFadeSpeed;
  if (bs.colorFadeBlend >= 1.0f) {
    bs.colorFadeBlend = 0.0f;
    bs.colorFadeIdx = (bs.colorFadeIdx + 1) % bs.colorFadeCount;
  }
}

void updateFade(int b) {
  blocks[b].fadeHue++;
  for (int i = 0; i < LEDS_PER_BLOCK; i++)
    setPixel(b, i, hsvToRgbw(blocks[b].fadeHue + (i*255/LEDS_PER_BLOCK), 255, 255));
}

void updateSnake(int b) {
  fadeBlock(b, 30);
  for (int i = 0; i < SNAKE_LENGTH; i++) {
    int pos = (blocks[b].snakePos - i + LEDS_PER_BLOCK) % LEDS_PER_BLOCK;
    setPixel(b, pos, hsvToRgbw((blocks[b].fadeHue + i*3)%255, 255, 255-(i*(255/SNAKE_LENGTH))));
  }
  blocks[b].snakePos = (blocks[b].snakePos + 1) % LEDS_PER_BLOCK;
  blocks[b].fadeHue += 2;
}

void updateRain(int b) {
  fadeBlock(b, 20);
  for (int i = 0; i < 20; i++) {
    auto& d = blocks[b].drops[i];
    if (!d.active) {
      if (random(100) < 8) { d.pos=random(LEDS_PER_BLOCK); d.bri=random(200,255); d.active=true; }
    } else {
      d.pos = (d.pos+1) % LEDS_PER_BLOCK;
      d.bri -= 3;
      if (d.bri > 0) {
        uint8_t bri = (uint8_t)d.bri;
        setPixel(b, d.pos, RgbwColor(0, bri/5, bri, 0));
        for (int j=1; j<=3; j++) {
          uint8_t tb = bri > j*20 ? bri-j*20 : 0;
          if (tb > 0) setPixel(b, (d.pos-j+LEDS_PER_BLOCK)%LEDS_PER_BLOCK, RgbwColor(0, tb/8, tb/2, 0));
        }
      } else d.active = false;
    }
  }
}

void updateRainbow(int b) {
  blocks[b].fadeHue++;
  for (int i = 0; i < LEDS_PER_BLOCK; i++)
    setPixel(b, i, hsvToRgbw(blocks[b].fadeHue + (uint8_t)(i*255/LEDS_PER_BLOCK), 255, 255));
}

void updateFire(int b) {
  uint8_t* h = blocks[b].heat;
  for (int i=0; i<LEDS_PER_BLOCK; i++) { int c=(int)h[i]-random(0,40); h[i]=c<0?0:(uint8_t)c; }
  for (int i=0; i<LEDS_PER_BLOCK; i++) {
    h[i] = (h[(i-1+LEDS_PER_BLOCK)%LEDS_PER_BLOCK] +
            h[(i-2+LEDS_PER_BLOCK)%LEDS_PER_BLOCK] +
            h[(i-3+LEDS_PER_BLOCK)%LEDS_PER_BLOCK]) / 3;
  }
  for (int j=0; j<3; j++) { int s=random(LEDS_PER_BLOCK); int sp=(int)h[s]+random(180,255); h[s]=sp>255?255:(uint8_t)sp; }
  for (int i=0; i<LEDS_PER_BLOCK; i++) setPixel(b, i, heatColor(h[i]));
}

void updatePulse(int b) {
  uint8_t bri = beatsin8val(12, 0, 255);
  for (int i=0; i<LEDS_PER_BLOCK; i++) setPixel(b, i, RgbwColor(0, 0, 0, bri));
}

void updateRandom(int b) {
  fadeBlock(b, 5);
  for (int i=0; i<10; i++)
    setPixel(b, random(LEDS_PER_BLOCK), RgbwColor(random(255), random(255), random(255), 0));
}

// ─── Mode init ────────────────────────────────────────────────
void initializeMode(int b, Mode mode) {
  switch(mode) {
    case MODE_OFF:         setBlockSolid(b,0,0,0,0);   break;
    case MODE_ALL_ON:      setBlockSolid(b,0,0,0,255); break;
    case MODE_SOLID_COLOR: setBlockSolid(b,blocks[b].solidR,blocks[b].solidG,blocks[b].solidB,blocks[b].solidW); break;
    case MODE_RAIN:   for (int i=0;i<20;i++){blocks[b].drops[i].active=false;blocks[b].drops[i].pos=random(LEDS_PER_BLOCK);} break;
    case MODE_FIRE:   memset(blocks[b].heat,0,LEDS_PER_BLOCK); break;
    case MODE_SNAKE:  blocks[b].snakePos=0; break;
    case MODE_FADE:
    case MODE_RAINBOW: blocks[b].fadeHue=0; break;
    case MODE_COLOR_FADE:
      blocks[b].colorFadeIdx   = 0;
      blocks[b].colorFadeBlend = 0.0f;
      break;
    default: break;
  }
  blocks[b].mode      = mode;
  blocks[b].modeStart = millis();
}

void updateAutoCycle(int b) {
  if (millis()-blocks[b].modeStart >= MODE_DURATION) {
    blocks[b].cycleIndex = (blocks[b].cycleIndex+1) % AUTO_CYCLE_COUNT;
    initializeMode(b, AUTO_CYCLE_ORDER[blocks[b].cycleIndex]);
  }
}

// ─── Color parser "(R,G,B,W)" ─────────────────────────────────
bool parseColorRGBW(String s, uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &w) {
  s.trim(); s.replace("(",""); s.replace(")",""); s.replace(" ","");
  int v[4]={0,0,0,0}, idx=0, start=0;
  for (int i=0; i<=(int)s.length() && idx<4; i++) {
    if (i==(int)s.length() || s[i]==',') { v[idx++]=s.substring(start,i).toInt(); start=i+1; }
  }
  if (idx<4) return false;
  r=constrain(v[0],0,255); g=constrain(v[1],0,255);
  b=constrain(v[2],0,255); w=constrain(v[3],0,255);
  return true;
}

// ─── Block command handler ────────────────────────────────────
void handleBlockCommand(int b, String cmd) {
  cmd.trim();
  Serial.printf("[BLOCK %d | Pin %d] %s\n", b+1, BLOCK_PINS[b], cmd.c_str());
  blocks[b].autoCycle = false;
  if      (cmd=="off")     initializeMode(b, MODE_OFF);
  else if (cmd=="allOn")   initializeMode(b, MODE_ALL_ON);
  else if (cmd=="fade")    initializeMode(b, MODE_FADE);
  else if (cmd=="snake")   initializeMode(b, MODE_SNAKE);
  else if (cmd=="rain")    initializeMode(b, MODE_RAIN);
  else if (cmd=="random")  initializeMode(b, MODE_RANDOM);
  else if (cmd=="rainbow") initializeMode(b, MODE_RAINBOW);
  else if (cmd=="fire")    initializeMode(b, MODE_FIRE);
  else if (cmd=="pulse")   initializeMode(b, MODE_PULSE);
  else if (cmd=="auto"||cmd=="cycle") { blocks[b].autoCycle=true; blocks[b].cycleIndex=0; initializeMode(b, AUTO_CYCLE_ORDER[0]); }
  else if (cmd.startsWith("color ")||cmd.startsWith("solid ")) {
    String cs=cmd.substring(cmd.indexOf(' ')+1);
    uint8_t r,g,bl,w;
    if (parseColorRGBW(cs,r,g,bl,w)) { blocks[b].solidR=r; blocks[b].solidG=g; blocks[b].solidB=bl; blocks[b].solidW=w; initializeMode(b, MODE_SOLID_COLOR); }
    else Serial.println("[WARN] Формат: (R,G,B,W) например (0,0,0,255)");
  }
  else if (cmd.startsWith("cfade")) {
    // cfade               — дефолтные 5 цветов
    // cfade (R,G,B,W),... — до 8 своих цветов
    setDefaultCFadePalette(b);  // начальный сброс
    String rest = cmd.substring(5); rest.trim();
    if (rest.length() > 0) {
      uint8_t cnt = 0;
      int pos = 0;
      while (pos < (int)rest.length() && cnt < 8) {
        int open  = rest.indexOf('(', pos);
        int close = rest.indexOf(')', pos);
        if (open == -1 || close == -1) break;
        String token = rest.substring(open, close + 1);
        uint8_t r, g, bl, w;
        if (parseColorRGBW(token, r, g, bl, w))
          blocks[b].colorFadeColors[cnt++] = RgbwColor(r, g, bl, w);
        pos = close + 1;
      }
      if (cnt >= 1) blocks[b].colorFadeCount = cnt;
    }
    initializeMode(b, MODE_COLOR_FADE);
  }
  else if (cmd.startsWith("(")) {
    uint8_t r,g,bl,w;
    if (parseColorRGBW(cmd,r,g,bl,w)) { blocks[b].solidR=r; blocks[b].solidG=g; blocks[b].solidB=bl; blocks[b].solidW=w; initializeMode(b, MODE_SOLID_COLOR); }
    else Serial.println("[WARN] Формат: (R,G,B,W) например (0,0,0,255)");
  }
  else Serial.printf("[WARN] Неизвестная команда: %s\n", cmd.c_str());
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("block ")) {
    String rest=cmd.substring(6); int sp=rest.indexOf(' ');
    if (sp==-1) { Serial.println("[WARN] Использование: block <1-16|all> <mode>"); return; }
    String target=rest.substring(0,sp), subcmd=rest.substring(sp+1);
    target.trim(); subcmd.trim();
    if (target.equalsIgnoreCase("all")) { for (int b=0;b<NUM_BLOCKS;b++) handleBlockCommand(b,subcmd); }
    else {
      int blockNum=target.toInt();
      if (blockNum<1||blockNum>NUM_BLOCKS) { Serial.printf("[WARN] Блок должен быть 1-%d\n",NUM_BLOCKS); return; }
      handleBlockCommand(blockNum-1, subcmd);
    }
  } else { for (int b=0;b<NUM_BLOCKS;b++) handleBlockCommand(b,cmd); }
}

void onDataReceive(const uint8_t* mac, const uint8_t* data, int len) {
  String msg=""; for (int i=0;i<len;i++) msg+=(char)data[i]; msg.trim();
  int colon=msg.indexOf(':'); if (colon==-1) return;
  String targetId=msg.substring(0,colon), cmd=msg.substring(colon+1);
  if (!targetId.equalsIgnoreCase(MY_DEVICE_ID)) return;
  Serial.printf("[ESP-NOW] %s\n", cmd.c_str());
  handleCommand(cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  strip0.Begin();  strip1.Begin();  strip2.Begin();  strip3.Begin();
  strip4.Begin();  strip5.Begin();  strip6.Begin();  strip7.Begin();
  strip8.Begin();  strip9.Begin();  strip10.Begin(); strip11.Begin();
  strip12.Begin(); strip13.Begin(); strip14.Begin(); strip15.Begin();
  clearAll();

  WiFi.mode(WIFI_STA); WiFi.disconnect();
  if (esp_now_init()!=ESP_OK) { Serial.println("[ERROR] ESP-NOW init failed!"); return; }
  esp_now_register_recv_cb(onDataReceive);

  for (int b=0;b<NUM_BLOCKS;b++) { setDefaultCFadePalette(b); initializeMode(b, MODE_PULSE); }

  Serial.printf("LED Slave [%s] ready. MAC: %s\n", MY_DEVICE_ID, WiFi.macAddress().c_str());
  Serial.println("Добавь в platformio.ini: lib_deps = makuna/NeoPixelBus @ ^2.7.9");
  Serial.println("Режимы: off|allOn|fade|snake|rain|random|rainbow|fire|pulse|cfade|auto");
  Serial.println("cfade — переход между цветами. cfade (R,G,B,W),(R,G,B,W),... — свои цвета");
  Serial.println("Примеры: (0,0,0,255)=белый  (255,0,0,0)=красный");
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

  if (millis()-lastUpdate >= UPDATE_DELAY) {
    lastUpdate = millis();
    for (int b=0;b<NUM_BLOCKS;b++) {
      if (blocks[b].autoCycle) updateAutoCycle(b);
      switch(blocks[b].mode) {
        case MODE_COLOR_FADE: updateColorFade(b); break;
        case MODE_FADE:    updateFade(b);    break;
        case MODE_SNAKE:   updateSnake(b);   break;
        case MODE_RAIN:    updateRain(b);    break;
        case MODE_RANDOM:
          if (millis()-blocks[b].lastRandom > RANDOM_DELAY) { updateRandom(b); blocks[b].lastRandom=millis(); }
          break;
        case MODE_RAINBOW: updateRainbow(b); break;
        case MODE_FIRE:    updateFire(b);    break;
        case MODE_PULSE:   updatePulse(b);   break;
        default: break;
      }
    }
    showAll();
  }
  delay(10);
}
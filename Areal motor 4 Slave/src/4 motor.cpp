/*
 * 5-Motor Slave Controller
 *
 * Lift order (0=top, 4=bottom):
 *   M1: dir=4  step=2   TMC2209 | UART: RX=18, TX=19 (1kOhm between 19→18)
 *   M2: dir=15 step=21  TMC2209 | UART: RX=16(RX2), TX=17(TX2) — hardware Serial2
 *   M3: dir=25 step=26  TB6600
 *   M4: dir=27 step=14  TB6600
 *   M5: dir=12 step=13  TB6600  (GPIO12 is safe — eFuse burned)
 *
 * Limit switches (NC, INPUT_PULLUP, HIGH=triggered):
 *   M1→5   M2→33  M3→32  M4→22  M5→23
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <TMCStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_coexist.h>

// =====================
// Pins
// =====================
#define STEP1   2
#define STEP2  21
#define STEP3  26
#define STEP4  14
#define STEP5  13

#define DIR1    4
#define DIR2   15
#define DIR3   25
#define DIR4   27
#define DIR5   12   // GPIO12 is safe after burning the eFuse

#define LIM1    5   // M1 (top)
#define LIM2   33   // M2
#define LIM3   32   // M3
#define LIM4   22   // M4
#define LIM5   23   // M5 (bottom)

// TMC2209 UART
// M1: Serial1, RX=18 (wire from PDN_UART), TX=19 (via 1kOhm to 18)
// M2: Serial2, RX=16(RX2), TX=17(TX2) — hardware UART2, no conflicts
#define TMC1_RX  18
#define TMC1_TX  19
#define TMC2_RX  16
#define TMC2_TX  17

#define TMC_R_SENSE    0.11f
#define TMC_DEFAULT_MA  800

static const uint8_t LIM_PINS[5]  = { LIM1, LIM2, LIM3, LIM4, LIM5 };
static const uint8_t STEP_PINS[5] = { STEP1, STEP2, STEP3, STEP4, STEP5 };
static const uint8_t DIR_PINS[5]  = { DIR1,  DIR2,  DIR3,  DIR4,  DIR5  };

// =====================
// Constants
// =====================
#define NUM_MOTORS     5
#define UP_MOTORS      4   // M5 stays in place during a full raise
#define DEFAULT_SPEED  8000.0f
#define DEFAULT_ACCEL   600.0f
#define EXTRA_STEPS     800L
#define DOWN_DIR        1
#define UP_DIR         -1

// Factory default steps (on first boot / after RESET), per motor
static const long DEFAULT_STEPS[NUM_MOTORS] = { 140000L, 100000L, 60000L, 40000L, 50000L };

// =====================
// AccelStepper
// =====================
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);
AccelStepper motor4(AccelStepper::DRIVER, STEP4, DIR4);
AccelStepper motor5(AccelStepper::DRIVER, STEP5, DIR5);

AccelStepper* const motors[NUM_MOTORS] = {
  &motor1, &motor2, &motor3, &motor4, &motor5
};

// =====================
// TMC2209
// Serial1 (M1): RX=18, TX=19
// Serial2 (M2): RX=16(RX2), TX=17(TX2) — hardware UART2, no conflicts
// =====================
TMC2209Stepper tmc1(&Serial1, TMC_R_SENSE, 0);
TMC2209Stepper tmc2(&Serial2, TMC_R_SENSE, 0);

static int tmcCurrentMA = TMC_DEFAULT_MA;
static int tmcMicrostep = 16;

void configureTMC1(int mA, int mstep) {
  tmc1.begin();
  tmc1.pdn_disable(true);
  tmc1.I_scale_analog(false);
  tmc1.mstep_reg_select(true);
  tmc1.rms_current(mA);
  tmc1.microsteps(mstep);
  tmc1.en_spreadCycle(false);
  tmc1.pwm_autoscale(true);
  tmc1.toff(3);
}

void configureTMC2(int mA, int mstep) {
  tmc2.begin();
  tmc2.pdn_disable(true);
  tmc2.I_scale_analog(false);
  tmc2.mstep_reg_select(true);
  tmc2.rms_current(mA);
  tmc2.microsteps(mstep);
  tmc2.en_spreadCycle(false);
  tmc2.pwm_autoscale(true);
  tmc2.toff(3);
}

void setupTMC() {
  Serial1.begin(115200, SERIAL_8N1, TMC1_RX, TMC1_TX);
  Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);
  delay(150);
  configureTMC1(tmcCurrentMA, tmcMicrostep);
  configureTMC2(tmcCurrentMA, tmcMicrostep);
  Serial.println("TMC2209: configured");
}

void applyTMCParams(int mA, int mstep) {
  tmcCurrentMA = mA;
  tmcMicrostep = mstep;
  configureTMC1(mA, mstep);
  configureTMC2(mA, mstep);
}

// =====================
// Other objects
// =====================
BluetoothSerial SerialBT;
Preferences     prefs;

// =====================
// Runtime parameters
// =====================
long  moveSteps[NUM_MOTORS];
long  trimSteps[NUM_MOTORS];
float maxSpeed     = DEFAULT_SPEED;
float acceleration = DEFAULT_ACCEL;

// =====================
// System state
// =====================
enum SystemState { STATE_INIT, STATE_IDLE, STATE_GOING_DOWN, STATE_GOING_UP, STATE_ADJ, STATE_ERROR };
enum Position    { POS_UNKNOWN = 0, POS_DOWN = 1, POS_UP = 2 };
enum HomingPhase { HP_IDLE, HP_BACKOFF, HP_SEEKING };

SystemState sysState    = STATE_IDLE;
Position    sysPosition = POS_UNKNOWN;
Position    motorPos[NUM_MOTORS];
HomingPhase homingPhase = HP_IDLE;
int         seqIndex    = 0;
bool        extraStep[NUM_MOTORS] = {};
int         adjMotorIdx = -1;
int         adjDir      = 0;
unsigned long errorTimer = 0;

// Motion mode: false=simultaneous, true=sequential (DOWN: M5→M1, UP: M1→M4)
bool seqMode = false;

// =====================
// Output
// =====================
#define BOTH_PRINTLN(s)       do { Serial.println(s);                 SerialBT.println(s);                 } while(0)
#define BOTH_PRINTF(fmt, ...) do { Serial.printf(fmt, ##__VA_ARGS__); SerialBT.printf(fmt, ##__VA_ARGS__); } while(0)

// =====================
// ESP-NOW
// =====================
typedef struct { uint8_t cmd; uint8_t motorIdx; } EspNowCmd;
volatile bool    espNowPending  = false;
volatile uint8_t espNowCmd      = 0;
volatile uint8_t espNowMotorIdx = 0;

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < (int)sizeof(EspNowCmd)) return;
  EspNowCmd p; memcpy(&p, data, sizeof(p));
  espNowCmd      = p.cmd;
  espNowMotorIdx = p.motorIdx;
  espNowPending  = true;
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // The slave keeps BluetoothSerial always on alongside ESP-NOW.
  // On the classic ESP32, Wi-Fi and BT Classic time-share a single radio.
  // IMPORTANT: esp_wifi_set_ps(WIFI_PS_NONE) must NOT be used here — with BT Classic active
  // the closed-source Wi-Fi lib (libnet80211) strictly requires modem sleep and
  // calls abort() ("Should enable WiFi modem sleep when both WiFi and
  // Bluetooth are enabled"); this caused a real crash/boot loop in testing. We keep
  // the stock WIFI_PS_MIN_MODEM (default) and only bias radio arbitration
  // toward Wi-Fi — the BT console is not time-critical, motor commands are.
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW FAIL"); return; }
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.printf("ESP-NOW OK  MAC=%s\n", WiFi.macAddress().c_str());
}

// =====================
// NVS
// =====================
void loadSettings() {
  prefs.begin("slave5", true);
  for (int i = 0; i < NUM_MOTORS; i++) {
    char k[8];
    snprintf(k, sizeof(k), "stp%d", i); moveSteps[i] = prefs.getLong(k, DEFAULT_STEPS[i]);
    snprintf(k, sizeof(k), "trm%d", i); trimSteps[i] = prefs.getLong(k, 0);
  }
  maxSpeed     = prefs.getFloat("spd", DEFAULT_SPEED);
  acceleration = prefs.getFloat("acc", DEFAULT_ACCEL);
  tmcCurrentMA = prefs.getInt("tmc_ma",  TMC_DEFAULT_MA);
  tmcMicrostep = prefs.getInt("tmc_ms",  16);
  seqMode      = prefs.getBool("seqmode", false);
  prefs.end();
}

void saveSettings() {
  prefs.begin("slave5", false);
  for (int i = 0; i < NUM_MOTORS; i++) {
    char k[8];
    snprintf(k, sizeof(k), "stp%d", i); prefs.putLong(k, moveSteps[i]);
    snprintf(k, sizeof(k), "trm%d", i); prefs.putLong(k, trimSteps[i]);
  }
  prefs.putFloat("spd", maxSpeed);
  prefs.putFloat("acc", acceleration);
  prefs.putInt("tmc_ma", tmcCurrentMA);
  prefs.putInt("tmc_ms", tmcMicrostep);
  prefs.putBool("seqmode", seqMode);
  prefs.end();
  BOTH_PRINTLN("NVS: saved");
}

void applyMotorSettings() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i]->setMaxSpeed(maxSpeed);
    motors[i]->setAcceleration(acceleration);
  }
}

void resetSettings() {
  for (int i = 0; i < NUM_MOTORS; i++) { moveSteps[i] = DEFAULT_STEPS[i]; trimSteps[i] = 0; }
  maxSpeed = DEFAULT_SPEED; acceleration = DEFAULT_ACCEL;
  tmcCurrentMA = TMC_DEFAULT_MA; tmcMicrostep = 16;
  saveSettings(); applyMotorSettings();
  applyTMCParams(tmcCurrentMA, tmcMicrostep);
  BOTH_PRINTLN("Settings: reset");
}

// =====================
// Utilities
// =====================
bool limitTriggered(int i) { return digitalRead(LIM_PINS[i]) == HIGH; } // NC limit switches: HIGH=triggered

void hardStop(int i) { motors[i]->setCurrentPosition(motors[i]->currentPosition()); }
void hardStopAll()   { for (int i = 0; i < NUM_MOTORS; i++) hardStop(i); }

long effectiveSteps(int i, int dir) {
  long s = moveSteps[i] + (dir == DOWN_DIR ? trimSteps[i] : -trimSteps[i]);
  return s * dir;
}

static const char* posName(Position p) {
  return p == POS_DOWN ? "DOWN" : p == POS_UP ? "UP" : "UNK";
}

bool upperMotorsUp(int idx) {
  for (int i = 0; i < idx; i++)
    if (motorPos[i] != POS_UP) return false;
  return true;
}

// =====================
// Homing
// =====================
void startHoming() {
  hardStopAll();
  for (int i = 0; i < NUM_MOTORS; i++) { extraStep[i] = false; motorPos[i] = POS_UNKNOWN; }
  sysState = STATE_INIT; sysPosition = POS_UNKNOWN;

  if (seqMode) {
    seqIndex = NUM_MOTORS - 1; homingPhase = HP_IDLE;
    BOTH_PRINTLN("HOMING: start (SEQ, M5→M1)");
  } else {
    homingPhase = HP_SEEKING;
    BOTH_PRINTLN("HOMING: start (SIM, all at once)");
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (limitTriggered(i)) {
        motors[i]->setCurrentPosition(0);
        motorPos[i] = POS_DOWN;  // already in place
      } else {
        motors[i]->move((moveSteps[i] + EXTRA_STEPS * 10) * DOWN_DIR);
      }
    }
  }
}

// --- SIM homing: all motors at once ---
void runInitSim() {
  bool allDone = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorPos[i] == POS_DOWN) continue;  // loop() already stopped it at the limit switch
    if (motors[i]->distanceToGo() == 0 && !extraStep[i]) {
      motors[i]->move(EXTRA_STEPS * DOWN_DIR); extraStep[i] = true;
    }
    if (motors[i]->distanceToGo() != 0) { allDone = false; continue; }
    // Ran out of steps without triggering the limit switch — treat as bottom position
    motorPos[i] = POS_DOWN;
  }
  if (allDone) {
    homingPhase = HP_IDLE;
    sysPosition = POS_DOWN; sysState = STATE_IDLE;
    BOTH_PRINTLN("HOMING_DONE");
  }
}

// --- SEQ homing: M5→M4→...→M1 ---
void runInitSeq() {
  AccelStepper &m = *motors[seqIndex];
  switch (homingPhase) {
    case HP_IDLE:
      if (limitTriggered(seqIndex)) {
        m.move(2000L * UP_DIR); homingPhase = HP_BACKOFF;
        BOTH_PRINTF("HOMING M%d: backoff\n", seqIndex+1);
      } else {
        m.move((moveSteps[seqIndex] + EXTRA_STEPS*10) * DOWN_DIR); homingPhase = HP_SEEKING;
        BOTH_PRINTF("HOMING M%d: seeking...\n", seqIndex+1);
      }
      break;
    case HP_BACKOFF:
      if (m.distanceToGo() == 0) {
        m.move((moveSteps[seqIndex] + EXTRA_STEPS*10) * DOWN_DIR); homingPhase = HP_SEEKING;
        BOTH_PRINTF("HOMING M%d: seeking...\n", seqIndex+1);
      }
      break;
    case HP_SEEKING:
      if (limitTriggered(seqIndex)) {
        hardStop(seqIndex); motors[seqIndex]->setCurrentPosition(0);
        motorPos[seqIndex] = POS_DOWN;
        BOTH_PRINTF("HOMING M%d: DONE\n", seqIndex+1);
        seqIndex--; homingPhase = HP_IDLE;
        if (seqIndex < 0) {
          sysPosition = POS_DOWN; sysState = STATE_IDLE; seqIndex = 0;
          BOTH_PRINTLN("HOMING_DONE");
        }
        return;
      }
      if (m.distanceToGo() == 0 && !extraStep[seqIndex]) {
        m.move(EXTRA_STEPS * DOWN_DIR); extraStep[seqIndex] = true;
      }
      if (m.distanceToGo() == 0 && extraStep[seqIndex]) {
        motorPos[seqIndex] = POS_DOWN;
        seqIndex--; homingPhase = HP_IDLE;
        if (seqIndex < 0) {
          sysPosition = POS_DOWN; sysState = STATE_IDLE; seqIndex = 0;
          BOTH_PRINTLN("HOMING_DONE");
        }
      }
      break;
  }
}

void runInit() { if (seqMode) runInitSeq(); else runInitSim(); }

// =====================
// Move down
// =====================

// --- Simultaneous (all motors at once) ---
void startGoDownSim() {
  for (int i = 0; i < NUM_MOTORS; i++) extraStep[i] = false;
  sysState = STATE_GOING_DOWN;
  bool anyStarted = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorPos[i] != POS_DOWN) {
      motors[i]->move(effectiveSteps(i, DOWN_DIR));
      anyStarted = true;
    }
  }
  if (!anyStarted) {
    sysPosition = POS_DOWN; sysState = STATE_IDLE;
    BOTH_PRINTLN("DOWN: all already down"); return;
  }
  BOTH_PRINTLN("CMD: DOWN (simultaneous)");
}

void runGoDownSim() {
  bool allDone = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorPos[i] == POS_DOWN) continue;

    if (limitTriggered(i)) {
      hardStop(i); motors[i]->setCurrentPosition(0);
      motorPos[i] = POS_DOWN;
      BOTH_PRINTF("DOWN: M%d limit OK\n", i+1);
      continue;
    }
    if (motors[i]->distanceToGo() == 0 && !extraStep[i]) {
      motors[i]->move(EXTRA_STEPS * DOWN_DIR); extraStep[i] = true;
    }
    if (motors[i]->distanceToGo() != 0) {
      allDone = false;
    } else {
      BOTH_PRINTF("WARN: M%d lim?\n", i+1);
      motorPos[i] = POS_DOWN;
    }
  }
  if (allDone) {
    sysPosition = POS_DOWN; sysState = STATE_IDLE;
    BOTH_PRINTLN("DOWN_DONE");
  }
}

// --- Sequential (M5→M4→M3→M2→M1) ---
static void downSeqStartNext() {
  while (seqIndex >= 0 && motorPos[seqIndex] == POS_DOWN) seqIndex--;
  if (seqIndex < 0) {
    sysPosition = POS_DOWN; sysState = STATE_IDLE;
    BOTH_PRINTLN("DOWN_DONE"); return;
  }
  extraStep[seqIndex] = false;
  motors[seqIndex]->move(effectiveSteps(seqIndex, DOWN_DIR));
  BOTH_PRINTF("DOWN: M%d...\n", seqIndex+1);
}

void startGoDownSeq() {
  for (int i = 0; i < NUM_MOTORS; i++) extraStep[i] = false;
  seqIndex = NUM_MOTORS - 1;
  sysState = STATE_GOING_DOWN;
  BOTH_PRINTLN("CMD: DOWN (M5→M1)");
  downSeqStartNext();
}

void runGoDownSeq() {
  int i = seqIndex;
  if (i < 0) { sysPosition = POS_DOWN; sysState = STATE_IDLE; BOTH_PRINTLN("DOWN_DONE"); return; }

  if (limitTriggered(i)) {
    hardStop(i); motors[i]->setCurrentPosition(0);
    motorPos[i] = POS_DOWN;
    BOTH_PRINTF("DOWN: M%d limit OK\n", i+1);
    seqIndex--; downSeqStartNext(); return;
  }
  if (motors[i]->distanceToGo() == 0 && !extraStep[i]) {
    motors[i]->move(EXTRA_STEPS * DOWN_DIR); extraStep[i] = true;
  }
  if (motors[i]->distanceToGo() == 0 && extraStep[i]) {
    BOTH_PRINTF("WARN: M%d lim?\n", i+1);
    motorPos[i] = POS_DOWN;
    seqIndex--; downSeqStartNext();
  }
}

// --- Dispatcher ---
void startGoDown() { if (seqMode) startGoDownSeq(); else startGoDownSim(); }
void runGoDown()   { if (seqMode) runGoDownSeq();   else runGoDownSim();   }

// =====================
// Move up
// =====================

// --- Simultaneous (M1..M4) ---
void startGoUpSim() {
  sysState = STATE_GOING_UP;
  bool anyStarted = false;
  for (int i = 0; i < UP_MOTORS; i++) {
    if (motorPos[i] != POS_UP) {
      motors[i]->move(effectiveSteps(i, UP_DIR)); anyStarted = true;
    }
  }
  if (!anyStarted) {
    sysPosition = POS_UP; sysState = STATE_IDLE;
    BOTH_PRINTLN("UP: all already up"); return;
  }
  BOTH_PRINTLN("CMD: UP (M1..M4 simultaneous, M5 stays)");
}

void runGoUpSim() {
  bool allDone = true;
  for (int i = 0; i < UP_MOTORS; i++) {
    if (motorPos[i] == POS_UP) continue;
    if (motors[i]->distanceToGo() != 0) { allDone = false; continue; }
    motorPos[i] = POS_UP;
    BOTH_PRINTF("UP: M%d done\n", i+1);
  }
  if (allDone) {
    sysPosition = POS_UP; sysState = STATE_IDLE;
    BOTH_PRINTLN("UP_DONE");
  }
}

// --- Sequential (M1→M2→M3→M4) ---
void startGoUpSeq() {
  seqIndex = 0; sysState = STATE_GOING_UP;
  // skip motors already raised
  while (seqIndex < UP_MOTORS && motorPos[seqIndex] == POS_UP) seqIndex++;
  if (seqIndex >= UP_MOTORS) {
    sysPosition = POS_UP; sysState = STATE_IDLE;
    BOTH_PRINTLN("UP: all already up"); return;
  }
  motors[seqIndex]->move(effectiveSteps(seqIndex, UP_DIR));
  BOTH_PRINTLN("CMD: UP (M1→M4 one by one, M5 stays)");
  BOTH_PRINTF("UP: M%d...\n", seqIndex+1);
}

void runGoUpSeq() {
  if (motors[seqIndex]->distanceToGo() != 0) return;
  motorPos[seqIndex] = POS_UP;
  BOTH_PRINTF("UP: M%d done\n", seqIndex+1);
  seqIndex++;
  while (seqIndex < UP_MOTORS && motorPos[seqIndex] == POS_UP) seqIndex++;
  if (seqIndex >= UP_MOTORS) {
    sysPosition = POS_UP; sysState = STATE_IDLE; seqIndex = 0;
    BOTH_PRINTLN("UP_DONE"); return;
  }
  motors[seqIndex]->move(effectiveSteps(seqIndex, UP_DIR));
  BOTH_PRINTF("UP: M%d...\n", seqIndex+1);
}

// --- Dispatcher ---
void startGoUp() { if (seqMode) startGoUpSeq(); else startGoUpSim(); }
void runGoUp()   { if (seqMode) runGoUpSeq();   else runGoUpSim();   }

// =====================
// Adjustment
// =====================
void runAdj() {
  if (adjMotorIdx < 0) { sysState = STATE_IDLE; return; }
  // Check only the motor that is moving
  if (motors[adjMotorIdx]->distanceToGo() != 0) return;
  motorPos[adjMotorIdx] = (adjDir == UP_DIR) ? POS_UP : POS_DOWN;
  BOTH_PRINTF("ADJ done: M%d pos=%ld [%s]\n",
    adjMotorIdx+1, motors[adjMotorIdx]->currentPosition(), posName(motorPos[adjMotorIdx]));
  adjMotorIdx = -1;
  sysState = STATE_IDLE;
}

// =====================
// TEST
// =====================
void runTest() {
  if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
  BOTH_PRINTLN("TEST: 400 steps forward/back...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    BOTH_PRINTF("  M%d (step=%d dir=%d)... ", i+1, STEP_PINS[i], DIR_PINS[i]);
    motors[i]->setMaxSpeed(400); motors[i]->setAcceleration(200); motors[i]->move(400);
    unsigned long t = millis();
    while (motors[i]->distanceToGo() != 0) {
      motors[i]->run();
      if (millis()-t > 5000) { BOTH_PRINTLN("TIMEOUT!"); goto nextM; }
    }
    delay(200);
    motors[i]->move(-400); t = millis();
    while (motors[i]->distanceToGo() != 0) {
      motors[i]->run();
      if (millis()-t > 5000) { BOTH_PRINTLN("TIMEOUT!"); goto nextM; }
    }
    BOTH_PRINTLN("OK");
    nextM: delay(300);
  }
  applyMotorSettings();
  BOTH_PRINTLN("TEST done");
}

// =====================
// Help
// =====================
void printHelp() {
  BOTH_PRINTLN("===== 5-MOTOR SLAVE =====");
  BOTH_PRINTLN("M1: dir=4  step=2  (top) TMC2209  lim=5   UART RX=18 TX=19");
  BOTH_PRINTLN("M2: dir=15 step=21       TMC2209  lim=33  UART RX=16(RX2) TX=17(TX2)");
  BOTH_PRINTLN("M3: dir=25 step=26       TB6600   lim=32");
  BOTH_PRINTLN("M4: dir=27 step=14       TB6600   lim=22");
  BOTH_PRINTLN("M5: dir=12 step=13 (bot) TB6600   lim=23");
  BOTH_PRINTLN("Limit switches: NC (HIGH=triggered)");
  BOTH_PRINTF("Mode: %s (MODE:SIM / MODE:SEQ)\n", seqMode ? "SEQ (one by one)" : "SIM (simultaneous)");
  BOTH_PRINTLN("--- All motors ---");
  BOTH_PRINTLN("  1        : DOWN (to limit switches)");
  BOTH_PRINTLN("  2        : UP  (M1..M4, M5 stays)");
  BOTH_PRINTLN("  0        : EMERGENCY STOP");
  BOTH_PRINTLN("  HOME     : homing");
  BOTH_PRINTLN("  TEST     : test each motor");
  BOTH_PRINTLN("  MODE:SIM : simultaneous (default)");
  BOTH_PRINTLN("  MODE:SEQ : one by one (DOWN: M5→M1, UP: M1→M4)");
  BOTH_PRINTLN("--- Single motor (N=1..5) ---");
  BOTH_PRINTLN("  MN:UP      : raise N");
  BOTH_PRINTLN("  MN:DOWN    : lower N");
  BOTH_PRINTLN("  MN:STOP    : stop N");
  BOTH_PRINTLN("  MN:ADJ:<n> : move (+n=UP, -n=DOWN)");
  BOTH_PRINTLN("  MN:S:<n>   : steps (100..500000)");
  BOTH_PRINTLN("  MN:T:<n>   : offset (-50000..50000)");
  BOTH_PRINTLN("--- TMC2209 ---");
  BOTH_PRINTLN("  CURR:<mA>  : M1+M2 current (100..2000)");
  BOTH_PRINTLN("  MSTEP:<n>  : M1+M2 microsteps (1/2/4/8/16/32/64/128/256)");
  BOTH_PRINTLN("  TMCSTAT    : driver status");
  BOTH_PRINTLN("--- All motors at once ---");
  BOTH_PRINTLN("  S:<n>      : steps for all (100..500000)");
  BOTH_PRINTLN("  T:<n>      : offset for all (-50000..50000)");
  BOTH_PRINTLN("--- Parameters ---");
  BOTH_PRINTLN("  SPD:<n>    : speed (100..8000)");
  BOTH_PRINTLN("  ACC:<n>    : acceleration (50..3000)");
  BOTH_PRINTLN("  POS / LIM / RESET / i");
  for (int i = 0; i < NUM_MOTORS; i++)
    BOTH_PRINTF("  M%d: pos=%ld steps=%ld trim=%+ld [%s]\n",
      i+1, motors[i]->currentPosition(), moveSteps[i], trimSteps[i], posName(motorPos[i]));
  BOTH_PRINTF("  Spd=%.0f Acc=%.0f CURR=%dmA MSTEP=%d SysPos=%s Mode=%s\n",
    maxSpeed, acceleration, tmcCurrentMA, tmcMicrostep, posName(sysPosition),
    seqMode ? "SEQ" : "SIM");
  BOTH_PRINTF("  MAC=%s\n", WiFi.macAddress().c_str());
  BOTH_PRINTLN("=========================");
}

// =====================
// Command handling
// =====================
String btBuf = "", usbBuf = "";

void processLine(const String &line) {
  if (line.equalsIgnoreCase("i") || line.equalsIgnoreCase("help")) { printHelp(); return; }
  if (line.equalsIgnoreCase("TEST")) { runTest(); return; }
  if (line.equalsIgnoreCase("HOME")) {
    if (sysState == STATE_GOING_DOWN || sysState == STATE_GOING_UP) { BOTH_PRINTLN("BUSY"); return; }
    startHoming(); return;
  }
  if (line.equalsIgnoreCase("POS")) {
    for (int i = 0; i < NUM_MOTORS; i++)
      BOTH_PRINTF("M%d: pos=%ld [%s]\n", i+1, motors[i]->currentPosition(), posName(motorPos[i]));
    return;
  }
  if (line.equalsIgnoreCase("LIM")) {
    for (int i = 0; i < NUM_MOTORS; i++)
      BOTH_PRINTF("  LIM%d pin%d: %s (raw=%d)\n",
        i+1, LIM_PINS[i], limitTriggered(i) ? "TRIGGERED" : "open", digitalRead(LIM_PINS[i]));
    return;
  }
  if (line.equalsIgnoreCase("TMCSTAT")) {
    BOTH_PRINTF("TMC M1: DRV_STATUS=0x%08X IFCNT=%d\n", tmc1.DRV_STATUS(), tmc1.IFCNT());
    BOTH_PRINTF("TMC M2: DRV_STATUS=0x%08X IFCNT=%d\n", tmc2.DRV_STATUS(), tmc2.IFCNT());
    return;
  }
  if (line.equalsIgnoreCase("RESET")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    resetSettings(); return;
  }
  if (line.equalsIgnoreCase("MODE:SIM")) {
    seqMode = false; saveSettings();
    BOTH_PRINTLN("MODE: SIM (simultaneous)"); return;
  }
  if (line.equalsIgnoreCase("MODE:SEQ")) {
    seqMode = true; saveSettings();
    BOTH_PRINTLN("MODE: SEQ (one by one, DOWN: M5→M1, UP: M1→M4)"); return;
  }
  if (line == "0") {
    hardStopAll(); sysState = STATE_IDLE; sysPosition = POS_UNKNOWN;
    seqIndex = 0; homingPhase = HP_IDLE; adjMotorIdx = -1;
    for (int i = 0; i < NUM_MOTORS; i++) motorPos[i] = POS_UNKNOWN;
    BOTH_PRINTLN("EMERGENCY_STOP"); return;
  }
  if (line == "1") {
    if (sysState == STATE_ERROR) { BOTH_PRINTLN("ERROR: send 0"); return; }
    if (sysState != STATE_IDLE)  { BOTH_PRINTLN("BUSY"); return; }
    if (sysPosition == POS_DOWN) { BOTH_PRINTLN("ALREADY_DOWN"); return; }
    startGoDown(); return;
  }
  if (line == "2") {
    if (sysState == STATE_ERROR) { BOTH_PRINTLN("ERROR: send 0"); return; }
    if (sysState != STATE_IDLE)  { BOTH_PRINTLN("BUSY"); return; }
    if (sysPosition == POS_UP)   { BOTH_PRINTLN("ALREADY_UP"); return; }
    startGoUp(); return;
  }
  if (line.startsWith("CURR:") || line.startsWith("curr:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    int v = line.substring(5).toInt();
    if (v < 100 || v > 2000) { BOTH_PRINTLN("ERR: 100..2000"); return; }
    applyTMCParams(v, tmcMicrostep); saveSettings();
    BOTH_PRINTF("TMC current: %d mA\n", v); return;
  }
  if (line.startsWith("MSTEP:") || line.startsWith("mstep:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    int v = line.substring(6).toInt();
    applyTMCParams(tmcCurrentMA, v); saveSettings();
    BOTH_PRINTF("TMC microsteps: %d\n", v); return;
  }
  if (line.startsWith("SPD:") || line.startsWith("spd:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long v = line.substring(4).toInt();
    if (v < 100 || v > 8000) { BOTH_PRINTLN("ERR: 100..8000"); return; }
    maxSpeed = (float)v; applyMotorSettings(); saveSettings();
    BOTH_PRINTF("SPD=%.0f\n", maxSpeed); return;
  }
  if (line.startsWith("ACC:") || line.startsWith("acc:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long v = line.substring(4).toInt();
    if (v < 50 || v > 3000) { BOTH_PRINTLN("ERR: 50..3000"); return; }
    acceleration = (float)v; applyMotorSettings(); saveSettings();
    BOTH_PRINTF("ACC=%.0f\n", acceleration); return;
  }
  // MN:CMD
  if (line.length() >= 4 && (line[0]=='M'||line[0]=='m') && isDigit(line[1]) && line[2]==':') {
    int idx = (int)(line[1] - '1');
    if (idx < 0 || idx >= NUM_MOTORS) { BOTH_PRINTLN("ERR: M1..M5"); return; }
    String cmd = line.substring(3);
    if (cmd.equalsIgnoreCase("UP")) {
      if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
      if (!upperMotorsUp(idx)) { BOTH_PRINTF("SAFETY: raise M1..M%d first\n", idx); return; }
      adjMotorIdx = idx; adjDir = UP_DIR;
      motors[idx]->move(effectiveSteps(idx, UP_DIR)); sysState = STATE_ADJ;
      BOTH_PRINTF("M%d: UP\n", idx+1); return;
    }
    if (cmd.equalsIgnoreCase("DOWN")) {
      if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
      adjMotorIdx = idx; adjDir = DOWN_DIR;
      motors[idx]->move(effectiveSteps(idx, DOWN_DIR)); sysState = STATE_ADJ;
      BOTH_PRINTF("M%d: DOWN\n", idx+1); return;
    }
    if (cmd.equalsIgnoreCase("STOP")) {
      hardStop(idx);
      if (sysState == STATE_ADJ) { sysState = STATE_IDLE; adjMotorIdx = -1; }
      BOTH_PRINTF("M%d: STOP pos=%ld\n", idx+1, motors[idx]->currentPosition()); return;
    }
    if (cmd.startsWith("ADJ:") || cmd.startsWith("adj:")) {
      if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
      long v = cmd.substring(4).toInt();
      if (v == 0) { BOTH_PRINTLN("ERR: ADJ=0"); return; }
      // + = UP (raise), - = DOWN (lower)
      int  dir   = (v > 0) ? UP_DIR : DOWN_DIR;
      long steps = abs(v) * dir;   // UP_DIR=-1: steps is negative (up)
      adjMotorIdx = idx; adjDir = dir;
      motors[idx]->move(steps); sysState = STATE_ADJ;
      BOTH_PRINTF("M%d: ADJ %+ld (%s)\n", idx+1, v, (v>0)?"UP":"DOWN"); return;
    }
    if (cmd.startsWith("S:") || cmd.startsWith("s:")) {
      if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
      long v = cmd.substring(2).toInt();
      if (v < 100 || v > 500000) { BOTH_PRINTLN("ERR: 100..500000"); return; }
      moveSteps[idx] = v; saveSettings();
      BOTH_PRINTF("M%d: steps=%ld\n", idx+1, v); return;
    }
    if (cmd.startsWith("T:") || cmd.startsWith("t:")) {
      if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
      long v = cmd.substring(2).toInt();
      if (v < -50000 || v > 50000) { BOTH_PRINTLN("ERR: -50000..50000"); return; }
      trimSteps[idx] = v; saveSettings();
      BOTH_PRINTF("M%d: trim=%+ld\n", idx+1, v); return;
    }
  }
  // S:<n>  — steps for all motors
  if (line.startsWith("S:") || line.startsWith("s:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long v = line.substring(2).toInt();
    if (v < 100 || v > 500000) { BOTH_PRINTLN("ERR: 100..500000"); return; }
    for (int i = 0; i < NUM_MOTORS; i++) moveSteps[i] = v;
    saveSettings();
    BOTH_PRINTF("ALL steps=%ld\n", v); return;
  }
  // T:<n>  — offset for all motors
  if (line.startsWith("T:") || line.startsWith("t:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long v = line.substring(2).toInt();
    if (v < -50000 || v > 50000) { BOTH_PRINTLN("ERR: -50000..50000"); return; }
    for (int i = 0; i < NUM_MOTORS; i++) trimSteps[i] = v;
    saveSettings();
    BOTH_PRINTF("ALL trim=%+ld\n", v); return;
  }
  BOTH_PRINTF("UNKNOWN: '%s' (type 'i')\n", line.c_str());
}

void handleSerial() {
  // USB Serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      usbBuf.trim();
      if (usbBuf.length() > 0) { String s = usbBuf; usbBuf = ""; processLine(s); }
    } else { usbBuf += c; if (usbBuf.length() > 64) usbBuf = ""; }
  }
  // BT — always read, otherwise STOP only arrives after the motor has already stopped on its own
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      btBuf.trim();
      if (btBuf.length() > 0) { String s = btBuf; btBuf = ""; processLine(s); }
    } else { btBuf += c; if (btBuf.length() > 64) btBuf = ""; }
  }
}

// =====================
// ESP-NOW
// =====================
void handleEspNow() {
  if (!espNowPending) return;
  espNowPending = false;
  uint8_t cmd = espNowCmd, mIdx = espNowMotorIdx;
  if (mIdx >= 1 && mIdx <= NUM_MOTORS) {
    int idx = mIdx - 1;
    if (cmd == 0) { hardStop(idx); return; }
    if (cmd == 2) {
      if (sysState != STATE_IDLE || !upperMotorsUp(idx)) return;
      adjMotorIdx = idx; adjDir = UP_DIR;
      motors[idx]->move(effectiveSteps(idx, UP_DIR)); sysState = STATE_ADJ; return;
    }
    if (cmd == 1) {
      if (sysState != STATE_IDLE) return;
      adjMotorIdx = idx; adjDir = DOWN_DIR;
      motors[idx]->move(effectiveSteps(idx, DOWN_DIR)); sysState = STATE_ADJ; return;
    }
    return;
  }
  if (cmd == 0) {
    hardStopAll(); sysState = STATE_IDLE; sysPosition = POS_UNKNOWN;
    seqIndex = 0; adjMotorIdx = -1;
    for (int i = 0; i < NUM_MOTORS; i++) motorPos[i] = POS_UNKNOWN; return;
  }
  if (cmd == 1) { if (sysState == STATE_IDLE && sysPosition != POS_DOWN) startGoDown(); return; }
  if (cmd == 2) { if (sysState == STATE_IDLE && sysPosition != POS_UP)   startGoUp();   return; }
  if (cmd == 3) { seqMode = true;  saveSettings(); BOTH_PRINTLN("MODE: SEQ (one by one)");   return; }
  if (cmd == 4) { seqMode = false; saveSettings(); BOTH_PRINTLN("MODE: SIM (simultaneous)"); return; }
}

// =====================
// SETUP / LOOP
// =====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 5-MOTOR SLAVE BOOT ===");

  for (int i = 0; i < NUM_MOTORS; i++) motorPos[i] = POS_UNKNOWN;

  for (int i = 0; i < NUM_MOTORS; i++)
    pinMode(LIM_PINS[i], INPUT_PULLUP);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(STEP_PINS[i], OUTPUT); digitalWrite(STEP_PINS[i], LOW);
    pinMode(DIR_PINS[i],  OUTPUT); digitalWrite(DIR_PINS[i],  LOW);
  }

  motor3.setMinPulseWidth(5);
  motor4.setMinPulseWidth(5);
  motor5.setMinPulseWidth(5);

  loadSettings();
  setupTMC();
  applyMotorSettings();

  SerialBT.begin("Slave Motor 5");
  Serial.println("BT: Slave Motor 5");

  initEspNow();
  Serial.println("READY. Commands: TEST, HOME, LIM, TMCSTAT, i");

  // Automatically move down on power-up
  startHoming();
}

void loop() {
  // === Instant stop at limit switch — before any run() ===
  if (sysState == STATE_GOING_DOWN) {
    if (seqMode) {
      // Sequential: watch only the current motor
      if (seqIndex >= 0 && motorPos[seqIndex] != POS_DOWN && limitTriggered(seqIndex)) {
        hardStop(seqIndex); motors[seqIndex]->setCurrentPosition(0);
        motorPos[seqIndex] = POS_DOWN;
      }
    } else {
      // Simultaneous: watch all of them
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (motorPos[i] != POS_DOWN && limitTriggered(i)) {
          hardStop(i); motors[i]->setCurrentPosition(0);
          motorPos[i] = POS_DOWN;
        }
      }
    }
  } else if (sysState == STATE_INIT) {
    if (seqMode) {
      if (limitTriggered(seqIndex)) hardStop(seqIndex);
    } else {
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (motorPos[i] != POS_DOWN && limitTriggered(i)) {
          hardStop(i); motors[i]->setCurrentPosition(0);
          motorPos[i] = POS_DOWN;
        }
      }
    }
  }

  for (int i = 0; i < NUM_MOTORS; i++) motors[i]->run();
  handleSerial();
  for (int i = 0; i < NUM_MOTORS; i++) motors[i]->run();
  handleEspNow();
  for (int i = 0; i < NUM_MOTORS; i++) motors[i]->run();

  switch (sysState) {
    case STATE_INIT:       runInit();   break;
    case STATE_GOING_DOWN: runGoDown(); break;
    case STATE_GOING_UP:   runGoUp();   break;
    case STATE_ADJ:        runAdj();    break;
    case STATE_IDLE:                    break;
    case STATE_ERROR: break;
  }
}

#include <Arduino.h>
#include <AccelStepper.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <TMCStepper.h>

BluetoothSerial SerialBT;
Preferences     prefs;

// =====================
// Пины
// =====================
#define M1_STEP_PIN    2
#define M1_DIR_PIN     15
#define M2_STEP_PIN    5
#define M2_DIR_PIN     18
#define LIMIT_PIN      4   // INPUT_PULLUP; HIGH = сработал (NC-выключатель), общий для обоих

// AS5600 I2C
#define AS5600_SDA      21
#define AS5600_SCL      22
#define AS5600_DIR_PIN  23

// TMC UART
#define TMC1_RX   16   // Serial2, TX через ~1кОм к UART драйвера 1
#define TMC1_TX   17
#define TMC2_RX   12   // Serial1, TX через ~1кОм к UART драйвера 2
#define TMC2_TX   13
#define R_SENSE       0.11f
#define DRIVER_ADDR   0

// =====================
// Настройки по умолчанию
// =====================
#define DEFAULT_STEPS      50000L
#define DEFAULT_MAX_SPEED  3000.0f
#define DEFAULT_ACCEL      800.0f
#define DEFAULT_RMS_MA     800
#define EXTRA_STEPS        500L

// =====================
// Рабочие параметры
// =====================
long  moveStepsM1  = DEFAULT_STEPS;
long  moveStepsM2  = DEFAULT_STEPS;
long  trimM1       = 0;   // постоянная поправка шагов M1 (NVS)
long  trimM2       = 0;   // постоянная поправка шагов M2 (NVS)
float maxSpeed     = DEFAULT_MAX_SPEED;
float acceleration = DEFAULT_ACCEL;

// =====================
// Объекты
// =====================
AccelStepper   motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper   motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
TMC2209Stepper driver1(&Serial2, R_SENSE, DRIVER_ADDR);
TMC2209Stepper driver2(&Serial1, R_SENSE, DRIVER_ADDR);

// =====================
// Конечный автомат
// =====================
enum SystemState { STATE_INIT, STATE_IDLE, STATE_GOING_DOWN, STATE_GOING_UP, STATE_ADJ, STATE_ERROR };
enum Position    { POS_UNKNOWN = 0, POS_DOWN = 1, POS_UP = 2 };
enum HomingPhase { HOMING_IDLE, HOMING_BACKOFF, HOMING_SEEKING };

SystemState sysState    = STATE_INIT;
Position    sysPosition = POS_UNKNOWN;
HomingPhase homingPhase = HOMING_IDLE;

bool m1Extra = false, m2Extra = false;
unsigned long errorTimer = 0;

#define DOWN_DIR  1
#define UP_DIR   -1

// =====================
// Вывод в оба интерфейса
// =====================
#define BOTH_PRINTLN(s)      do { Serial.println(s);                SerialBT.println(s);                } while(0)
#define BOTH_PRINTF(fmt,...) do { Serial.printf(fmt,##__VA_ARGS__); SerialBT.printf(fmt,##__VA_ARGS__); } while(0)

// =====================
// ESP-NOW
// =====================
typedef struct { uint8_t cmd; uint8_t reserved; } EspNowCmd;
volatile bool    espNowCmdPending = false;
volatile uint8_t espNowCmdVal     = 0;

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < (int)sizeof(EspNowCmd)) return;
  EspNowCmd pkt;
  memcpy(&pkt, data, sizeof(pkt));
  espNowCmdVal     = pkt.cmd;
  espNowCmdPending = true;
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init FAILED"); return; }
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.printf("ESP-NOW ready. MAC: %s\n", WiFi.macAddress().c_str());
}

// =====================
// AS5600
// =====================
uint16_t as5600ReadAngle() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom((uint8_t)0x36, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

void initAS5600() {
  pinMode(AS5600_DIR_PIN, OUTPUT);
  digitalWrite(AS5600_DIR_PIN, LOW);
  Wire.begin(AS5600_SDA, AS5600_SCL);
  uint16_t angle = as5600ReadAngle();
  if (angle == 0xFFFF) Serial.println("AS5600: not found!");
  else                 Serial.printf("AS5600 init OK, angle: %u\n", angle);
}

// =====================
// NVS
// =====================
void loadSettings() {
  prefs.begin("slave2", true);
  moveStepsM1  = prefs.getLong  ("stepsM1",  DEFAULT_STEPS);
  moveStepsM2  = prefs.getLong  ("stepsM2",  DEFAULT_STEPS);
  trimM1       = prefs.getLong  ("trimM1",   0);
  trimM2       = prefs.getLong  ("trimM2",   0);
  maxSpeed     = prefs.getFloat ("maxSpeed", DEFAULT_MAX_SPEED);
  acceleration = prefs.getFloat ("accel",    DEFAULT_ACCEL);
  prefs.end();
  Serial.printf("NVS: M1=%ld M2=%ld trim1=%ld trim2=%ld spd=%.0f acc=%.0f\n",
                moveStepsM1, moveStepsM2, trimM1, trimM2, maxSpeed, acceleration);
}

void saveSettings() {
  prefs.begin("slave2", false);
  prefs.putLong  ("stepsM1",  moveStepsM1);
  prefs.putLong  ("stepsM2",  moveStepsM2);
  prefs.putLong  ("trimM1",   trimM1);
  prefs.putLong  ("trimM2",   trimM2);
  prefs.putFloat ("maxSpeed", maxSpeed);
  prefs.putFloat ("accel",    acceleration);
  prefs.end();
  BOTH_PRINTLN("NVS saved.");
}

void applyMotorSettings() {
  motor1.setMaxSpeed(maxSpeed); motor1.setAcceleration(acceleration);
  motor2.setMaxSpeed(maxSpeed); motor2.setAcceleration(acceleration);
}

void resetSettings() {
  moveStepsM1  = DEFAULT_STEPS;
  moveStepsM2  = DEFAULT_STEPS;
  trimM1       = 0;
  trimM2       = 0;
  maxSpeed     = DEFAULT_MAX_SPEED;
  acceleration = DEFAULT_ACCEL;
  saveSettings();
  applyMotorSettings();
  BOTH_PRINTLN("Settings reset.");
}

// =====================
// Утилиты
// =====================
bool limitTriggered() { return digitalRead(LIMIT_PIN) == HIGH; }

void hardStop1()   { motor1.setCurrentPosition(motor1.currentPosition()); }
void hardStop2()   { motor2.setCurrentPosition(motor2.currentPosition()); }
void hardStopAll() { hardStop1(); hardStop2(); }

// Итоговые шаги с учётом TRIM
long stepsM1(int dir) { return (moveStepsM1 + (dir == DOWN_DIR ? trimM1 : -trimM1)) * dir; }
long stepsM2(int dir) { return (moveStepsM2 + (dir == DOWN_DIR ? trimM2 : -trimM2)) * dir; }

// =====================
// Help
// =====================
void printHelp() {
  BOTH_PRINTLN("===== SLAVE CONTROLLER 2 =====");
  BOTH_PRINTLN("-- Управление --");
  BOTH_PRINTLN("  1           : DOWN (к концевику)");
  BOTH_PRINTLN("  2           : UP");
  BOTH_PRINTLN("  0           : EMERGENCY STOP");
  BOTH_PRINTLN("  HOME        : запустить хоминг");
  BOTH_PRINTLN("-- Шаги --");
  BOTH_PRINTLN("  M1:<N>      : шаги M1 (100..500000)");
  BOTH_PRINTLN("  M2:<N>      : шаги M2 (100..500000)");
  BOTH_PRINTLN("-- Подстройка (если разъехались) --");
  BOTH_PRINTLN("  ADJ1:<N>    : сдвинуть M1 на N шагов прямо сейчас (+/-)");
  BOTH_PRINTLN("  ADJ2:<N>    : сдвинуть M2 на N шагов прямо сейчас (+/-)");
  BOTH_PRINTLN("  TRIM1:<N>   : постоянная поправка M1 шагов (+/-), сохр. NVS");
  BOTH_PRINTLN("  TRIM2:<N>   : постоянная поправка M2 шагов (+/-), сохр. NVS");
  BOTH_PRINTLN("-- Параметры движения --");
  BOTH_PRINTLN("  SPD:<N>     : макс. скорость шаг/с (100..10000)");
  BOTH_PRINTLN("  ACC:<N>     : ускорение шаг/с^2 (50..5000)");
  BOTH_PRINTLN("-- Прочее --");
  BOTH_PRINTLN("  ANG         : угол AS5600 (0..4095)");
  BOTH_PRINTLN("  POS         : текущие позиции моторов в шагах");
  BOTH_PRINTLN("  i / help    : эта справка");
  BOTH_PRINTLN("  RESET       : заводские настройки");
  BOTH_PRINTLN("----- Текущие настройки -----");
  BOTH_PRINTF ("  M1 steps : %ld  (trim: %+ld)\n", moveStepsM1, trimM1);
  BOTH_PRINTF ("  M2 steps : %ld  (trim: %+ld)\n", moveStepsM2, trimM2);
  BOTH_PRINTF ("  MaxSpeed : %.0f шаг/с\n",  maxSpeed);
  BOTH_PRINTF ("  Accel    : %.0f шаг/с^2\n", acceleration);
  BOTH_PRINTF ("  AS5600   : %u\n",            as5600ReadAngle());
  BOTH_PRINTF ("  Position : %s\n",
    sysPosition == POS_DOWN ? "DOWN" :
    sysPosition == POS_UP   ? "UP"   : "UNKNOWN");
  BOTH_PRINTF ("  State    : %s\n",
    sysState == STATE_IDLE       ? "IDLE"       :
    sysState == STATE_GOING_DOWN ? "GOING_DOWN" :
    sysState == STATE_GOING_UP   ? "GOING_UP"   :
    sysState == STATE_INIT       ? "INIT"        :
    sysState == STATE_ADJ        ? "ADJUSTING"   : "ERROR");
  BOTH_PRINTF ("  MAC      : %s\n", WiFi.macAddress().c_str());
  BOTH_PRINTLN("==============================");
}

// =====================
// Хоминг (оба параллельно, один концевик)
// =====================
void startHoming() {
  hardStopAll();
  homingPhase = HOMING_IDLE;
  m1Extra = m2Extra = false;
  sysState    = STATE_INIT;
  sysPosition = POS_UNKNOWN;
  BOTH_PRINTLN("HOMING: started");
}

void runInit() {
  switch (homingPhase) {
    case HOMING_IDLE:
      if (limitTriggered()) {
        // уже на концевике — оба отъезжают
        motor1.move(2000L * UP_DIR);
        motor2.move(2000L * UP_DIR);
        homingPhase = HOMING_BACKOFF;
      } else {
        motor1.move((moveStepsM1 + EXTRA_STEPS * 10) * DOWN_DIR);
        motor2.move((moveStepsM2 + EXTRA_STEPS * 10) * DOWN_DIR);
        homingPhase = HOMING_SEEKING;
      }
      break;

    case HOMING_BACKOFF:
      // ждём пока оба отъедут
      if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
        motor1.move((moveStepsM1 + EXTRA_STEPS * 10) * DOWN_DIR);
        motor2.move((moveStepsM2 + EXTRA_STEPS * 10) * DOWN_DIR);
        homingPhase = HOMING_SEEKING;
      }
      break;

    case HOMING_SEEKING:
      if (limitTriggered()) {
        hardStopAll();
        motor1.setCurrentPosition(0);
        motor2.setCurrentPosition(0);
        sysPosition = POS_DOWN;
        sysState    = STATE_IDLE;
        homingPhase = HOMING_IDLE;
        BOTH_PRINTLN("HOMING_DONE");
        return;
      }
      // Добавляем экстра-шаги если один из моторов заканчивает раньше
      if (motor1.distanceToGo() == 0 && !m1Extra) { motor1.move(EXTRA_STEPS * DOWN_DIR); m1Extra = true; }
      if (motor2.distanceToGo() == 0 && !m2Extra) { motor2.move(EXTRA_STEPS * DOWN_DIR); m2Extra = true; }
      // Оба закончили — концевик так и не сработал
      if (motor1.distanceToGo() == 0 && m1Extra &&
          motor2.distanceToGo() == 0 && m2Extra) {
        sysState    = STATE_ERROR;
        homingPhase = HOMING_IDLE;
        BOTH_PRINTLN("ERROR: концевик не сработал при хоминге!");
      }
      break;
  }
}

// =====================
// Движение вниз (оба параллельно)
// =====================
void startGoDown() {
  m1Extra = m2Extra = false;
  sysState = STATE_GOING_DOWN;
  motor1.move(stepsM1(DOWN_DIR));
  motor2.move(stepsM2(DOWN_DIR));
  BOTH_PRINTLN("CMD: DOWN");
}

void runGoDown() {
  bool m1Done = (motor1.distanceToGo() == 0);
  bool m2Done = (motor2.distanceToGo() == 0);

  if (limitTriggered()) {
    hardStopAll();
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    sysPosition = POS_DOWN;
    sysState    = STATE_IDLE;
    BOTH_PRINTLN("DOWN_DONE (limit)");
    return;
  }
  // Экстра-шаги если концевик ещё не сработал
  if (m1Done && !m1Extra) { motor1.move(EXTRA_STEPS * DOWN_DIR); m1Extra = true; }
  if (m2Done && !m2Extra) { motor2.move(EXTRA_STEPS * DOWN_DIR); m2Extra = true; }

  if (motor1.distanceToGo() == 0 && m1Extra &&
      motor2.distanceToGo() == 0 && m2Extra) {
    sysState = STATE_ERROR;
    BOTH_PRINTLN("ERROR: концевик не сработал при опускании!");
  }
}

// =====================
// Движение вверх (оба параллельно)
// =====================
void startGoUp() {
  sysState = STATE_GOING_UP;
  motor1.move(stepsM1(UP_DIR));
  motor2.move(stepsM2(UP_DIR));
  BOTH_PRINTLN("CMD: UP");
}

void runGoUp() {
  if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
    sysPosition = POS_UP;
    sysState    = STATE_IDLE;
    BOTH_PRINTLN("UP_DONE");
  }
}

// =====================
// Подстройка (ADJ) — ручное смещение одного мотора
// =====================
void runAdj() {
  if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
    sysState = STATE_IDLE;
    BOTH_PRINTF("ADJ done. M1 pos=%ld  M2 pos=%ld\n",
                motor1.currentPosition(), motor2.currentPosition());
  }
}

// =====================
// Команды (USB Serial + BT)
// =====================
String btBuffer  = "";
String usbBuffer = "";

void processLine(const String &line) {
  if (line.equalsIgnoreCase("i") || line.equalsIgnoreCase("help")) {
    printHelp(); return;
  }
  if (line.equalsIgnoreCase("RESET")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    resetSettings(); return;
  }
  if (line.equalsIgnoreCase("ANG")) {
    BOTH_PRINTF("AS5600: %u\n", as5600ReadAngle()); return;
  }
  if (line.equalsIgnoreCase("POS")) {
    BOTH_PRINTF("M1 pos=%ld  M2 pos=%ld\n",
                motor1.currentPosition(), motor2.currentPosition()); return;
  }
  if (line.equalsIgnoreCase("HOME")) {
    if (sysState == STATE_GOING_DOWN || sysState == STATE_GOING_UP) { BOTH_PRINTLN("BUSY"); return; }
    startHoming(); return;
  }
  if (line == "0") {
    hardStopAll();
    sysState = STATE_IDLE; sysPosition = POS_UNKNOWN;
    BOTH_PRINTLN("EMERGENCY_STOP"); return;
  }
  if (line == "1") {
    if (sysState == STATE_ERROR) { BOTH_PRINTLN("ERROR_STATE: send 0 to reset"); return; }
    if (sysState != STATE_IDLE)  { BOTH_PRINTLN("BUSY"); return; }
    if (sysPosition == POS_DOWN) { BOTH_PRINTLN("ALREADY_DOWN"); return; }
    startGoDown(); return;
  }
  if (line == "2") {
    if (sysState == STATE_ERROR) { BOTH_PRINTLN("ERROR_STATE: send 0 to reset"); return; }
    if (sysState != STATE_IDLE)  { BOTH_PRINTLN("BUSY"); return; }
    if (sysPosition == POS_UP)   { BOTH_PRINTLN("ALREADY_UP"); return; }
    startGoUp(); return;
  }

  // --- Шаги ---
  if (line.startsWith("M1:") || line.startsWith("m1:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(3).toInt();
    if (val < 100 || val > 500000) { BOTH_PRINTLN("ERROR: M1 out of range (100..500000)"); return; }
    moveStepsM1 = val; saveSettings();
    BOTH_PRINTF("OK: M1 steps = %ld\n", moveStepsM1); return;
  }
  if (line.startsWith("M2:") || line.startsWith("m2:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(3).toInt();
    if (val < 100 || val > 500000) { BOTH_PRINTLN("ERROR: M2 out of range (100..500000)"); return; }
    moveStepsM2 = val; saveSettings();
    BOTH_PRINTF("OK: M2 steps = %ld\n", moveStepsM2); return;
  }

  // --- Подстройка: немедленный сдвиг ---
  if (line.startsWith("ADJ1:") || line.startsWith("adj1:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(5).toInt();
    if (val == 0) { BOTH_PRINTLN("ERROR: ADJ1 = 0"); return; }
    motor1.move(val);
    sysState = STATE_ADJ;
    BOTH_PRINTF("ADJ1: moving M1 by %+ld steps\n", val); return;
  }
  if (line.startsWith("ADJ2:") || line.startsWith("adj2:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(5).toInt();
    if (val == 0) { BOTH_PRINTLN("ERROR: ADJ2 = 0"); return; }
    motor2.move(val);
    sysState = STATE_ADJ;
    BOTH_PRINTF("ADJ2: moving M2 by %+ld steps\n", val); return;
  }

  // --- Подстройка: постоянная поправка ---
  if (line.startsWith("TRIM1:") || line.startsWith("trim1:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(6).toInt();
    if (val < -50000 || val > 50000) { BOTH_PRINTLN("ERROR: TRIM1 out of range (-50000..50000)"); return; }
    trimM1 = val; saveSettings();
    BOTH_PRINTF("OK: TRIM1 = %+ld steps\n", trimM1); return;
  }
  if (line.startsWith("TRIM2:") || line.startsWith("trim2:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(6).toInt();
    if (val < -50000 || val > 50000) { BOTH_PRINTLN("ERROR: TRIM2 out of range (-50000..50000)"); return; }
    trimM2 = val; saveSettings();
    BOTH_PRINTF("OK: TRIM2 = %+ld steps\n", trimM2); return;
  }

  // --- Скорость / ускорение ---
  if (line.startsWith("SPD:") || line.startsWith("spd:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(4).toInt();
    if (val < 100 || val > 10000) { BOTH_PRINTLN("ERROR: SPD out of range (100..10000)"); return; }
    maxSpeed = (float)val; applyMotorSettings(); saveSettings();
    BOTH_PRINTF("OK: maxSpeed = %.0f steps/s\n", maxSpeed); return;
  }
  if (line.startsWith("ACC:") || line.startsWith("acc:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(4).toInt();
    if (val < 50 || val > 5000) { BOTH_PRINTLN("ERROR: ACC out of range (50..5000)"); return; }
    acceleration = (float)val; applyMotorSettings(); saveSettings();
    BOTH_PRINTF("OK: accel = %.0f steps/s^2\n", acceleration); return;
  }

  BOTH_PRINTF("UNKNOWN: %s\n", line.c_str());
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      usbBuffer.trim();
      if (usbBuffer.length() > 0) { String s = usbBuffer; usbBuffer = ""; processLine(s); }
    } else {
      usbBuffer += c;
      if (usbBuffer.length() > 64) usbBuffer = "";
    }
  }
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) { String s = btBuffer; btBuffer = ""; processLine(s); }
    } else {
      btBuffer += c;
      if (btBuffer.length() > 64) btBuffer = "";
    }
  }
}

// =====================
// ESP-NOW команды
// =====================
void handleEspNow() {
  if (!espNowCmdPending) return;
  espNowCmdPending = false;
  uint8_t cmd = espNowCmdVal;
  Serial.printf("ESP-NOW cmd: %d\n", cmd);

  if (cmd == 0) {
    hardStopAll();
    sysState = STATE_IDLE; sysPosition = POS_UNKNOWN;
    BOTH_PRINTLN("ESP-NOW: STOP"); return;
  }
  if (cmd == 1) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("ESP-NOW: BUSY"); return; }
    if (sysPosition == POS_DOWN) { BOTH_PRINTLN("ESP-NOW: ALREADY_DOWN"); return; }
    startGoDown(); return;
  }
  if (cmd == 2) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("ESP-NOW: BUSY"); return; }
    if (sysPosition == POS_UP)  { BOTH_PRINTLN("ESP-NOW: ALREADY_UP"); return; }
    startGoUp(); return;
  }
  Serial.printf("ESP-NOW: unknown cmd %d\n", cmd);
}

// =====================
// SETUP / LOOP
// =====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("Slave Motor 2-2");

  pinMode(LIMIT_PIN,   INPUT_PULLUP);
  pinMode(M1_STEP_PIN, OUTPUT); pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT); pinMode(M2_DIR_PIN, OUTPUT);

  initAS5600();

  Serial2.begin(115200, SERIAL_8N1, TMC1_RX, TMC1_TX);
  driver1.begin(); driver1.toff(5); driver1.rms_current(DEFAULT_RMS_MA);
  driver1.microsteps(16); driver1.en_spreadCycle(false); driver1.pwm_autoscale(true);
  Serial.printf("TMC driver1 IOIN: 0x%08X\n", driver1.IOIN());

  Serial1.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);
  driver2.begin(); driver2.toff(5); driver2.rms_current(DEFAULT_RMS_MA);
  driver2.microsteps(16); driver2.en_spreadCycle(false); driver2.pwm_autoscale(true);
  Serial.printf("TMC driver2 IOIN: 0x%08X\n", driver2.IOIN());

  loadSettings();
  applyMotorSettings();
  initEspNow();

  startHoming();
  Serial.println("BOOT: homing... (send 'i' for help)");
}

void loop() {
  motor1.run();
  motor2.run();

  handleSerial();
  handleEspNow();

  switch (sysState) {
    case STATE_INIT:       runInit();   break;
    case STATE_GOING_DOWN: runGoDown(); break;
    case STATE_GOING_UP:   runGoUp();   break;
    case STATE_ADJ:        runAdj();    break;
    case STATE_IDLE:                    break;
    case STATE_ERROR:
      if (millis() - errorTimer > 3000) {
        BOTH_PRINTLN("ERROR: halted. Send 0 to reset.");
        errorTimer = millis();
      }
      break;
  }
}

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
#define STEP_PIN      4
#define DIR_PIN       2
#define LIMIT_PIN     15   // INPUT_PULLUP; HIGH = концевик сработал (NC-выключатель)

// AS5600 I2C
#define AS5600_SDA    21
#define AS5600_SCL    22
#define AS5600_DIR_PIN 23  // DIR-пин AS5600 (задаёт направление счёта)

// TMC UART: RX2=16, TX2=17 (TX через резистор ~1кОм к RX контроллера)
#define TMC_RX        16
#define TMC_TX        17
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
long  moveSteps   = DEFAULT_STEPS;
float maxSpeed    = DEFAULT_MAX_SPEED;
float acceleration = DEFAULT_ACCEL;

// =====================
// Объекты
// =====================
AccelStepper  motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDR);

// =====================
// Конечный автомат
// =====================
enum SystemState { STATE_INIT, STATE_IDLE, STATE_GOING_DOWN, STATE_GOING_UP, STATE_ERROR };
enum Position    { POS_UNKNOWN = 0, POS_DOWN = 1, POS_UP = 2 };

SystemState sysState    = STATE_INIT;
Position    sysPosition = POS_UNKNOWN;

enum HomingPhase { HOMING_IDLE, HOMING_BACKOFF, HOMING_SEEKING };
HomingPhase homingPhase = HOMING_IDLE;
bool motorDone  = false;
bool motorExtra = false;
unsigned long errorTimer = 0;

#define DOWN_DIR  1
#define UP_DIR   -1

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
  Wire.write(0x0E);  // угол (фильтрованный), старший байт
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom((uint8_t)0x36, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

void initAS5600() {
  pinMode(AS5600_DIR_PIN, OUTPUT);
  digitalWrite(AS5600_DIR_PIN, LOW);  // CW — при необходимости поменять на HIGH
  Wire.begin(AS5600_SDA, AS5600_SCL);
  uint16_t angle = as5600ReadAngle();
  if (angle == 0xFFFF)
    Serial.println("AS5600: not found!");
  else
    Serial.printf("AS5600 init OK, angle: %u\n", angle);
}

// =====================
// NVS
// =====================
void loadSettings() {
  prefs.begin("slave", true);
  moveSteps    = prefs.getLong  ("steps",    DEFAULT_STEPS);
  maxSpeed     = prefs.getFloat ("maxSpeed", DEFAULT_MAX_SPEED);
  acceleration = prefs.getFloat ("accel",    DEFAULT_ACCEL);
  prefs.end();
  Serial.printf("NVS: steps=%ld spd=%.0f acc=%.0f\n", moveSteps, maxSpeed, acceleration);
}

void saveSettings() {
  prefs.begin("slave", false);
  prefs.putLong  ("steps",    moveSteps);
  prefs.putFloat ("maxSpeed", maxSpeed);
  prefs.putFloat ("accel",    acceleration);
  prefs.end();
  Serial.println("NVS saved.");
  SerialBT.println("NVS saved.");
}

void applyMotorSettings() {
  motor.setMaxSpeed(maxSpeed);
  motor.setAcceleration(acceleration);
}

void resetSettings() {
  moveSteps    = DEFAULT_STEPS;
  maxSpeed     = DEFAULT_MAX_SPEED;
  acceleration = DEFAULT_ACCEL;
  saveSettings();
  applyMotorSettings();
  Serial.println("Settings reset.");
  SerialBT.println("Settings reset.");
}

// =====================
// Утилиты
// =====================
bool limitTriggered() { return digitalRead(LIMIT_PIN) == HIGH; }

void hardStop() { motor.setCurrentPosition(motor.currentPosition()); }

void moveMotor(long steps, int dir) { motor.move((long)steps * dir); }

// =====================
// Help / статус
// =====================
// Вывод в оба интерфейса одновременно
#define BOTH_PRINTLN(s)      do { Serial.println(s);      SerialBT.println(s);      } while(0)
#define BOTH_PRINTF(fmt,...) do { Serial.printf(fmt,##__VA_ARGS__); SerialBT.printf(fmt,##__VA_ARGS__); } while(0)

void printHelp() {
  BOTH_PRINTLN("===== SLAVE CONTROLLER =====");
  BOTH_PRINTLN("  1          : go DOWN (к концевику)");
  BOTH_PRINTLN("  2          : go UP");
  BOTH_PRINTLN("  0          : EMERGENCY STOP");
  BOTH_PRINTLN("  HOME       : запустить хоминг");
  BOTH_PRINTLN("  STEPS:<N>  : шаги движения (100..500000)");
  BOTH_PRINTLN("  SPD:<N>    : макс. скорость шаг/с (100..10000)");
  BOTH_PRINTLN("  ACC:<N>    : ускорение шаг/с^2 (50..5000)");
  BOTH_PRINTLN("  ANG        : прочитать угол AS5600 (0..4095)");
  BOTH_PRINTLN("  i / help   : эта справка");
  BOTH_PRINTLN("  RESET      : заводские настройки");
  BOTH_PRINTLN("----- Текущие настройки -----");
  BOTH_PRINTF ("  Steps    : %ld\n",        moveSteps);
  BOTH_PRINTF ("  MaxSpeed : %.0f шаг/с\n", maxSpeed);
  BOTH_PRINTF ("  Accel    : %.0f шаг/с^2\n", acceleration);
  BOTH_PRINTF ("  AS5600   : %u\n",          as5600ReadAngle());
  BOTH_PRINTF ("  Position : %s\n",
    sysPosition == POS_DOWN ? "DOWN" :
    sysPosition == POS_UP   ? "UP"   : "UNKNOWN");
  BOTH_PRINTF ("  State    : %s\n",
    sysState == STATE_IDLE       ? "IDLE"       :
    sysState == STATE_GOING_DOWN ? "GOING_DOWN" :
    sysState == STATE_GOING_UP   ? "GOING_UP"   :
    sysState == STATE_INIT       ? "INIT"        : "ERROR");
  BOTH_PRINTF ("  MAC      : %s\n", WiFi.macAddress().c_str());
  BOTH_PRINTLN("============================");
}

// =====================
// Хоминг
// =====================
void startHoming() {
  hardStop();
  homingPhase = HOMING_IDLE;
  motorExtra  = false;
  motorDone   = false;
  sysState    = STATE_INIT;
  sysPosition = POS_UNKNOWN;
  Serial.println("HOMING: started");
  SerialBT.println("HOMING: started");
}

void runInit() {
  switch (homingPhase) {
    case HOMING_IDLE:
      if (limitTriggered()) {
        // уже на концевике — сначала отъехать
        moveMotor(2000, UP_DIR);
        homingPhase = HOMING_BACKOFF;
      } else {
        moveMotor(moveSteps + EXTRA_STEPS * 10, DOWN_DIR);
        homingPhase = HOMING_SEEKING;
      }
      break;

    case HOMING_BACKOFF:
      // ждём, пока отъедем от концевика
      if (motor.distanceToGo() == 0) {
        moveMotor(moveSteps + EXTRA_STEPS * 10, DOWN_DIR);
        homingPhase = HOMING_SEEKING;
      }
      break;

    case HOMING_SEEKING:
      if (limitTriggered()) {
        hardStop();
        motor.setCurrentPosition(0);
        sysPosition = POS_DOWN;
        sysState    = STATE_IDLE;
        homingPhase = HOMING_IDLE;
        Serial.println("HOMING_DONE");
        SerialBT.println("HOMING_DONE");
      } else if (motor.distanceToGo() == 0 && !motorExtra) {
        moveMotor(EXTRA_STEPS, DOWN_DIR);
        motorExtra = true;
      } else if (motor.distanceToGo() == 0 && motorExtra) {
        sysState    = STATE_ERROR;
        homingPhase = HOMING_IDLE;
        Serial.println("ERROR: концевик не сработал при хоминге!");
        SerialBT.println("ERROR: limit not triggered during homing!");
      }
      break;
  }
}

// =====================
// Движение вниз (к концевику)
// =====================
void startGoDown() {
  motorDone  = false;
  motorExtra = false;
  sysState   = STATE_GOING_DOWN;
  moveMotor(moveSteps, DOWN_DIR);
  Serial.println("CMD: DOWN");
  SerialBT.println("CMD: DOWN");
}

void runGoDown() {
  if (motorDone) return;

  if (limitTriggered()) {
    hardStop();
    motor.setCurrentPosition(0);
    motorDone   = true;
    sysPosition = POS_DOWN;
    sysState    = STATE_IDLE;
    Serial.println("DOWN_DONE");
    SerialBT.println("DOWN_DONE");
    return;
  }

  if (motor.distanceToGo() == 0 && !motorExtra) {
    moveMotor(EXTRA_STEPS, DOWN_DIR);
    motorExtra = true;
  } else if (motor.distanceToGo() == 0 && motorExtra) {
    motorDone = true;
    sysState  = STATE_ERROR;
    Serial.println("ERROR: концевик не сработал при опускании!");
    SerialBT.println("ERROR: limit not triggered going down!");
  }
}

// =====================
// Движение вверх
// =====================
void startGoUp() {
  motorDone  = false;
  motorExtra = false;
  sysState   = STATE_GOING_UP;
  moveMotor(moveSteps, UP_DIR);
  Serial.println("CMD: UP");
  SerialBT.println("CMD: UP");
}

void runGoUp() {
  if (motorDone) return;
  if (motor.distanceToGo() == 0) {
    motorDone   = true;
    sysPosition = POS_UP;
    sysState    = STATE_IDLE;
    Serial.println("UP_DONE");
    SerialBT.println("UP_DONE");
  }
}

// =====================
// Команды (Serial + BT)
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
  if (line.equalsIgnoreCase("HOME")) {
    if (sysState == STATE_GOING_DOWN || sysState == STATE_GOING_UP) {
      BOTH_PRINTLN("BUSY"); return;
    }
    startHoming(); return;
  }
  if (line == "0") {
    hardStop();
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
  if (line.startsWith("STEPS:") || line.startsWith("steps:")) {
    if (sysState != STATE_IDLE) { BOTH_PRINTLN("BUSY"); return; }
    long val = line.substring(6).toInt();
    if (val < 100 || val > 500000) { BOTH_PRINTLN("ERROR: STEPS out of range (100..500000)"); return; }
    moveSteps = val; saveSettings();
    BOTH_PRINTF("OK: steps = %ld\n", moveSteps); return;
  }
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
  // USB Serial Monitor
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
  // Bluetooth
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
// ESP-NOW-команды
// =====================
void handleEspNow() {
  if (!espNowCmdPending) return;
  espNowCmdPending = false;
  uint8_t cmd = espNowCmdVal;
  Serial.printf("ESP-NOW cmd: %d\n", cmd);

  if (cmd == 0) {
    hardStop();
    sysState = STATE_IDLE; sysPosition = POS_UNKNOWN;
    Serial.println("ESP-NOW: STOP"); return;
  }
  if (cmd == 1) {
    if (sysState != STATE_IDLE) { Serial.println("ESP-NOW: BUSY"); return; }
    if (sysPosition == POS_DOWN) { Serial.println("ESP-NOW: ALREADY_DOWN"); return; }
    startGoDown(); return;
  }
  if (cmd == 2) {
    if (sysState != STATE_IDLE) { Serial.println("ESP-NOW: BUSY"); return; }
    if (sysPosition == POS_UP)  { Serial.println("ESP-NOW: ALREADY_UP"); return; }
    startGoUp(); return;
  }
  Serial.printf("ESP-NOW: unknown cmd %d\n", cmd);
}

// =====================
// SETUP / LOOP
// =====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("Slave Motor 1-1");
  // 1-1 1-2 для разных моторов

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN,  OUTPUT);
  pinMode(DIR_PIN,   OUTPUT);

  initAS5600();

  // TMC2209 UART: RX2=16, TX2=17 (TX2 через ~1кОм резистор к TMC UART)
  Serial2.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
  driver.begin();
  driver.toff(5);
  driver.rms_current(DEFAULT_RMS_MA);
  driver.microsteps(16);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  Serial.printf("TMC2209 ready. IOIN: 0x%08X\n", driver.IOIN());

  loadSettings();
  applyMotorSettings();
  initEspNow();

  startHoming();

  Serial.println("BOOT: homing...");
  SerialBT.println("BOOT: homing... (send 'i' for help)");
}

void loop() {
  motor.run();

  handleSerial();
  handleEspNow();

  switch (sysState) {
    case STATE_INIT:       runInit();   break;
    case STATE_GOING_DOWN: runGoDown(); break;
    case STATE_GOING_UP:   runGoUp();   break;
    case STATE_IDLE:                    break;
    case STATE_ERROR:
      if (millis() - errorTimer > 3000) {
        Serial.println("ERROR: halted. Send 0 to reset.");
        SerialBT.println("ERROR: halted. Send 0 to reset.");
        errorTimer = millis();
      }
      break;
  }
}

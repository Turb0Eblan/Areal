/*
 * ============================================================
 *  Master Controller — ESP32
 *  Управление слейвами через ESP-NOW.
 *  Команды через USB Serial и Bluetooth.
 *
 *  Motor slaves:
 *    m1-1  FC:E8:C0:DF:33:50   motor1  (2-мотор слейв)
 *    m1-2  F8:B3:B7:74:A9:B8   motor1  (2-мотор слейв)
 *    m2-1  FC:E8:C0:E1:5D:64   motor2  (2-мотор слейв)
 *    m2-2  FC:E8:C0:DF:39:F0   motor2  (2-мотор слейв)
 *    m3-1  [placeholder]
 *    m3-2  [placeholder]
 *    m4    FC:E8:C0:DF:3F:B4   motor4  (5-мотор слейв)
 *      Моторы m4 (M1=верхний, M5=нижний):
 *        M1: dir=4  step=2  (верхний)   TMC2209  lim=5
 *        M2: dir=15 step=21             TMC2209  lim=33
 *        M3: dir=25 step=26             TB6600   lim=32
 *        M4: dir=27 step=14             TB6600   lim=22
 *        M5: dir=12 step=13 (нижний)   TB6600   lim=23
 *      Концевики NC (HIGH=сработал). Автохоминг при включении (M5→M1).
 *      Режимы DOWN/UP: SIM=одновременно (умолч.), SEQ=по очереди.
 *        SEQ DOWN: M5→M4→M3→M2→M1    SEQ UP: M1→M2→M3→M4 (M5 стоит)
 *
 *  LED slaves (NeoPixelBus, 16 блоков по 300 LED, SK6812 RGBW):
 *    red      20:50:0D:D1:32:70
 *    lights   [placeholder]
 *
 *  Motor command examples:
 *    m1-1 down          — опустить один слейв
 *    motor1 up          — поднять группу (m1-1 + m1-2)
 *    motors stop        — остановить все моторы
 *    motors down        — опустить все моторы
 *    m4 up              — поднять M1..M4 (M5 стоит)
 *    m4 down            — опустить все 5 до концевиков
 *    m4 mode:sim        — переключить m4 в одновременный режим
 *    m4 mode:seq        — переключить m4 в последовательный режим
 *    m4 M1 up           — поднять только M1 на слейве m4
 *    m4 M3 down         — опустить только M3 на слейве m4
 *    m4 M2 stop         — остановить M2 на слейве m4
 *    lube motors 3 5000 — режим смазки: 3 цикла вниз/вверх, пауза 5с на движение
 *
 *  ESP-NOW MotorCmd protocol:
 *    { cmd, motorIdx }
 *    cmd:      0=STOP  1=DOWN  2=UP  3=MODE_SEQ  4=MODE_SIM
 *    motorIdx: 0=все моторы,  1..5=один мотор (только m4 поддерживает)
 *
 *  LED command examples (протокол блочного слейва NeoPixelBus):
 *    red pulse                — белый пульс на всех 16 блоках слейва red
 *    red block 3 fire         — блок №3 слейва red в режим "огонь"
 *    red block all off        — все блоки слейва red выключить
 *    red (0,0,0,255)          — залить все блоки red сплошным белым
 *    red cfade                — плавный переход по палитре (все блоки)
 *    off                      — выключить все LED-слейвы
 *    pulse                    — белый пульс на всех LED-слейвах
 *    block 5 rainbow          — блок №5 на ВСЕХ LED-слейвах — радуга
 *  Режимы: off|allOn|fade|snake|rain|random|rainbow|fire|pulse|cfade|auto
 *  Цвет:   (R,G,B,W), например (0,0,0,255)=белый (255,0,0,0)=красный
 * ============================================================
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ─── Вывод в оба интерфейса ───────────────────────────────────
#define BOTH_PRINTLN(s)      do { Serial.println(s);                SerialBT.println(s);                } while(0)
#define BOTH_PRINTF(fmt,...) do { Serial.printf(fmt,##__VA_ARGS__); SerialBT.printf(fmt,##__VA_ARGS__); } while(0)

// ─── Пакет ESP-NOW для моторных слейвов ──────────────────────
// cmd:      0=STOP  1=DOWN  2=UP  3=MODE_SEQ  4=MODE_SIM
// motorIdx: 0=все моторы,  1..5=один мотор (только m4 / 5-мотор слейв)
typedef struct { uint8_t cmd; uint8_t motorIdx; } MotorCmd;

// ─── Send callback + retry механизм ──────────────────────────
static volatile bool   s_sendDone   = false;
static volatile bool   s_sendOk     = false;

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    s_sendOk   = (status == ESP_NOW_SEND_SUCCESS);
    s_sendDone = true;
}

// Отправить с retry. Возвращает true если доставлено.
static bool espNowSendWithRetry(const uint8_t* mac, const uint8_t* data, size_t len,
                                int retries = 3, uint32_t retryDelayMs = 50) {
    for (int attempt = 0; attempt < retries; attempt++) {
        s_sendDone = false;
        s_sendOk   = false;
        if (esp_now_send(mac, data, len) != ESP_OK) {
            delay(retryDelayMs);
            continue;
        }
        // Ждём callback (max 200ms)
        uint32_t t = millis();
        while (!s_sendDone && (millis() - t) < 200) { delay(1); }
        if (s_sendOk) return true;
        if (attempt < retries - 1) delay(retryDelayMs);
    }
    return false;
}

// Одна "сырая" отправка без внутреннего retry — используется в sendMotorCmd()
// как один "выстрел" при бомбардировке для STOP/DOWN/UP (см. MOTOR_MOVE_FLOOD_MS).
// waitMs должен быть с запасом: при активных BT+WiFi (coexistence) callback
// send-статуса иногда приходит с задержкой. Если ждать мало и при этом слать
// следующий пакет почти сразу — можно затереть s_sendDone/s_sendOk раньше,
// чем придёт запоздавший callback от ПРЕДЫДУЩЕГО пакета: пакет реально
// доставлен и слейв его выполнил, а мастер всё равно посчитает это FAIL.
// Наблюдалось живьём: механизм ехал, а лог писал "FAIL" на всех попытках.
static bool espNowSendOnce(const uint8_t* mac, const uint8_t* data, size_t len, uint32_t waitMs = 400) {
    s_sendDone = false;
    s_sendOk   = false;
    if (esp_now_send(mac, data, len) != ESP_OK) return false;
    uint32_t t = millis();
    while (!s_sendDone && (millis() - t) < waitMs) { delay(1); }
    return s_sendOk;
}

// ─── Бомбардировка для команд движения (STOP/DOWN/UP) ────────────
// Слейвы игнорируют DOWN/UP пока не в IDLE и не шлют NACK — поэтому мастер
// не может узнать, принял ли слейв команду, только что пакет доставлен по радио.
// Чтобы пережить потерю пакетов, шлём команду раз за разом до
// MOTOR_MOVE_FLOOD_MS, пока не придёт ACK о доставке или не выйдет время.
// Повтор DOWN/UP, когда слейв уже в этом состоянии, — no-op на его стороне,
// так что бомбардировка безопасна и не вызывает лишних движений.
// Пауза между "выстрелами" намеренно не короткая — см. комментарий у
// espNowSendOnce: слишком частая отправка ломает подсчёт ACK гонкой.
#define MOTOR_MOVE_FLOOD_MS      10000  // макс. длительность бомбардировки, мс
#define MOTOR_MOVE_REPEAT_GAP_MS 200    // пауза между "выстрелами", мс

// ─── Моторные слейвы ─────────────────────────────────────────
struct MotorSlave {
    uint8_t    mac[6];
    const char *id;      // имя для команд (m1-1, m2-1, ...)
    const char *group;   // группа (motor1, motor2, ...)
    bool        active;  // false = плейсхолдер, не отправлять
};

MotorSlave motorSlaves[] = {
    //  MAC                                    id       group     active
    {{0xFC,0xE8,0xC0,0xDF,0x33,0x50},  "m1-1",  "motor1",  true },
    {{0xFC,0xE8,0xC0,0xDF,0x3F,0xB4},  "m1-2",  "motor1",  true }, //0xF8,0xB3,0xB7,0x74,0xA9,0xB8
    {{0xFC,0xE8,0xC0,0xE1,0x5D,0x64},  "m2-1",  "motor2",  true },
    {{0xFC,0xE8,0xC0,0xDF,0x39,0xF0},  "m2-2",  "motor2",  true },
    // ── Плейсхолдеры — заполни MAC когда будет готово ──
    {{0x00,0x00,0x00,0x00,0x00,0x00},  "m3-1",  "motor3",  false},
    {{0x00,0x00,0x00,0x00,0x00,0x00},  "m3-2",  "motor3",  false},
    // m4 — 5-мотор слейв (FC:E8:C0:DF:3F:B4)
    {{0xF8,0xB3,0xB7,0x74,0xA9,0xB8},  "m4",    "motor4",  true }, //0xFC,0xE8,0xC0,0xDF,0x3F,0xB4
};
const int MOTOR_COUNT = sizeof(motorSlaves) / sizeof(motorSlaves[0]);

// ─── LED-слейвы ──────────────────────────────────────────────
struct LedDevice {
    uint8_t    mac[6];
    const char *id;
    bool        active;
};

LedDevice ledDevices[] = {
    {{0x20,0x50,0x0D,0xD1,0x32,0x70}, "blue",    true },
    // ── Плейсхолдер ──
    {{0x00,0x00,0x00,0x00,0x00,0x00}, "lights", false},
};
const int LED_COUNT = sizeof(ledDevices) / sizeof(ledDevices[0]);

// ─── Утилиты ─────────────────────────────────────────────────
void printMac(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
}

bool isMacEmpty(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) if (mac[i]) return false;
    return true;
}

void registerPeer(const uint8_t* mac, const char* label) {
    if (isMacEmpty(mac)) {
        Serial.printf("  [~] %-8s  placeholder (MAC not set)\n", label);
        return;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 1;  // должен совпадать с каналом слейвов
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.printf("  [+] %-8s  MAC: ", label);
        printMac(mac); Serial.println();
    } else {
        Serial.printf("  [!] Failed: %s\n", label);
    }
}

// ─── Отправка мотору ─────────────────────────────────────────
// motorIdx=0 → все моторы; motorIdx=1..5 → один мотор (только m4)
void sendMotorCmd(MotorSlave& s, uint8_t cmd, uint8_t motorIdx = 0) {
    if (!s.active || isMacEmpty(s.mac)) {
        BOTH_PRINTF("  [~] %s  skipped (inactive)\n", s.id);
        return;
    }
    MotorCmd pkt = {cmd, motorIdx};
    const char* act = (cmd == 0) ? "STOP" : (cmd == 1) ? "DOWN" : (cmd == 2) ? "UP" :
                      (cmd == 3) ? "MODE:SEQ" : "MODE:SIM";

    // STOP/DOWN/UP — команды движения: бомбардируем слейв пакетом до
    // MOTOR_MOVE_FLOOD_MS, пока не придёт ACK о доставке. Как только слейв
    // ответил (или начал реагировать на предыдущий "выстрел" и перешёл
    // в busy-состояние) — дальше слать бессмысленно, выходим раньше.
    bool     moveCmd  = (cmd == 0 || cmd == 1 || cmd == 2);
    bool     ok       = false;
    int      attempts = 0;
    uint32_t t0       = millis();

    if (moveCmd) {
        while (millis() - t0 < MOTOR_MOVE_FLOOD_MS) {
            attempts++;
            if (espNowSendOnce(s.mac, (uint8_t*)&pkt, sizeof(pkt))) { ok = true; break; }
            delay(MOTOR_MOVE_REPEAT_GAP_MS);
        }
    } else {
        attempts = 1;
        ok = espNowSendWithRetry(s.mac, (uint8_t*)&pkt, sizeof(pkt));
    }

    uint32_t elapsed = millis() - t0;
    if (motorIdx == 0)
        BOTH_PRINTF("  --> %-6s  %s  %s (%d попыт., %lums)\n",
                     s.id, act, ok ? "OK" : "FAIL", attempts, (unsigned long)elapsed);
    else
        BOTH_PRINTF("  --> %-6s  M%d %s  %s (%d попыт., %lums)\n",
                     s.id, motorIdx, act, ok ? "OK" : "FAIL", attempts, (unsigned long)elapsed);
}

// Найти слейв по id
MotorSlave* findMotorById(const String& id) {
    for (int i = 0; i < MOTOR_COUNT; i++)
        if (String(motorSlaves[i].id).equalsIgnoreCase(id)) return &motorSlaves[i];
    return nullptr;
}

// Отправить команду по имени слейва (m1-1, m2-1, m4 ...)
bool sendMotorById(const String& id, uint8_t cmd, uint8_t motorIdx = 0) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (String(motorSlaves[i].id).equalsIgnoreCase(id)) {
            sendMotorCmd(motorSlaves[i], cmd, motorIdx);
            return true;
        }
    }
    return false;
}

// Отправить команду всей группе (motor1, motor2 ...)
bool sendMotorByGroup(const String& group, uint8_t cmd) {
    bool found = false;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (String(motorSlaves[i].group).equalsIgnoreCase(group)) {
            sendMotorCmd(motorSlaves[i], cmd);
            found = true;
        }
    }
    return found;
}

// Отправить всем активным моторам
void sendMotorAll(uint8_t cmd) {
    for (int i = 0; i < MOTOR_COUNT; i++)
        sendMotorCmd(motorSlaves[i], cmd);
}

// ─── Разбор моторной команды ─────────────────────────────────
// Формат: <target> <action>
//   target : m1-1 | m1-2 | m2-1 | m2-2 | motor1 | motor2 | motors
//   action : down | up | stop  (или 1 | 2 | 0)
uint8_t parseMotorAction(const String& act) {
    if (act.equalsIgnoreCase("down") || act == "1") return 1;
    if (act.equalsIgnoreCase("up")   || act == "2") return 2;
    if (act.equalsIgnoreCase("stop") || act == "0") return 0;
    return 0xFF; // неизвестно
}

bool processMotorCommand(const String& cmd) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) return false;
    String target = cmd.substring(0, sp);
    String rest   = cmd.substring(sp + 1);
    rest.trim();

    // ── Команды переключения режима (MODE:SIM / MODE:SEQ) ──────────
    if (rest.equalsIgnoreCase("mode:sim") || rest.equalsIgnoreCase("mode:seq")) {
        uint8_t modeCmd = rest.equalsIgnoreCase("mode:sim") ? 4 : 3;
        BOTH_PRINTF("\n>>> [MOTOR] %s → %s\n", target.c_str(), rest.c_str());
        if (target.equalsIgnoreCase("motors") || target.equalsIgnoreCase("all-motors")) {
            sendMotorAll(modeCmd); return true;
        }
        { String t = target; t.toLowerCase(); if (t.startsWith("motor")) { if (sendMotorByGroup(target, modeCmd)) return true; } }
        if (sendMotorById(target, modeCmd)) return true;
        return false;
    }

    // ── Команда для одного мотора слейва m4: "m4 MN up/down/stop" ──
    // Формат: m4 M<1..5> <action>
    if (target.equalsIgnoreCase("m4") &&
        rest.length() >= 4 &&
        (rest[0] == 'M' || rest[0] == 'm') &&
        isDigit(rest[1]) && rest[2] == ' ')
    {
        int mIdx = rest[1] - '0'; // 1..5
        if (mIdx >= 1 && mIdx <= 5) {
            String actStr = rest.substring(3);
            actStr.trim();
            uint8_t action = parseMotorAction(actStr);
            if (action != 0xFF) {
                BOTH_PRINTF("\n>>> [M4 индивид.] M%d → %s\n", mIdx, actStr.c_str());
                sendMotorById("m4", action, (uint8_t)mIdx);
                return true;
            }
        }
    }

    uint8_t action = parseMotorAction(rest);
    if (action == 0xFF) return false;

    BOTH_PRINTF("\n>>> [MOTOR] %s → %s\n", target.c_str(), rest.c_str());

    // "motors" = все
    if (target.equalsIgnoreCase("motors") || target.equalsIgnoreCase("all-motors")) {
        sendMotorAll(action); return true;
    }
    // Группа (motor1, motor2, motor3 ...) — проверяем без toLowerCase()
    {
        String t = target;
        t.toLowerCase();
        if (t.startsWith("motor")) {
            if (sendMotorByGroup(target, action)) return true;
        }
    }
    // Конкретный слейв (m1-1, m2-1, m4 ...)
    if (sendMotorById(target, action)) return true;

    return false;
}

// ─── Режим смазки (lube) ──────────────────────────────────────
// Циклически опускает и поднимает механизм, чтобы прогнать смазку.
// Формат: lube <target> [cycles] [waitMs]
//   target : m1-1 | m1-2 | m2-1 | m2-2 | m4 | motor1..4 | motors
//   cycles : число циклов вниз-вверх (по умолч. 3)
//   waitMs : пауза на каждое движение, чтобы механизм успел доехать (по умолч. 5000 мс)
// Во время выполнения можно ввести "stop" (Serial/BT), чтобы прервать цикл.
bool lubeSendAction(const String& target, uint8_t action) {
    if (target.equalsIgnoreCase("motors") || target.equalsIgnoreCase("all-motors")) {
        sendMotorAll(action); return true;
    }
    String t = target; t.toLowerCase();
    if (t.startsWith("motor")) {
        if (sendMotorByGroup(target, action)) return true;
    }
    return sendMotorById(target, action);
}

// Ждёт ms, попутно слушая Serial/BT на предмет "stop". true = прервано.
bool lubeWaitOrAbort(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (Serial.available()) {
            String s = Serial.readStringUntil('\n'); s.trim();
            if (s.equalsIgnoreCase("stop")) return true;
        }
        if (SerialBT.available()) {
            String s = SerialBT.readStringUntil('\n'); s.trim();
            if (s.equalsIgnoreCase("stop")) return true;
        }
        delay(20);
    }
    return false;
}

void processLubeCommand(const String& args) {
    String a = args; a.trim();
    int sp1 = a.indexOf(' ');
    String target = (sp1 == -1) ? a : a.substring(0, sp1);
    String rest    = (sp1 == -1) ? String("") : a.substring(sp1 + 1);
    rest.trim();

    if (target.length() == 0) {
        BOTH_PRINTLN("[LUBE] Не указана цель. Пример: lube motors 3 5000");
        return;
    }

    int cycles = 3;
    uint32_t waitMs = 5000;
    if (rest.length()) {
        int sp2 = rest.indexOf(' ');
        String c = (sp2 == -1) ? rest : rest.substring(0, sp2);
        int cv = c.toInt();
        if (cv > 0) cycles = cv;
        if (sp2 != -1) {
            String w = rest.substring(sp2 + 1); w.trim();
            long wv = w.toInt();
            if (wv > 0) waitMs = (uint32_t)wv;
        }
    }

    BOTH_PRINTF("\n>>> [LUBE] %s  циклов=%d  пауза=%lu мс  (введите 'stop' для отмены)\n",
                target.c_str(), cycles, (unsigned long)waitMs);

    for (int i = 0; i < cycles; i++) {
        BOTH_PRINTF("  [LUBE] цикл %d/%d: вниз...\n", i + 1, cycles);
        lubeSendAction(target, 1); // down
        if (lubeWaitOrAbort(waitMs)) {
            lubeSendAction(target, 0); // stop
            BOTH_PRINTLN("[LUBE] Остановлено пользователем.");
            return;
        }

        BOTH_PRINTF("  [LUBE] цикл %d/%d: вверх...\n", i + 1, cycles);
        lubeSendAction(target, 2); // up
        if (lubeWaitOrAbort(waitMs)) {
            lubeSendAction(target, 0); // stop
            BOTH_PRINTLN("[LUBE] Остановлено пользователем.");
            return;
        }
    }

    BOTH_PRINTLN("[LUBE] Готово.");
}

// ─── LED: отправка ───────────────────────────────────────────
void sendToLed(LedDevice& dev, const String& payload) {
    if (!dev.active || isMacEmpty(dev.mac)) return;
    String full = String(dev.id) + ":" + payload;
    Serial.printf("  --> %-8s  %s\n", dev.id, payload.c_str());
    bool ok = espNowSendWithRetry(dev.mac, (uint8_t*)full.c_str(), full.length());
    if (!ok) Serial.printf("    [!] FAIL (no ACK after retries)\n");
}

void sendToAllLed(const String& payload) {
    for (int i = 0; i < LED_COUNT; i++) sendToLed(ledDevices[i], payload);
}

LedDevice* findLed(const String& id) {
    for (int i = 0; i < LED_COUNT; i++)
        if (String(ledDevices[i].id).equalsIgnoreCase(id)) return &ledDevices[i];
    return nullptr;
}

bool processLedCommand(const String& cmd) {
    int sp = cmd.indexOf(' ');
    String devId  = (sp == -1) ? cmd                    : cmd.substring(0, sp);
    String subcmd = (sp == -1) ? String("pulse")        : cmd.substring(sp + 1);
    subcmd.trim();

    LedDevice* dev = findLed(devId);
    if (!dev) return false;

    Serial.printf("\n>>> [LED: %s] %s\n", dev->id, subcmd.c_str());

    if (subcmd.equalsIgnoreCase("solo")) {
        for (int i = 0; i < LED_COUNT; i++)
            if (!String(ledDevices[i].id).equalsIgnoreCase(devId))
                sendToLed(ledDevices[i], "off");
        sendToLed(*dev, "pulse");
        return true;
    }
    sendToLed(*dev, subcmd);
    return true;
}

// ─── Help ─────────────────────────────────────────────────────
void printHelp() {
    BOTH_PRINTLN("\n=== MASTER COMMAND REFERENCE ===");

    BOTH_PRINTLN("── Моторы ────────────────────────────────────");
    BOTH_PRINTLN("  Targets  : m1-1 | m1-2 | m2-1 | m2-2 | m4");
    BOTH_PRINTLN("             motor1 | motor2 | motor3 | motor4");
    BOTH_PRINTLN("             motors  (все активные)");
    BOTH_PRINTLN("  Actions  : down | up | stop  (или 1 | 2 | 0)");
    BOTH_PRINTLN("  Examples:");
    BOTH_PRINTLN("    m1-1 down          — опустить слейв m1-1");
    BOTH_PRINTLN("    motor1 up          — поднять группу motor1 (m1-1 + m1-2)");
    BOTH_PRINTLN("    motors stop        — остановить всё");
    BOTH_PRINTLN("    motors down        — все вниз");
    BOTH_PRINTLN("    m4 up              — поднять M1..M4 (M5 стоит)");
    BOTH_PRINTLN("    m4 down            — опустить все 5 до концевиков");
    BOTH_PRINTLN("── m4 — 5-мотор слейв (FC:E8:C0:DF:3F:B4) ───");
    BOTH_PRINTLN("  m4 M<N> up/down/stop  — один мотор N (1..5)");
    BOTH_PRINTLN("  m4 mode:sim           — одновременный режим DOWN/UP (умолч.)");
    BOTH_PRINTLN("  m4 mode:seq           — по очереди: DOWN M5→M1, UP M1→M4");
    BOTH_PRINTLN("  motors mode:sim/seq   — переключить режим на всех слейвах");
    BOTH_PRINTLN("  Пины m4:  M1=верх(dir4/step2) M2(dir15/step21)");
    BOTH_PRINTLN("            M3(dir25/step26) M4(dir27/step14) M5=низ(dir12/step13)");
    BOTH_PRINTLN("  Автохоминг при включении (M5→M1). Концевики NC HIGH=сработал.");

    BOTH_PRINTLN("\n── Смазка (lube) ─────────────────────────────");
    BOTH_PRINTLN("  lube <target> [cycles] [waitMs]");
    BOTH_PRINTLN("    target : m1-1|m1-2|m2-1|m2-2|m4|motor1..4|motors");
    BOTH_PRINTLN("    cycles : число циклов вниз-вверх (умолч. 3)");
    BOTH_PRINTLN("    waitMs : пауза на движение, чтобы механизм успел доехать (умолч. 5000)");
    BOTH_PRINTLN("  Пример: lube motors 3 5000");
    BOTH_PRINTLN("  Во время выполнения введите 'stop' чтобы прервать.");

    BOTH_PRINTLN("\n── LED-устройства (слейв: 16 блоков NeoPixelBus) ─");
    BOTH_PRINTLN("  Устройства : red");
    BOTH_PRINTLN("  <device> <command>       — отправить команду устройству");
    BOTH_PRINTLN("  <device>                 — без команды = pulse (белый пульс)");
    BOTH_PRINTLN("  <device> solo            — включить одно (pulse), остальные выкл.");
    BOTH_PRINTLN("  <device> block <1-16|all> <mode>");
    BOTH_PRINTLN("  <device> block <1-16> color (R,G,B,W)");
    BOTH_PRINTLN("  <device> (R,G,B,W)       — залить все блоки сплошным цветом");
    BOTH_PRINTLN("  <device> cfade [(R,G,B,W),...]  — плавный переход по палитре");
    BOTH_PRINTLN("  Режимы     : off | allOn | fade | snake | rain | random |");
    BOTH_PRINTLN("               rainbow | fire | pulse | cfade | auto");
    BOTH_PRINTLN("  Цвет       : (R,G,B,W), например (0,0,0,255)=белый (255,0,0,0)=красный");
    BOTH_PRINTLN("  Глобальные (все LED-устройства, без указания <device>):");
    BOTH_PRINTLN("    off | random | pulse | allOn | fade | snake | rain |");
    BOTH_PRINTLN("    rainbow | fire | auto | cfade | (R,G,B,W)");
    BOTH_PRINTLN("    block <1-16|all> <mode>  — тот же блок на всех LED-устройствах");

    BOTH_PRINTLN("\n── Система ──────────────────────────────────");
    BOTH_PRINTLN("  status             — список всех устройств");
    BOTH_PRINTLN("  help               — эта справка");
    BOTH_PRINTLN("=================================\n");
}

// ─── Status ──────────────────────────────────────────────────
void printStatus() {
    BOTH_PRINTLN("\n=== DEVICE STATUS ===");
    BOTH_PRINTLN("  Motor slaves:");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Serial.printf("    %-6s  %-7s  %s  MAC: ",
            motorSlaves[i].id, motorSlaves[i].group,
            motorSlaves[i].active ? "[active]" : "[placeholder]");
        printMac(motorSlaves[i].mac); Serial.println();
    }
    BOTH_PRINTLN("  LED devices:");
    for (int i = 0; i < LED_COUNT; i++) {
        Serial.printf("    %-8s  %s  MAC: ",
            ledDevices[i].id,
            ledDevices[i].active ? "[active]" : "[placeholder]");
        printMac(ledDevices[i].mac); Serial.println();
    }
    BOTH_PRINTLN("====================\n");
}

// ─── Main command dispatcher ──────────────────────────────────
void processCommand(const String& originalCmd) {
    String cmd = originalCmd;
    cmd.trim();
    if (cmd.length() == 0) return;

    BOTH_PRINTF("\n[CMD] %s\n", cmd.c_str());

    if (cmd.equalsIgnoreCase("help"))   { printHelp();   return; }
    if (cmd.equalsIgnoreCase("status")) { printStatus();  return; }

    // ── Режим смазки ──────────────────────────────────────────
    if (cmd.startsWith("lube ")) { processLubeCommand(cmd.substring(5)); return; }
    if (cmd.equalsIgnoreCase("lube")) { BOTH_PRINTLN("Использование: lube <target> [cycles] [waitMs]"); return; }

    // ── Моторные команды ─────────────────────────────────────
    if (processMotorCommand(cmd)) return;

    // ── LED команды устройства ───────────────────────────────
    if (processLedCommand(cmd)) return;

    // ── Глобальные LED-команды (применяются ко всем LED-слейвам) ──
    if (cmd.equalsIgnoreCase("off"))    { sendToAllLed("off");    return; }
    if (cmd.equalsIgnoreCase("random")) { sendToAllLed("random"); return; }
    if (cmd.equalsIgnoreCase("pulse"))  { sendToAllLed("pulse");  return; }

    if (cmd.startsWith("all "))         { sendToAllLed(cmd.substring(4)); return; }

    if (cmd.startsWith("block ")        ||
        cmd.equalsIgnoreCase("allOn")   || cmd.equalsIgnoreCase("fade")    ||
        cmd.equalsIgnoreCase("snake")   || cmd.equalsIgnoreCase("rain")    ||
        cmd.equalsIgnoreCase("rainbow") || cmd.equalsIgnoreCase("fire")    ||
        cmd.equalsIgnoreCase("auto")    || cmd.equalsIgnoreCase("cycle")   ||
        cmd.startsWith("color ")        || cmd.startsWith("solid ")       ||
        cmd.startsWith("cfade")         || cmd.startsWith("("))
    {
        sendToAllLed(cmd); return;
    }

    BOTH_PRINTF("[ERROR] Unknown command: '%s'\n  Type 'help' for reference.\n", cmd.c_str());
}

// ─── Setup ────────────────────────────────────────────────────
void setup() {
    delay(1000);
    Serial.begin(115200);

    BOTH_PRINTLN("\n=== MASTER CONTROLLER ===");

    if (!SerialBT.begin("N_A_I_Master"))
        Serial.println("[ERROR] Bluetooth init failed!");
    else
        Serial.println("[OK] Bluetooth: N_A_I_Master");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Фиксируем канал 1 — должен совпадать со слейвами
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(100);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW init failed!");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    Serial.println("[OK] ESP-NOW ready");

    Serial.println("\nRegistering motor slaves:");
    for (int i = 0; i < MOTOR_COUNT; i++)
        registerPeer(motorSlaves[i].mac, motorSlaves[i].id);

    Serial.println("\nRegistering LED devices:");
    for (int i = 0; i < LED_COUNT; i++)
        registerPeer(ledDevices[i].mac, ledDevices[i].id);

    printHelp();
    while (Serial.available()) Serial.read();
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length()) processCommand(cmd);
    }
    if (SerialBT.available()) {
        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();
        if (cmd.length()) {
            Serial.printf("[BT] %s\n", cmd.c_str());
            processCommand(cmd);
        }
    }
}

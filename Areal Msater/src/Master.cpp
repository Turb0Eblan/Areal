/*
 * ============================================================
 *  Master Controller — ESP32
 *  Controls the slaves over ESP-NOW.
 *  Commands via USB Serial and Bluetooth.
 *
 *  Motor slaves:
 *    m1-1  FC:E8:C0:DF:33:50   motor1  (2-motor slave)
 *    m1-2  F8:B3:B7:74:A9:B8   motor1  (2-motor slave)
 *    m2-1  FC:E8:C0:E1:5D:64   motor2  (2-motor slave)
 *    m2-2  FC:E8:C0:DF:39:F0   motor2  (2-motor slave)
 *    m3-1  [placeholder]
 *    m3-2  [placeholder]
 *    m4    FC:E8:C0:DF:3F:B4   motor4  (5-motor slave)
 *      m4 motors (M1=top, M5=bottom):
 *        M1: dir=4  step=2  (top)   TMC2209  lim=5
 *        M2: dir=15 step=21             TMC2209  lim=33
 *        M3: dir=25 step=26             TB6600   lim=32
 *        M4: dir=27 step=14             TB6600   lim=22
 *        M5: dir=12 step=13 (bottom)   TB6600   lim=23
 *      Limit switches NC (HIGH=triggered). Auto-homing on power-up (M5→M1).
 *      DOWN/UP modes: SIM=simultaneous (default), SEQ=one by one.
 *        SEQ DOWN: M5→M4→M3→M2→M1    SEQ UP: M1→M2→M3→M4 (M5 stays)
 *
 *  LED slaves (NeoPixelBus, 16 blocks of 300 LEDs, SK6812 RGBW):
 *    red      20:50:0D:D1:32:70
 *    lights   [placeholder]
 *
 *  Motor command examples:
 *    m1-1 down          — lower a single slave
 *    motor1 up          — raise a group (m1-1 + m1-2)
 *    motors stop        — stop all motors
 *    motors down        — lower all motors
 *    m4 up              — raise M1..M4 (M5 stays)
 *    m4 down            — lower all 5 to the limit switches
 *    m4 mode:sim        — switch m4 to simultaneous mode
 *    m4 mode:seq        — switch m4 to sequential mode
 *    m4 M1 up           — raise only M1 on slave m4
 *    m4 M3 down         — lower only M3 on slave m4
 *    m4 M2 stop         — stop M2 on slave m4
 *    lube motors 3 5000 — lube mode: 3 down/up cycles, 5 s pause per move
 *
 *  ESP-NOW MotorCmd protocol:
 *    { cmd, motorIdx }
 *    cmd:      0=STOP  1=DOWN  2=UP  3=MODE_SEQ  4=MODE_SIM
 *    motorIdx: 0=all motors,  1..5=single motor (m4 only)
 *
 *  LED command examples (NeoPixelBus block slave protocol):
 *    red pulse                — white pulse on all 16 blocks of slave red
 *    red block 3 fire         — block #3 of slave red to "fire" mode
 *    red block all off        — turn off all blocks of slave red
 *    red (0,0,0,255)          — fill all red blocks with solid white
 *    red cfade                — smooth palette transition (all blocks)
 *    off                      — turn off all LED slaves
 *    pulse                    — white pulse on all LED slaves
 *    block 5 rainbow          — block #5 on ALL LED slaves — rainbow
 *  Modes: off|allOn|fade|snake|rain|random|rainbow|fire|pulse|cfade|auto
 *  Color:   (R,G,B,W), e.g. (0,0,0,255)=white (255,0,0,0)=red
 * ============================================================
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ─── Output to both interfaces ───────────────────────────────────
#define BOTH_PRINTLN(s)      do { Serial.println(s);                SerialBT.println(s);                } while(0)
#define BOTH_PRINTF(fmt,...) do { Serial.printf(fmt,##__VA_ARGS__); SerialBT.printf(fmt,##__VA_ARGS__); } while(0)

// ─── ESP-NOW packet for motor slaves ──────────────────────
// cmd:      0=STOP  1=DOWN  2=UP  3=MODE_SEQ  4=MODE_SIM
// motorIdx: 0=all motors,  1..5=single motor (m4 / 5-motor slave only)
typedef struct { uint8_t cmd; uint8_t motorIdx; } MotorCmd;

// ─── Send callback + retry mechanism ──────────────────────────
static volatile bool   s_sendDone   = false;
static volatile bool   s_sendOk     = false;

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    s_sendOk   = (status == ESP_NOW_SEND_SUCCESS);
    s_sendDone = true;
}

// Send with retry. Returns true if delivered.
static bool espNowSendWithRetry(const uint8_t* mac, const uint8_t* data, size_t len,
                                int retries = 3, uint32_t retryDelayMs = 50) {
    for (int attempt = 0; attempt < retries; attempt++) {
        s_sendDone = false;
        s_sendOk   = false;
        if (esp_now_send(mac, data, len) != ESP_OK) {
            delay(retryDelayMs);
            continue;
        }
        // Wait for callback (max 200ms)
        uint32_t t = millis();
        while (!s_sendDone && (millis() - t) < 200) { delay(1); }
        if (s_sendOk) return true;
        if (attempt < retries - 1) delay(retryDelayMs);
    }
    return false;
}

// A single "raw" send without internal retry — used in sendMotorCmd()
// as one "shot" of the flood for STOP/DOWN/UP (see MOTOR_MOVE_FLOOD_MS).
// waitMs needs headroom: with BT+WiFi active (coexistence) the send-status
// callback sometimes arrives late. If we wait too little and send the
// next packet almost immediately, s_sendDone/s_sendOk can be overwritten before
// the late callback of the PREVIOUS packet arrives: the packet was actually
// delivered and executed by the slave, yet the master still counts it as FAIL.
// Seen in practice: the mechanism moved while the log showed "FAIL" on every attempt.
static bool espNowSendOnce(const uint8_t* mac, const uint8_t* data, size_t len, uint32_t waitMs = 400) {
    s_sendDone = false;
    s_sendOk   = false;
    if (esp_now_send(mac, data, len) != ESP_OK) return false;
    uint32_t t = millis();
    while (!s_sendDone && (millis() - t) < waitMs) { delay(1); }
    return s_sendOk;
}

// ─── Flooding for motion commands (STOP/DOWN/UP) ────────────
// Slaves ignore DOWN/UP until they are IDLE and send no NACK — so the master
// cannot tell whether the slave accepted the command, only that the packet was delivered over the air.
// To survive packet loss, the command is sent over and over for up to
// MOTOR_MOVE_FLOOD_MS, until a delivery ACK arrives or time runs out.
// Repeating DOWN/UP while the slave is already in that state is a no-op on its side,
// so flooding is safe and causes no extra movement.
// The pause between "shots" is intentionally not short — see the comment on
// espNowSendOnce: sending too often breaks ACK accounting via a race.
#define MOTOR_MOVE_FLOOD_MS      10000  // max flood duration, ms
#define MOTOR_MOVE_REPEAT_GAP_MS 200    // pause between "shots", ms

// ─── Motor slaves ─────────────────────────────────────────
struct MotorSlave {
    uint8_t    mac[6];
    const char *id;      // name used in commands (m1-1, m2-1, ...)
    const char *group;   // group (motor1, motor2, ...)
    bool        active;  // false = placeholder, do not send
};

MotorSlave motorSlaves[] = {
    //  MAC                                    id       group     active
    {{0xFC,0xE8,0xC0,0xDF,0x33,0x50},  "m1-1",  "motor1",  true },
    {{0xFC,0xE8,0xC0,0xDF,0x3F,0xB4},  "m1-2",  "motor1",  true }, //0xF8,0xB3,0xB7,0x74,0xA9,0xB8
    {{0xFC,0xE8,0xC0,0xE1,0x5D,0x64},  "m2-1",  "motor2",  true },
    {{0xFC,0xE8,0xC0,0xDF,0x39,0xF0},  "m2-2",  "motor2",  true },
    // ── Placeholders — fill in the MAC when ready ──
    {{0x00,0x00,0x00,0x00,0x00,0x00},  "m3-1",  "motor3",  false},
    {{0x00,0x00,0x00,0x00,0x00,0x00},  "m3-2",  "motor3",  false},
    // m4 — 5-motor slave (FC:E8:C0:DF:3F:B4)
    {{0xF8,0xB3,0xB7,0x74,0xA9,0xB8},  "m4",    "motor4",  true }, //0xFC,0xE8,0xC0,0xDF,0x3F,0xB4
};
const int MOTOR_COUNT = sizeof(motorSlaves) / sizeof(motorSlaves[0]);

// ─── LED slaves ──────────────────────────────────────────────
struct LedDevice {
    uint8_t    mac[6];
    const char *id;
    bool        active;
};

LedDevice ledDevices[] = {
    {{0x20,0x50,0x0D,0xD1,0x32,0x70}, "blue",    true },
    // ── Placeholder ──
    {{0x00,0x00,0x00,0x00,0x00,0x00}, "lights", false},
};
const int LED_COUNT = sizeof(ledDevices) / sizeof(ledDevices[0]);

// ─── Utilities ─────────────────────────────────────────────────
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
    peer.channel = 1;  // must match the slaves' channel
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.printf("  [+] %-8s  MAC: ", label);
        printMac(mac); Serial.println();
    } else {
        Serial.printf("  [!] Failed: %s\n", label);
    }
}

// ─── Sending to motors ─────────────────────────────────────────
// motorIdx=0 → all motors; motorIdx=1..5 → single motor (m4 only)
void sendMotorCmd(MotorSlave& s, uint8_t cmd, uint8_t motorIdx = 0) {
    if (!s.active || isMacEmpty(s.mac)) {
        BOTH_PRINTF("  [~] %s  skipped (inactive)\n", s.id);
        return;
    }
    MotorCmd pkt = {cmd, motorIdx};
    const char* act = (cmd == 0) ? "STOP" : (cmd == 1) ? "DOWN" : (cmd == 2) ? "UP" :
                      (cmd == 3) ? "MODE:SEQ" : "MODE:SIM";

    // STOP/DOWN/UP are motion commands: flood the slave with the packet for up to
    // MOTOR_MOVE_FLOOD_MS until a delivery ACK arrives. As soon as the slave
    // has answered (or started reacting to a previous "shot" and switched
    // to a busy state) there is no point sending more, so exit early.
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
        BOTH_PRINTF("  --> %-6s  %s  %s (%d attempts, %lums)\n",
                     s.id, act, ok ? "OK" : "FAIL", attempts, (unsigned long)elapsed);
    else
        BOTH_PRINTF("  --> %-6s  M%d %s  %s (%d attempts, %lums)\n",
                     s.id, motorIdx, act, ok ? "OK" : "FAIL", attempts, (unsigned long)elapsed);
}

// Find a slave by id
MotorSlave* findMotorById(const String& id) {
    for (int i = 0; i < MOTOR_COUNT; i++)
        if (String(motorSlaves[i].id).equalsIgnoreCase(id)) return &motorSlaves[i];
    return nullptr;
}

// Send a command by slave name (m1-1, m2-1, m4 ...)
bool sendMotorById(const String& id, uint8_t cmd, uint8_t motorIdx = 0) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (String(motorSlaves[i].id).equalsIgnoreCase(id)) {
            sendMotorCmd(motorSlaves[i], cmd, motorIdx);
            return true;
        }
    }
    return false;
}

// Send a command to a whole group (motor1, motor2 ...)
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

// Send to all active motors
void sendMotorAll(uint8_t cmd) {
    for (int i = 0; i < MOTOR_COUNT; i++)
        sendMotorCmd(motorSlaves[i], cmd);
}

// ─── Motor command parsing ─────────────────────────────────
// Format: <target> <action>
//   target : m1-1 | m1-2 | m2-1 | m2-2 | motor1 | motor2 | motors
//   action : down | up | stop  (or 1 | 2 | 0)
uint8_t parseMotorAction(const String& act) {
    if (act.equalsIgnoreCase("down") || act == "1") return 1;
    if (act.equalsIgnoreCase("up")   || act == "2") return 2;
    if (act.equalsIgnoreCase("stop") || act == "0") return 0;
    return 0xFF; // unknown
}

bool processMotorCommand(const String& cmd) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) return false;
    String target = cmd.substring(0, sp);
    String rest   = cmd.substring(sp + 1);
    rest.trim();

    // ── Mode switch commands (MODE:SIM / MODE:SEQ) ──────────
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

    // ── Single-motor command for slave m4: "m4 MN up/down/stop" ──
    // Format: m4 M<1..5> <action>
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
                BOTH_PRINTF("\n>>> [M4 single] M%d → %s\n", mIdx, actStr.c_str());
                sendMotorById("m4", action, (uint8_t)mIdx);
                return true;
            }
        }
    }

    uint8_t action = parseMotorAction(rest);
    if (action == 0xFF) return false;

    BOTH_PRINTF("\n>>> [MOTOR] %s → %s\n", target.c_str(), rest.c_str());

    // "motors" = all
    if (target.equalsIgnoreCase("motors") || target.equalsIgnoreCase("all-motors")) {
        sendMotorAll(action); return true;
    }
    // Group (motor1, motor2, motor3 ...) — checked without toLowerCase()
    {
        String t = target;
        t.toLowerCase();
        if (t.startsWith("motor")) {
            if (sendMotorByGroup(target, action)) return true;
        }
    }
    // Specific slave (m1-1, m2-1, m4 ...)
    if (sendMotorById(target, action)) return true;

    return false;
}

// ─── Lube mode ──────────────────────────────────────
// Repeatedly lowers and raises the mechanism to work the lubricant in.
// Format: lube <target> [cycles] [waitMs]
//   target : m1-1 | m1-2 | m2-1 | m2-2 | m4 | motor1..4 | motors
//   cycles : number of down-up cycles (default 3)
//   waitMs : pause per move so the mechanism can finish travelling (default 5000 ms)
// While running, type "stop" (Serial/BT) to abort the cycle.
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

// Waits ms while listening on Serial/BT for "stop". true = aborted.
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
        BOTH_PRINTLN("[LUBE] No target given. Example: lube motors 3 5000");
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

    BOTH_PRINTF("\n>>> [LUBE] %s  cycles=%d  pause=%lu ms  (type 'stop' to cancel)\n",
                target.c_str(), cycles, (unsigned long)waitMs);

    for (int i = 0; i < cycles; i++) {
        BOTH_PRINTF("  [LUBE] cycle %d/%d: down...\n", i + 1, cycles);
        lubeSendAction(target, 1); // down
        if (lubeWaitOrAbort(waitMs)) {
            lubeSendAction(target, 0); // stop
            BOTH_PRINTLN("[LUBE] Stopped by user.");
            return;
        }

        BOTH_PRINTF("  [LUBE] cycle %d/%d: up...\n", i + 1, cycles);
        lubeSendAction(target, 2); // up
        if (lubeWaitOrAbort(waitMs)) {
            lubeSendAction(target, 0); // stop
            BOTH_PRINTLN("[LUBE] Stopped by user.");
            return;
        }
    }

    BOTH_PRINTLN("[LUBE] Done.");
}

// ─── LED: sending ───────────────────────────────────────────
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

    BOTH_PRINTLN("── Motors ────────────────────────────────────");
    BOTH_PRINTLN("  Targets  : m1-1 | m1-2 | m2-1 | m2-2 | m4");
    BOTH_PRINTLN("             motor1 | motor2 | motor3 | motor4");
    BOTH_PRINTLN("             motors  (all active)");
    BOTH_PRINTLN("  Actions  : down | up | stop  (or 1 | 2 | 0)");
    BOTH_PRINTLN("  Examples:");
    BOTH_PRINTLN("    m1-1 down          — lower slave m1-1");
    BOTH_PRINTLN("    motor1 up          — raise group motor1 (m1-1 + m1-2)");
    BOTH_PRINTLN("    motors stop        — stop everything");
    BOTH_PRINTLN("    motors down        — all down");
    BOTH_PRINTLN("    m4 up              — raise M1..M4 (M5 stays)");
    BOTH_PRINTLN("    m4 down            — lower all 5 to the limit switches");
    BOTH_PRINTLN("── m4 — 5-motor slave (FC:E8:C0:DF:3F:B4) ───");
    BOTH_PRINTLN("  m4 M<N> up/down/stop  — single motor N (1..5)");
    BOTH_PRINTLN("  m4 mode:sim           — simultaneous DOWN/UP mode (default)");
    BOTH_PRINTLN("  m4 mode:seq           — one by one: DOWN M5→M1, UP M1→M4");
    BOTH_PRINTLN("  motors mode:sim/seq   — switch mode on all slaves");
    BOTH_PRINTLN("  m4 pins:  M1=top(dir4/step2) M2(dir15/step21)");
    BOTH_PRINTLN("            M3(dir25/step26) M4(dir27/step14) M5=bottom(dir12/step13)");
    BOTH_PRINTLN("  Auto-homing on power-up (M5→M1). Limit switches NC, HIGH=triggered.");

    BOTH_PRINTLN("\n── Lube ─────────────────────────────");
    BOTH_PRINTLN("  lube <target> [cycles] [waitMs]");
    BOTH_PRINTLN("    target : m1-1|m1-2|m2-1|m2-2|m4|motor1..4|motors");
    BOTH_PRINTLN("    cycles : number of down-up cycles (default 3)");
    BOTH_PRINTLN("    waitMs : pause per move so the mechanism can finish travelling (default 5000)");
    BOTH_PRINTLN("  Example: lube motors 3 5000");
    BOTH_PRINTLN("  While running, type 'stop' to abort.");

    BOTH_PRINTLN("\n── LED devices (slave: 16 NeoPixelBus blocks) ─");
    BOTH_PRINTLN("  Devices    : red");
    BOTH_PRINTLN("  <device> <command>       — send a command to the device");
    BOTH_PRINTLN("  <device>                 — no command = pulse (white pulse)");
    BOTH_PRINTLN("  <device> solo            — turn on this one (pulse), others off");
    BOTH_PRINTLN("  <device> block <1-16|all> <mode>");
    BOTH_PRINTLN("  <device> block <1-16> color (R,G,B,W)");
    BOTH_PRINTLN("  <device> (R,G,B,W)       — fill all blocks with a solid color");
    BOTH_PRINTLN("  <device> cfade [(R,G,B,W),...]  — smooth palette transition");
    BOTH_PRINTLN("  Modes      : off | allOn | fade | snake | rain | random |");
    BOTH_PRINTLN("               rainbow | fire | pulse | cfade | auto");
    BOTH_PRINTLN("  Color      : (R,G,B,W), e.g. (0,0,0,255)=white (255,0,0,0)=red");
    BOTH_PRINTLN("  Global (all LED devices, no <device> given):");
    BOTH_PRINTLN("    off | random | pulse | allOn | fade | snake | rain |");
    BOTH_PRINTLN("    rainbow | fire | auto | cfade | (R,G,B,W)");
    BOTH_PRINTLN("    block <1-16|all> <mode>  — the same block on all LED devices");

    BOTH_PRINTLN("\n── System ──────────────────────────────────");
    BOTH_PRINTLN("  status             — list all devices");
    BOTH_PRINTLN("  help               — this help");
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

    // ── Lube mode ──────────────────────────────────────────
    if (cmd.startsWith("lube ")) { processLubeCommand(cmd.substring(5)); return; }
    if (cmd.equalsIgnoreCase("lube")) { BOTH_PRINTLN("Usage: lube <target> [cycles] [waitMs]"); return; }

    // ── Motor commands ─────────────────────────────────────
    if (processMotorCommand(cmd)) return;

    // ── Device LED commands ───────────────────────────────
    if (processLedCommand(cmd)) return;

    // ── Global LED commands (applied to all LED slaves) ──
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
    // Pin channel 1 — must match the slaves
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

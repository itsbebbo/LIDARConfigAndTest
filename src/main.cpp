#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <driver/gpio.h>

// ── Port A GPIO pins (M5Stack CoreS3 / CoreS3 SE) ─────────────────────────
static constexpr int PORT_A_SCL = 1;   // yellow wire
static constexpr int PORT_A_SDA = 2;   // white wire

// ── TFmini-S settings ──────────────────────────────────────────────────────
static constexpr uint32_t LIDAR_BAUD      = 115200;
static constexpr uint8_t  LIDAR_I2C_ADDR  = 0x10;
static constexpr int      MAX_RANGE_CM    = 1200;   // 12 m

// Command packets (header 0x5A + len + cmd_id + payload + checksum)
static const uint8_t CMD_SET_UART[] = {0x5A, 0x05, 0x0A, 0x00, 0x69};
static const uint8_t CMD_SET_I2C[]  = {0x5A, 0x05, 0x0A, 0x01, 0x6A};
static const uint8_t CMD_SAVE[]     = {0x5A, 0x04, 0x11, 0x6F};
// Request a measurement in centimetre format (required before each I2C read)
static const uint8_t CMD_I2C_CM[]   = {0x5A, 0x05, 0x00, 0x01, 0x60};

// ── Layout constants (320 × 240) ───────────────────────────────────────────
static constexpr int SCR_W   = 320;
static constexpr int SCR_H   = 240;
static constexpr int HDR_H   = 36;
static constexpr int BAR_M   = 12;    // horizontal margin
static constexpr int BAR_X   = BAR_M;
static constexpr int BAR_W   = SCR_W - 2 * BAR_M;
static constexpr int BAR_Y   = 58;
static constexpr int BAR_H   = 48;
static constexpr int BTN_H   = 48;
static constexpr int BTN_Y   = SCR_H - BTN_H - 8;
static constexpr int BTN_X   = BAR_M;
static constexpr int BTN_W   = SCR_W - 2 * BAR_M;

// ── App state ──────────────────────────────────────────────────────────────
enum class State { UART_MODE, I2C_MODE, NO_LIDAR };
static State appState = State::NO_LIDAR;

// Last valid measurement
static int16_t  g_dist_cm    = 0;
static uint16_t g_strength   = 0;
static uint8_t  g_readFails  = 0;       // consecutive read failures
static constexpr uint8_t MAX_FAILS = 20; // ~20 frames before redetect

// Double-buffer sprite
static M5Canvas canvas(&M5.Display);

// ── Forward declarations ───────────────────────────────────────────────────
static void redetect();
static bool detectUART();
static bool detectI2C();
static bool readFrameUART(int16_t &dist, uint16_t &strength);
static bool readFrameI2C(int16_t &dist, uint16_t &strength);
static bool isNoObject(int16_t dist, uint16_t strength);
static void drawHeader(const char *mode);
static void drawBar(bool valid, int16_t dist, uint16_t strength);
static void drawSwitchButton(const char *label, uint32_t bg);
static void drawNoLidarScreen();
static bool showConfirm(const char *line1, const char *line2);
static void doSwitch(State target);

// ═══════════════════════════════════════════════════════════════════════════
// Detection
// ═══════════════════════════════════════════════════════════════════════════

static bool detectUART() {
    Serial1.begin(LIDAR_BAUD, SERIAL_8N1, PORT_A_SCL, PORT_A_SDA);
    delay(300);
    while (Serial1.available()) Serial1.read(); // flush stale bytes

    uint32_t start = millis();
    uint8_t  buf[9];
    uint8_t  idx = 0;

    while (millis() - start < 1500) {
        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (idx == 0) {
                if (b == 0x59) buf[idx++] = b;
            } else if (idx == 1) {
                if (b == 0x59) buf[idx++] = b; else idx = 0;
            } else {
                buf[idx++] = b;
                if (idx == 9) {
                    uint8_t cs = 0;
                    for (int i = 0; i < 8; i++) cs += buf[i];
                    if (cs == buf[8]) return true; // valid frame found
                    idx = 0;
                }
            }
        }
        delay(10);
    }
    Serial1.end();
    return false;
}

static bool detectI2C() {
    delay(50); // let pins settle after Serial1.end() releases them

    Wire.begin(PORT_A_SDA, PORT_A_SCL);
    Wire.setClock(100000);

    // Enable internal pull-ups — Grove Port A has no external pull-ups
    gpio_set_pull_mode((gpio_num_t)PORT_A_SDA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)PORT_A_SCL, GPIO_PULLUP_ONLY);

    delay(200);

    // Full bus scan so we can see what's actually on the bus
    Serial.println("[I2C] Scanning bus...");
    bool anyFound = false;
    for (int addr = 1; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C]   Device at 0x%02X\n", addr);
            anyFound = true;
        }
    }
    if (!anyFound) Serial.println("[I2C]   No devices found on bus");

    // Try up to 8 reads at the expected TFmini-S address
    for (int attempt = 0; attempt < 8; attempt++) {
        // Send the CM-format request command; sensor populates its output on receipt
        Wire.beginTransmission(LIDAR_I2C_ADDR);
        Wire.write(CMD_I2C_CM, sizeof(CMD_I2C_CM));
        Wire.endTransmission();
        delay(10);

        uint8_t got = Wire.requestFrom((int)LIDAR_I2C_ADDR, 9, 1);
        uint8_t avail = Wire.available();
        Serial.printf("[I2C] Attempt %d: got=%d avail=%d\n", attempt, got, avail);

        if (got == 9 && avail == 9) {
            uint8_t buf[9];
            for (int i = 0; i < 9; i++) buf[i] = Wire.read();
            Serial.printf("[I2C]   Raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);

            if (buf[0] == 0x59 && buf[1] == 0x59) {
                uint8_t cs = 0;
                for (int i = 0; i < 8; i++) cs += buf[i];
                if (cs == buf[8]) {
                    Serial.println("[I2C]   Valid TFmini-S frame — detected!");
                    return true;
                }
                Serial.printf("[I2C]   Headers OK but checksum fail (calc=%02X got=%02X)\n", cs, buf[8]);
            } else {
                Serial.println("[I2C]   Wrong headers");
            }
        } else {
            while (Wire.available()) Wire.read();
            if (got == 0) {
                Serial.println("[I2C]   No ACK from 0x10 — aborting");
                Wire.end();
                return false;
            }
        }
        delay(20);
    }

    Wire.end();
    return false;
}

static void redetect() {
    g_readFails = 0;
    if      (detectUART()) appState = State::UART_MODE;
    else if (detectI2C())  appState = State::I2C_MODE;
    else                   appState = State::NO_LIDAR;
}

// ═══════════════════════════════════════════════════════════════════════════
// Reading
// ═══════════════════════════════════════════════════════════════════════════

static bool readFrameUART(int16_t &dist, uint16_t &strength) {
    // Discard all buffered bytes so we read the next frame off the wire,
    // not stale frames that accumulated while the display was rendering.
    while (Serial1.available()) Serial1.read();

    uint8_t buf[9], idx = 0;
    uint32_t start = millis();

    while (millis() - start < 150) {
        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (idx == 0) {
                if (b == 0x59) buf[idx++] = b;
            } else if (idx == 1) {
                if (b == 0x59) buf[idx++] = b; else idx = 0;
            } else {
                buf[idx++] = b;
                if (idx == 9) {
                    uint8_t cs = 0;
                    for (int i = 0; i < 8; i++) cs += buf[i];
                    if (cs == buf[8]) {
                        dist     = (int16_t)(buf[2] | (buf[3] << 8));
                        strength = (uint16_t)(buf[4] | (buf[5] << 8));
                        return true;
                    }
                    idx = 0;
                }
            }
        }
        delay(2);
    }
    return false;
}

static bool readFrameI2C(int16_t &dist, uint16_t &strength) {
    // Send CM-format request; sensor populates its 9-byte output on receipt
    Wire.beginTransmission(LIDAR_I2C_ADDR);
    Wire.write(CMD_I2C_CM, sizeof(CMD_I2C_CM));
    if (Wire.endTransmission() != 0) return false;
    delay(10);

    Wire.requestFrom((int)LIDAR_I2C_ADDR, 9, 1);
    if (Wire.available() < 9) {
        while (Wire.available()) Wire.read();
        return false;
    }

    uint8_t buf[9];
    for (int i = 0; i < 9; i++) buf[i] = Wire.read();

    if (buf[0] != 0x59 || buf[1] != 0x59) return false;
    uint8_t cs = 0;
    for (int i = 0; i < 8; i++) cs += buf[i];
    if (cs != buf[8]) return false;

    dist     = (int16_t)(buf[2] | (buf[3] << 8));
    strength = (uint16_t)(buf[4] | (buf[5] << 8));
    return true;
}

static bool isNoObject(int16_t dist, uint16_t strength) {
    return (strength < 100 || strength == 0xFFFF || dist <= 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Drawing (all to canvas, then pushed)
// ═══════════════════════════════════════════════════════════════════════════

static void drawHeader(const char *mode) {
    // Colors are RGB888 (uint32_t 0xRRGGBB): 0x006000 = dark green, 0xA00000 = medium red
    uint32_t bg = (uint32_t)NAVY;
    if (mode) bg = (strcmp(mode, "UART") == 0) ? (uint32_t)0x6000 : (uint32_t)0xA00000;

    canvas.fillRect(0, 0, SCR_W, HDR_H, bg);
    canvas.setTextColor(WHITE, bg);
    canvas.setTextSize(2);

    char buf[32];
    if (mode) snprintf(buf, sizeof(buf), "LIDAR DETECTED - %s", mode);
    else      strcpy(buf, "TFmini-S LIDAR");

    canvas.setCursor((SCR_W - canvas.textWidth(buf)) / 2, (HDR_H - 16) / 2);
    canvas.print(buf);
}

static void drawBar(bool valid, int16_t dist, uint16_t strength) {
    // Range axis labels
    canvas.setTextSize(1);
    canvas.setTextColor(0x7BEF, BLACK); // mid-grey
    canvas.setCursor(BAR_X, BAR_Y - 14);
    canvas.print("0m");
    const char *maxLbl = "12m";
    canvas.setCursor(BAR_X + BAR_W - canvas.textWidth(maxLbl), BAR_Y - 14);
    canvas.print(maxLbl);

    // Bar track
    canvas.fillRoundRect(BAR_X, BAR_Y, BAR_W, BAR_H, 4, 0x39E7); // dark grey
    canvas.drawRoundRect(BAR_X, BAR_Y, BAR_W, BAR_H, 4, WHITE);

    bool noObj = !valid || isNoObject(dist, strength);

    if (!noObj) {
        int fill = constrain((int)((float)dist / MAX_RANGE_CM * (BAR_W - 2)), 1, BAR_W - 2);
        canvas.fillRoundRect(BAR_X + 1, BAR_Y + 1, fill, BAR_H - 2, 3, TFT_GREEN);
    }

    // Distance readout
    canvas.setTextSize(3);
    canvas.setTextColor(WHITE);
    char distBuf[20];
    if (!valid)       strcpy(distBuf, "--- m");
    else if (noObj)   strcpy(distBuf, "NO OBJECT");
    else              snprintf(distBuf, sizeof(distBuf), "%.2f m", dist / 100.0f);
    canvas.setCursor((SCR_W - canvas.textWidth(distBuf)) / 2, BAR_Y + BAR_H + 6);
    canvas.print(distBuf);

    // Signal strength annotation
    if (valid) {
        canvas.setTextSize(1);
        canvas.setTextColor(0x7BEF, BLACK);
        char sigBuf[24];
        snprintf(sigBuf, sizeof(sigBuf), "Signal: %u", strength);
        canvas.setCursor(BAR_X, BAR_Y + BAR_H + 38);
        canvas.print(sigBuf);
    }
}

static void drawSwitchButton(const char *label, uint32_t bg) {
    canvas.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 8, bg);
    canvas.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 8, WHITE);
    canvas.setTextColor(WHITE, bg);
    canvas.setTextSize(2);
    canvas.setCursor((SCR_W - canvas.textWidth(label)) / 2, BTN_Y + (BTN_H - 16) / 2);
    canvas.print(label);
}

static void drawNoLidarScreen() {
    canvas.fillSprite(BLACK);
    drawHeader(nullptr);

    canvas.setTextSize(2);
    canvas.setTextColor(TFT_RED, BLACK);
    const char *l1 = "NO LIDAR DETECTED";
    const char *l2 = "ON PORT A";
    canvas.setCursor((SCR_W - canvas.textWidth(l1)) / 2, 85);
    canvas.print(l1);
    canvas.setCursor((SCR_W - canvas.textWidth(l2)) / 2, 110);
    canvas.print(l2);

    canvas.setTextSize(1);
    canvas.setTextColor(0x7BEF, BLACK);
    const char *ck = "Checking...";
    canvas.setCursor((SCR_W - canvas.textWidth(ck)) / 2, BTN_Y + (BTN_H - 8) / 2);
    canvas.print(ck);
}

// ═══════════════════════════════════════════════════════════════════════════
// Confirmation dialog (blocking, draws directly to display)
// ═══════════════════════════════════════════════════════════════════════════

static bool showConfirm(const char *line1, const char *line2) {
    constexpr int DX = 20, DY = 55, DW = SCR_W - 40, DH = 130;

    M5.Display.fillRoundRect(DX, DY, DW, DH, 10, (uint32_t)0x000080);
    M5.Display.drawRoundRect(DX, DY, DW, DH, 10, WHITE);
    M5.Display.setTextColor(WHITE, (uint32_t)0x000080);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(DX + 8, DY + 12);
    M5.Display.print(line1);
    if (line2) {
        M5.Display.setTextSize(1);
        M5.Display.setCursor(DX + 8, DY + 40);
        M5.Display.print(line2);
    }

    // OK button
    constexpr int OK_X = DX + 8, OK_Y = DY + 75, OK_W = 115, OK_H = 40;
    M5.Display.fillRoundRect(OK_X, OK_Y, OK_W, OK_H, 6, TFT_GREEN);
    M5.Display.setTextColor(BLACK, TFT_GREEN);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(OK_X + (OK_W - 24) / 2, OK_Y + 11);
    M5.Display.print("OK");

    // Cancel button
    constexpr int CX = DX + DW - 123, CY = DY + 75, CW = 115, CH = 40;
    M5.Display.fillRoundRect(CX, CY, CW, CH, 6, TFT_RED);
    M5.Display.setTextColor(WHITE, TFT_RED);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(CX + 6, CY + 11);
    M5.Display.print("CANCEL");

    uint32_t deadline = millis() + 10000;
    while (millis() < deadline) {
        M5.update();
        auto t = M5.Touch.getDetail();
        if (t.wasClicked()) {
            if (t.x >= OK_X && t.x < OK_X + OK_W && t.y >= OK_Y && t.y < OK_Y + OK_H)
                return true;
            if (t.x >= CX && t.x < CX + CW && t.y >= CY && t.y < CY + CH)
                return false;
        }
        delay(10);
    }
    return false; // timeout → cancel
}

// ═══════════════════════════════════════════════════════════════════════════
// Mode switching
// ═══════════════════════════════════════════════════════════════════════════

static void sendViaUART(const uint8_t *cmd, size_t len) {
    Serial1.write(cmd, len);
    Serial1.flush();
    delay(100);
}

static void sendViaI2C(const uint8_t *cmd, size_t len) {
    Wire.beginTransmission(LIDAR_I2C_ADDR);
    Wire.write(cmd, len);
    Wire.endTransmission();
    delay(100);
}

static void doSwitch(State target) {
    if (target == appState) return; // no-op guard
    if (target == State::I2C_MODE) {
        if (!showConfirm("Are you sure?", "The drone uses UART mode LIDARs.")) return;
    }

    if (appState == State::UART_MODE) {
        sendViaUART(target == State::I2C_MODE ? CMD_SET_I2C : CMD_SET_UART,
                    target == State::I2C_MODE ? sizeof(CMD_SET_I2C) : sizeof(CMD_SET_UART));
        sendViaUART(CMD_SAVE, sizeof(CMD_SAVE));
        delay(200);
        Serial1.end();
    } else {
        if (target == State::UART_MODE) {
            M5.Display.fillScreen(BLACK);
            M5.Display.setTextColor(WHITE, BLACK);
            M5.Display.setTextSize(2);
            const char *msg = "Changing to UART...";
            M5.Display.setCursor((SCR_W - M5.Display.textWidth(msg)) / 2, (SCR_H - 16) / 2);
            M5.Display.print(msg);
        }
        sendViaI2C(target == State::UART_MODE ? CMD_SET_UART : CMD_SET_I2C,
                   target == State::UART_MODE ? sizeof(CMD_SET_UART) : sizeof(CMD_SET_I2C));
        sendViaI2C(CMD_SAVE, sizeof(CMD_SAVE));
        delay(200);
        Wire.end();
    }

    delay(500); // allow sensor to restart in new mode
    redetect();
}

// ═══════════════════════════════════════════════════════════════════════════
// Arduino entry points
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);

    Serial.begin(115200);
    delay(500); // allow USB CDC to enumerate before first serial output

    canvas.createSprite(SCR_W, SCR_H);
    drawNoLidarScreen();   // show "Checking..." while first detection runs
    canvas.pushSprite(0, 0);
    redetect();
}

void loop() {
    M5.update();
    auto touch = M5.Touch.getDetail(); // read once per frame

    // ── No LIDAR ──────────────────────────────────────────────────────────
    if (appState == State::NO_LIDAR) {
        drawNoLidarScreen();
        canvas.pushSprite(0, 0);
        redetect(); // blocks ~2 s per attempt; updates appState when found
        return;
    }

    // ── Read measurement ───────────────────────────────────────────────────
    int16_t  dist     = 0;
    uint16_t strength = 0;
    bool ok = (appState == State::UART_MODE)
              ? readFrameUART(dist, strength)
              : readFrameI2C(dist, strength);

    if (ok) {
        g_readFails = 0;
    } else if (++g_readFails >= MAX_FAILS) {
        g_readFails = 0;
        redetect();
        return;
    }

    // ── Render frame ───────────────────────────────────────────────────────
    canvas.fillSprite(BLACK);
    if (appState == State::UART_MODE) {
        drawHeader("UART");
        drawBar(ok, dist, strength);
        drawSwitchButton("SWITCH TO I2C", 0xA00000);
    } else {
        drawHeader("I2C");
        drawBar(ok, dist, strength);
        drawSwitchButton("SWITCH TO UART", 0x006000);
    }
    canvas.pushSprite(0, 0);

    // ── Handle button tap ──────────────────────────────────────────────────
    if (touch.wasClicked() &&
        touch.x >= BTN_X && touch.x < BTN_X + BTN_W &&
        touch.y >= BTN_Y && touch.y < BTN_Y + BTN_H) {
        doSwitch(appState == State::UART_MODE ? State::I2C_MODE : State::UART_MODE);
    }
}

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ==================== 1. RS485 Sniffer Configuration ====================
// Pins 43 (TX) and 44 (RX) are free on Waveshare ESP32-S3-Touch-LCD-4.3B
// GPIO4 = touch INT, GPIO5 = LCD RGB DE — do NOT use those.
#define ENABLE_RS485_SNIFFER   1
#define RS485_RX_PIN           44
#define RS485_TX_PIN           43
// No DE/RTS pin needed: we are receive-only sniffer, pulled LOW by transceiver
// If your module has an RE/DE pin, wire it to GND permanently for pure RX mode.

// Self-test: uses Serial2 to loop TX→RX on the same UART pair so we can
// verify capture logic before the hardware is even wired to the bus.
// Set to 1 to enable loopback self-test (connect pin 43 to pin 44 with a wire).
#define RS485_SELF_TEST        1

// ==================== 2. RS485 Config Table ====================
// Covers all common AC controller protocols (Daikin, Mitsubishi, Haier, etc.)
struct RS485Config {
    uint32_t    baud;
    uint32_t    serial_cfg;   // Arduino SERIAL_8x1 constant
    const char* label;        // short label for on-screen display
    const char* desc;         // verbose label for serial monitor
};

static const RS485Config rs485_configs[] = {
    { 9600,  SERIAL_8N1, "9600-8N1",  "9600 baud 8N1 (no parity)"          },
    { 9600,  SERIAL_8E1, "9600-8E1",  "9600 baud 8E1 (even - Haier/Gree)"  },
    { 9600,  SERIAL_8O1, "9600-8O1",  "9600 baud 8O1 (odd)"                },
    { 4800,  SERIAL_8N1, "4800-8N1",  "4800 baud 8N1"                       },
    { 4800,  SERIAL_8E1, "4800-8E1",  "4800 baud 8E1 (even)"               },
    { 19200, SERIAL_8N1, "19200-8N1", "19200 baud 8N1"                      },
    { 19200, SERIAL_8E1, "19200-8E1", "19200 baud 8E1 (even - Daikin)"      },
    { 38400, SERIAL_8N1, "38400-8N1", "38400 baud 8N1"                      },
    { 38400, SERIAL_8E1, "38400-8E1", "38400 baud 8E1"                      },
    { 2400,  SERIAL_8E1, "2400-8E1",  "2400 baud 8E1 (slow legacy)"         },
};
static const int RS485_NUM_CONFIGS = sizeof(rs485_configs) / sizeof(rs485_configs[0]);
static const uint32_t SCAN_INTERVAL_MS = 6000; // ms per config slot

// ==================== 3. Shared Sniffer State (task → UI) ====================
// Written by sniffer FreeRTOS task, read by LVGL timer — using volatile +
// simple byte-aligned types so reads are atomic enough for a status display.
struct SnifferStatus {
    volatile int     cfg_idx;          // current config index
    volatile uint32_t bytes_total;     // total raw bytes seen this slot
    volatile uint32_t pkts_valid;      // packets that passed heuristic
    volatile uint32_t pkts_invalid;    // packets that failed heuristic
    volatile uint32_t slots_scanned;   // how many config slots completed
    volatile bool    self_test_ok;     // did self-test TX→RX loop pass?
    char             status_line[64];  // human-readable, set by sniffer task
};
static SnifferStatus g_sniffer = {0, 0, 0, 0, 0, false, "Initialising..."};

// ==================== 4. Central AC State ====================
bool ac_master_power  = true;
int  ac_target_temp   = 24;
int  ac_current_mode  = 0;  // 0:Cool 1:Heat 2:Fan 3:Dry 4:Auto
int  ac_current_fan   = 1;  // 0:Low  1:Med  2:High
bool ac_zones[]       = {true, false, false, false, false};

// ==================== 5. UI Widget Pointers ====================
lv_obj_t *left_panel;
lv_obj_t *right_panel;
lv_obj_t *label_room_temp;
lv_obj_t *label_target_temp;
lv_obj_t *lbl_btn_mode;
lv_obj_t *lbl_btn_fan;
lv_obj_t *lbl_power;
lv_obj_t *btn_power;
lv_obj_t *btn_zones[5];
lv_obj_t *lbl_zones[5];
lv_obj_t *lbl_sniffer_bar = NULL;  // status bar label (right panel, below zones)

const char* mode_names[] = {"COOL", "HEAT", "FAN ONLY", "DRY", "AUTO"};
const char* fan_names[]  = {"LOW", "MEDIUM", "HIGH"};
const char* zone_names[] = {"Upstairs Bed", "Rumpus", "Dining", "Downstairs Bed", "Living Room"};

#define UI_FONT_CN LV_FONT_DEFAULT

static const lv_color_t COLOR_BG            = lv_color_make(18,  22,  30 );
static const lv_color_t COLOR_PANEL_ON      = lv_color_make(30,  35,  45 );
static const lv_color_t COLOR_PANEL_OFF     = lv_color_make(32,  40,  52 );
static const lv_color_t COLOR_BUTTON_ON     = lv_color_make(50,  60,  80 );
static const lv_color_t COLOR_BUTTON_OFF    = lv_color_make(58,  70,  88 );
static const lv_color_t COLOR_ZONE_OFF      = lv_color_make(60,  65,  75 );
static const lv_color_t COLOR_ZONE_DISABLED = lv_color_make(52,  64,  82 );
static const lv_color_t COLOR_TEXT_MUTED    = lv_color_make(150, 164, 180);
static const lv_color_t COLOR_TEXT_OFF      = lv_color_make(128, 150, 170);
static const lv_color_t COLOR_SNIFFER_BG    = lv_color_make(20,  28,  42 );

// Forward declarations
static void refresh_air_con_ui(void);
static void refresh_zone_button(int zone_idx);

// ==================== 6. UI Event Callbacks ====================
static void temp_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    const char *action = (const char *)lv_event_get_user_data(e);
    if (!action) return;
    if (strcmp(action, "+") == 0) ac_target_temp++;
    else                          ac_target_temp--;
    if (ac_target_temp > 30) ac_target_temp = 30;
    if (ac_target_temp < 16) ac_target_temp = 16;
    lv_label_set_text_fmt(label_target_temp, "%d\xC2\xB0" "C", ac_target_temp);
    Serial.printf("[UI] Target temp -> %d°C\n", ac_target_temp);
}

static void zone_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    int zone_idx = (int)(uintptr_t)lv_event_get_user_data(e);
    ac_zones[zone_idx] = !ac_zones[zone_idx];
    refresh_zone_button(zone_idx);
    Serial.printf("[UI] Zone %d -> %s\n", zone_idx + 1, ac_zones[zone_idx] ? "ON" : "OFF");
}

static void mode_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    ac_current_mode = (ac_current_mode + 1) % 5;
    refresh_air_con_ui();
    Serial.printf("[UI] Mode -> %s\n", mode_names[ac_current_mode]);
}

static void fan_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    ac_current_fan = (ac_current_fan + 1) % 3;
    refresh_air_con_ui();
    Serial.printf("[UI] Fan -> %s\n", fan_names[ac_current_fan]);
}

static void power_btn_cb(lv_event_t *e) {
    ac_master_power = !ac_master_power;
    refresh_air_con_ui();
    Serial.printf("[UI] Power -> %s\n", ac_master_power ? "ON" : "OFF");
}

// ==================== 7. Zone Button Visual Refresh ====================
static void refresh_zone_button(int zone_idx) {
    if (!btn_zones[zone_idx] || !lbl_zones[zone_idx]) return;
    if (!ac_master_power) {
        lv_obj_set_style_bg_color(btn_zones[zone_idx], COLOR_ZONE_DISABLED, 0);
        lv_obj_set_style_text_color(lbl_zones[zone_idx], COLOR_TEXT_OFF, 0);
        lv_label_set_text_fmt(lbl_zones[zone_idx], "%s: StandBy", zone_names[zone_idx]);
        return;
    }
    if (ac_zones[zone_idx]) {
        lv_obj_set_style_bg_color(btn_zones[zone_idx], lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_text_color(lbl_zones[zone_idx], lv_color_white(), 0);
        lv_label_set_text_fmt(lbl_zones[zone_idx], "%s: ON", zone_names[zone_idx]);
    } else {
        lv_obj_set_style_bg_color(btn_zones[zone_idx], COLOR_ZONE_OFF, 0);
        lv_obj_set_style_text_color(lbl_zones[zone_idx], COLOR_TEXT_MUTED, 0);
        lv_label_set_text_fmt(lbl_zones[zone_idx], "%s: OFF", zone_names[zone_idx]);
    }
}

// ==================== 8. Full Panel Refresh ====================
static void refresh_air_con_ui(void) {
    lv_color_t panel_color   = ac_master_power ? COLOR_PANEL_ON    : COLOR_PANEL_OFF;
    lv_color_t control_color = ac_master_power ? COLOR_BUTTON_ON   : COLOR_BUTTON_OFF;
    lv_color_t text_color    = ac_master_power ? lv_color_white()  : COLOR_TEXT_OFF;
    lv_color_t muted_color   = ac_master_power ? COLOR_TEXT_MUTED  : COLOR_TEXT_OFF;

    if (left_panel)       lv_obj_set_style_bg_color(left_panel,       panel_color,   0);
    if (right_panel)      lv_obj_set_style_bg_color(right_panel,      panel_color,   0);
    if (label_room_temp)  lv_obj_set_style_text_color(label_room_temp,  muted_color, 0);
    if (label_target_temp)lv_obj_set_style_text_color(label_target_temp,text_color,  0);

    if (btn_power)
        lv_obj_set_style_bg_color(btn_power,
            ac_master_power ? lv_palette_main(LV_PALETTE_GREEN) : COLOR_ZONE_DISABLED, 0);
    if (lbl_power) {
        lv_obj_set_style_text_color(lbl_power, lv_color_white(), 0);
        lv_label_set_text(lbl_power, ac_master_power ? "Power: ON" : "Power: OFF");
    }
    if (lbl_btn_mode) {
        lv_label_set_text_fmt(lbl_btn_mode, "Mode\n%s", mode_names[ac_current_mode]);
        lv_obj_set_style_text_color(lbl_btn_mode, text_color, 0);
    }
    if (lbl_btn_fan) {
        lv_label_set_text_fmt(lbl_btn_fan, "Fan\n%s", fan_names[ac_current_fan]);
        lv_obj_set_style_text_color(lbl_btn_fan, text_color, 0);
    }
    lv_obj_t *mode_btn = lbl_btn_mode ? lv_obj_get_parent(lbl_btn_mode) : NULL;
    lv_obj_t *fan_btn  = lbl_btn_fan  ? lv_obj_get_parent(lbl_btn_fan)  : NULL;
    if (mode_btn) lv_obj_set_style_bg_color(mode_btn, control_color, 0);
    if (fan_btn)  lv_obj_set_style_bg_color(fan_btn,  control_color, 0);

    for (int i = 0; i < 5; i++) refresh_zone_button(i);
}

// ==================== 9. LVGL Sniffer Status Bar Timer ====================
// Called from the LVGL task every 500 ms — safe to update widgets here.
static void sniffer_status_timer_cb(lv_timer_t *timer) {
    if (!lbl_sniffer_bar) return;

    // Snapshot volatile fields (no mutex needed — single-byte or word reads)
    int   idx   = g_sniffer.cfg_idx;
    if (idx < 0 || idx >= RS485_NUM_CONFIGS) idx = 0;

    uint32_t valid   = g_sniffer.pkts_valid;
    uint32_t invalid = g_sniffer.pkts_invalid;
    uint32_t bytes   = g_sniffer.bytes_total;
    bool     st_ok   = g_sniffer.self_test_ok;

#if RS485_SELF_TEST
    // Show self-test result prominently until real traffic arrives
    if (bytes == 0) {
        lv_label_set_text_fmt(lbl_sniffer_bar,
            "SELF-TEST: %s | waiting…",
            st_ok ? "PASS" : "pending");
        lv_obj_set_style_text_color(lbl_sniffer_bar,
            st_ok ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_YELLOW), 0);
        return;
    }
#endif

    const char *lbl = rs485_configs[idx].label;
    lv_label_set_text_fmt(lbl_sniffer_bar,
        "%s | B:%lu V:%lu I:%lu",
        lbl, (unsigned long)bytes,
        (unsigned long)valid, (unsigned long)invalid);

    // Color: green = got valid pkts, orange = data but invalid, grey = no data
    if (valid > 0)
        lv_obj_set_style_text_color(lbl_sniffer_bar, lv_palette_main(LV_PALETTE_GREEN), 0);
    else if (bytes > 0)
        lv_obj_set_style_text_color(lbl_sniffer_bar, lv_palette_main(LV_PALETTE_ORANGE), 0);
    else
        lv_obj_set_style_text_color(lbl_sniffer_bar, COLOR_TEXT_MUTED, 0);
}

// ==================== 10. Main Dashboard UI Build ====================
void create_air_con_dashboard(void) {
    lv_obj_set_style_bg_color(lv_scr_act(), COLOR_BG, 0);

    // ── Left panel ──────────────────────────────────────────────────────────
    left_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(left_panel, lv_pct(45), lv_pct(90));
    lv_obj_align(left_panel, LV_ALIGN_LEFT_MID, lv_pct(3), 0);
    lv_obj_set_style_bg_color(left_panel, COLOR_PANEL_ON, 0);
    lv_obj_set_style_border_width(left_panel, 0, 0);
    lv_obj_set_style_radius(left_panel, 12, 0);

    // Power button
    btn_power = lv_btn_create(left_panel);
    lv_obj_set_size(btn_power, lv_pct(86), lv_pct(14));
    lv_obj_align(btn_power, LV_ALIGN_TOP_MID, 0, lv_pct(4));
    lv_obj_set_style_bg_color(btn_power, lv_palette_main(LV_PALETTE_GREEN), 0);
    lbl_power = lv_label_create(btn_power);
    lv_label_set_text(lbl_power, "Power: ON");
    lv_obj_set_style_text_font(lbl_power, UI_FONT_CN, 0);
    lv_obj_center(lbl_power);
    lv_obj_add_event_cb(btn_power, power_btn_cb, LV_EVENT_CLICKED, NULL);

    // Room temp display
    label_room_temp = lv_label_create(left_panel);
    lv_label_set_text(label_room_temp, "Room temp: 21.5\xC2\xB0" "C");
    lv_obj_set_style_text_font(label_room_temp, UI_FONT_CN, 0);
    lv_obj_set_style_text_color(label_room_temp, COLOR_TEXT_MUTED, 0);
    lv_obj_align(label_room_temp, LV_ALIGN_CENTER, 0, lv_pct(-24));

    // Target temp
    label_target_temp = lv_label_create(left_panel);
    lv_label_set_text_fmt(label_target_temp, "%d\xC2\xB0" "C", ac_target_temp);
    lv_obj_set_style_text_font(label_target_temp, &lv_font_montserrat_30, 0);
    lv_obj_set_style_text_color(label_target_temp, lv_color_white(), 0);
    lv_obj_align(label_target_temp, LV_ALIGN_CENTER, 0, lv_pct(-10));

    // Temp ± buttons
    lv_obj_t *btn_minus = lv_btn_create(left_panel);
    lv_obj_set_size(btn_minus, lv_pct(22), lv_pct(13));
    lv_obj_align(btn_minus, LV_ALIGN_CENTER, lv_pct(-30), lv_pct(-10));
    lv_obj_set_style_bg_color(btn_minus, lv_color_make(45, 50, 65), 0);
    lv_obj_t *lbl_minus = lv_label_create(btn_minus);
    lv_label_set_text(lbl_minus, "-");
    lv_obj_set_style_text_font(lbl_minus, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_minus);
    lv_obj_add_event_cb(btn_minus, temp_btn_cb, LV_EVENT_CLICKED, (void *)"-");

    lv_obj_t *btn_plus = lv_btn_create(left_panel);
    lv_obj_set_size(btn_plus, lv_pct(22), lv_pct(13));
    lv_obj_align(btn_plus, LV_ALIGN_CENTER, lv_pct(30), lv_pct(-10));
    lv_obj_set_style_bg_color(btn_plus, lv_color_make(45, 50, 65), 0);
    lv_obj_t *lbl_plus = lv_label_create(btn_plus);
    lv_label_set_text(lbl_plus, "+");
    lv_obj_set_style_text_font(lbl_plus, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_plus);
    lv_obj_add_event_cb(btn_plus, temp_btn_cb, LV_EVENT_CLICKED, (void *)"+");

    // Mode / Fan square buttons
    lv_obj_t *btn_mode = lv_btn_create(left_panel);
    lv_obj_set_size(btn_mode, 128, 128);
    lv_obj_align(btn_mode, LV_ALIGN_BOTTOM_LEFT, lv_pct(8), lv_pct(-5));
    lv_obj_set_style_bg_color(btn_mode, COLOR_BUTTON_ON, 0);
    lbl_btn_mode = lv_label_create(btn_mode);
    lv_label_set_text_fmt(lbl_btn_mode, "Mode\n%s", mode_names[ac_current_mode]);
    lv_obj_set_style_text_font(lbl_btn_mode, UI_FONT_CN, 0);
    lv_obj_set_style_text_align(lbl_btn_mode, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_btn_mode);
    lv_obj_add_event_cb(btn_mode, mode_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_fan = lv_btn_create(left_panel);
    lv_obj_set_size(btn_fan, 128, 128);
    lv_obj_align(btn_fan, LV_ALIGN_BOTTOM_RIGHT, lv_pct(-8), lv_pct(-5));
    lv_obj_set_style_bg_color(btn_fan, COLOR_BUTTON_ON, 0);
    lbl_btn_fan = lv_label_create(btn_fan);
    lv_label_set_text_fmt(lbl_btn_fan, "Fan\n%s", fan_names[ac_current_fan]);
    lv_obj_set_style_text_font(lbl_btn_fan, UI_FONT_CN, 0);
    lv_obj_set_style_text_align(lbl_btn_fan, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_btn_fan);
    lv_obj_add_event_cb(btn_fan, fan_btn_cb, LV_EVENT_CLICKED, NULL);

    // ── Right panel ─────────────────────────────────────────────────────────
    right_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(right_panel, lv_pct(45), lv_pct(90));
    lv_obj_align(right_panel, LV_ALIGN_RIGHT_MID, lv_pct(-3), 0);
    lv_obj_set_style_bg_color(right_panel, COLOR_PANEL_ON, 0);
    lv_obj_set_style_border_width(right_panel, 0, 0);
    lv_obj_set_style_radius(right_panel, 12, 0);

    lv_obj_t *zone_title = lv_label_create(right_panel);
    lv_label_set_text(zone_title, "Zone Control");
    lv_obj_set_style_text_font(zone_title, UI_FONT_CN, 0);
    lv_obj_set_style_text_color(zone_title, lv_palette_main(LV_PALETTE_LIGHT_BLUE), 0);
    lv_obj_align(zone_title, LV_ALIGN_TOP_MID, 0, lv_pct(4));

    // Zone buttons — 5 buttons at 11% height each, starting at 13% top offset
    for (int i = 0; i < 5; i++) {
        btn_zones[i] = lv_btn_create(right_panel);
        lv_obj_set_size(btn_zones[i], lv_pct(86), lv_pct(11));
        lv_obj_align(btn_zones[i], LV_ALIGN_TOP_MID, 0, lv_pct(13 + (i * 14)));
        lbl_zones[i] = lv_label_create(btn_zones[i]);
        lv_obj_set_style_text_font(lbl_zones[i], UI_FONT_CN, 0);
        lv_obj_center(lbl_zones[i]);
        lv_obj_add_event_cb(btn_zones[i], zone_btn_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    // ── RS485 Sniffer Status Bar (below zone buttons) ───────────────────────
    // Sits at ~83% from top of right panel (5 zones × 14% + 13% start = 83%)
    lv_obj_t *sniffer_bar = lv_obj_create(right_panel);
    lv_obj_set_size(sniffer_bar, lv_pct(90), lv_pct(9));
    lv_obj_align(sniffer_bar, LV_ALIGN_TOP_MID, 0, lv_pct(84));
    lv_obj_set_style_bg_color(sniffer_bar, COLOR_SNIFFER_BG, 0);
    lv_obj_set_style_border_color(sniffer_bar, lv_palette_main(LV_PALETTE_BLUE_GREY), 0);
    lv_obj_set_style_border_width(sniffer_bar, 1, 0);
    lv_obj_set_style_radius(sniffer_bar, 6, 0);
    lv_obj_set_style_pad_all(sniffer_bar, 4, 0);

    lbl_sniffer_bar = lv_label_create(sniffer_bar);
    lv_obj_set_style_text_font(lbl_sniffer_bar, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_sniffer_bar, COLOR_TEXT_MUTED, 0);
    lv_label_set_long_mode(lbl_sniffer_bar, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(lbl_sniffer_bar, lv_pct(100));
    lv_label_set_text(lbl_sniffer_bar, "RS485 sniffer starting...");
    lv_obj_align(lbl_sniffer_bar, LV_ALIGN_LEFT_MID, 0, 0);

    // LVGL timer: refresh status bar every 500 ms from within the LVGL task
    lv_timer_create(sniffer_status_timer_cb, 500, NULL);

    refresh_air_con_ui();
}

// ==================== 11. Packet Validation Heuristic ====================
// A "packet" is a burst of bytes separated by a silence gap of >5 ms.
// Valid if: length >= 3, first byte repeats across multiple packets (master address),
// or last byte could be a checksum (XOR/sum of prior bytes ≈ last byte).
static bool validate_packet(const uint8_t *buf, int len) {
    if (len < 3) return false;
    // Heuristic 1: XOR checksum of all bytes except last == last byte
    uint8_t xor_sum = 0;
    for (int i = 0; i < len - 1; i++) xor_sum ^= buf[i];
    if (xor_sum == buf[len - 1]) return true;
    // Heuristic 2: arithmetic sum (low byte) of all bytes except last == last byte
    uint8_t arith_sum = 0;
    for (int i = 0; i < len - 1; i++) arith_sum += buf[i];
    if (arith_sum == buf[len - 1]) return true;
    // Heuristic 3: reasonable length (4–32 bytes) with non-all-zero content
    if (len >= 4 && len <= 32) {
        bool all_zero = true;
        for (int i = 0; i < len; i++) if (buf[i] != 0) { all_zero = false; break; }
        if (!all_zero) return true;
    }
    return false;
}

// ==================== 12. RS485 Sniffer FreeRTOS Task (Core 1) ====================
#if ENABLE_RS485_SNIFFER
void TaskRS485Sniffer(void *pvParameters) {
    (void)pvParameters;

    // ── Self-test: TX known bytes via Serial2, receive on Serial1 ──────────
#if RS485_SELF_TEST
    // Serial1 = RX on pin 44, TX on pin 43 (sniffer UART, receive-only in production)
    // Serial2 = TX on pin 43 only for self-test — wire pin 43 to pin 44 externally.
    // Self-test runs at 9600 8N1 for simplicity.
    Serial.println("[Sniffer] Self-test: begin (wire pin 43 → pin 44 for loopback)");
    Serial2.begin(9600, SERIAL_8N1, -1, RS485_TX_PIN); // TX=43, no RX
    Serial1.begin(9600, SERIAL_8N1, RS485_RX_PIN, -1); // RX=44, no TX
    vTaskDelay(pdMS_TO_TICKS(100));

    // Transmit a known sequence
    const uint8_t test_seq[] = {0xAA, 0x01, 0x02, 0x03, 0xAB};
    Serial2.write(test_seq, sizeof(test_seq));
    Serial2.flush();
    vTaskDelay(pdMS_TO_TICKS(50));

    int matched = 0;
    int test_idx = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 500) {
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (test_idx < (int)sizeof(test_seq) && b == test_seq[test_idx]) {
                test_idx++;
                if (test_idx == (int)sizeof(test_seq)) { matched = 1; break; }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    g_sniffer.self_test_ok = (matched == 1);
    Serial.printf("[Sniffer] Self-test result: %s\n", g_sniffer.self_test_ok ? "PASS" : "FAIL (pin 43→44 not connected?)");

    Serial1.end();
    Serial2.end();
    vTaskDelay(pdMS_TO_TICKS(50));
#endif // RS485_SELF_TEST

    // ── Main sniff loop ────────────────────────────────────────────────────
    int cfg_idx = 0;
    g_sniffer.cfg_idx = cfg_idx;

    Serial1.begin(rs485_configs[cfg_idx].baud,
                  rs485_configs[cfg_idx].serial_cfg,
                  RS485_RX_PIN, -1); // RX only, no TX
    Serial.printf("[Sniffer] Start scanning: %s\n", rs485_configs[cfg_idx].desc);

    // Per-slot state
    uint32_t slot_bytes   = 0;
    uint32_t slot_valid   = 0;
    uint32_t slot_invalid = 0;

    // Packet assembly buffer
    static uint8_t pkt_buf[64];
    int pkt_len = 0;
    unsigned long last_byte_time = millis();
    const uint32_t PKT_GAP_MS = 5; // silence > 5 ms = end of packet

    unsigned long slot_start = millis();

    for (;;) {
        // ── Read incoming bytes ────────────────────────────────────────────
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            last_byte_time = millis();
            slot_bytes++;
            g_sniffer.bytes_total++;

            // Print hex to Serial monitor
            if (b < 0x10) Serial.print('0');
            Serial.print(b, HEX);
            Serial.print(' ');

            if (pkt_len < (int)sizeof(pkt_buf)) {
                pkt_buf[pkt_len++] = b;
            }
        }

        // ── Detect end of packet by silence gap ───────────────────────────
        if (pkt_len > 0 && (millis() - last_byte_time) > PKT_GAP_MS) {
            bool ok = validate_packet(pkt_buf, pkt_len);
            if (ok) { slot_valid++;   g_sniffer.pkts_valid++;   }
            else    { slot_invalid++; g_sniffer.pkts_invalid++; }

            Serial.printf("\n[Sniffer] PKT len=%d %s\n", pkt_len, ok ? "VALID" : "invalid");
            pkt_len = 0;
        }

        // ── Advance to next config every SCAN_INTERVAL_MS ─────────────────
        if (millis() - slot_start > SCAN_INTERVAL_MS) {
            Serial.printf("\n[Sniffer] Slot done: %s | bytes=%lu valid=%lu invalid=%lu\n",
                rs485_configs[cfg_idx].desc,
                (unsigned long)slot_bytes,
                (unsigned long)slot_valid,
                (unsigned long)slot_invalid);

            cfg_idx = (cfg_idx + 1) % RS485_NUM_CONFIGS;
            g_sniffer.cfg_idx   = cfg_idx;
            g_sniffer.slots_scanned++;

            // Reset per-slot counters
            slot_bytes = slot_valid = slot_invalid = 0;
            pkt_len = 0;

            Serial1.end();
            vTaskDelay(pdMS_TO_TICKS(30));
            Serial1.begin(rs485_configs[cfg_idx].baud,
                          rs485_configs[cfg_idx].serial_cfg,
                          RS485_RX_PIN, -1);
            Serial.printf("[Sniffer] -> %s\n", rs485_configs[cfg_idx].desc);
            slot_start = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif // ENABLE_RS485_SNIFFER

// ==================== 13. setup() ====================
void setup() {
    Serial.begin(115200);
    Serial.println("=== AC Controller + RS485 Sniffer ===");
    Serial.printf("[Config] RS485 RX=pin%d  TX=pin%d  configs=%d  self_test=%s\n",
        RS485_RX_PIN, RS485_TX_PIN, RS485_NUM_CONFIGS,
        RS485_SELF_TEST ? "ON (wire 43->44)" : "OFF");

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    if (lcd != nullptr) {
        lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
        auto lcd_bus = lcd->getBus();
        if (lcd_bus != nullptr &&
            lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(
                lcd->getFrameWidth() * 10);
        }
#endif
    }
#endif

    assert(board->begin());

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    Serial.println("Creating UI");
    lvgl_port_lock(-1);
    lv_obj_set_size(lv_scr_act(), lv_pct(100), lv_pct(100));
    create_air_con_dashboard();
    lvgl_port_unlock();

#if ENABLE_RS485_SNIFFER
    xTaskCreatePinnedToCore(
        TaskRS485Sniffer,
        "RS485Sniffer",
        6144,   // stack — slightly larger for loopback test buffers
        NULL,
        3,
        NULL,
        1       // Core 1 (LVGL runs on Core 0 via Arduino)
    );
    Serial.println("[System] RS485 sniffer task started on Core 1");
#else
    Serial.println("[System] RS485 sniffer disabled");
#endif

    Serial.println("[System] Ready.");
}

// ==================== 14. loop() ====================
void loop() {
    // Everything is handled by FreeRTOS tasks and LVGL timer callbacks.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

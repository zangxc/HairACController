#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "driver/uart.h"   // for uart_set_line_inverse / UART_SIGNAL_RXD_INV

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ==================== 1. RS485 Sniffer Configuration ====================
#define ENABLE_RS485_SNIFFER   1
#define RS485_RX_PIN           43   // GPIO43 = RS485_RXD per Waveshare schematic
#define RS485_TX_PIN           44   // GPIO44 = RS485_TXD per Waveshare schematic

// Set to 1 if the line sits LOW at idle (A/B wires swapped).
// The raw sampler will invert its GPIO reads, and the UART will use
// hardware RX inversion so you don't need to physically swap wires.
// Start with 0; if raw output shows L60000 (always LOW), set to 1.
#define RS485_INVERTED         0

// ==================== 2. RS485 Config Table ====================
// Baud is confirmed 9600 via logic analyser (104µs bit period).
// Exhaust all parity/stop-bit combos at 9600 only.
struct RS485Config {
    uint32_t    baud;
    uint32_t    serial_cfg;
    const char* label;
    const char* desc;
};

static const RS485Config rs485_configs[] = {
    { 9600, SERIAL_8N1, "8N1", "9600 8N1 (no parity, 1 stop)"    },
    { 9600, SERIAL_8N2, "8N2", "9600 8N2 (no parity, 2 stop)"    },
    { 9600, SERIAL_8E1, "8E1", "9600 8E1 (even parity, 1 stop)"  },
    { 9600, SERIAL_8E2, "8E2", "9600 8E2 (even parity, 2 stop)"  },
    { 9600, SERIAL_8O1, "8O1", "9600 8O1 (odd parity,  1 stop)"  },
    { 9600, SERIAL_8O2, "8O2", "9600 8O2 (odd parity,  2 stop)"  },
};
static const int RS485_NUM_CONFIGS = sizeof(rs485_configs) / sizeof(rs485_configs[0]);
static const uint32_t SCAN_INTERVAL_MS = 8000; // 8 s per config (bus is ~1 msg/s)

// ==================== 3. Shared State (tasks → UI) ====================
struct SnifferStatus {
    volatile int      cfg_idx;
    volatile uint32_t bytes_total;
    volatile uint32_t pkts_valid;
    volatile uint32_t pkts_invalid;
    volatile uint32_t slots_scanned;
    volatile bool     self_test_ok;
    // Raw sampler fields — written by sampler task
    volatile uint32_t raw_captures;   // how many triggered captures done
    volatile uint32_t raw_bit_us;     // last measured bit period in µs
    volatile uint8_t  raw_frame_bits; // last decoded frame: bit count hint (8 or 9)
    char              status_line[80];
};
static SnifferStatus g_sniffer = {};

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

    int      idx     = g_sniffer.cfg_idx;
    if (idx < 0 || idx >= RS485_NUM_CONFIGS) idx = 0;
    uint32_t valid   = g_sniffer.pkts_valid;
    uint32_t invalid = g_sniffer.pkts_invalid;
    uint32_t caps    = g_sniffer.raw_captures;
    uint32_t bit_us  = g_sniffer.raw_bit_us;

    const char *lbl = rs485_configs[idx].label;

    if (caps == 0) {
        lv_label_set_text_fmt(lbl_sniffer_bar,
            "UART:%s V:%lu I:%lu | RAW: waiting...",
            lbl, (unsigned long)valid, (unsigned long)invalid);
        lv_obj_set_style_text_color(lbl_sniffer_bar, COLOR_TEXT_MUTED, 0);
    } else {
        lv_label_set_text_fmt(lbl_sniffer_bar,
            "UART:%s V:%lu I:%lu | RAW:%luus(%lubaud) #%lu",
            lbl,
            (unsigned long)valid, (unsigned long)invalid,
            (unsigned long)bit_us,
            bit_us > 0 ? (unsigned long)(1000000UL / bit_us) : 0UL,
            (unsigned long)caps);
        lv_obj_set_style_text_color(lbl_sniffer_bar,
            valid > 0 ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_ORANGE), 0);
    }
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
// Gap-based framing: silence > PKT_GAP_MS after last byte = packet boundary.
// PKT_GAP_MS must be >> one byte time at the current baud:
//   9600 baud  → 1 byte ≈ 1.04 ms → gap 30 ms is safe
//   4800 baud  → 1 byte ≈ 2.08 ms → gap 30 ms is safe
//   38400 baud → 1 byte ≈ 0.26 ms → gap 30 ms is safe
// 30 ms covers even protocols that stretch inter-byte gaps to ~3-4 ms.
static const uint32_t PKT_GAP_MS = 30;

// Framing noise filter: RS485 idle line can produce 0x00 or 0xFF framing
// error bytes at wrong baud/parity. Skip lone 0x00/0xFF before reassembling.
static inline bool is_framing_noise(const uint8_t *buf, int len) {
    if (len > 2) return false; // real packet, not noise
    for (int i = 0; i < len; i++)
        if (buf[i] != 0x00 && buf[i] != 0xFF) return false;
    return true; // all bytes are 0x00 or 0xFF → likely framing error on idle line
}

static bool validate_packet(const uint8_t *buf, int len) {
    if (len < 3) return false;
    if (is_framing_noise(buf, len)) return false;

    // H1: XOR of all bytes (including last) == 0  (common in HVAC)
    uint8_t xor_all = 0;
    for (int i = 0; i < len; i++) xor_all ^= buf[i];
    if (xor_all == 0x00) return true;

    // H2: XOR checksum — XOR of bytes[0..n-2] == bytes[n-1]
    uint8_t xor_sum = 0;
    for (int i = 0; i < len - 1; i++) xor_sum ^= buf[i];
    if (xor_sum == buf[len - 1]) return true;

    // H3: Arithmetic sum (low byte) of bytes[0..n-2] == bytes[n-1]
    uint8_t arith_sum = 0;
    for (int i = 0; i < len - 1; i++) arith_sum += buf[i];
    if (arith_sum == buf[len - 1]) return true;

    // H4: Arithmetic sum + 1 (Mitsubishi/some Daikin variants)
    if ((uint8_t)(arith_sum + 1) == buf[len - 1]) return true;

    // H5: Packet is plausible length (4–48 bytes) AND has structural byte variation
    // (not all identical, not random noise = some bytes repeat like address fields)
    if (len >= 4 && len <= 48) {
        int unique = 0;
        uint8_t seen[256] = {0};
        for (int i = 0; i < len; i++) { if (!seen[buf[i]]) { seen[buf[i]] = 1; unique++; } }
        // Real protocol frames have 2–20 unique byte values; pure noise has many
        if (unique >= 2 && unique <= 20) return true;
    }

    return false;
}


// ==================== 12. Raw GPIO Sampler Task ====================
// Edge-triggered: waits for falling edge on pin 44, then captures 60ms
// at 100kHz (10µs/sample = ~10 samples per 104µs bit at 9600 baud).
// Decodes each UART frame and reports which config (8N1/8E1/8O1/8N2...) fits.
// Runs on Core 1. Pauses Serial1 during the 60ms capture window, then
// signals the UART sniffer task to re-open Serial1.

#define RAW_SAMPLE_RATE_US  10
#define RAW_CAPTURE_MS      60
#define RAW_MAX_SAMPLES     (RAW_CAPTURE_MS * 1000 / RAW_SAMPLE_RATE_US)  // 6000

// Global buffers (not on task stack — 30KB total, fine in PSRAM-extended heap)
static uint8_t  raw_buf[RAW_MAX_SAMPLES];   // 6 KB
static uint32_t raw_ts[RAW_MAX_SAMPLES];    // 24 KB

// Coordination: sampler pauses UART task during capture
static volatile bool g_sampler_active = false;

void TaskRawSampler(void *pvParameters) {
    (void)pvParameters;
    Serial.printf("[Raw] GPIO sampler on pin %d ready\n", RS485_RX_PIN);

    for (;;) {
        // ── 1. Wait for falling edge (start bit) with 3s timeout ─────────
        pinMode(RS485_RX_PIN, INPUT);  // ensure GPIO mode while waiting
        uint32_t wait_start = millis();
        bool got_edge = false;
        while (millis() - wait_start < 3000) {
#if RS485_INVERTED
            if (digitalRead(RS485_RX_PIN) == HIGH) { got_edge = true; break; } // inverted: idle=LOW, start=HIGH
#else
            if (digitalRead(RS485_RX_PIN) == LOW)  { got_edge = true; break; } // normal:   idle=HIGH, start=LOW
#endif
            taskYIELD();
        }

        if (!got_edge) {
            Serial.println("[Raw] No edge in 3s. Check: A/B polarity? RE/DE pin grounded? Vcc?");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // ── 2. Signal UART task to stop (it will end Serial1) ────────────
        g_sampler_active = true;
        vTaskDelay(pdMS_TO_TICKS(5)); // let UART task notice and end Serial1

        // Re-assert GPIO input (Serial1.end may have changed pin mode)
        pinMode(RS485_RX_PIN, INPUT);

        // ── 3. Capture 60ms at 100kHz ─────────────────────────────────────
        int n = 0;
        uint64_t t0     = esp_timer_get_time();
        uint64_t t_next = t0;
        while (n < RAW_MAX_SAMPLES) {
            while ((int64_t)(esp_timer_get_time() - t_next) < 0) {}  // busy-wait
#if RS485_INVERTED
            raw_buf[n] = (uint8_t)(digitalRead(RS485_RX_PIN) ^ 1);  // flip: HIGH=0, LOW=1
#else
            raw_buf[n] = (uint8_t)digitalRead(RS485_RX_PIN);
#endif
            raw_ts[n]  = (uint32_t)(esp_timer_get_time() - t0);
            t_next += RAW_SAMPLE_RATE_US;
            n++;
        }
        g_sniffer.raw_captures++;
        g_sampler_active = false;  // release UART task

        // ── 4. Run-length encode for waveform printout ───────────────────
        Serial.printf("\n=== RAW #%lu (pin%d, 100kHz, 60ms) ===\n",
            (unsigned long)g_sniffer.raw_captures, RS485_RX_PIN);

        uint8_t  prev = raw_buf[0];
        int      run  = 1;
        uint32_t min_run = 99999, max_run = 0, run_cnt = 0;
        bool     any_low = false;
        char     line[120]; int lpos = snprintf(line, sizeof(line), "  ");

        for (int i = 1; i <= n; i++) {
            bool flush = (i == n) || (raw_buf[i] != prev);
            if (flush) {
                uint32_t us = (uint32_t)run * RAW_SAMPLE_RATE_US;
                if (run >= 3) {
                    if (us < min_run) min_run = us;
                    if (us > max_run) max_run = us;
                    run_cnt++;
                }
                if (!prev) any_low = true;
                lpos += snprintf(line+lpos, sizeof(line)-lpos, "%c%lu ", prev?'H':'L', (unsigned long)us);
                if (lpos > 90) { Serial.println(line); lpos = snprintf(line, sizeof(line), "  "); }
                if (i < n) { prev = raw_buf[i]; run = 1; }
            } else { run++; }
        }
        if (lpos > 2) Serial.println(line);

        if (!any_low) {
            Serial.println("  *** No signal transitions in 60ms capture ***");
#if RS485_INVERTED
            Serial.println("  RS485_INVERTED=1: idle=LOW expected, got all HIGH.");
            Serial.println("  -> Try RS485_INVERTED=0 (standard polarity)");
#else
            Serial.println("  RS485_INVERTED=0: idle=HIGH expected, got all LOW.");
            Serial.println("  -> Try RS485_INVERTED=1 (swap A/B in software)");
            Serial.println("  -> Or physically swap A and B wires on RS485 module");
#endif
            Serial.println("  Also check: RE/DE pin LOW? Module Vcc? Pin 44 connected?");
            g_sniffer.raw_bit_us = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // ── 5. Estimate bit period ────────────────────────────────────────
        if (min_run <  50) min_run = 104;  // sub-50µs = glitch, floor at 9600 baud
        if (min_run > 500) min_run = 104;  // >500µs = below 2000 baud, unexpected
        g_sniffer.raw_bit_us = min_run;
        uint32_t baud_est = 1000000UL / min_run;
        Serial.printf("  bit=%luus  baud~%lu  runs=%lu  min=%luus  max=%luus\n",
            (unsigned long)min_run, (unsigned long)baud_est,
            (unsigned long)run_cnt, (unsigned long)min_run, (unsigned long)max_run);

        // ── 6. Soft-decode UART frames ────────────────────────────────────
        int  spb  = (int)(min_run / RAW_SAMPLE_RATE_US);  // samples per bit (~10)
        if (spb < 1) spb = 1;
        int  half = spb / 2;
        int  frames = 0;
        Serial.printf("  Frames (spb=%d):\n", spb);

        for (int si = 1; si < n - spb * 13; si++) {
            // Falling edge = start bit candidate
            if (raw_buf[si-1] != 1 || raw_buf[si] != 0) continue;
            // Verify start bit centre is LOW
            if (si + half >= n || raw_buf[si + half] != 0) continue;

            // Sample bits: positions relative to start edge
            // bit slot k centre = si + half + k*spb
            // k=0: start (verified above)
            // k=1..8: data bits d0..d7
            // k=9: either parity or stop1
            // k=10: stop1 or stop2
            uint16_t samp = 0;
            bool ok = true;
            for (int k = 1; k <= 11; k++) {
                int idx = si + half + k * spb;
                if (idx >= n) { ok = false; break; }
                if (raw_buf[idx]) samp |= (1 << (k-1));
            }
            if (!ok) break;

            uint8_t data = (uint8_t)(samp & 0xFF);          // bits k=1..8
            uint8_t b8   = (samp >> 8)  & 1;                // k=9
            uint8_t b9   = (samp >> 9)  & 1;                // k=10
            uint8_t b10  = (samp >> 10) & 1;                // k=11

            uint8_t ones = (uint8_t)__builtin_popcount(data);
            // 8N1: b8 must be 1 (stop)
            bool n1 = (b8 == 1);
            // 8N2: b8 and b9 both 1
            bool n2 = (b8 == 1 && b9 == 1);
            // 8E1: (ones + b8) is even → b8 = ones%2
            bool ep = (b8 == (ones & 1));
            // 8O1: (ones + b8) is odd → b8 != ones%2
            bool op = (b8 != (ones & 1));
            // 8E2: even parity then stop → ep && b9==1
            bool ep2 = (ep && b9 == 1);
            // 8O2: odd parity then stop
            bool op2 = (op && b9 == 1);

            Serial.printf("    0x%02X '%c'  b8=%d b9=%d b10=%d | 8N1:%s 8N2:%s 8E1:%s 8E2:%s 8O1:%s 8O2:%s\n",
                data, (data >= 0x20 && data < 0x7F) ? (char)data : '.',
                b8, b9, b10,
                n1 ?"OK":"--", n2 ?"OK":"--",
                ep ?"OK":"--", ep2?"OK":"--",
                op ?"OK":"--", op2?"OK":"--");

            frames++;
            si += spb * 10;  // skip rest of frame
        }
        if (frames == 0)
            Serial.println("  (0 frames decoded — signal may be inverted or too noisy)");

        Serial.printf("=== END RAW #%lu (%d frames) ===\n\n",
            (unsigned long)g_sniffer.raw_captures, frames);

        vTaskDelay(pdMS_TO_TICKS(200));  // brief pause before next trigger-wait
    }
}

// ==================== 13. UART Sniffer Task ====================
// Cycles through all 6 parity/stop combos at 9600 baud.
// Suspends Serial1 while the raw sampler is capturing.
#if ENABLE_RS485_SNIFFER
void TaskRS485Sniffer(void *pvParameters) {
    (void)pvParameters;

    int      cfg_idx    = 0;
    g_sniffer.cfg_idx   = cfg_idx;
    bool     uart_open  = false;

    auto open_uart = [&]() {
        Serial1.begin(rs485_configs[cfg_idx].baud,
                      rs485_configs[cfg_idx].serial_cfg,
                      RS485_RX_PIN, -1);
#if RS485_INVERTED
        // Hardware RX signal inversion — no need to physically swap A/B wires
        uart_set_line_inverse(1, UART_SIGNAL_RXD_INV);
#endif
        uart_open = true;
    };
    auto close_uart = [&]() {
        Serial1.end();
        uart_open = false;
    };

    open_uart();
    Serial.printf("[UART] Start: %s\n", rs485_configs[cfg_idx].desc);

    uint32_t slot_bytes = 0, slot_valid = 0, slot_invalid = 0;
    static uint8_t pkt_buf[128];
    int pkt_len = 0;
    unsigned long last_byte_ms = millis();
    unsigned long slot_start   = millis();

    for (;;) {
        // Yield pin to raw sampler when it's active
        if (g_sampler_active) {
            if (uart_open) close_uart();
            while (g_sampler_active) vTaskDelay(pdMS_TO_TICKS(5));
            open_uart();
        }

        // Read bytes
        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            last_byte_ms = millis();
            slot_bytes++;
            g_sniffer.bytes_total++;
            if (pkt_len == 0) Serial.printf("[%s] ", rs485_configs[cfg_idx].label);
            Serial.printf("%02X ", b);
            if (pkt_len < (int)sizeof(pkt_buf)) pkt_buf[pkt_len++] = b;
        }

        // Packet boundary on silence
        if (pkt_len > 0 && (millis() - last_byte_ms) > PKT_GAP_MS) {
            bool noise = is_framing_noise(pkt_buf, pkt_len);
            bool ok    = !noise && validate_packet(pkt_buf, pkt_len);
            if (!noise) {
                if (ok) { slot_valid++;   g_sniffer.pkts_valid++;   }
                else    { slot_invalid++; g_sniffer.pkts_invalid++; }
                Serial.printf(" ← len=%d %s\n", pkt_len, ok ? "OK" : "inv");
            }
            pkt_len = 0;
        }

        // Rotate config
        if (millis() - slot_start > SCAN_INTERVAL_MS) {
            Serial.printf("[UART] %s done: B=%lu V=%lu I=%lu\n",
                rs485_configs[cfg_idx].label,
                (unsigned long)slot_bytes,
                (unsigned long)slot_valid,
                (unsigned long)slot_invalid);
            cfg_idx = (cfg_idx + 1) % RS485_NUM_CONFIGS;
            g_sniffer.cfg_idx = cfg_idx;
            g_sniffer.slots_scanned++;
            slot_bytes = slot_valid = slot_invalid = pkt_len = 0;
            close_uart();
            vTaskDelay(pdMS_TO_TICKS(30));
            open_uart();
            Serial.printf("[UART] -> %s\n", rs485_configs[cfg_idx].desc);
            slot_start = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif

// ==================== 14. setup() ====================
void setup() {
    Serial.begin(115200);
    Serial.println("=== AC Controller + RS485 Dual Sniffer ===");
    Serial.printf("[Config] RX=pin%d  TX=pin%d\n", RS485_RX_PIN, RS485_TX_PIN);
    Serial.println("[Config] Raw GPIO sampler: edge-triggered, 100kHz, 60ms window");
    Serial.printf("[Config] UART scanner: 9600 baud x %d configs x %lus each\n",
        RS485_NUM_CONFIGS, (unsigned long)(SCAN_INTERVAL_MS / 1000));

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
    lvgl_port_init(board->getLCD(), board->getTouch());

    lvgl_port_lock(-1);
    lv_obj_set_size(lv_scr_act(), lv_pct(100), lv_pct(100));
    create_air_con_dashboard();
    lvgl_port_unlock();

    // Raw GPIO sampler — higher priority so it wins the pin during capture
    xTaskCreatePinnedToCore(TaskRawSampler, "RawSampler", 8192, NULL, 5, NULL, 1);

#if ENABLE_RS485_SNIFFER
    xTaskCreatePinnedToCore(TaskRS485Sniffer, "RS485Sniffer", 6144, NULL, 3, NULL, 1);
#endif

    Serial.println("[System] Ready.");
}

// ==================== 15. loop() ====================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

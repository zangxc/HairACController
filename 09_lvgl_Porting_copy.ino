#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ==================== 1. RS485 Configuration ====================
// CONFIRMED: 9600 baud, 8N2 (no parity, 2 stop bits)
// GPIO43 = RXD (input), GPIO44 = TXD (output) — per Waveshare schematic
// Board has onboard auto-DE/RE transceiver — no manual direction pin needed.
#define ENABLE_RS485_SNIFFER   1
#define RS485_RX_PIN           43
#define RS485_TX_PIN           44
#define RS485_BAUD             9600
#define RS485_CONFIG           SERIAL_8N2   // CONFIRMED by raw GPIO analysis

// ==================== 2. Shared Sniffer State (task → UI) ====================
struct SnifferStatus {
    volatile uint32_t bytes_total;
    volatile uint32_t pkts_valid;
    volatile uint32_t pkts_invalid;
    volatile uint32_t pkts_captured;
};
static SnifferStatus g_sniffer = {};

// ==================== 3. Parsed Bus State (sniffer task → LVGL timer) ====================
// Written by sniffer task on Core 1, read by LVGL timer on Core 0.
// The dirty flag signals the UI timer to copy and refresh.
struct BusACState {
    bool     power;         // true=ON, false=OFF
    uint8_t  zone_mask;     // bits 0–4 = zones 1–5
    uint8_t  fan_speed;     // raw byte from bus
    uint8_t  mode;          // raw byte from bus
    uint16_t room_temp_raw; // byte[2], divide by 10 for °C
    uint16_t setpoint;      // byte[8], divide by 10 for °C
    uint8_t  status_flag;   // 0x40=running, 0x00=idle
    // Time from bus (0x61 frame bytes 14-15)
    uint8_t  bus_hour;
    uint8_t  bus_minute;
};
static BusACState g_bus_state = {};
static volatile bool g_bus_state_dirty = false;

// ==================== 4. Central AC State ====================
bool ac_master_power  = false;
int  ac_target_temp   = 30;
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
lv_obj_t *lbl_bus_time = NULL;     // bus time display
lv_obj_t *lbl_tx_bar = NULL;       // TX status bar label
lv_obj_t *btn_tx_toggle = NULL;    // TX enable toggle button
lv_obj_t *lbl_tx_toggle = NULL;    // TX toggle label

// ==================== 5b. TX State ====================
static bool    g_tx_enabled = false;   // Transmission disabled by default
static uint32_t g_tx_count = 0;        // Packets transmitted

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
    send_ac_command();
}

static void zone_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    int zone_idx = (int)(uintptr_t)lv_event_get_user_data(e);
    ac_zones[zone_idx] = !ac_zones[zone_idx];
    refresh_zone_button(zone_idx);
    Serial.printf("[UI] Zone %d -> %s\n", zone_idx + 1, ac_zones[zone_idx] ? "ON" : "OFF");
    send_ac_command();
}

static void mode_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    ac_current_mode = (ac_current_mode + 1) % 5;
    refresh_air_con_ui();
    Serial.printf("[UI] Mode -> %s\n", mode_names[ac_current_mode]);
    send_ac_command();
}

static void fan_btn_cb(lv_event_t *e) {
    if (!ac_master_power) return;
    ac_current_fan = (ac_current_fan + 1) % 3;
    refresh_air_con_ui();
    Serial.printf("[UI] Fan -> %s\n", fan_names[ac_current_fan]);
    send_ac_command();
}

static void power_btn_cb(lv_event_t *e) {
    ac_master_power = !ac_master_power;
    refresh_air_con_ui();
    Serial.printf("[UI] Power -> %s\n", ac_master_power ? "ON" : "OFF");
    send_ac_command();
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

    // ── Apply parsed bus state to UI if new data arrived ──
    if (g_bus_state_dirty) {
        g_bus_state_dirty = false;

        // Copy struct (single writer on Core 1, safe on ESP32 with portENTER_CRITICAL not needed for byte-aligned POD)
        BusACState s;
        memcpy(&s, (const void *)&g_bus_state, sizeof(BusACState));

        // Update global AC state variables
        ac_master_power = s.power;
        ac_current_mode = bus_mode_to_ui(s.mode);
        ac_current_fan  = bus_fan_to_ui(s.fan_speed);

        // Setpoint from bus: byte[8] ÷ 10 = °C
        int setpt = s.setpoint / 10;
        if (setpt >= 16 && setpt <= 30) {
            ac_target_temp = setpt;
        }

        // Update zone states from bitmask
        for (int i = 0; i < 5; i++) {
            ac_zones[i] = (s.zone_mask >> i) & 0x01;
        }

        // Update room temperature + time display (combined label)
        if (label_room_temp) {
            int room_whole = s.room_temp_raw / 10;
            int room_frac  = s.room_temp_raw % 10;
            if (s.bus_hour <= 23 && s.bus_minute <= 59) {
                lv_label_set_text_fmt(label_room_temp, "%02d:%02d  Room: %d.%dC",
                    s.bus_hour, s.bus_minute, room_whole, room_frac);
            } else {
                lv_label_set_text_fmt(label_room_temp, "--:--  Room: %d.%dC", room_whole, room_frac);
            }
        }

        // Update target temp display
        if (label_target_temp) {
            lv_label_set_text_fmt(label_target_temp, "%d\xC2\xB0" "C", ac_target_temp);
        }

        // Refresh all buttons/panels to reflect new state
        refresh_air_con_ui();
    }

    // ── Update sniffer packet counter bar ──
    uint32_t valid   = g_sniffer.pkts_valid;
    uint32_t invalid = g_sniffer.pkts_invalid;
    uint32_t total   = g_sniffer.pkts_captured;
    lv_label_set_text_fmt(lbl_sniffer_bar,
        "9600 8N2 | pkts:%lu V:%lu I:%lu",
        (unsigned long)total, (unsigned long)valid, (unsigned long)invalid);
    lv_obj_set_style_text_color(lbl_sniffer_bar,
        valid > 0   ? lv_palette_main(LV_PALETTE_GREEN)  :
        total > 0   ? lv_palette_main(LV_PALETTE_ORANGE) :
                      COLOR_TEXT_MUTED, 0);

    // ── Update TX status bar ──
    if (lbl_tx_bar) {
        lv_label_set_text_fmt(lbl_tx_bar, "TX: %s | sent:%lu",
            g_tx_enabled ? "READY" : "off", (unsigned long)g_tx_count);
        lv_obj_set_style_text_color(lbl_tx_bar,
            g_tx_enabled ? lv_palette_main(LV_PALETTE_RED) : COLOR_TEXT_MUTED, 0);
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
    // Room temp + bus time on same line
    label_room_temp = lv_label_create(left_panel);
    lv_label_set_text(label_room_temp, "--:--  Room: --.-C");
    lv_obj_set_style_text_font(label_room_temp, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(label_room_temp, COLOR_TEXT_MUTED, 0);
    lv_obj_align(label_room_temp, LV_ALIGN_CENTER, 0, lv_pct(-24));

    // lbl_bus_time is not needed as separate label — we combine into label_room_temp
    lbl_bus_time = NULL;

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

    // ── TX Enable Toggle (left panel, bottom-center) ────────────────────────
    btn_tx_toggle = lv_btn_create(left_panel);
    lv_obj_set_size(btn_tx_toggle, lv_pct(86), lv_pct(8));
    lv_obj_align(btn_tx_toggle, LV_ALIGN_BOTTOM_MID, 0, lv_pct(-1));
    lv_obj_set_style_bg_color(btn_tx_toggle, COLOR_ZONE_DISABLED, 0);
    lbl_tx_toggle = lv_label_create(btn_tx_toggle);
    lv_obj_set_style_text_font(lbl_tx_toggle, &lv_font_montserrat_12, 0);
    lv_label_set_text(lbl_tx_toggle, "TX: DISABLED");
    lv_obj_set_style_text_color(lbl_tx_toggle, COLOR_TEXT_MUTED, 0);
    lv_obj_center(lbl_tx_toggle);
    lv_obj_add_event_cb(btn_tx_toggle, [](lv_event_t *e) {
        g_tx_enabled = !g_tx_enabled;
        if (g_tx_enabled) {
            lv_obj_set_style_bg_color(btn_tx_toggle, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(lbl_tx_toggle, "TX: ENABLED");
            lv_obj_set_style_text_color(lbl_tx_toggle, lv_color_white(), 0);
        } else {
            lv_obj_set_style_bg_color(btn_tx_toggle, COLOR_ZONE_DISABLED, 0);
            lv_label_set_text(lbl_tx_toggle, "TX: DISABLED");
            lv_obj_set_style_text_color(lbl_tx_toggle, COLOR_TEXT_MUTED, 0);
        }
        Serial.printf("[UI] TX -> %s\n", g_tx_enabled ? "ENABLED" : "DISABLED");
    }, LV_EVENT_CLICKED, NULL);

    // ── TX Status Bar (right panel, below sniffer bar) ──────────────────────
    lv_obj_t *tx_bar = lv_obj_create(right_panel);
    lv_obj_set_size(tx_bar, lv_pct(90), lv_pct(7));
    lv_obj_align(tx_bar, LV_ALIGN_TOP_MID, 0, lv_pct(93));
    lv_obj_set_style_bg_color(tx_bar, COLOR_SNIFFER_BG, 0);
    lv_obj_set_style_border_color(tx_bar, lv_palette_main(LV_PALETTE_BLUE_GREY), 0);
    lv_obj_set_style_border_width(tx_bar, 1, 0);
    lv_obj_set_style_radius(tx_bar, 6, 0);
    lv_obj_set_style_pad_all(tx_bar, 4, 0);

    lbl_tx_bar = lv_label_create(tx_bar);
    lv_obj_set_style_text_font(lbl_tx_bar, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl_tx_bar, COLOR_TEXT_MUTED, 0);
    lv_label_set_text(lbl_tx_bar, "TX: off | sent:0");
    lv_obj_align(lbl_tx_bar, LV_ALIGN_LEFT_MID, 0, 0);

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



// ==================== 12. Parse 0x61 Status Broadcast ====================
// The indoor unit broadcasts a 27-byte status frame every ~1s.
// Layout (from protocol_analysis.md):
//   [0]  = 0x61 device addr
//   [2]  = power: 0xB0=ON, 0xAF=OFF
//   [4]  = zone/status flags (zone bitmask in low nibble)
//   [5]  = fan speed: 0x01=low, 0x03=med, 0x04=auto, 0x05=high
//   [6]  = mode: 0x02=Heat, 0x03=Cool, 0x04=Fan/Dry
//   [8]  = room temp raw (÷10 = °C)
//   [17] = setpoint (direct °C)
//   [21] = status flag: 0x40=running
//   [25–26] = CRC16
static void parse_status_broadcast(const uint8_t *buf, int len) {
    if (len < 27) return;       // too short for 0x61 frame
    if (buf[0] != 0x61) return; // not a status broadcast

    // Verify Modbus CRC16 on the frame (bytes 0..24, CRC at 25-26)
    // We already validated the packet via validate_packet(), but double-check addr
    BusACState state;
    state.power         = (buf[21] == 0x40); // 0x40=running/ON, 0x00=idle/OFF
    // Zone bitmask: byte[18] echoes the 0x9C49 register write when AC is on
    // byte[4] sometimes has zone info too but less reliable
    state.zone_mask     = (buf[18] != 0x00) ? (buf[18] & 0x1F) : (buf[4] & 0x1F);
    state.fan_speed     = buf[5];
    state.mode          = buf[6];
    state.room_temp_raw = buf[2];            // ÷10 = room temp °C (0xBE=190→19.0°C)
    state.setpoint      = buf[8];            // ÷10 = setpoint °C (0xD2=210→21.0°C)
    state.status_flag   = buf[21];
    // Note: byte[14]=hour, byte[15]=minute (NOT setpoint)
    state.bus_hour      = buf[14];
    state.bus_minute    = buf[15];

    // Atomic-ish write (single-core writer, no tearing on ESP32 for aligned structs)
    g_bus_state = state;
    g_bus_state_dirty = true;

    Serial.printf("[%lu][Parse] 0x61: pwr=%s zones=0x%02X fan=%d mode=%d room=%.1fC set=%.1fC flag=0x%02X\n",
        millis(),
        state.power ? "ON" : "OFF",
        state.zone_mask,
        state.fan_speed,
        state.mode,
        state.room_temp_raw / 10.0f,
        state.setpoint / 10.0f,
        state.status_flag);
}

// ==================== 12b. Register Change Tracker ====================
// Tracks last-seen value for each register written via FC06.
// Logs when any register value changes.
#define REG_TRACK_COUNT 8
static struct {
    uint16_t reg;
    uint16_t val;
    bool     seen;
} g_reg_track[REG_TRACK_COUNT] = {};

static int reg_track_find_or_add(uint16_t reg) {
    for (int i = 0; i < REG_TRACK_COUNT; i++) {
        if (g_reg_track[i].seen && g_reg_track[i].reg == reg) return i;
    }
    // Add new
    for (int i = 0; i < REG_TRACK_COUNT; i++) {
        if (!g_reg_track[i].seen) {
            g_reg_track[i].reg = reg;
            g_reg_track[i].seen = true;
            g_reg_track[i].val = 0xFFFF; // sentinel
            return i;
        }
    }
    return -1; // full
}

// FC06 writes from master carry actual control values
// Format: [addr=02] [FC=06] [reg_H] [reg_L] [val_H] [val_L] [CRC_L] [CRC_H]
static void parse_fc06_write(const uint8_t *buf, int len) {
    if (len != 8) return;
    if (buf[0] != 0x02 || buf[1] != 0x06) return;

    uint16_t reg = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t val = ((uint16_t)buf[4] << 8) | buf[5];

    // Track register changes
    int idx = reg_track_find_or_add(reg);
    if (idx >= 0) {
        uint16_t old_val = g_reg_track[idx].val;
        if (old_val != val) {
            if (old_val == 0xFFFF) {
                Serial.printf("[%lu][REG] 0x%04X = 0x%04X (first)\n", millis(), reg, val);
            } else {
                Serial.printf("[%lu][REG] 0x%04X: 0x%04X -> 0x%04X\n", millis(), reg, old_val, val);
            }
            g_reg_track[idx].val = val;
        }
    }

    // Update UI state from known registers
    if (reg == 0x9C49 && val != 0x00FF) {
        g_bus_state.zone_mask = val & 0x1F;
        g_bus_state_dirty = true;
    } else if (reg == 0x9C4A) {
        g_bus_state.power = (val == 0x0001);
        g_bus_state.status_flag = val ? 0x40 : 0x00;
        g_bus_state_dirty = true;
    }
}

// Map bus fan byte to UI index: 0=Low, 1=Med, 2=High
static int bus_fan_to_ui(uint8_t fan_byte) {
    switch (fan_byte) {
        case 0x01: return 0; // Low
        case 0x03: return 1; // Med
        case 0x05: return 2; // High
        case 0x04: return 1; // Auto → show as Med
        default:   return 1; // unknown → Med
    }
}

// Map bus mode byte to UI index: 0=Cool, 1=Heat, 2=Fan, 3=Dry, 4=Auto
static int bus_mode_to_ui(uint8_t mode_byte) {
    switch (mode_byte) {
        case 0x03: return 0; // Cool
        case 0x02: return 1; // Heat
        case 0x04: return 2; // Fan Only (or Dry — same byte per analysis)
        default:   return 0; // unknown → Cool
    }
}

// ==================== 12c. RS485 TX — Slave Controller Replacement ====================
// Sends FC06 write commands onto the bus when TX is enabled.
// The onboard RS485 transceiver has auto DE/RE — just write to Serial1.
//
// TODO/TEST:
// - Verify that sending during bus idle (~900ms gap) doesn't collide with master
// - Verify the indoor unit accepts writes from a second device on the bus
// - Verify the full command sequence (power, damper, fan/mode, zones, enable) is correct
// - The 26-byte slave echo may be needed for the master to recognize us — test without first
// - Timing: the slave might need to wait for a specific window (after master's poll cycle)

// Modbus CRC16 (standard polynomial 0xA001)
static uint16_t modbus_crc16(const uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}

// Send a single FC06 write: addr=0x02, FC=0x06, reg, value (+ CRC)
// Returns true if TX is enabled and data was sent.
static bool rs485_send_fc06(uint16_t reg, uint16_t value) {
    if (!g_tx_enabled) return false;

    uint8_t frame[8];
    frame[0] = 0x02;              // slave address (indoor unit)
    frame[1] = 0x06;              // function code: write single register
    frame[2] = (reg >> 8) & 0xFF; // register high
    frame[3] = reg & 0xFF;        // register low
    frame[4] = (value >> 8) & 0xFF; // value high
    frame[5] = value & 0xFF;      // value low
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;        // CRC low
    frame[7] = (crc >> 8) & 0xFF; // CRC high

    Serial1.write(frame, 8);
    Serial1.flush();  // wait for TX to complete before releasing bus
    g_tx_count++;

    Serial.printf("[TX] FC06 reg=0x%04X val=0x%04X\n", reg, value);
    return true;
}

// Send the full AC command sequence (mimics what the slave controller sends)
// Call this after any UI change (power, zones, temp, mode, fan)
static void send_ac_command(void) {
    if (!g_tx_enabled) return;

    // Build zone bitmask from current state
    uint8_t zone_mask = 0;
    if (ac_master_power) {
        for (int i = 0; i < 5; i++) {
            if (ac_zones[i]) zone_mask |= (1 << i);
        }
    } else {
        zone_mask = 0xFF;  // power-off marker
    }

    // Map UI mode to bus mode byte
    // UI: 0=Cool 1=Heat 2=Fan 3=Dry 4=Auto
    // Bus: 0x03=Cool 0x02=Heat 0x04=Fan/Dry (tentative)
    uint8_t mode_byte;
    switch (ac_current_mode) {
        case 0: mode_byte = 0x03; break; // Cool
        case 1: mode_byte = 0x02; break; // Heat
        case 2: mode_byte = 0x04; break; // Fan Only
        case 3: mode_byte = 0x04; break; // Dry (same as fan? TODO: verify)
        case 4: mode_byte = 0x03; break; // Auto (TODO: find correct byte)
        default: mode_byte = 0x03; break;
    }

    // Map UI fan to bus fan byte (tentative, from protocol analysis)
    // UI: 0=Low 1=Med 2=High
    // Bus: 0x01=Low 0x03=Med 0x05=High
    uint8_t fan_byte;
    switch (ac_current_fan) {
        case 0: fan_byte = 0x01; break;
        case 1: fan_byte = 0x03; break;
        case 2: fan_byte = 0x05; break;
        default: fan_byte = 0x03; break;
    }

    // TODO: fan_byte and mode_byte are combined into register 0x9C4C
    // From captures: 0x9C4C = 0x0048, 0x004C, 0x0050 seen
    // For now use 0x004C as a safe default (observed during heat mode)
    uint16_t fan_mode_reg = 0x004C;  // TODO: decode properly

    // TODO: damper (0x9C4B) and zone enable (0x9C4D) values are not fully decoded
    // Using observed values: damper=0x0058, enable=0x0060
    uint16_t damper_reg = 0x0058;    // TODO: decode properly
    uint16_t zone_enable_reg = 0x0060; // TODO: decode properly

    // Send the full sequence with small gaps between frames
    // Sequence observed from slave: zones, power, damper, fan/mode, zone_enable
    rs485_send_fc06(0x9C49, zone_mask);
    delay(20);  // inter-frame gap
    rs485_send_fc06(0x9C4A, ac_master_power ? 0x0001 : 0x0000);
    delay(20);
    rs485_send_fc06(0x9C4B, damper_reg);
    delay(20);
    rs485_send_fc06(0x9C4C, fan_mode_reg);
    delay(20);
    rs485_send_fc06(0x9C4D, zone_enable_reg);

    Serial.printf("[TX] Command sent: pwr=%d zones=0x%02X\n", ac_master_power, zone_mask);
}

// ==================== 13. RS485 Sniffer Task (Core 1) ====================
// Locked to 9600 8N2 — confirmed by raw GPIO analysis.
// Prints all packets to serial as hex + validity marker.
#if ENABLE_RS485_SNIFFER
void TaskRS485Sniffer(void *pvParameters) {
    (void)pvParameters;

    Serial1.begin(RS485_BAUD, RS485_CONFIG, RS485_RX_PIN, RS485_TX_PIN);
    Serial.printf("[Sniffer] 9600 8N2 RX=%d TX=%d\n", RS485_RX_PIN, RS485_TX_PIN);

    static uint8_t pkt[128];
    int pkt_len = 0;
    unsigned long last_byte_ms = millis();

    for (;;) {
        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            last_byte_ms = millis();
            g_sniffer.bytes_total++;
            if (pkt_len == 0) Serial.printf("[%lu][PKT] ", millis());
            Serial.printf("%02X ", b);
            if (pkt_len < (int)sizeof(pkt)) pkt[pkt_len++] = b;
        }

        if (pkt_len > 0 && (millis() - last_byte_ms) > PKT_GAP_MS) {
            bool noise = is_framing_noise(pkt, pkt_len);
            bool ok    = !noise && validate_packet(pkt, pkt_len);
            if (!noise) {
                g_sniffer.pkts_captured++;
                if (ok) {
                    g_sniffer.pkts_valid++;
                    Serial.printf(" [V len=%d]\n", pkt_len);
                    // Attempt to parse 0x61 status broadcasts
                    if (pkt[0] == 0x61 && pkt_len >= 27) {
                        parse_status_broadcast(pkt, pkt_len);
                    }
                    // Attempt to parse FC06 write commands (8 bytes, addr=0x02, FC=0x06)
                    if (pkt[0] == 0x02 && pkt[1] == 0x06 && pkt_len == 8) {
                        parse_fc06_write(pkt, pkt_len);
                    }
                    // Also scan for FC06 writes merged in longer packets (request+response in same gap)
                    // Look for pattern: 02 06 9C xx at any offset
                    if (pkt_len > 8) {
                        for (int i = 0; i <= pkt_len - 8; i++) {
                            if (pkt[i] == 0x02 && pkt[i+1] == 0x06 && pkt[i+2] == 0x9C) {
                                parse_fc06_write(pkt + i, 8);
                            }
                        }
                    }
                } else {
                    g_sniffer.pkts_invalid++;
                    Serial.printf(" [? len=%d]\n", pkt_len);
                    // Scan invalid packets too — merged frames often fail checksum
                    // but still contain valid FC06 writes
                    for (int i = 0; i <= pkt_len - 8; i++) {
                        if (pkt[i] == 0x02 && pkt[i+1] == 0x06 && pkt[i+2] == 0x9C) {
                            parse_fc06_write(pkt + i, 8);
                        }
                    }
                }
            }
            pkt_len = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif

// ==================== 14. setup() ====================
void setup() {
    Serial.begin(115200);
    Serial.println("=== AC Controller + RS485 Sniffer (9600 8N2 confirmed) ===");

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

#if ENABLE_RS485_SNIFFER
    xTaskCreatePinnedToCore(TaskRS485Sniffer, "RS485Sniffer", 4096, NULL, 3, NULL, 1);
    Serial.println("[System] Sniffer started on Core 1 (9600 8N2, pin 43)");
#endif
    Serial.println("[System] Ready.");
}

// ==================== 15. loop() ====================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

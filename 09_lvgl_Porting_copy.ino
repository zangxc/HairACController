#include <Arduino.h>
#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ==================== 1. 硬件引脚定义 (微雪 4.3B 专属) ====================
// GPIO4 is used by touch INT and GPIO5 is used by LCD RGB DE on this board.
// Keep the sniffer disabled until RS485 is moved to free pins.
#define ENABLE_RS485_SNIFFER 0
#define RS485_RX_PIN  4
#define RS485_TX_PIN  5
#define RS485_RTS_PIN 6   // 高电平发送，低电平接收

// ==================== 2. 中央空调核心状态机变量 ====================
bool ac_master_power = true;
int ac_target_temp = 24;
int ac_current_mode = 0; // 0: Cool, 1: Heat, 2: Fan, 3: Dry, 4: Auto
int ac_current_fan = 1;  // 0: Low, 1: Med, 2: High
bool ac_zones[] = {true, false, false, false, false}; // 5个分区的开关状态

// UI 元素全局指针，用于在事件中动态刷新
lv_obj_t *left_panel;
lv_obj_t *right_panel;
lv_obj_t *label_room_temp;
lv_obj_t *label_target_temp;
lv_obj_t *lbl_btn_mode; // 模式按钮上的文本指针
lv_obj_t *lbl_btn_fan;  // 风速按钮上的文本指针
lv_obj_t *lbl_power;
lv_obj_t *btn_power;
lv_obj_t *btn_zones[5]; // 固定大小数组，防止内存踩踏
lv_obj_t *lbl_zones[5];

const char* mode_names[] = {"COOL", "HEAT", "FAN ONLY", "DRY", "AUTO"};
const char* fan_names[] = {"LOW", "MEDIUM", "HIGH"};
const char* zone_names[] = {"Upstairs Bed", "Rumpus", "Dining", "Downstairs Bed", "Living Room"};

#define UI_FONT_CN LV_FONT_DEFAULT

static const lv_color_t COLOR_BG = lv_color_make(18, 22, 30);
static const lv_color_t COLOR_PANEL_ON = lv_color_make(30, 35, 45);
static const lv_color_t COLOR_PANEL_OFF = lv_color_make(32, 40, 52);
static const lv_color_t COLOR_BUTTON_ON = lv_color_make(50, 60, 80);
static const lv_color_t COLOR_BUTTON_OFF = lv_color_make(58, 70, 88);
static const lv_color_t COLOR_ZONE_OFF = lv_color_make(60, 65, 75);
static const lv_color_t COLOR_ZONE_DISABLED = lv_color_make(52, 64, 82);
static const lv_color_t COLOR_TEXT_MUTED = lv_color_make(150, 164, 180);
static const lv_color_t COLOR_TEXT_OFF = lv_color_make(128, 150, 170);

static void refresh_air_con_ui(void);
static void refresh_zone_button(int zone_idx);

// ==================== 3. 485 自动化永续盲测数据结构 ====================
#if ENABLE_RS485_SNIFFER
struct SerialConfigCombination {
    uint32_t baud;
    uint32_t config;
    const char* desc;
};

SerialConfigCombination rs485_configs[] = {
    {9600,  SERIAL_8N1, "9600-8N1 (无校验)"},
    {9600,  SERIAL_8E1, "9600-8E1 (偶校验-海尔常见)"},
    {19200, SERIAL_8N1, "19200-8N1 (无校验)"},
    {19200, SERIAL_8E1, "19200-8E1 (偶校验-大金常见)"},
    {4800,  SERIAL_8E1, "4800-8E1 (低速偶校验)"}
};

int current_485_idx = 0;
#endif

// ==================== 4. LVGL UI 交互事件回调 ====================
static void temp_btn_cb(lv_event_t * e) {
    if(!ac_master_power) return; 

    char *action = (char *)lv_event_get_user_data(e);
    if(action == NULL) return;

    if(strcmp(action, "+") == 0) ac_target_temp++;
    else ac_target_temp--;
    
    if(ac_target_temp > 30) ac_target_temp = 30;
    if(ac_target_temp < 16) ac_target_temp = 16;
    
    lv_label_set_text_fmt(label_target_temp, "%d°C", ac_target_temp);
    Serial.printf("[UI] 目标温度调整为: %d°C\n", ac_target_temp);
}

static void zone_btn_cb(lv_event_t * e) {
    if(!ac_master_power) return; 

    lv_obj_t * btn = lv_event_get_target(e);
    int zone_idx = (int)(uintptr_t)lv_event_get_user_data(e);
    
    ac_zones[zone_idx] = !ac_zones[zone_idx]; 
    refresh_zone_button(zone_idx);
    Serial.printf("[UI] 区域 Zone %d 状态切换为: %s\n", zone_idx + 1, ac_zones[zone_idx] ? "ON" : "OFF");
}

static void mode_btn_cb(lv_event_t * e) {
    if(!ac_master_power) return;
    ac_current_mode = (ac_current_mode + 1) % 5;
    refresh_air_con_ui();
    Serial.printf("[UI] 空调模式切换为: %s\n", mode_names[ac_current_mode]);
}

static void fan_btn_cb(lv_event_t * e) {
    if(!ac_master_power) return;
    ac_current_fan = (ac_current_fan + 1) % 3;
    refresh_air_con_ui();
    Serial.printf("[UI] 空调风速切换为: %s\n", fan_names[ac_current_fan]);
}

static void power_btn_cb(lv_event_t * e) {
    ac_master_power = !ac_master_power;
    refresh_air_con_ui();
    Serial.printf("[UI] 中央空调主开关: %s\n", ac_master_power ? "ON" : "OFF");
}

static void refresh_zone_button(int zone_idx) {
    if(btn_zones[zone_idx] == NULL || lbl_zones[zone_idx] == NULL) return;

    if(!ac_master_power) {
        lv_obj_set_style_bg_color(btn_zones[zone_idx], COLOR_ZONE_DISABLED, 0);
        lv_obj_set_style_text_color(lbl_zones[zone_idx], COLOR_TEXT_OFF, 0);
        lv_label_set_text_fmt(lbl_zones[zone_idx], "%s: StandBy", zone_names[zone_idx]);
        return;
    }

    if(ac_zones[zone_idx]) {
        lv_obj_set_style_bg_color(btn_zones[zone_idx], lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_text_color(lbl_zones[zone_idx], lv_color_white(), 0);
        lv_label_set_text_fmt(lbl_zones[zone_idx], "%s: ON", zone_names[zone_idx]);
    } else {
        lv_obj_set_style_bg_color(btn_zones[zone_idx], COLOR_ZONE_OFF, 0);
        lv_obj_set_style_text_color(lbl_zones[zone_idx], COLOR_TEXT_MUTED, 0);
        lv_label_set_text_fmt(lbl_zones[zone_idx], "%s: OFF", zone_names[zone_idx]);
    }
}

static void refresh_air_con_ui(void) {
    lv_color_t panel_color = ac_master_power ? COLOR_PANEL_ON : COLOR_PANEL_OFF;
    lv_color_t control_color = ac_master_power ? COLOR_BUTTON_ON : COLOR_BUTTON_OFF;
    lv_color_t text_color = ac_master_power ? lv_color_white() : COLOR_TEXT_OFF;
    lv_color_t muted_color = ac_master_power ? COLOR_TEXT_MUTED : COLOR_TEXT_OFF;

    if(left_panel != NULL) lv_obj_set_style_bg_color(left_panel, panel_color, 0);
    if(right_panel != NULL) lv_obj_set_style_bg_color(right_panel, panel_color, 0);
    if(label_room_temp != NULL) lv_obj_set_style_text_color(label_room_temp, muted_color, 0);
    if(label_target_temp != NULL) lv_obj_set_style_text_color(label_target_temp, text_color, 0);

    if(btn_power != NULL) {
        lv_obj_set_style_bg_color(btn_power, ac_master_power ? lv_palette_main(LV_PALETTE_GREEN) : COLOR_ZONE_DISABLED, 0);
    }
    if(lbl_power != NULL) {
        lv_obj_set_style_text_color(lbl_power, lv_color_white(), 0);
        lv_label_set_text(lbl_power, ac_master_power ? "Power: ON" : "Power: OFF");
    }

    if(lbl_btn_mode != NULL) {
        lv_label_set_text_fmt(lbl_btn_mode, "Mode\n%s", mode_names[ac_current_mode]);
        lv_obj_set_style_text_color(lbl_btn_mode, text_color, 0);
    }
    if(lbl_btn_fan != NULL) {
        lv_label_set_text_fmt(lbl_btn_fan, "Fan\n%s", fan_names[ac_current_fan]);
        lv_obj_set_style_text_color(lbl_btn_fan, text_color, 0);
    }

    lv_obj_t *mode_btn = (lbl_btn_mode != NULL) ? lv_obj_get_parent(lbl_btn_mode) : NULL;
    lv_obj_t *fan_btn = (lbl_btn_fan != NULL) ? lv_obj_get_parent(lbl_btn_fan) : NULL;
    if(mode_btn != NULL) lv_obj_set_style_bg_color(mode_btn, control_color, 0);
    if(fan_btn != NULL) lv_obj_set_style_bg_color(fan_btn, control_color, 0);

    for(int i = 0; i < 5; i++) {
        refresh_zone_button(i);
    }
}

// ==================== 5. 百分比(lv_pct)纯自适应 UI 渲染函数 ====================
void create_air_con_dashboard(void) {
    // 设置高档次的家居深色暗黑风格底板背景
    lv_obj_set_style_bg_color(lv_scr_act(), COLOR_BG, 0);

    // 🌟【1. 左侧大面板】(使用百分比：占屏幕总宽度的 45%，总高度的 90%)
    left_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(left_panel, lv_pct(45), lv_pct(90));
    lv_obj_align(left_panel, LV_ALIGN_LEFT_MID, lv_pct(3), 0); // 靠左边缘向右漂移 3%
    lv_obj_set_style_bg_color(left_panel, COLOR_PANEL_ON, 0);
    lv_obj_set_style_border_width(left_panel, 0, 0);
    lv_obj_set_style_radius(left_panel, 12, 0);

    // 总开关
    btn_power = lv_btn_create(left_panel);
    lv_obj_set_size(btn_power, lv_pct(86), lv_pct(14));
    lv_obj_align(btn_power, LV_ALIGN_TOP_MID, 0, lv_pct(4));
    lv_obj_set_style_bg_color(btn_power, lv_palette_main(LV_PALETTE_GREEN), 0);
    lbl_power = lv_label_create(btn_power);
    lv_label_set_text(lbl_power, "Power: ON");
    lv_obj_set_style_text_font(lbl_power, UI_FONT_CN, 0);
    lv_obj_center(lbl_power);
    lv_obj_add_event_cb(btn_power, power_btn_cb, LV_EVENT_CLICKED, NULL);

    label_room_temp = lv_label_create(left_panel);
    lv_label_set_text(label_room_temp, "Room temp: 21.5°C");
    lv_obj_set_style_text_font(label_room_temp, UI_FONT_CN, 0);
    lv_obj_set_style_text_color(label_room_temp, COLOR_TEXT_MUTED, 0);
    lv_obj_align(label_room_temp, LV_ALIGN_CENTER, 0, lv_pct(-24));

    label_target_temp = lv_label_create(left_panel);
    lv_label_set_text_fmt(label_target_temp, "%d°C", ac_target_temp);
    lv_obj_set_style_text_font(label_target_temp, &lv_font_montserrat_30, 0); 
    lv_obj_set_style_text_color(label_target_temp, lv_color_white(), 0);
    lv_obj_align(label_target_temp, LV_ALIGN_CENTER, 0, lv_pct(-10));

    // 温度调节按钮也完全切换为百分比
    lv_obj_t * btn_minus = lv_btn_create(left_panel);
    lv_obj_set_size(btn_minus, lv_pct(22), lv_pct(13));
    lv_obj_align(btn_minus, LV_ALIGN_CENTER, lv_pct(-30), lv_pct(-10));
    lv_obj_set_style_bg_color(btn_minus, lv_color_make(45, 50, 65), 0);
    lv_obj_t * lbl_minus = lv_label_create(btn_minus);
    lv_label_set_text(lbl_minus, "-");
    lv_obj_set_style_text_font(lbl_minus, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_minus);
    lv_obj_add_event_cb(btn_minus, temp_btn_cb, LV_EVENT_CLICKED, (void*)"-");

    lv_obj_t * btn_plus = lv_btn_create(left_panel);
    lv_obj_set_size(btn_plus, lv_pct(22), lv_pct(13));
    lv_obj_align(btn_plus, LV_ALIGN_CENTER, lv_pct(30), lv_pct(-10));
    lv_obj_set_style_bg_color(btn_plus, lv_color_make(45, 50, 65), 0);
    lv_obj_t * lbl_plus = lv_label_create(btn_plus);
    lv_label_set_text(lbl_plus, "+");
    lv_obj_set_style_text_font(lbl_plus, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_plus);
    lv_obj_add_event_cb(btn_plus, temp_btn_cb, LV_EVENT_CLICKED, (void*)"+");

    // 模式与风力大方形按钮
    lv_obj_t * btn_mode = lv_btn_create(left_panel);
    lv_obj_set_size(btn_mode, 128, 128);
    lv_obj_align(btn_mode, LV_ALIGN_BOTTOM_LEFT, lv_pct(8), lv_pct(-5));
    lv_obj_set_style_bg_color(btn_mode, COLOR_BUTTON_ON, 0);
    lbl_btn_mode = lv_label_create(btn_mode);
    lv_label_set_text_fmt(lbl_btn_mode, "Mode\n%s", mode_names[ac_current_mode]);
    lv_obj_set_style_text_font(lbl_btn_mode, UI_FONT_CN, 0);
    lv_obj_set_style_text_align(lbl_btn_mode, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_btn_mode);
    lv_obj_add_event_cb(btn_mode, mode_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_fan = lv_btn_create(left_panel);
    lv_obj_set_size(btn_fan, 128, 128);
    lv_obj_align(btn_fan, LV_ALIGN_BOTTOM_RIGHT, lv_pct(-8), lv_pct(-5));
    lv_obj_set_style_bg_color(btn_fan, COLOR_BUTTON_ON, 0);
    lbl_btn_fan = lv_label_create(btn_fan);
    lv_label_set_text_fmt(lbl_btn_fan, "Fan\n%s", fan_names[ac_current_fan]);
    lv_obj_set_style_text_font(lbl_btn_fan, UI_FONT_CN, 0);
    lv_obj_set_style_text_align(lbl_btn_fan, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl_btn_fan);
    lv_obj_add_event_cb(btn_fan, fan_btn_cb, LV_EVENT_CLICKED, NULL);

    // 🌟【2. 右侧大面板】(使用百分比：同样对称占宽 45%，高 90%)
    right_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(right_panel, lv_pct(45), lv_pct(90));
    lv_obj_align(right_panel, LV_ALIGN_RIGHT_MID, lv_pct(-3), 0); // 靠右边缘向左漂移 3%
    lv_obj_set_style_bg_color(right_panel, COLOR_PANEL_ON, 0);
    lv_obj_set_style_border_width(right_panel, 0, 0);
    lv_obj_set_style_radius(right_panel, 12, 0);

    lv_obj_t * zone_title = lv_label_create(right_panel);
    lv_label_set_text(zone_title, "Zone Control");
    lv_obj_set_style_text_font(zone_title, UI_FONT_CN, 0);
    lv_obj_set_style_text_color(zone_title, lv_palette_main(LV_PALETTE_LIGHT_BLUE), 0);
    lv_obj_align(zone_title, LV_ALIGN_TOP_MID, 0, lv_pct(4));

    // 🌟 纯自适应垂直分布：让 5 个 Zone 按钮完全以百分比间距均摊在面板内，永不溢出
    for(int i = 0; i < 5; i++) {
        btn_zones[i] = lv_btn_create(right_panel);
        lv_obj_set_size(btn_zones[i], lv_pct(86), lv_pct(11)); // 高度占面板的 11%
        
        // 核心技术点：利用百分比累加定位坐标，等比例纵向平铺，自适应缩放
        lv_obj_align(btn_zones[i], LV_ALIGN_TOP_MID, 0, lv_pct(13 + (i * 15))); 
        
        lbl_zones[i] = lv_label_create(btn_zones[i]);
        lv_obj_set_style_text_font(lbl_zones[i], UI_FONT_CN, 0);
        lv_obj_center(lbl_zones[i]);

        lv_obj_add_event_cb(btn_zones[i], zone_btn_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }

    refresh_air_con_ui();
}
// ==================== 6. FreeRTOS 后台不间断多轨自动嗅探任务 (独立运行于 Core 1) ====================
#if ENABLE_RS485_SNIFFER
void TaskRS485Sniffer(void *pvParameters) 
{
    (void) pvParameters;
    
    pinMode(RS485_RTS_PIN, OUTPUT);
    digitalWrite(RS485_RTS_PIN, LOW); // 硬件永久处于高阻接收状态，绝对安全

    unsigned long last_switch_time = millis();
    const unsigned long SCAN_INTERVAL = 6000; // 6 秒自动切换一次盲测参数
    int total_configs = sizeof(rs485_configs) / sizeof(rs485_configs[0]); // 严谨计算长度
    bool has_data_in_current_config = false;

    // 启动初始盲测
    Serial1.begin(rs485_configs[current_485_idx].baud, rs485_configs[current_485_idx].config, RS485_RX_PIN, RS485_TX_PIN);
    Serial.printf("\n[485 Sniffer] 永续盲测启动！正在扫轨: %s\n", rs485_configs[current_485_idx].desc);

    for (;;) {
        // A. 持续非阻塞读取字节并打印
        if (Serial1.available() > 0) {
            if (!has_data_in_current_config) {
                has_data_in_current_config = true;
                Serial.printf("\n\n📊 [DATA IN] 在当前轨【%s】捕获到通信流: \n", rs485_configs[current_485_idx].desc);
            }

            uint8_t inByte = Serial1.read();
            if (inByte < 0x10) {
                Serial.print("0");
            }
            Serial.print(inByte, HEX);
            Serial.print(" ");
        }

        // B. 强制非锁死状态机：定时一到立即无条件换轨
        if (millis() - last_switch_time > SCAN_INTERVAL) {
            current_485_idx = (current_485_idx + 1) % total_configs;
            has_data_in_current_config = false; 
            
            Serial1.end();
            vTaskDelay(pdMS_TO_TICKS(50)); // 释放短暂时间片让硬件串口复位

            Serial1.begin(rs485_configs[current_485_idx].baud, rs485_configs[current_485_idx].config, RS485_RX_PIN, RS485_TX_PIN);
            digitalWrite(RS485_RTS_PIN, LOW); // 重新强制拉回纯接收区
            
            Serial.printf("\n[485 Sniffer] 时间到，自动切轨 -> %s", rs485_configs[current_485_idx].desc);
            last_switch_time = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // 喂狗，维持多线程分时调度
    }
}
#endif

// ==================== 7. 主系统初始化入口 ====================
void setup()
{
    Serial.begin(115200);

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    if (lcd != nullptr) {
        lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
        #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
        auto lcd_bus = lcd->getBus();
        if (lcd_bus != nullptr && lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
        }
        #endif
    }
#endif

    // 启动板级硬件
    assert(board->begin());

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    Serial.println("Creating UI");
    lvgl_port_lock(-1);

    // 🚀【彻底净化逻辑】：完全删除了任何硬编码 hor_res 和显存改动的代码！
    // 强制把底层渲染面板大小直接映射给活动画布，防止由于硬件 Stride 运算错误导致的花屏
    lv_obj_set_size(lv_scr_act(), lv_pct(100), lv_pct(100));

    // 渲染为您全新编写的、百分比自适应的二合一控制面板
    create_air_con_dashboard(); 
    
    lvgl_port_unlock();

    // 启动独立 Core 1 后台永续自动轮巡嗅探线程
#if ENABLE_RS485_SNIFFER
    xTaskCreatePinnedToCore(
        TaskRS485Sniffer,     
        "RS485_Sniffer_Task", 
        4096,                 
        NULL,                 
        3,                    
        NULL,                 
        1                     
    );
#else
    Serial.println("[System] RS485 sniffer disabled: GPIO4 conflicts with touch INT, GPIO5 conflicts with LCD DE.");
#endif
    Serial.println("[System] 百分比自适应多核系统已完美合龙！");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}

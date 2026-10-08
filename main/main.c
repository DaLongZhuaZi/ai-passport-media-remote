// 媒体遥控器主程序:基于 FoloToy BSP 的自研界面(与官方 demo 菜单无关)。
// 三键操作:
//   上 短按=上一曲   下 短按=下一曲   OK 短按=播放/暂停
//   上 长按=音量+    下 长按=音量-    OK 长按=静音   OK 双击=手机桌面(Home)
#include <stdbool.h>
#include <stdint.h>
#include "bsp_i2c.h"
#include "bsp_display.h"
#include "bsp_button.h"
#include "bsp_battery.h"
#include "hid_remote.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "remote";

// Consumer Control 用法
#define CC_PLAY_PAUSE 0x00CD
#define CC_NEXT       0x00B5
#define CC_PREV       0x00B6
#define CC_VOL_UP     0x00E9
#define CC_VOL_DOWN   0x00EA
#define CC_MUTE       0x00E2
#define KEY_HOME      0x4A

typedef struct {
    bsp_btn_t btn;
    bsp_btn_ev_t ev;
} input_event_t;

static QueueHandle_t s_input_queue;
static volatile bool s_ble_ready;
static volatile bool s_ble_connected_seen;
static volatile int64_t s_last_activity_us;
static int s_battery_soc = -1;
static bool s_battery_ok;

// LVGL 对象(仅在 LVGL 任务/持锁时访问)
static lv_obj_t *s_status_label;
static lv_obj_t *s_batt_label;
static lv_obj_t *s_symbol_label;
static lv_obj_t *s_action_label;
static lv_obj_t *s_hint_label;

static void on_hid_state(bool ready, void *user)
{
    (void)user;
    s_ble_ready = ready;
    if (ready) s_ble_connected_seen = true;
}

// ---------------- UI ----------------
static void ui_build(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x10151B), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // 顶栏:左侧连接状态,右侧电量(避开顶部圆角,自 y=36 起)
    s_status_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x8FD0FF), 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_LEFT, 16, 36);
    lv_label_set_text(s_status_label, LV_SYMBOL_BLUETOOTH " Pairing...");

    s_batt_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_batt_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_batt_label, lv_color_hex(0xA8E063), 0);
    lv_obj_align(s_batt_label, LV_ALIGN_TOP_RIGHT, -16, 36);
    lv_label_set_text(s_batt_label, "--%");

    // 中央动作符号与文字
    s_symbol_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_symbol_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_symbol_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_symbol_label, LV_ALIGN_CENTER, 0, -58);
    lv_label_set_text(s_symbol_label, LV_SYMBOL_BLUETOOTH);

    s_action_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_action_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_action_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_action_label, LV_ALIGN_CENTER, 0, -8);
    lv_label_set_text(s_action_label, "MEDIA REMOTE");

    s_hint_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_hint_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_hint_label, lv_color_hex(0x9AA7B4), 0);
    lv_obj_set_width(s_hint_label, 220);
    lv_label_set_long_mode(s_hint_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(s_hint_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_hint_label, LV_ALIGN_CENTER, 0, 26);
    lv_label_set_text(s_hint_label,
                      "Pair \"Passport Remote\"\nfrom phone Bluetooth settings");

    // 底部按键图例
    lv_obj_t *legend = lv_label_create(scr);
    lv_obj_set_style_text_font(legend, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(legend, lv_color_hex(0x6E7B8A), 0);
    lv_obj_set_style_text_align(legend, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -34);
    lv_label_set_text(legend,
                      "UP Prev    DOWN Next\n"
                      "OK Play/Pause\n"
                      "Hold UP/DN Volume  Hold OK Mute\n"
                      "OK x2 Home");
}

static void ui_show_action(const char *symbol, const char *text)
{
    if (!bsp_lvgl_lock(200)) return;
    lv_label_set_text(s_symbol_label, symbol);
    lv_label_set_text(s_action_label, text);
    bsp_lvgl_unlock();
}

// 每 500ms 在 LVGL 任务里刷新状态与背光(状态来自原子标志,不阻塞)
static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    // 状态栏
    if (s_ble_ready) {
        lv_label_set_text(s_status_label, LV_SYMBOL_BLUETOOTH " Connected");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xA8E063), 0);
        lv_label_set_text(s_hint_label,
                          "Controlling your phone.\nKeys work even while music apps\nare in background.");
    } else if (s_ble_connected_seen) {
        lv_label_set_text(s_status_label, LV_SYMBOL_BLUETOOTH " Pairing...");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x8FD0FF), 0);
    }
    if (s_battery_ok && s_battery_soc >= 0) {
        lv_label_set_text_fmt(s_batt_label, "%d%%", s_battery_soc);
    }
    // 30 秒无操作后调暗背光省电
    int64_t idle_us = esp_timer_get_time() - s_last_activity_us;
    bsp_display_backlight(idle_us > 30 * 1000 * 1000 ? 25 : 100);
}

// ---------------- 按键处理 ----------------
static void do_consumer(uint16_t usage, const char *symbol, const char *text)
{
    if (!s_ble_ready) {
        ui_show_action(LV_SYMBOL_BLUETOOTH, "NOT CONNECTED");
        return;
    }
    if (hid_remote_send_consumer(usage) == ESP_OK) {
        ui_show_action(symbol, text);
    } else {
        ui_show_action(LV_SYMBOL_WARNING, "SEND FAILED");
    }
}

static void do_key(uint8_t keycode, const char *symbol, const char *text)
{
    if (!s_ble_ready) {
        ui_show_action(LV_SYMBOL_BLUETOOTH, "NOT CONNECTED");
        return;
    }
    if (hid_remote_send_key(keycode) == ESP_OK) {
        ui_show_action(symbol, text);
    } else {
        ui_show_action(LV_SYMBOL_WARNING, "SEND FAILED");
    }
}

static void handle_input(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    s_last_activity_us = esp_timer_get_time();
    if (ev == BSP_BTN_CLICK) {
        if (btn == BSP_BTN_UP)        do_consumer(CC_PREV, LV_SYMBOL_PREV, "PREV TRACK");
        else if (btn == BSP_BTN_DOWN) do_consumer(CC_NEXT, LV_SYMBOL_NEXT, "NEXT TRACK");
        else if (btn == BSP_BTN_OK)   do_consumer(CC_PLAY_PAUSE, LV_SYMBOL_PLAY, "PLAY / PAUSE");
    } else if (ev == BSP_BTN_LONG) {
        if (btn == BSP_BTN_UP)        do_consumer(CC_VOL_UP, LV_SYMBOL_VOLUME_MAX, "VOLUME +");
        else if (btn == BSP_BTN_DOWN) do_consumer(CC_VOL_DOWN, LV_SYMBOL_VOLUME_MID, "VOLUME -");
        else if (btn == BSP_BTN_OK)   do_consumer(CC_MUTE, LV_SYMBOL_MUTE, "MUTE");
    } else if (ev == BSP_BTN_DOUBLE && btn == BSP_BTN_OK) {
        do_key(KEY_HOME, LV_SYMBOL_HOME, "HOME");
    }
}

static void input_task(void *arg)
{
    (void)arg;
    input_event_t ev;
    for (;;) {
        if (xQueueReceive(s_input_queue, &ev, portMAX_DELAY) == pdTRUE) {
            handle_input(ev.btn, ev.ev);
        }
    }
}

static void battery_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_battery_ok) {
            int soc = bsp_battery_soc();
            if (soc >= 0) {
                s_battery_soc = soc;
                hid_remote_set_battery((uint8_t)soc);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

// button 回调跑在 esp_timer 任务:只入队
static void on_key(bsp_btn_t btn, bsp_btn_ev_t ev, void *user)
{
    (void)user;
    if (!s_input_queue) return;
    const input_event_t item = { .btn = btn, .ev = ev };
    (void)xQueueSend(s_input_queue, &item, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "媒体遥控器启动");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    bsp_i2c_init();
    s_battery_ok = (bsp_battery_init() == ESP_OK);
    if (!s_battery_ok) ESP_LOGW(TAG, "电量计不可用,状态页电量将显示 --");

    if (bsp_display_init() != ESP_OK || !bsp_lvgl_init()) {
        ESP_LOGE(TAG, "显示初始化失败");
        return;
    }
    bsp_display_backlight(100);
    s_last_activity_us = esp_timer_get_time();

    if (bsp_lvgl_lock(1000)) {
        ui_build();
        lv_timer_create(ui_timer_cb, 500, NULL);
        bsp_lvgl_unlock();
    }

    s_input_queue = xQueueCreate(8, sizeof(input_event_t));
    if (!s_input_queue || xTaskCreate(input_task, "input", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "输入任务创建失败");
        return;
    }
    if (bsp_button_init(on_key, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "按键初始化失败");
        return;
    }

    if (hid_remote_init(on_hid_state, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "BLE HID 初始化失败");
        if (bsp_lvgl_lock(200)) {
            lv_label_set_text(s_action_label, "BLE ERROR");
            bsp_lvgl_unlock();
        }
        return;
    }

    xTaskCreate(battery_task, "battery", 3072, NULL, 3, NULL);
    // 初始电量尽快上报一次
    if (s_battery_ok) {
        int soc = bsp_battery_soc();
        if (soc >= 0) { s_battery_soc = soc; hid_remote_set_battery((uint8_t)soc); }
    }
    ESP_LOGI(TAG, "就绪:在手机蓝牙设置里配对 Passport Remote");
}

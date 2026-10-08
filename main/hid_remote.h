#pragma once
// BLE HID 媒体遥控器:NimBLE 外设,实现 HID over GATT(0x1812)。
// 提供 Consumer Control 报告(播放/暂停/上下曲/音量/静音)与键盘报告(Home 等),
// 外加电池服务(0x180F)把工牌自身电量报给手机。配对为 Just Works + 绑定,NVS 持久化。
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// 连接状态回调:connected=已建立连接且加密就绪(可发键),由 BLE 任务上下文调用,
// 实现方只应做置标志/入队等轻量操作。
typedef void (*hid_remote_state_cb_t)(bool connected, void *user);

esp_err_t hid_remote_init(hid_remote_state_cb_t cb, void *user);

// 是否已连接且可发送报告。
bool hid_remote_ready(void);

// 发送 Consumer Control 用法(按下+抬起)。常用值:
// 播放/暂停 0x00CD,下一曲 0x00B5,上一曲 0x00B6,音量+ 0x00E9,音量- 0x00EA,静音 0x00E2。
esp_err_t hid_remote_send_consumer(uint16_t usage);

// 发送键盘键码(按下+抬起),如 Home=0x4A, Enter=0x28, Esc=0x29。
esp_err_t hid_remote_send_key(uint8_t keycode);

// 更新电池服务电量百分比(0..100)。
void hid_remote_set_battery(uint8_t percent);

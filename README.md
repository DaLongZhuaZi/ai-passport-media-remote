# 媒体遥控器 Media Remote（AI Passport 玩法）

把 FoloToy AI Passport 变成安卓手机的蓝牙媒体遥控器。标准 BLE HID（over GATT）实现，
手机在蓝牙设置里配对 `Passport Remote` 即可，不用装任何 App。

## 按键

| 操作 | 功能 |
| --- | --- |
| 上 短按 | 上一曲 |
| 下 短按 | 下一曲 |
| OK 短按 | 播放 / 暂停 |
| 上 长按 | 音量 + |
| 下 长按 | 音量 − |
| OK 长按 | 静音 |
| OK 双击 | 回到手机桌面（Home） |

屏幕显示连接状态与工牌电量；工牌电量同时通过蓝牙电池服务（0x180F）同步给手机。
30 秒无操作自动调暗背光。配对绑定信息存 NVS，重启后自动回连，不必重复配对。

## 构建

基于 FoloToy 官方基线仓库的 BSP（ESP-IDF 5.5.3，ESP32-C3）：

```bash
. $IDF_PATH/export.sh
idf.py build
```

产物 `build/ai_passport_remote.bin` 是标准 ESP 应用镜像（约 1.05 MB），
可通过 AI Passport 玩法库安装（写入 0x10000 应用区，或由设备端应用商店装入玩法分区）：

https://github.com/DaLongZhuaZi/ai-passport-plays

## 实现要点

- NimBLE 外设，HID 服务 0x1812：报告 1 = Consumer Control（播放/暂停、上下曲、音量、静音），报告 2 = 标准键盘（Home 等）；Just Works 配对 + 绑定，报告特征要求加密链路。
- 三键事件走 BSP 的 ADC 按键回调 → 队列 → 输入任务，HID 发送不在回调里做。
- UI 为自研的符号化界面（LVGL 内置符号 + Montserrat），不带中文字体，控制体积。

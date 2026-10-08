// BLE HID 媒体遥控器实现(NimBLE)。
// 报告映射:报告 ID 1 = Consumer Control(16 位用法数组,发 0 表示抬起);
//           报告 ID 2 = 标准键盘(修饰键 + 预留 + 6 键位)。
#include "hid_remote.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

extern void ble_store_config_init(void);

static const char *TAG = "hid_remote";
static const char *DEVICE_NAME = "Passport Remote";

#define REPORT_ID_CONSUMER 1
#define REPORT_ID_KEYBOARD 2

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_encrypted = false;
static bool s_sub_consumer = false;
static bool s_sub_keyboard = false;
static uint8_t s_own_addr_type;
static uint8_t s_battery_level = 100;
static uint8_t s_proto_mode = 1; // 1=Report 协议

static uint16_t s_consumer_val_handle;
static uint16_t s_keyboard_val_handle;
static uint16_t s_battery_val_handle;

static hid_remote_state_cb_t s_state_cb;
static void *s_state_user;

// HID 报告描述符
static const uint8_t s_report_map[] = {
    // Consumer Control(报告 ID 1):16 位用法数组
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x03, //   Logical Maximum (1023)
    0x19, 0x00,       //   Usage Minimum (0)
    0x2A, 0xFF, 0x03, //   Usage Maximum (1023)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x00,       //   Input (Data, Array, Abs)
    0xC0,             // End Collection
    // 键盘(报告 ID 2):标准 8 字节报告
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       //   Report ID (2)
    0x05, 0x07,       //   Usage Page (Key Codes)
    0x19, 0xE0,       //   Usage Minimum (224)
    0x29, 0xE7,       //   Usage Maximum (231)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Var, Abs) 修饰键
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x01,       //   Input (Const) 预留字节
    0x95, 0x06,       //   Report Count (6)
    0x75, 0x08,       //   Report Size (8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x05, 0x07,       //   Usage Page (Key Codes)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, 0xFF,       //   Usage Maximum (255)
    0x81, 0x00,       //   Input (Data, Array, Abs) 键位数组
    0xC0,             // End Collection
};

static const uint8_t s_hid_info[] = {0x11, 0x01, 0x00, 0x03}; // bcdHID 1.11, 国家码 0, 远程唤醒|常连接
static const uint8_t s_pnp_id[] = {0x02, 0x3A, 0x30, 0x01, 0x80, 0x00, 0x01}; // USB-IF, VID 0x303A, PID 0x8001, v1.00
static const char s_manufacturer[] = "Community DIY";

static bool conn_encrypted(uint16_t conn_handle)
{
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0) return false;
    return desc.sec_state.encrypted;
}

static void notify_state(void)
{
    if (s_state_cb) s_state_cb(hid_remote_ready(), s_state_user);
}

static int append_bytes(struct ble_gatt_access_ctxt *ctxt, const void *buf, size_t len)
{
    return os_mbuf_append(ctxt->om, buf, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// ---------- GATT 访问回调 ----------
static int hid_info_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return append_bytes(ctxt, s_hid_info, sizeof(s_hid_info));
}

static int report_map_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return append_bytes(ctxt, s_report_map, sizeof(s_report_map));
}

static int ctrl_point_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    return 0; // 接受并忽略(挂起/退出挂起命令对本设备无意义)
}

static int proto_mode_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return append_bytes(ctxt, &s_proto_mode, 1);
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
            uint8_t v = 1;
            ble_hs_mbuf_to_flat(ctxt->om, &v, 1, NULL);
            s_proto_mode = v ? 1 : 0;
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int report_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle;
    int report_id = (int)(intptr_t)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    // HID 报告必须在加密链路上读取;未加密时回 Insufficient Encryption,促使手机发起配对绑定。
    if (!conn_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
    uint8_t zeros[8] = {0};
    size_t len = (report_id == REPORT_ID_CONSUMER) ? 2 : 8;
    return append_bytes(ctxt, zeros, len);
}

static int report_ref_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
    uint8_t ref[2] = {(uint8_t)(intptr_t)arg, 0x01}; // {报告 ID, 类型=Input}
    return append_bytes(ctxt, ref, sizeof(ref));
}

static int fixed_str_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    const char *s = (const char *)arg;
    return append_bytes(ctxt, s, strlen(s));
}

static int pnp_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return append_bytes(ctxt, s_pnp_id, sizeof(s_pnp_id));
}

static int battery_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return append_bytes(ctxt, &s_battery_level, 1);
}

// ---------- GATT 服务定义 ----------
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812), // HID 服务
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4A), // HID Information
                .access_cb = hid_info_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4B), // Report Map
                .access_cb = report_map_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4C), // HID Control Point
                .access_cb = ctrl_point_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4E), // Protocol Mode
                .access_cb = proto_mode_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4D), // Report: Consumer
                .access_cb = report_access,
                .arg = (void *)(intptr_t)REPORT_ID_CONSUMER,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_consumer_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2908), // Report Reference
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = report_ref_access,
                        .arg = (void *)(intptr_t)REPORT_ID_CONSUMER,
                    },
                    { 0 },
                },
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4D), // Report: Keyboard
                .access_cb = report_access,
                .arg = (void *)(intptr_t)REPORT_ID_KEYBOARD,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_keyboard_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2908),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = report_ref_access,
                        .arg = (void *)(intptr_t)REPORT_ID_KEYBOARD,
                    },
                    { 0 },
                },
            },
            { 0 },
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A), // Device Information
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29), // Manufacturer Name
                .access_cb = fixed_str_access,
                .arg = (void *)s_manufacturer,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A50), // PnP ID
                .access_cb = pnp_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 },
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F), // Battery Service
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A19), // Battery Level
                .access_cb = battery_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_battery_val_handle,
            },
            { 0 },
        },
    },
    { 0 },
};

// ---------- 广播与连接 ----------
static int gap_event_cb(struct ble_gap_event *event, void *arg);

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(0x1812) };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.appearance = 0x03C1; // 键盘外观,确保安卓按输入设备处理
    fields.appearance_is_present = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv set fields rc=%d", rc); return; }

    struct ble_hs_adv_fields rsp = {0};
    rsp.name = (uint8_t *)DEVICE_NAME;
    rsp.name_len = strlen(DEVICE_NAME);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) { ESP_LOGE(TAG, "adv rsp set fields rc=%d", rc); return; }

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event_cb, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "adv start rc=%d", rc); return; }
    ESP_LOGI(TAG, "开始广播: %s", DEVICE_NAME);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_encrypted = false;
            s_sub_consumer = false;
            s_sub_keyboard = false;
            ESP_LOGI(TAG, "已连接 handle=%d,等待加密/订阅", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "连接失败 status=%d,恢复广播", event->connect.status);
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "断开 reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_encrypted = false;
        s_sub_consumer = false;
        s_sub_keyboard = false;
        notify_state();
        start_advertising();
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            s_encrypted = true;
            ESP_LOGI(TAG, "链路已加密/绑定完成");
            notify_state();
        } else {
            ESP_LOGW(TAG, "加密失败 status=%d", event->enc_change.status);
        }
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_consumer_val_handle) {
            s_sub_consumer = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_keyboard_val_handle) {
            s_sub_keyboard = event->subscribe.cur_notify;
        }
        if (s_sub_consumer || s_sub_keyboard) notify_state();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) { ESP_LOGE(TAG, "ensure_addr rc=%d", rc); return; }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "infer_auto rc=%d", rc); return; }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ---------- 对外 API ----------
bool hid_remote_ready(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_encrypted &&
           (s_sub_consumer || s_sub_keyboard);
}

static esp_err_t notify_report(uint16_t val_handle, const uint8_t *data, size_t len)
{
    if (!hid_remote_ready()) return ESP_ERR_INVALID_STATE;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(s_conn_handle, val_handle, om);
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t hid_remote_send_consumer(uint16_t usage)
{
    uint8_t press[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
    uint8_t release[2] = {0, 0};
    esp_err_t err = notify_report(s_consumer_val_handle, press, sizeof(press));
    if (err != ESP_OK) return err;
    return notify_report(s_consumer_val_handle, release, sizeof(release));
}

esp_err_t hid_remote_send_key(uint8_t keycode)
{
    uint8_t press[8] = {0, 0, keycode, 0, 0, 0, 0, 0};
    uint8_t release[8] = {0};
    esp_err_t err = notify_report(s_keyboard_val_handle, press, sizeof(press));
    if (err != ESP_OK) return err;
    return notify_report(s_keyboard_val_handle, release, sizeof(release));
}

void hid_remote_set_battery(uint8_t percent)
{
    if (percent > 100) percent = 100;
    s_battery_level = percent;
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_battery_level, 1);
        if (om) ble_gatts_notify_custom(s_conn_handle, s_battery_val_handle, om);
    }
}

esp_err_t hid_remote_init(hid_remote_state_cb_t cb, void *user)
{
    s_state_cb = cb;
    s_state_user = user;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err)); return err; }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT; // Just Works
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc); return ESP_FAIL; }

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) { ESP_LOGE(TAG, "device_name_set rc=%d", rc); return ESP_FAIL; }

    ble_store_config_init();
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "HID 遥控器已启动");
    return ESP_OK;
}

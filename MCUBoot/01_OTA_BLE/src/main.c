/**
 * @file main.c
 * @brief BLE Dynamic Beacon Example using Zephyr
 *
 * This program demonstrates how to set up a BLE beacon in Zephyr,
 * advertise manufacturer data and a URI, and handle connection/
 * disconnection events with automatic advertising restart.
 *
 * Later, the updated zephyr.signed.bin will be uploaded from the
 * nRF Connect App for DFU (Device Firmware Update). During this
 * process, the advertised name will change from "Test 1.0" to
 * "Test 1.1".
 */

/* ========================== HEADER FILES ========================== */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

/* ========================== MACROS & CONSTANTS ========================== */

/* Device name from Zephyr Kconfig (set via prj.conf) */
#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

/* Company Identifier Code (Assigned Numbers) */
#define COMPANY_ID_CODE     0x02E5  /* Espressif Systems (Shanghai) Co., Ltd. */

/* ========================== LOGGING MODULE ========================== */
LOG_MODULE_REGISTER(DFU_OTA_BLE , LOG_LEVEL_DBG);

/* ========================== DATA STRUCTURES ========================== */

/**
 * @brief Manufacturer-specific advertising data format
 *
 * You can extend this structure for custom beacon payloads.
 */
typedef struct {
    uint16_t company_code;   /* Company Identifier Code (16-bit) */
    uint16_t count;          /* Example counter field */
} adv_mfg_data_type;

/* ========================== GLOBAL VARIABLES ========================== */

char addr_s[BT_ADDR_LE_STR_LEN];     /* String to hold BT address */
bt_addr_le_t addr = {0};             /* Local device Bluetooth address */
size_t count = 1;                    /* Used in bt_id_get() */

/* Advertising parameters:
 * - Connectable advertising
 * - Interval ~100ms (160 * 0.625ms = 100ms)
 */
static const struct bt_le_adv_param *adv_param =
    BT_LE_ADV_PARAM(
        BT_LE_ADV_OPT_CONNECTABLE,  /* Allow connections */
        260,                        /* Min interval = 260 * 0.625ms = 162.5ms */
        261,                        /* Max interval = 261 * 0.625ms = 163.125ms */
        NULL                        /* Undirected advertising */
    );

/* Manufacturer data structure (example) */
static adv_mfg_data_type adv_mfg_data = {
    .company_code = COMPANY_ID_CODE,
    .count        = 0x00
};

/* For demo: only advertise company code (instead of full struct) */
static uint16_t data = COMPANY_ID_CODE;

/* ========================== ADVERTISING DATA ========================== */

/* Primary advertising payload */
static const struct bt_data ad[] = {
    /* Device Name */
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

    /* General discoverable flag */
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL),

    /* Manufacturer-specific data (only company ID here) */
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&data, sizeof(data)),
    // Alternative (uncomment to use full struct):
    // BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data)),
};

/* Example URI (GitHub profile link) */
static unsigned char url_data[] = {
    0x17, '/', '/', 'g', 'i', 't', 'h', 'u', 'b', '.', 'c', 'o', 'm',
    '/', 'D', 'i', 's', 'h', 'a', 'n', 't', '7', '9', '9', '0'
};

/* Scan Response Data (advertises URI) */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),
};

/* ========================== CALLBACKS ========================== */

/**
 * @brief Bluetooth stack ready callback
 *
 * This is called once the Bluetooth subsystem is enabled.
 */
static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    /* Start advertising */
    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    /* Get local identity address */
    bt_id_get(&addr, &count);
    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

    LOG_INF("Beacon started, advertising as %s", addr_s);
}

/**
 * @brief Connection established callback
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
    }
}

/**
 * @brief Disconnection callback
 *
 * Automatically restarts advertising after disconnection.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);

    /* Stop advertising first */
    bt_le_adv_stop();
    LOG_INF("Advertising stopped");

    /* Small delay before restart */
    k_sleep(K_MSEC(100));

    /* Restart advertising */
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to restart (err %d)", err);
    } else {
        LOG_INF("Advertising restarted");
    }
}

/* Register connection callbacks */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* ========================== MAIN APPLICATION ========================== */

int main(void)
{
    int err;

    LOG_INF("Dynamic Beacon Demo Started");

    /* Enable Bluetooth subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
    }

    /* Log sizes of advertisement & scan response */
    LOG_INF("Advertising packet size = %d", sizeof(ad));
    LOG_INF("Scan response packet size = %d", sizeof(sd));

    /* Periodic logging loop */
    while (1) {
        LOG_INF("BLE Device Name : %s", DEVICE_NAME);

        bt_id_get(&addr, &count);
        bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));
        LOG_INF("Beacon active, advertising as %s", addr_s);

        k_sleep(K_MSEC(1000));
    }

    return 0;
}

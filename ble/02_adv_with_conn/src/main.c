/**
 * @file main.c
 * @brief BLE Connectable Advertising Demo using Zephyr
 *
 * This program demonstrates how to set up a BLE peripheral in Zephyr
 * that performs **connectable advertising** with custom parameters.
 *
 * Features included:
 *  - Uses a fixed random static Bluetooth address created at runtime.
 *  - Configures connectable advertising with a defined advertising interval.
 *  - Advertises:
 *      - Complete device name
 *      - Manufacturer-specific data (Company ID)
 *  - Includes a URI (e.g., GitHub profile link) in the scan response.
 *  - Handles connection and disconnection events with callbacks.
 *  - Restarts advertising automatically after disconnection.
 *  - Uses Zephyr’s work queue (`k_work`) to defer advertising start.
 */

/* ========================== HEADER FILES ========================== */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

/* ========================== LOGGING MODULE ========================== */
LOG_MODULE_REGISTER(Connectable_Adv, LOG_LEVEL_DBG);

/* ========================== MACROS & CONSTANTS ========================== */

/* Device name defined via Kconfig (CONFIG_BT_DEVICE_NAME in prj.conf) */
#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

/* Company Identifier Code (Bluetooth SIG Assigned Numbers) */
#define COMPANY_ID_CODE     0x02E5  /* Espressif Systems (Shanghai) Co., Ltd. */

/* ========================== ADVERTISING PARAMETERS ========================== */

/**
 * @brief Advertising parameters
 *
 * - Connectable advertising
 * - Uses identity address
 * - Advertising interval ~162.5 ms
 */
static const struct bt_le_adv_param *adv_param = 
    BT_LE_ADV_PARAM(
        (_BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising */
        800,    /*Min Advertising Interval 500ms (800*0.625ms) */
        801,    /*Max Advertising Interval 500.625ms (801*0.625ms)*/
        NULL    /* Undirected advertising */
    );

/* ========================== GLOBAL VARIABLES ========================== */

/* Manufacturer-specific data: Company ID */
static uint16_t data = COMPANY_ID_CODE;

/* Work structure for deferred advertising start */
static struct k_work adv_work;

/* ========================== ADVERTISING DATA ========================== */

/* Primary advertising data */
static const struct bt_data ad[] = {
    /* Complete device name */
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

    /* Flags: General Discoverable Mode, BR/EDR not supported */
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

    /* Manufacturer-specific data */
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (const uint8_t *)&data, sizeof(data)),
};

/* Example URI for Scan Response (e.g., GitHub profile link) */
static const unsigned char url_data[] = {
    0x17, '/', '/', 'g', 'i', 't', 'h', 'u', 'b', '.', 'c', 'o', 'm',
    '/', 'D', 'i', 's', 'h', 'a', 'n', 't', '7', '9', '9', '0'
};

// static const uint8_t lbs_uuid[16] = {
//     0x23, 0x15, 0x00, 0x00,
//     0x12, 0x12,
//     0xde, 0xef,
//     0x23, 0x15,
//     0x7f, 0x5e, 0xab, 0xcd, 0xd1, 0x23
// };
    
/* Scan response data */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),

    /* If you want to add a 128-bit UUID, define it as a byte array and add it here */
    // BT_DATA(BT_DATA_UUID128_ALL, lbs_uuid, sizeof(lbs_uuid)),
};

/* ========================== FUNCTION DEFINITIONS ========================== */

/**
 * @brief Advertising work handler to start advertising asynchronously.
 */
static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

/**
 * @brief Submit work item to start advertising.
 */
static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

/* ========================== CONNECTION CALLBACKS ========================== */

/**
 * @brief Callback function invoked when a connection is established.
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
    } else {
        LOG_INF("Connected");
    }
}

/**
 * @brief Callback function invoked when a connection is disconnected.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);

    /* Restart advertising to allow new connections */
    advertising_start();
}

/* Register connection callbacks */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ========================== MAIN FUNCTION ========================== */

int main(void)
{
    int err;

    LOG_INF("Connectable Advertising Demo Started");

    bt_addr_le_t addr;
    err = bt_addr_le_from_str("DD:11:07:20:03:AA", "random", &addr);
    if (err) {
        LOG_ERR("Invalid BT address (err %d)", err);
    }

    err = bt_id_create(&addr, NULL);
    if (err) {
        LOG_ERR("Creating new ID failed (err %d)", err);
    }

    /* Initialize Bluetooth subsystem */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }
    LOG_INF("Bluetooth initialized");

    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* Initialize advertising work */
    k_work_init(&adv_work, adv_work_handler);

    /* Start advertising */
    advertising_start();

    /* Main loop: periodically log device name */
    while (1) {
        LOG_INF("Bluetooth Device name: %s", DEVICE_NAME);
        k_sleep(K_SECONDS(3));
    }

    return 0;
}

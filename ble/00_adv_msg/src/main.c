#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Include bluetooth related necessary header files
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

LOG_MODULE_REGISTER(Beacon, LOG_LEVEL_INF);

#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

// Declare the advertising packet
static const struct bt_data ad[] = {
    // Set the advertising flags
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),

    // Set the advertising packet data
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// Set scan responce data
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0Xaa, 0xfe),

    BT_DATA_BYTES(BT_DATA_SVC_DATA16,
                0xaa, 0xfe, // Eddystone UUID
                0x10,       // Eddystone-URL frame type
                0x00,       // Calibrated Tx-power at 0m 
                0x00,       // URL scheme prefix http://www. 
                'z', 'e', 'p', 'h', 'y', 'r',
                'p', 'r', 'o', 'j', 'e', 'c', 't',
                0x08)       // .org
};

static void bt_ready(int err) {
    char addr_s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t addr = {0};
    size_t count = 1;

    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized !!");

    /* Start advertising */
    err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    /**
     * For conncetable advertising you would use
     * bt_le_oob_get_local(). For non-conncetable non-identity
     * advertising on non-resolveable private address is used;
     * there is no API to retrieve that.
     */
    bt_id_get(&addr, &count);
    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

    LOG_INF("Beacon started, advertising as %s", addr_s);
}

int main(void) {
    int err;

    LOG_INF("Starting Beacon Demo");

    /* Initialize the bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Failed to initialize Bluetooth (err %d)", err);
    }

    
    return 0;
} 
#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Include nluetooth related necessary header files */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

LOG_MODULE_REGISTER(Dynamic_Beacon, LOG_LEVEL_DBG);

/* Define the device name from Kconfig and calculate its length */
#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) -1)

/* Define the company identifier (Company ID) 
 * Which you can find here : 
 * https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf 
 */
#define COMPANY_ID_CODE     0x02E5 // Espressif Systems (Shanghai) Co., Ltd.

/* Declare the structure for your custom data */
typedef struct {
    uint16_t company_code;  /* Company Identifier Code */
    uint16_t count;         /* Counter */
} adv_mfg_data_type;

/* Create an LE Advertising Parameters variable */
static const struct bt_le_adv_param *adv_param = 
    BT_LE_ADV_PARAM(
        BT_LE_ADV_OPT_NONE,     /* No options specified */
        160,                   /* Min Advertising Interval 100ms (160 * 0.625) */
        161,                   /* Max Advertising Interval 100.625ms (161 * 0.625) */
        NULL                    /* Set to NULL for undefined advertising */
    );

/* Define and initialize a variable of type adv_mfg_data_type */
static adv_mfg_data_type adv_mfg_data = {COMPANY_ID_CODE, 0x00};

/* Set Advertising data */
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),    
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data)),
};

static unsigned char url_data[] = {
    0x17, '/', '/', 'g', 'i', 't', 'h', 'u', 'b', '.', 'c', 'o', 'm',
    '/', 'D', 'i', 's', 'h', 'a', 'n', 't', '7', '9', '9', '0'
};

/* Set scan response data */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_URI, url_data, sizeof(url_data)),
};

static void bt_ready(int err)
{
    char addr_s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t addr = {0};
    size_t count = 1;

    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    /* Starting advertising */
    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    /* For connectable advertising you would use
	 * bt_le_oob_get_local().  For non-connectable non-identity
	 * advertising an non-resolvable private address is used;
	 * there is no API to retrieve that.
	 */

    bt_id_get(&addr, &count);
    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

    LOG_INF("Beacon started, advertising as %s", addr_s);
}

int main(void)
{
    int err;

    LOG_INF("Dynamically Update the message demo");

    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
    }

    LOG_INF("Advertising packet size = %d", sizeof(ad));
    LOG_INF("Scan responce packet size = %d", sizeof(sd));

    while(1) {
        if(adv_mfg_data.count >= 500){
            adv_mfg_data.count = 0;
        } else {
            adv_mfg_data.count += 3;
        }

        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

        k_msleep(1000);
    }

    return 0;
}
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcuboot, LOG_LEVEL_INF);

int main(void) {
    while(1) {
        LOG_INF("Hello world.. mcuboot\n");
        k_sleep(K_SECONDS(2));
    }

    return 0;
}
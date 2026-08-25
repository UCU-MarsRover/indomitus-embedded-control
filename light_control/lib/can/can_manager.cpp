// Stubbed can manager so builds in workspace won't pull complex cross-project code.
#include "can_driver.hpp"
#include "freertos/queue.h"

void can_rx_task(void*) { vTaskDelay(portMAX_DELAY); }
void can_tx_enqueue(uint32_t, const uint8_t*, uint8_t) { }
void can_tx_task(void*) { vTaskDelay(portMAX_DELAY); }
void can_monitor_task(void*) { vTaskDelay(portMAX_DELAY); }

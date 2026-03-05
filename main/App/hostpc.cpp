#include "hostpc.hpp"
#include "uart.hpp"

void HostPC::HostPCTask(void *pvParameters)
{
    HostPC &hostPC = HostPC::GetInstance();

    hostPC.muart1.initialize();
    // muart2.initialize();
    // hostPC.muart1.write((const uint8_t *)"hello world1\n", strlen("hello world1\n"));
    // muart2.write((const uint8_t *)"hello world2\n", strlen("hello world2\n"));

    //========================================================
    xSemaphoreGive(rtoshandler.xInitCountSem);
    xEventGroupWaitBits(rtoshandler.xStartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(HostPC::TAG, "HostPCTask started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
        BaseType_t ret = xQueueReceive(rtoshandler.xSensorQueue, &hostPC.imuAttitude, 0);
        if (ret != pdPASS) {
            continue;
        }
        else {
            hostPC.muart1.write((uint8_t *)&hostPC.imuAttitude, sizeof(hostPC.imuAttitude));
        }
    }
    vTaskDelete(NULL);
}

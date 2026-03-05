#include "hostpc.hpp"
#include "uart.hpp"

void HostPC::HostPCTask(void *pvParameters)
{
    HostPC &hostPC = HostPC::GetInstance();

    // uart muart2(GPIO_NUM_12, GPIO_NUM_7, 115200, UART_NUM_2);
    hostPC.muart1.initialize();
    // muart2.initialize();
    hostPC.muart1.write((const uint8_t *)"hello world1\n", strlen("hello world1\n"));
    // muart2.write((const uint8_t *)"hello world2\n", strlen("hello world2\n"));

    //========================================================
    xSemaphoreGive(xInitCountSem);
    xEventGroupWaitBits(xStartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(HostPC::TAG, "HostPCTask started");

    while (1) {
        BaseType_t ret = xQueueReceive(xSensorQueue, &hostPC.imuAttitude, portMAX_DELAY);
        if (ret != pdPASS) {
            continue;
        }
        else {
            hostPC.muart1.write((uint8_t *)&hostPC.imuAttitude, sizeof(hostPC.imuAttitude));
        }
    }
}

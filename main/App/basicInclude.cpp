#include "basicInclude.hpp"

rtosHandler rtoshandler = {
    .xImuQueue = NULL,
    .xBaroQueue = NULL,
    .xInitCountSem = NULL,
    .xStartSyncGroup = NULL,
};

esp_err_t rtosHandlerInit(void)
{
    rtoshandler.xImuQueue = xQueueCreate(10, sizeof(xAxisIMU::IMUAttitude));
    rtoshandler.xBaroQueue = xQueueCreate(10, sizeof(sensor::MS5611::ConvertData));
    rtoshandler.xInitCountSem = xSemaphoreCreateCounting(10, 0);
    rtoshandler.xStartSyncGroup = xEventGroupCreate();
    if (rtoshandler.xImuQueue == NULL || rtoshandler.xBaroQueue == NULL || rtoshandler.xInitCountSem == NULL ||
        rtoshandler.xStartSyncGroup == NULL) {
        ESP_LOGE("rtosHandler", "Failed to create RTOS handler");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
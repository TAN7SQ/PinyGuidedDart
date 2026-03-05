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

uint16_t Tools::crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}
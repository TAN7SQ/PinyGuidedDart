#include "basicInclude.hpp"

rtosHandler rtoshandler = {
    .imuQueue = NULL,
    .BaroQueue = NULL,
    .ControlQueue = NULL,

    .InitCountSem = NULL,
    .StartSyncGroup = NULL,
};

esp_err_t rtosHandlerInit(void)
{
    rtoshandler.imuQueue = xQueueCreate(10, sizeof(xAxisIMU::IMUAttitude));
    rtoshandler.BaroQueue = xQueueCreate(10, sizeof(sensor::MS5611::ConvertData));
    rtoshandler.ControlQueue = xQueueCreate(10, sizeof(Comm::ControlData));

    rtoshandler.InitCountSem = xSemaphoreCreateCounting(10, 0);
    rtoshandler.StartSyncGroup = xEventGroupCreate();
    if (rtoshandler.imuQueue == NULL || rtoshandler.BaroQueue == NULL || rtoshandler.InitCountSem == NULL ||
        rtoshandler.StartSyncGroup == NULL) {
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
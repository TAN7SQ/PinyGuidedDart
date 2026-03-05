#include "hostpc.hpp"
#include "uart.hpp"

void HostPC::HostPCTask(void *pvParameters)
{
    HostPC &hostPC = HostPC::GetInstance();

    ESP_LOGI(HostPC::TAG, "HostPCTask init");

    //========================================================
    xSemaphoreGive(rtoshandler.xInitCountSem);
    xEventGroupWaitBits(rtoshandler.xStartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(HostPC::TAG, "HostPCTask started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2));
        if (xQueueReceive(rtoshandler.xImuQueue, &hostPC.imuAttitude, 0) == pdPASS) {
            hostPC.sendData(msgType_e::MSG_IMU, &hostPC.imuAttitude, sizeof(hostPC.imuAttitude));
        }
        if (xQueueReceive(rtoshandler.xBaroQueue, &hostPC.baro, 0) == pdPASS) {
            hostPC.sendData(msgType_e::MSG_BARO, &hostPC.baro, sizeof(hostPC.baro));
        }
        // ...
    }
    vTaskDelete(NULL);
}

esp_err_t HostPC::sendData(msgType_e type, void *payload, uint16_t payloadLength)
{
    if (payload == nullptr || payloadLength == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // frame = header + type + len(2) + payload + crc(2)
    const uint8_t HEAD = 0xAA;

    uint16_t frameLength = 1 + 1 + 2 + payloadLength + 2;
    if (frameLength > 256) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t buffer[256];
    uint16_t index = 0;

    buffer[index++] = HEAD;
    buffer[index++] = (uint8_t)type;
    buffer[index++] = payloadLength & 0xFF;
    buffer[index++] = (payloadLength >> 8) & 0xFF;
    memcpy(&buffer[index], payload, payloadLength);
    index += payloadLength;
    uint16_t crc = Tools::crc16_ccitt(buffer, index);
    buffer[index++] = crc & 0xFF;
    buffer[index++] = (crc >> 8) & 0xFF;

    int ret = 0;
    if (type > msgType_e::MSG_IMU) {
        ret = muart1.write(buffer, index);
    }
    else {
        ret = muart2.write(buffer, index);
    }

    if (ret < 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}
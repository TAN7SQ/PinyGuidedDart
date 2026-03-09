#include "hostpc.hpp"
#include "uart.hpp"

void HostPC::HostPCTask(void *pvParameters)
{
    HostPC &hostPC = HostPC::GetInstance();

    ESP_LOGI(HostPC::TAG, "HostPCTask init");

    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(HostPC::TAG, "HostPCTask started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2));
        if (xQueueReceive(rtoshandler.imuQueue, &hostPC.imuAttitude, 0) == pdPASS) {
            // hostPC.sendData(msgType::MSG_IMU, &hostPC.imuAttitude, sizeof(hostPC.imuAttitude));
        }
        if (xQueueReceive(rtoshandler.BaroQueue, &hostPC.baro, 0) == pdPASS) {
            // hostPC.sendData(msgType::MSG_BARO, &hostPC.baro, sizeof(hostPC.baro));
        }
        // ...
    }
    vTaskDelete(NULL);
}

esp_err_t HostPC::sendData(msgType type, void *payload, uint16_t payloadLength)
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
    if (type > msgType::MSG_IMU) {
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

esp_err_t HostPC::receiveData(void *payload, uint16_t *payloadLength)
{
    if (payload == nullptr || payloadLength == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t HEAD = 0xAA;
    uint8_t head;
    int ret;
    while (true) {
        ret = muart1.read(&head, 1);
        if (ret <= 0) {
            return ESP_FAIL;
        }
        if (head == HEAD) {
            break;
        }
    }

    uint8_t type;
    ret = muart1.read(&type, 1);
    if (ret <= 0) {
        return ESP_FAIL;
    }
    uint8_t lenBuf[2];
    ret = muart1.read(lenBuf, 2);
    if (ret <= 0) {
        return ESP_FAIL;
    }
    uint16_t len = lenBuf[0] | (lenBuf[1] << 8);
    if (len > 256) {
        return ESP_ERR_INVALID_SIZE;
    }
    ret = muart1.read((uint8_t *)payload, len);
    if (ret <= 0) {
        return ESP_FAIL;
    }
    uint8_t crcBuf[2];
    ret = muart1.read(crcBuf, 2);
    if (ret <= 0) {
        return ESP_FAIL;
    }

    uint16_t recv_crc = crcBuf[0] | (crcBuf[1] << 8);
    uint8_t crcFrame[256];
    uint16_t index = 0;
    crcFrame[index++] = HEAD;
    crcFrame[index++] = type;
    crcFrame[index++] = lenBuf[0];
    crcFrame[index++] = lenBuf[1];
    memcpy(&crcFrame[index], payload, len);
    index += len;
    uint16_t calc_crc = Tools::crc16_ccitt(crcFrame, index);
    if (calc_crc != recv_crc) {
        ESP_LOGW(TAG, "CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    *payloadLength = len;
    return parseData((msgType)type, payload, len);
}

esp_err_t HostPC::parseData(const msgType type, void *payload, uint16_t payloadLength)
{
    if (payload == nullptr || payloadLength == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (type) {
    case MSG_CONTROL: {
        if (payloadLength != sizeof(Comm::ControlData)) {
            ESP_LOGW(TAG, "ControlData size mismatch");
            return ESP_ERR_INVALID_SIZE;
        }
        Comm::ControlData ctrl;
        memcpy(&ctrl, payload, sizeof(ctrl));
        xQueueSend(rtoshandler.ControlQueue, &ctrl, 0);
        break;
    }
    case MSG_SYSTEM: {
        ESP_LOGI(TAG, "System message received");
        break;
    }
    default:
        ESP_LOGW(TAG, "Unknown msg type: %d", type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

#include "hostpc.hpp"
#include "uart.hpp"

void HostPC::run()
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
        if (xQueueReceive(rtoshandler.imuQueueFiltered, &hostPC.imuAttitude, 0) == pdPASS) {
            hostPC.sendData(MsgType::ATTITUDE, &hostPC.imuAttitude, sizeof(hostPC.imuAttitude));
        }
        if (xQueueReceive(rtoshandler.BaroQueue, &hostPC.baro, 0) == pdPASS) {
            hostPC.sendData(MsgType::BARO, &hostPC.baro, sizeof(hostPC.baro));
        }
    }
    vTaskDelete(NULL);
}

esp_err_t HostPC::sendData(MsgType type, const void *payload, uint16_t payloadLength)
{
    if (payload == nullptr || payloadLength == 0)
        return ESP_ERR_INVALID_ARG;

    const uint8_t SOF = 0xAA;

    // SOF | TYPE | LEN | PAYLOAD | CRC16
    uint16_t frameLength = 1 + 1 + 2 + payloadLength + 2;

    if (frameLength > 256)
        return ESP_ERR_INVALID_SIZE;

    uint8_t buffer[frameLength];
    uint16_t index = 0;

    buffer[index++] = SOF;
    buffer[index++] = (uint8_t)type;

    buffer[index++] = payloadLength & 0xFF;
    buffer[index++] = (payloadLength >> 8) & 0xFF;

    memcpy(&buffer[index], payload, payloadLength);
    index += payloadLength;

    uint16_t crc = Tools::crc16_ccitt(buffer, index);

    buffer[index++] = crc & 0xFF;
    buffer[index++] = crc >> 8;

    int ret = muart1.write(buffer, index);

    return (ret < 0) ? ESP_FAIL : ESP_OK;
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
    return parseData((MsgType)type, payload, len);
}

esp_err_t HostPC::parseData(const MsgType type, void *payload, uint16_t payloadLength)
{
    if (payload == nullptr || payloadLength == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (type) {
    case MsgType::CONTROL: {
        if (payloadLength != sizeof(Comm::ControlCmd_t)) {
            ESP_LOGW(TAG, "ControlCmd_t size mismatch");
            return ESP_ERR_INVALID_SIZE;
        }
        Comm::ControlCmd_t ctrl;
        memcpy(&ctrl, payload, sizeof(ctrl));
        xQueueSend(rtoshandler.ControlQueue, &ctrl, 0);
        break;
    }
    case MsgType::SYSTEM: {
        ESP_LOGI(TAG, "System message received");
        break;
    }
    default:
        ESP_LOGW(TAG, "Unknown msg type: %d", (int)type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

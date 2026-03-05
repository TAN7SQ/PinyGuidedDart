#include "control.hpp"
#include "servo.hpp"
void Control::ControlTask(void *pvParameters)
{
    Control &_control = Control::GetInstance();
    xAxisIMU::IMUAttitude imuAttitude;
    _control.mServo.Initialize();
    //========================================================
    xSemaphoreGive(xInitCountSem);
    xEventGroupWaitBits(xStartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(TAG, "ControlTask Start");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
        BaseType_t ret = xQueueReceive(xSensorQueue, &imuAttitude, 0);
        if (ret != pdPASS) {
            continue;
        }
        else {
        }
    }
    vTaskDelete(NULL);
}
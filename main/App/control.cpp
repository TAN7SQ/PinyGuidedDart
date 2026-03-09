#include "control.hpp"
#include "servo.hpp"
void Control::ControlTask(void *pvParameters)
{
    Control &_control = Control::GetInstance();
    xAxisIMU::IMUAttitude imuAttitude;
    _control.mServo.Initialize();
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(TAG, "ControlTask Start");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
        uint16_t angles[Servo::ALL] = {30, 30, 30, 30, 30, 30};
        _control.mServo.SetAngles(angles);
        BaseType_t ret = xQueueReceive(rtoshandler.imuQueue, &imuAttitude, 0);
        if (ret != pdPASS) {
            continue;
        }
        else {
        }
    }
    vTaskDelete(NULL);
}
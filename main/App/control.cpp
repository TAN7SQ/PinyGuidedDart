#include "control.hpp"
#include "servo.hpp"
void ControlTask(void *pvParameters)
{
    xAxisIMU::IMUAttitude imuAttitude;
    Servo mServo;
    mServo.Initialize();
    //========================================================
    xSemaphoreGive(xInitCountSem);
    xEventGroupWaitBits(xStartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
        BaseType_t ret = xQueueReceive(xSensorQueue, &imuAttitude, 0);
        if (ret != pdPASS) {
            continue;
        }
        else {
        }
    }
}
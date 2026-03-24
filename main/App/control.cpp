#include "control.hpp"
#include "servo.hpp"

void Control::run()
{
    Control &_control = Control::GetInstance();

    _control.mServo.Initialize();
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(TAG, "ControlTask Start");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2));

        if (uxQueueMessagesWaiting(rtoshandler.imuQueueFiltered)) {
            xQueueReceive(rtoshandler.imuQueueFiltered, &imuAttitude, 0);
        }
        if (uxQueueMessagesWaiting(rtoshandler.imuQueueRaw)) {
            xQueueReceive(rtoshandler.imuQueueRaw, &imuRawData, 0);
        }
        if (uxQueueMessagesWaiting(rtoshandler.ControlQueue)) {
            xQueueReceive(rtoshandler.ControlQueue, &bodyTarget, 0);
        }
        update();
    }
    vTaskDelete(NULL);
}

void Control::update()
{
    float yaw_rate_cmd = 0;
    float pitch_rate_cmd = 0;

    if (bodyTarget.valid) {
        yaw_rate_cmd = bodyTarget.yaw_error_rate;
        pitch_rate_cmd = bodyTarget.pitch_error_rate;
    }
    else {
        // 无目标：阻尼
        yaw_rate_cmd = -params.K_damp * imuRawData.gyro.z;
        pitch_rate_cmd = -params.K_damp * imuRawData.gyro.y;
    }

    float yaw_rate_err = yaw_rate_cmd - imuRawData.gyro.z;
    float pitch_rate_err = pitch_rate_cmd - imuRawData.gyro.y;

    float yaw_out = params.Kp_rate * yaw_rate_err + params.Kd_rate * (yaw_rate_err - params.last_yaw_err);

    float pitch_out = params.Kp_rate * pitch_rate_err + params.Kd_rate * (pitch_rate_err - params.last_pitch_err);

    params.last_yaw_err = yaw_rate_err;
    params.last_pitch_err = pitch_rate_err;

    yaw_out = std::clamp(yaw_out, -params.max_output, params.max_output);
    pitch_out = std::clamp(pitch_out, -params.max_output, params.max_output);

    // ===== X翼混控 =====
    float s1 = pitch_out + yaw_out;
    float s2 = pitch_out - yaw_out;
    float s3 = -pitch_out - yaw_out;
    float s4 = -pitch_out + yaw_out;

    mServo.SetAngle(Servo::LB0, s1);
    mServo.SetAngle(Servo::LB1, s2);
    mServo.SetAngle(Servo::RB0, s3);
    mServo.SetAngle(Servo::RB1, s4);
}
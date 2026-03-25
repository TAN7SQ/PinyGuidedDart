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
    int angle = 0;
    while (1) {
        lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));

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
        yaw_rate_cmd = -params.K_damp * imuRawData.gyro.yaw;
        pitch_rate_cmd = -params.K_damp * imuRawData.gyro.pitch;
    }

    // 防抖
    if (fabs(yaw_rate_cmd) < 0.02f)
        yaw_rate_cmd = 0;
    if (fabs(pitch_rate_cmd) < 0.02f)
        pitch_rate_cmd = 0;

    float yaw_rate_err = yaw_rate_cmd - imuRawData.gyro.yaw;
    float pitch_rate_err = pitch_rate_cmd - imuRawData.gyro.pitch;

    float yaw_out = params.Kp_rate * yaw_rate_err + params.Kd_rate * (yaw_rate_err - params.last_yaw_err);
    float pitch_out = params.Kp_rate * pitch_rate_err + params.Kd_rate * (pitch_rate_err - params.last_pitch_err);
    params.last_yaw_err = yaw_rate_err;
    params.last_pitch_err = pitch_rate_err;
    yaw_out = std::clamp(yaw_out, -params.max_output, params.max_output);
    pitch_out = std::clamp(pitch_out, -params.max_output, params.max_output);

    const float SERVO_MAX_DEFLECTION = 45.0f;

    // 线性映射：[-max_output, max_output] -> [-SERVO_MAX_DEFLECTION, SERVO_MAX_DEFLECTION]
    float yaw_servo = (yaw_out / params.max_output) * SERVO_MAX_DEFLECTION;
    float pitch_servo = (pitch_out / params.max_output) * SERVO_MAX_DEFLECTION;

    printf("%.3f,%.3f\n", yaw_out, pitch_out);

    // ===== X翼混控 =====
    /*
        HEAD
    CH2     CH1
     \     /
      \   /
       \ /
       / \
      /   \
     /     \
    CH3     CH4
      BACK
    */
    float s1 = pitch_servo + yaw_servo;
    float s2 = pitch_servo - yaw_servo;
    float s3 = -pitch_servo - yaw_servo;
    float s4 = -pitch_servo + yaw_servo;

    mServo.SetDelta(Servo::CH1, s1);
    mServo.SetDelta(Servo::CH2, s2);
    mServo.SetDelta(Servo::CH3, s3);
    mServo.SetDelta(Servo::CH4, s4);
}

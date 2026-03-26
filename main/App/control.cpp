#include "control.hpp"
#include "servo.hpp"

void Control::run()
{
    Control &_control = Control::GetInstance();
    params.yawLPF.setAlpha(0.9f);
    params.pitchLPF.setAlpha(0.9f);
    params.rollLPF.setAlpha(0.9f);
    _control.mServo.Initialize();
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI(TAG, "ControlTask Start");
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
    uint32_t now = xTaskGetTickCount();
    float dt = (now - params.last_time) * 0.001f;
    params.last_time = now;

    dt = std::clamp(dt, 0.002f, 0.02f);

    float yaw_rate_cmd = 0;
    float pitch_rate_cmd = 0;
    float roll_rate_cmd = 0;

    if (bodyTarget.valid) {
        yaw_rate_cmd = bodyTarget.yaw_rate_cmd;
        pitch_rate_cmd = bodyTarget.pitch_rate_cmd;
    }
    else {
        yaw_rate_cmd = -params.K_damp * imuRawData.gyro.yaw;
        pitch_rate_cmd = -params.K_damp * imuRawData.gyro.pitch;
    }

    // roll阻尼控制
    roll_rate_cmd = -params.K_roll_damp * imuRawData.gyro.roll;
    // 防抖
    if (fabsf(yaw_rate_cmd) < 0.02f)
        yaw_rate_cmd = 0;
    if (fabsf(pitch_rate_cmd) < 0.02f)
        pitch_rate_cmd = 0;
    if (fabsf(roll_rate_cmd) < 0.02f)
        roll_rate_cmd = 0;

    float yaw_err = yaw_rate_cmd - imuRawData.gyro.yaw;
    float pitch_err = pitch_rate_cmd - imuRawData.gyro.pitch;
    float roll_err = roll_rate_cmd - imuRawData.gyro.roll;

    float dYaw = (yaw_err - params.last_yaw_err) / dt;
    float dPitch = (pitch_err - params.last_pitch_err) / dt;
    float dRoll = (roll_err - params.last_roll_err) / dt;

    dYaw = params.yawLPF.update(dYaw);
    dPitch = params.pitchLPF.update(dPitch);
    dRoll = params.rollLPF.update(dRoll);

    float yaw_out = params.Kp_rate * yaw_err + params.Kd_rate * dYaw;
    float pitch_out = params.Kp_rate * pitch_err + params.Kd_rate * dPitch;
    float roll_out = params.Kp_roll * roll_err + params.Kd_roll * dRoll;

    params.last_yaw_err = yaw_err;
    params.last_pitch_err = pitch_err;
    params.last_roll_err = roll_err;

    yaw_out = std::clamp(yaw_out, -1.0f, 1.0f);
    pitch_out = std::clamp(pitch_out, -1.0f, 1.0f);
    roll_out = std::clamp(roll_out, -1.0f, 1.0f);

    if (fabsf(yaw_out) < 0.02f)
        yaw_out = 0;
    if (fabsf(pitch_out) < 0.02f)
        pitch_out = 0;
    if (fabsf(roll_out) < 0.02f)
        roll_out = 0;

    // ===== 气动权重 =====
    float u_yaw = yaw_out * 1.0f;
    float u_pitch = pitch_out * 1.0f;
    float u_roll = roll_out * 0.6f;

    float s1 = u_pitch + u_yaw + u_roll;
    float s2 = u_pitch - u_yaw - u_roll;
    float s3 = -u_pitch - u_yaw + u_roll;
    float s4 = -u_pitch + u_yaw - u_roll;

    float max_abs_val = std::max({fabsf(s1), fabsf(s2), fabsf(s3), fabsf(s4)});
    float max_val = std::max(max_abs_val, 1.0f);
    s1 /= max_val;
    s2 /= max_val;
    s3 /= max_val;
    s4 /= max_val;

    // ===== X翼混控 =====
    /*
        TOP
    CH2     CH1
     \     /           ==
      \   /             ======
       \ /=====================
       / \              ======
      /   \            ==
     /     \
    CH3     CH4
      BOTTOM
    */
    const float SERVO_MAX_DEFLECTION = 65.0f;

    mServo.SetDelta(Servo::CH1, s1 * SERVO_MAX_DEFLECTION);
    mServo.SetDelta(Servo::CH2, s2 * SERVO_MAX_DEFLECTION);
    mServo.SetDelta(Servo::CH3, s3 * SERVO_MAX_DEFLECTION);
    mServo.SetDelta(Servo::CH4, s4 * SERVO_MAX_DEFLECTION);

    printf("cmd:%.3f gyro:%.3f out:%.3f\n", yaw_rate_cmd, imuRawData.gyro.z, yaw_out);
}

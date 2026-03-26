#pragma once

#include "basicInclude.hpp"
#include "servo.hpp"

#include "lpf.hpp"

/********************************************************** */
enum Mode_e
{
    Steady,
    Track,
};

class Control
{
public:
    static constexpr const char *TAG = "Control";

    static Control &GetInstance()
    {
        static Control instance;
        return instance;
    }
    void start()
    {
        Tools::taskCreate(ControlTask, "control_task", 8192, this, tskIDLE_PRIORITY + 3, NULL, 0);
    }

private:
    Control()
    {
    }

    Control(const Control &) = delete;
    Control &operator=(const Control &) = delete;

    static void ControlTask(void *param)
    {
        Control *self = static_cast<Control *>(param);
        self->run();
    }

private:
    struct ControlParams
    {
        float Kp_angle = 3.0f;

        float Kp_rate = 0.6f;  // 先调小一点，防止震荡
        float Kd_rate = 0.01f; // 稍微加大 D 项

        float Kp_roll = 0.4f;
        float Kd_roll = 0.01f;

        float K_damp = 0.4f;
        float K_roll_damp = 0.4f;

        // 关键：把 max_output 改成 1.0，方便上面的映射计算
        // 意思是：算法内部计算出来的最大值是 1.0，对应舵机打满 45度
        float max_output = 1.0f;

        float last_yaw_err = 0;
        float last_pitch_err = 0;
        float last_roll_err = 0;

        uint32_t last_time = 0;

        LPF yawLPF;
        LPF pitchLPF;
        LPF rollLPF;
    };

    /*********************************************** */

    void run(void);
    void update();

private:
    Servo mServo;

    /*********************************** */
    xAxisIMU::IMUAttitude imuAttitude;
    xAxisIMU::IMURawData imuRawData;

    Comm::ControlCmd_t bodyTarget;
    Mode_e mode = Steady;

    ControlParams params;
    /*********************************** */
    TickType_t lastWakeTime = 0;
};
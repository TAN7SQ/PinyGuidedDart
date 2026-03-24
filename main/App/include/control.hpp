#pragma once

#include "basicInclude.hpp"
#include "servo.hpp"

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
        float Kp_rate = 0.8f;
        float Kd_rate = 0.02f;

        float K_damp = 0.3f;

        float max_output = 0.5f;

        float last_yaw_err = 0;
        float last_pitch_err = 0;
    };

    /*********************************************** */

    void run(void);
    void update();

private:
    Servo mServo;

    /*********************************** */
    xAxisIMU::IMUAttitude imuAttitude;
    xAxisIMU::IMURawData imuRawData;

    Comm::BodyTarget_t bodyTarget;
    Mode_e mode = Steady;

    ControlParams params;
    /*********************************** */
};
#pragma once

#include "basicInclude.hpp"
#include "servo.hpp"

/********************************************************** */
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

    /*********************************************** */

    void run(void);
    void update();

private:
    Servo mServo;

    /*********************************** */
    xAxisIMU::IMUAttitude imuAttitude;
    xAxisIMU::IMURawData imuRawData;
};
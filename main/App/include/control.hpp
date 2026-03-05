#pragma once

#include "basicInclude.hpp"

class Control
{
public:
    static constexpr const char *TAG = "Control";
    Control() {};

    static Control &GetInstance()
    {
        static Control instance;
        return instance;
    }

    Control(const Control &) = delete;
    Control &operator=(const Control &) = delete;

    static void ControlTask(void *pvParameters);

private:
};

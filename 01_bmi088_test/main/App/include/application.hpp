#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "i2c_bus.hpp"
#include "spi_bus.hpp"

#include <iostream>

class Application
{
public:
    static constexpr const char *TAG = "Application";
    static Application &GetInstance()
    {
        static Application instance;
        return instance;
    }
    // Delete copy constructor and assignment operator
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    void Initialize();
    void Run();


private:
    Application();
    ~Application();
};

#endif // APPLICATION_HPP
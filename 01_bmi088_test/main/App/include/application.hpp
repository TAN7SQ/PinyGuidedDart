#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

#include "spi_bus.hpp"
#include "i2c_bus.hpp"

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
    // Application(const Application &) = delete;
    // Application &operator=(const Application &) = delete;
    // Application();
    // ~Application();
    void Initialize();
    void Run();

private:
  
};

#endif // APPLICATION_HPP
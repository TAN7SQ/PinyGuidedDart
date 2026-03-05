#include "basicInclude.hpp"

QueueHandle_t xSensorQueue = NULL;
SemaphoreHandle_t xInitCountSem = NULL;
EventGroupHandle_t xStartSyncGroup = NULL;
#ifndef TF_CARD_HPP
#define TF_CARD_HPP

#include "iostream"

#include "esp_log.h"

#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "esp_vfs.h"
#include <dirent.h>

#define MOUNT_POINT "/sdcard"
#define SD_TAG "SD"
#define CONFIG_SOC_SDMMC_USE_GPIO_MATRIX 1

#define BSP_SD_CLK GPIO_NUM_5
#define BSP_SD_CMD GPIO_NUM_6
#define BSP_SD_D0 GPIO_NUM_4
class TF_Card
{
public:
    TF_Card();
    ~TF_Card();
    esp_err_t Initialize();

private:
    sdmmc_card_t *card = NULL;
    const char *mountPoint = MOUNT_POINT;
};

#endif // TF_CARD_HPP
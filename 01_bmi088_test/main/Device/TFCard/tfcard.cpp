
#include "tfcard.hpp"

TF_Card::TF_Card()
{
}

void TF_Card::Initialize()
{
    ESP_LOGI(SD_TAG, "TF_Card Initialize start");
    // 1. 挂载tf卡
    esp_err_t ret = 0;
    esp_vfs_fat_sdmmc_mount_config_t mountConfig = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slotConfig = SDMMC_SLOT_CONFIG_DEFAULT();

    slotConfig.width = 1; // 单线tf卡
    slotConfig.clk = BSP_SD_CLK;
    slotConfig.cmd = BSP_SD_CMD;
    slotConfig.d0 = BSP_SD_D0;
    slotConfig.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; // 内部上拉

    ESP_LOGI(SD_TAG, "Mounting SD card to %s", mountPoint);
    ret = esp_vfs_fat_sdmmc_mount(mountPoint, &host, &slotConfig, &mountConfig, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "Failed to mount SD card, err code: %d", ret);
        return;
    }
    ESP_LOGI(SD_TAG, "SD card mounted successfully");
    sdmmc_card_print_info(stdout, card);
}

TF_Card::~TF_Card()
{
    ESP_LOGI(SD_TAG, "TF_Card destructor");
}
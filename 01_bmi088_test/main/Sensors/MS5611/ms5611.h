#ifndef MS5611_H_
#define MS5611_H_

#define MS5611_SCL_GPIO_PIN GPIO_NUM_20
#define MS5611_SDA_GPIO_PIN GPIO_NUM_21
#define MS5611_ADDR 0x77
#define IIC_CLOCK_SPEED_HZ 400 * 1000 // 400 kHz

// ==================== MS5611-01BA 命令定义 ====================
#define MS5611_CMD_RESET 0x1E      // 复位命令
#define MS5611_CMD_PROM_READ 0xA2  // 读取校准系数起始地址
#define MS5611_CMD_CONVERT_D1 0x48 // 气压转换（4096 分辨率）
#define MS5611_CMD_CONVERT_D2 0x58 // 温度转换（4096 分辨率）
#define MS5611_CMD_ADC_READ 0x00   // 读取 ADC 转换结果

#define MS5611_TAG "MS5611_I2c"

#include "esp_err.h"
#include "stdint.h"

esp_err_t i2c_init(void);
esp_err_t i2c_scan_addr(uint8_t addr);
void i2c_full_scan(void);

esp_err_t ms5611_init(void);
void ms5611_task(void *arg);
#endif // MS5611_H_
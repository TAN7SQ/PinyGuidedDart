#include "ws2812.hpp"

/**
 * 配置LED灯带的函数
 *
 * 此函数初始化并配置一个LED灯带设备，包括灯带的基本配置和RMT（Remote Control Memory-mapped Timing）配置
 * 它指定了用于控制LED灯带的GPIO引脚、LED的数量、LED模型、颜色格式以及RMT的时钟源、分辨率和内存块大小等参数
 *
 * @return led_strip_handle_t 返回LED灯带的句柄，用于后续的操作如果配置失败，将返回NULL
 */

led_strip_handle_t configure_led(void)
{
    // LED灯带的基本配置
    led_strip_config_t strip_config = {.strip_gpio_num = WS2812B_GPIO_PIN, // 指定WS2812B灯带的GPIO引脚
                                       .max_leds = WS2812B_LED_NUM,        // 指定灯带上的最大LED数量
                                       .led_model = LED_MODEL_WS2812,      // 指定LED模型为WS2812
                                       .color_component_format =
                                           LED_STRIP_COLOR_COMPONENT_FMT_GRB, // 指定颜色格式为GRB（绿红蓝）
                                       .flags = {
                                           .invert_out = false, // 不反转输出信号
                                       }};

    // RMT配置，用于生成控制LED灯带所需的精确时序
    led_strip_rmt_config_t rmt_config = {.clk_src = RMT_CLK_SRC_DEFAULT,      // 使用默认的时钟源
                                         .resolution_hz = WS2812b_RMT_RES_HZ, // 设置RMT的分辨率
                                         .mem_block_symbols = 64,             // 每个RMT内存块的符号数
                                         .flags = {
                                             .with_dma = false, // 不使用DMA（直接存储访问）传输
                                         }};

    // LED灯带的句柄，用于操作灯带
    led_strip_handle_t led_strip;
    // 创建并初始化LED灯带设备
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    // 返回LED灯带的句柄
    return led_strip;
}

void ws2812b_RGBOn(led_strip_handle_t led_strip, uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(led_strip, index, g, r, b);
    led_strip_refresh(led_strip);
}
void ws2812b_HSVOn(led_strip_handle_t led_strip, uint32_t index, uint8_t h, uint8_t s, uint8_t v)
{
    led_strip_set_pixel_hsv(led_strip, index, h, s, v);
    led_strip_refresh(led_strip);
}
void ws2812b_Off(led_strip_handle_t led_strip, uint32_t index)
{
    led_strip_set_pixel(led_strip, index, 0, 0, 0);
    led_strip_refresh(led_strip);
}

/**
 * @brief 实现WS2812B LED灯带的呼吸效果，使用HSV色彩空间
 *
 * 此函数通过不断改变LED的亮度，实现呼吸效果。它使用HSV色彩空间来设置LED的颜色，
 * 其中H（色相）和S（饱和度）是输入参数，V（亮度）则在函数内部以步进方式递增或递减。
 *
 * @param led_strip LED灯带的句柄，用于标识和操作灯带
 * @param h 色相值，决定灯光的颜色
 * @param s 饱和度，决定颜色的纯度，范围0-255
 * @param v 初始亮度值，此值将在函数中根据呼吸效果的需要进行变化
 */
void ws2812b_breath_HSV(led_strip_handle_t led_strip, int h, uint8_t s, int v, const int step = 5)
{
    // 标记亮度是增加还是减少，初始设置为增加
    bool increase = true;
    // 定义亮度变化的步长，值越大，呼吸速度越快
    // 调整步长控制呼吸速度
    // 定义亮度的最大值和最小值，以控制呼吸效果的范围
    const int max_v = 100;
    const int min_v = 0;

    // 无限循环，实现持续的呼吸效果
    while (1) {
        // 根据increase标志决定亮度是增加还是减少
        v += (increase) ? step : -step;
        // 检查亮度是否超出范围，如果超出，则进行边界值处理并反转亮度变化方向
        if (v >= max_v) {
            v = max_v;
            increase = false;
        }
        else if (v <= min_v) {
            v = min_v;
            increase = true;
        }
        // 调用函数设置LED的HSV值，此处的v需要根据实际需要映射到0-255的范围
        for (uint32_t i = 0; i < WS2812B_LED_NUM; i++) {
            ws2812b_HSVOn(led_strip, i, h, s, v);
        }
        // 延时一段时间，以控制呼吸效果的快慢，单位为毫秒
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief 为WS2812B LED灯带生成彩虹效果
 *
 * 该函数通过不断改变LED灯带上的颜色来产生彩虹效果。它使用HSV颜色空间，
 * 并逐渐改变色调值，以实现颜色的平滑过渡。此函数适用于控制连接的WS2812B LED灯带。
 *
 * @param led_strip LED灯带的句柄，用于指定要操作的灯带
 */
void ws2812b_rainbow(led_strip_handle_t led_strip)
{
    // 初始化静态变量h、s、v，分别代表色相、饱和度和亮度
    static int h = 0, s = 255, v = 50;
    // 定义色相变化的步长，决定颜色变化的速度
    static int step = 3;
    // 无限循环，持续改变LED灯带的颜色以产生彩虹效果

    // 根据当前的HSV值设置LED灯带的颜色
    for (uint32_t i = 0; i < WS2812B_LED_NUM; i++) {
        ws2812b_HSVOn(led_strip, i, h, s, v);
    } // 增加色相值，以改变颜色
    h += step;
    // 如果色相值超过360，则减去360，保持色相值在有效范围内
    if (h >= 360) {
        h = h - 360;
    }
    // 延迟50毫秒，以控制颜色变化的速率
}

#ifndef WIFI_UDP_CLIENT_HPP
#define WIFI_UDP_CLIENT_HPP

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"

#include <cctype>
#include <cstdint>
#include <string>

// UDP发送队列配置（可根据需求调整）
#define UDP_SEND_QUEUE_LEN 32     // 队列最大缓存32条数据
#define UDP_SEND_DATA_MAX_LEN 128 // 单条UDP数据最大长度
#define UDP_SEND_TASK_STACK 4096  // UDP发送任务栈大小
#define UDP_SEND_TASK_PRIO 5      // UDP发送任务优先级

// 【核心新增】WiFi+UDP+静态IP 统一配置结构体
// 静态IP留空则使用DHCP自动获取，无需修改其他逻辑

// WiFi+UDP客户端类（单例模式，避免重复创建WiFi/UDP资源）
class WifiUdpClient
{
public:
    typedef struct
    {
        const char *wifi_ssid;      // WiFi名称（必填）
        const char *wifi_pass;      // WiFi密码（必填）
        const char *udp_server_ip;  // UDP服务端IP（必填）
        uint16_t udp_server_port;   // UDP服务端端口（必填）
        const char *static_ip;      // ESP32静态IP（留空=DHCP，例："192.168.137.99"）
        const char *static_gw;      // 网关IP（留空=DHCP，例："192.168.137.1"）
        const char *static_netmask; // 子网掩码（留空=DHCP，例："255.255.255.0"）
    } WifiUdpConfig;

    static constexpr const char *TAG = "WifiUdpClient";

    // 单例获取实例（全局唯一）
    static WifiUdpClient &getInstance();

    esp_err_t init(const WifiUdpConfig &config);
    esp_err_t init(const WifiUdpConfig &config, void (*cb)(void));

    // 非阻塞发送UDP数据（字节数组版）
    esp_err_t sendData(const uint8_t *data, uint16_t len);

    // 重载：非阻塞发送字符串（更易用，适配JSON/普通字符串）
    esp_err_t sendData(const std::string &data);

    // 释放资源（一般无需手动调用，程序退出时自动释放）
    void deinit();

    // 禁止拷贝和赋值（单例模式）
    WifiUdpClient(const WifiUdpClient &) = delete;
    WifiUdpClient &operator=(const WifiUdpClient &) = delete;

private:
    // 私有构造/析构，仅通过getInstance获取实例
    WifiUdpClient();
    ~WifiUdpClient();

    // 替换分散配置，保存统一配置
    WifiUdpConfig m_config;              // 全局统一配置
    EventGroupHandle_t m_wifiEventGroup; // WiFi事件组
    static const int WIFI_CONNECTED_BIT; // WiFi连接成功标志
    static const int WIFI_FAIL_BIT;      // WiFi连接失败标志
    int m_wifiRetryNum = 0;              // WiFi重连次数

    // UDP相关
    int m_udpSock = -1;            // UDP套接字句柄
    struct sockaddr_in m_destAddr; // 服务端地址结构体
    socklen_t m_destAddrLen;       // 地址结构体长度

    // 发送队列+任务
    QueueHandle_t m_udpSendQueue;     // UDP发送队列（存储待发送数据）
    TaskHandle_t m_udpSendTaskHandle; // UDP发送任务句柄
    bool m_isInited = false;          // 初始化完成标志

    // 【修改】适配ESP-IDF5.4：字符串IP转esp_ip4_addr_t（ESP官方封装类型）
    // ip_str: 字符串IP（"x.x.x.x"）, ip: 输出的esp_ip4_addr_t
    // 返回：ESP_OK=成功，ESP_FAIL=失败（IP格式错误）
    esp_err_t ipStrToIp4(const char *ip_str, esp_ip4_addr_t &ip);

    // WiFi事件处理函数（静态函数，适配ESP32事件回调）
    static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    // 初始化WiFi STA模式（支持静态IP/DHCP自动切换）
    esp_err_t initWifiSta();
    // 初始化UDP套接字
    esp_err_t initUdp();
    // UDP发送任务（静态函数，作为FreeRTOS任务入口）
    static void udpSendTask(void *arg);
    // 实际的UDP发送逻辑（非静态，供发送任务调用）
    void doUdpSend(const uint8_t *data, uint16_t len);
};

#endif // WIFI_UDP_CLIENT_HPP
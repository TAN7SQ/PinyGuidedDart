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

#define UDP_SEND_QUEUE_LEN 32     // 队列最大缓存
#define UDP_SEND_DATA_MAX_LEN 128 // 单条UDP数据最大长度
#define UDP_SEND_TASK_STACK 4096  // UDP发送任务栈大小
#define UDP_SEND_TASK_PRIO 5      // UDP发送任务优先级

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

    static WifiUdpClient &getInstance();

    esp_err_t init(const WifiUdpConfig &config);
    esp_err_t init(const WifiUdpConfig &config, void (*cb)(void));

    esp_err_t sendData(const uint8_t *data, uint16_t len);
    esp_err_t sendData(const std::string &data);

    void deinit();

    WifiUdpClient(const WifiUdpClient &) = delete;
    WifiUdpClient &operator=(const WifiUdpClient &) = delete;

private:
    WifiUdpClient();
    ~WifiUdpClient();

    WifiUdpConfig m_config;              
    EventGroupHandle_t m_wifiEventGroup; 
    static const int WIFI_CONNECTED_BIT; 
    static const int WIFI_FAIL_BIT;      
    int m_wifiRetryNum = 0;              

    int m_udpSock = -1;            
    struct sockaddr_in m_destAddr; 
    socklen_t m_destAddrLen;       

    QueueHandle_t m_udpSendQueue;     
    TaskHandle_t m_udpSendTaskHandle; 
    bool m_isInited = false;          

    esp_err_t ipStrToIp4(const char *ip_str, esp_ip4_addr_t &ip);

    static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    esp_err_t initWifiSta();
    esp_err_t initUdp();

    static void udpSendTask(void *arg);
    static void initTask(void *arg);

    void doUdpSend(const uint8_t *data, uint16_t len);
};

#endif // WIFI_UDP_CLIENT_HPP
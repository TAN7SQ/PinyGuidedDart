#include "wifi_udp_client.hpp"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include "nvs_flash.h"
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <sys/socket.h>

const int WifiUdpClient::WIFI_CONNECTED_BIT = BIT0;
const int WifiUdpClient::WIFI_FAIL_BIT = BIT1;

static void (*s_user_init_cb)(void) = nullptr;

void WifiUdpClient::initTask(void *arg)
{
    WifiUdpClient *instance = (WifiUdpClient *)arg;
    if (instance == nullptr) {
        ESP_LOGE(TAG, "Init task: invalid instance");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "wifi_udp_client init task start");
    esp_err_t ret = ESP_OK;
    do {
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS init fail: %s", esp_err_to_name(ret));
            break;
        }

        ret = instance->initWifiSta();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi init fail: %s", esp_err_to_name(ret));
            break;
        }

        ret = instance->initUdp();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "UDP init fail: %s", esp_err_to_name(ret));
            break;
        }

        BaseType_t taskRet = xTaskCreatePinnedToCore(udpSendTask,
                                                     "udp_send",
                                                     UDP_SEND_TASK_STACK,
                                                     instance,
                                                     UDP_SEND_TASK_PRIO,
                                                     &instance->m_udpSendTaskHandle,
                                                     1);

        if (taskRet != pdPASS) {
            ESP_LOGE(TAG, "Create UDP send task fail");
            close(instance->m_udpSock);
            ret = ESP_FAIL;
            break;
        }

        instance->m_isInited = true;
        ESP_LOGI(TAG,
                 "WifiUdpClient Create Success,UDP server: %s:%d",
                 instance->m_config.udp_server_ip,
                 instance->m_config.udp_server_port);

        if (s_user_init_cb != nullptr) {
            s_user_init_cb();
            s_user_init_cb = nullptr;
        }

    } while (0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WifiUdpClient Create Failed!");
        if (instance->m_wifiEventGroup != nullptr) {
            xEventGroupSetBits(instance->m_wifiEventGroup, WIFI_FAIL_BIT);
        }
    }

    vTaskDelete(NULL);
}

WifiUdpClient &WifiUdpClient::getInstance()
{
    static WifiUdpClient instance;
    return instance;
}

WifiUdpClient::WifiUdpClient() : m_wifiRetryNum(0), m_udpSock(-1), m_destAddrLen(sizeof(m_destAddr))
{
    m_udpSendQueue = xQueueCreate(UDP_SEND_QUEUE_LEN, UDP_SEND_DATA_MAX_LEN);
    if (m_udpSendQueue == nullptr) {
        ESP_LOGE(TAG, "Create UDP send queue fail");
        abort();
    }
    memset(&m_config, 0, sizeof(m_config));
    ESP_LOGI(TAG, "UDP send queue create success, len: %d, item size: %d", UDP_SEND_QUEUE_LEN, UDP_SEND_DATA_MAX_LEN);
}

WifiUdpClient::~WifiUdpClient()
{
    deinit();
}

esp_err_t WifiUdpClient::init(const WifiUdpConfig &config, void (*cb)(void))
{
    if (m_isInited) {
        ESP_LOGW(TAG, "Already inited, skip");
        return ESP_OK;
    }

    if (config.wifi_ssid == nullptr || config.wifi_pass == nullptr || config.udp_server_ip == nullptr ||
        config.udp_server_port == 0) {
        ESP_LOGE(TAG, "Invalid init params: WiFi/UDP config is required");
        return ESP_ERR_INVALID_ARG;
    }
    bool has_static_ip =
        (config.static_ip != nullptr) && (config.static_gw != nullptr) && (config.static_netmask != nullptr);
    bool has_part_static =
        (config.static_ip != nullptr) || (config.static_gw != nullptr) || (config.static_netmask != nullptr);
    if (has_part_static && !has_static_ip) {
        ESP_LOGE(TAG, "Static IP config error: ip/gw/netmask must be all set or all null");
        return ESP_ERR_INVALID_ARG;
    }

    m_config = config;
    s_user_init_cb = cb;

    BaseType_t ret = xTaskCreatePinnedToCore(initTask, "wifi_udp_init", 4096, this, 5, nullptr, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Create wifi udp init task fail");
        s_user_init_cb = nullptr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t WifiUdpClient::init(const WifiUdpConfig &config)
{
    return init(config, nullptr);
}

esp_err_t WifiUdpClient::ipStrToIp4(const char *ip_str, esp_ip4_addr_t &ip)
{
    if (ip_str == nullptr || strlen(ip_str) == 0) {
        ESP_LOGE(TAG, "IP string is null or empty");
        return ESP_FAIL;
    }

    char ip_buf[16] = {0};
    strncpy(ip_buf, ip_str, sizeof(ip_buf) - 1);
    char *p = strtok(ip_buf, ".");
    int ip_seg[4] = {0};
    int seg_cnt = 0;

    while (p != nullptr && seg_cnt < 4) {
        for (int i = 0; p[i] != '\0'; i++) {
            if (!isdigit(p[i])) {
                ESP_LOGE(TAG, "IP segment invalid: %s (not a number)", p);
                return ESP_FAIL;
            }
        }

        ip_seg[seg_cnt] = atoi(p);
        if (ip_seg[seg_cnt] < 0 || ip_seg[seg_cnt] > 255) {
            ESP_LOGE(TAG, "IP segment out of range: %d (must 0-255)", ip_seg[seg_cnt]);
            return ESP_FAIL;
        }
        seg_cnt++;
        p = strtok(nullptr, ".");
    }

    if (seg_cnt != 4) {
        ESP_LOGE(TAG, "IP format error: %s (must be x.x.x.x)", ip_str);
        return ESP_FAIL;
    }

    ip.addr = htonl((ip_seg[0] << 24) | (ip_seg[1] << 16) | (ip_seg[2] << 8) | ip_seg[3]);
    ESP_LOGD(TAG, "IP string convert success: %s → %d.%d.%d.%d", ip_str, ip_seg[0], ip_seg[1], ip_seg[2], ip_seg[3]);
    return ESP_OK;
}

esp_err_t WifiUdpClient::sendData(const uint8_t *data, uint16_t len)
{
    if (!m_isInited || data == nullptr || len == 0 || len > UDP_SEND_DATA_MAX_LEN) {
        ESP_LOGE(TAG, "Send data fail: invalid params or not inited");
        return ESP_FAIL;
    }

    BaseType_t queueRet = xQueueSend(m_udpSendQueue, data, pdMS_TO_TICKS(0));
    if (queueRet != pdPASS) {
        ESP_LOGE(TAG, "UDP send queue full, drop data (len: %d)", len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Data enqueue success (len: %d)", len);
    return ESP_OK;
}

esp_err_t WifiUdpClient::sendData(const std::string &data)
{
    return sendData((const uint8_t *)data.c_str(), data.length());
}

void WifiUdpClient::deinit()
{
    if (!m_isInited)
        return;

    if (m_udpSendTaskHandle != nullptr) {
        vTaskDelete(m_udpSendTaskHandle);
        m_udpSendTaskHandle = nullptr;
    }

    if (m_udpSock >= 0) {
        close(m_udpSock);
        m_udpSock = -1;
    }

    if (m_udpSendQueue != nullptr) {
        vQueueDelete(m_udpSendQueue);
        m_udpSendQueue = nullptr;
    }

    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_deinit();

    m_isInited = false;
    memset(&m_config, 0, sizeof(m_config));
    ESP_LOGI(TAG, "WifiUdpClient deinit success");
}

void WifiUdpClient::wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    WifiUdpClient *instance = (WifiUdpClient *)arg;
    if (instance == nullptr)
        return;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retry to connect");
        if (instance->m_wifiRetryNum < 5) {
            esp_wifi_connect();
            instance->m_wifiRetryNum++;
            ESP_LOGI(TAG, "WiFi retry connect num: %d", instance->m_wifiRetryNum);
        }
        else {
            xEventGroupSetBits(instance->m_wifiEventGroup, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connect fail after 5 retries");
            abort();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        instance->m_wifiRetryNum = 0;
        xEventGroupSetBits(instance->m_wifiEventGroup, WIFI_CONNECTED_BIT);
    }
}

esp_err_t WifiUdpClient::initWifiSta()
{
    m_wifiEventGroup = xEventGroupCreate();
    if (m_wifiEventGroup == nullptr) {
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == nullptr) {
        ESP_LOGE(TAG, "Create default WiFi STA netif fail");
        return ESP_FAIL;
    }

    wifi_init_config_t wifiCfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifiCfg));

    esp_event_handler_instance_t instanceAnyId = NULL;
    esp_event_handler_instance_t instanceGotIp = NULL;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, this, &instanceAnyId));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, this, &instanceGotIp));

    wifi_config_t wifiStaCfg = {};
    strncpy((char *)wifiStaCfg.sta.ssid, m_config.wifi_ssid, sizeof(wifiStaCfg.sta.ssid) - 1);
    strncpy((char *)wifiStaCfg.sta.password, m_config.wifi_pass, sizeof(wifiStaCfg.sta.password) - 1);
    wifiStaCfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifiStaCfg.sta.pmf_cfg.capable = true;
    wifiStaCfg.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiStaCfg));
    esp_wifi_set_ps(WIFI_PS_NONE); // 关闭低功耗
    ESP_ERROR_CHECK(esp_wifi_start());

    if (m_config.static_ip != nullptr) {
        esp_netif_ip_info_t ip_info = {};
        if (ipStrToIp4(m_config.static_ip, ip_info.ip) != ESP_OK ||
            ipStrToIp4(m_config.static_gw, ip_info.gw) != ESP_OK ||
            ipStrToIp4(m_config.static_netmask, ip_info.netmask) != ESP_OK) {
            ESP_LOGE(TAG, "Static IP parse fail");
            return ESP_FAIL;
        }

        // 配置静态IP
        esp_netif_dhcpc_stop(sta_netif);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
        ESP_LOGI(TAG,
                 "WiFi static IP config success: %s, gw: %s, netmask: %s",
                 m_config.static_ip,
                 m_config.static_gw,
                 m_config.static_netmask);
    }
    else {
        ESP_LOGI(TAG, "WiFi use DHCP mode, auto get IP");
    }

    EventBits_t bits =
        xEventGroupWaitBits(m_wifiEventGroup, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connect success: %s", m_config.wifi_ssid);
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connect fail: %s", m_config.wifi_ssid);
        return ESP_FAIL;
    }
    else {
        ESP_LOGE(TAG, "WiFi init unexpected event");
        return ESP_FAIL;
    }
}

esp_err_t WifiUdpClient::initUdp()
{
    m_udpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSock < 0) {
        ESP_LOGE(TAG, "UDP create socket fail, errno: %d", errno);
        return ESP_FAIL;
    }

    memset(&m_destAddr, 0, m_destAddrLen);
    m_destAddr.sin_family = AF_INET;
    m_destAddr.sin_port = htons(m_config.udp_server_port);

    // 转换UDP服务端IP
    if (inet_pton(AF_INET, m_config.udp_server_ip, &m_destAddr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "UDP server IP invalid: %s", m_config.udp_server_ip);
        close(m_udpSock);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP socket init success, server: %s:%d", m_config.udp_server_ip, m_config.udp_server_port);
    return ESP_OK;
}

void WifiUdpClient::udpSendTask(void *arg)
{
    WifiUdpClient *instance = (WifiUdpClient *)arg;
    if (instance == nullptr) {
        ESP_LOGE(TAG, "UDP send task: invalid instance");
        vTaskDelete(NULL);
        return;
    }

    uint8_t sendBuf[UDP_SEND_DATA_MAX_LEN] = {0};
    ESP_LOGI(TAG, "UDP send task start");

    while (1) {
        BaseType_t queueRet = xQueueReceive(instance->m_udpSendQueue, sendBuf, portMAX_DELAY);
        if (queueRet == pdPASS) {
            uint16_t dataLen = strlen((const char *)sendBuf);
            if (dataLen > 0) {
                instance->doUdpSend(sendBuf, dataLen);
            }
        }
        memset(sendBuf, 0, sizeof(sendBuf));
    }

    vTaskDelete(NULL);
}

void WifiUdpClient::doUdpSend(const uint8_t *data, uint16_t len)
{
    if (m_udpSock < 0) {
        ESP_LOGE(TAG, "UDP send fail: socket not init");
        return;
    }

    int sendLen = sendto(m_udpSock, data, len, 0, (struct sockaddr *)&m_destAddr, m_destAddrLen);
    if (sendLen < 0) {
        ESP_LOGE(TAG, "UDP send fail, errno: %d, data: %s", errno, (const char *)data);
    }
    else {
        ESP_LOGI(TAG, "UDP send success: %s (len: %d)", (const char *)data, sendLen);
    }
}
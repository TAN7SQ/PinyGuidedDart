#include "transport_interface.h"
#include <string.h>

#ifdef ESP_PLATFORM
    #include "driver/uart.h"
#else
    #include <errno.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>
#endif

class UartTransport : public TransportInterface
{
private:
    std::string port_name_;
    int baud_rate_;
    bool initialized_;
    bool async_running_;

#ifdef ESP_PLATFORM
    uart_port_t uart_num_;
#else
    int uart_fd_;
#endif

    DataReceivedCallback data_callback_;
    ErrorCallback error_callback_;

public:
    UartTransport(const std::string &port, int baudrate = 115200)
        : port_name_(port), baud_rate_(baudrate), initialized_(false), async_running_(false)
    {
#ifdef ESP_PLATFORM
        uart_num_ = UART_NUM_1; // 默认使用UART1
#endif
    }

    ~UartTransport() override
    {
        close();
    }

    int initialize() override
    {
        if (initialized_) {
            return 0;
        }

#ifdef ESP_PLATFORM
        // ESP-IDF UART配置
        uart_config_t uart_config = {
            .baud_rate = baud_rate_,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
        };

        // 安装UART驱动
        esp_err_t err = uart_param_config(uart_num_, &uart_config);
        if (err != ESP_OK) {
            LOG_ERROR("UART param config failed: %d", err);
            return -1;
        }

        err = uart_driver_install(uart_num_, 1024, 1024, 0, NULL, 0);
        if (err != ESP_OK) {
            LOG_ERROR("UART driver install failed: %d", err);
            return -1;
        }
#else
        // Linux串口配置
        uart_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ < 0) {
            LOG_ERROR("Failed to open UART port %s", port_name_.c_str());
            return -1;
        }

        // 配置串口参数
        struct termios options;
        tcgetattr(uart_fd_, &options);

        // 设置波特率
        cfsetispeed(&options, baud_rate_);
        cfsetospeed(&options, baud_rate_);

        // 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= (CLOCAL | CREAD);

        // 关闭软件流控
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcsetattr(uart_fd_, TCSANOW, &options);
#endif

        initialized_ = true;
        LOG_INFO("UART %s initialized at %d baud", port_name_.c_str(), baud_rate_);
        return 0;
    }

    int send(const ProtocolFrame &frame) override
    {
        if (!initialized_) {
            LOG_ERROR("UART not initialized");
            return -1;
        }

        // 发送整个帧
#ifdef ESP_PLATFORM
        int bytes_written = uart_write_bytes(uart_num_, (const char *)&frame, FRAME_SIZE);
#else
        int bytes_written = write(uart_fd_, &frame, FRAME_SIZE);
#endif

        if (bytes_written != FRAME_SIZE) {
            LOG_ERROR("UART write failed: wrote %d/%zu bytes", bytes_written, FRAME_SIZE);
            return -1;
        }

#ifdef ESP_PLATFORM
        uart_wait_tx_done(uart_num_, 100); // 等待发送完成
#endif

        return 0;
    }

    int receive(ProtocolFrame &frame, int timeout_ms = 1000) override
    {
        if (!initialized_) {
            return -1;
        }

        uint8_t *buffer = (uint8_t *)&frame;
        size_t total_received = 0;

#ifdef ESP_PLATFORM
        // 设置读取超时
        uart_set_rx_timeout(uart_num_, timeout_ms / portTICK_PERIOD_MS);

        // 读取数据
        int len = uart_read_bytes(uart_num_, buffer, FRAME_SIZE, pdMS_TO_TICKS(timeout_ms));
        if (len == FRAME_SIZE) {
            total_received = len;
        }
#else
        // Linux: 使用select实现超时
        fd_set read_fds;
        struct timeval timeout;

        FD_ZERO(&read_fds);
        FD_SET(uart_fd_, &read_fds);

        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(uart_fd_ + 1, &read_fds, NULL, NULL, &timeout);
        if (ret > 0) {
            if (FD_ISSET(uart_fd_, &read_fds)) {
                ssize_t len = read(uart_fd_, buffer, FRAME_SIZE);
                if (len > 0) {
                    total_received = len;
                }
            }
        }
#endif

        if (total_received == FRAME_SIZE) {
            if (validate_frame(&frame) == 0) {
                return 0;
            }
        }

        return -1;
    }

    int start_async_receive(DataReceivedCallback callback) override
    {
        if (!initialized_) {
            return -1;
        }

        data_callback_ = callback;
        async_running_ = true;

        // 在实际应用中，这里应该启动一个接收线程
        // 为了简化，这里只设置回调

        LOG_INFO("UART async receive started");
        return 0;
    }

    void stop_async_receive() override
    {
        async_running_ = false;
        LOG_INFO("UART async receive stopped");
    }

    void set_error_callback(ErrorCallback callback) override
    {
        error_callback_ = callback;
    }

    bool is_connected() const override
    {
        return initialized_;
    }

    void close() override
    {
        if (initialized_) {
#ifdef ESP_PLATFORM
            uart_driver_delete(uart_num_);
#else
            ::close(uart_fd_);
#endif
            initialized_ = false;
            LOG_INFO("UART closed");
        }
    }
};
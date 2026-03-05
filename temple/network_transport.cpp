#include "transport_interface.h"
#include <string.h>

#ifdef ESP_PLATFORM
    #include "lwip/netdb.h"
    #include "lwip/sockets.h"
#else
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <unistd.h>
#endif

class NetworkTransport : public TransportInterface
{
public:
    enum class Protocol
    {
        TCP,
        UDP,
        WEBSOCKET
    };

private:
    Protocol protocol_;
    std::string address_;
    int port_;
    int sock_fd_;
    bool connected_;
    bool async_running_;

    DataReceivedCallback data_callback_;
    ErrorCallback error_callback_;

public:
    NetworkTransport(Protocol protocol, const std::string &addr, int port)
        : protocol_(protocol), address_(addr), port_(port), sock_fd_(-1), connected_(false), async_running_(false)
    {
    }

    ~NetworkTransport() override
    {
        close();
    }

    int initialize() override
    {
        if (connected_) {
            return 0;
        }

        // 创建socket
        if (protocol_ == Protocol::TCP || protocol_ == Protocol::WEBSOCKET) {
            sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        }
        else { // UDP
            sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        }

        if (sock_fd_ < 0) {
            LOG_ERROR("Failed to create socket");
            return -1;
        }

        // 设置服务器地址
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);

        // 解析地址
        if (inet_pton(AF_INET, address_.c_str(), &server_addr.sin_addr) <= 0) {
            LOG_ERROR("Invalid address: %s", address_.c_str());
            close();
            return -1;
        }

        // 对于TCP/WebSocket，需要连接
        if (protocol_ == Protocol::TCP || protocol_ == Protocol::WEBSOCKET) {
            if (connect(sock_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                LOG_ERROR("Failed to connect to %s:%d", address_.c_str(), port_);
                close();
                return -1;
            }
        }

        connected_ = true;
        LOG_INFO("Network transport initialized: %s:%d", address_.c_str(), port_);
        return 0;
    }

    int send(const ProtocolFrame &frame) override
    {
        if (!connected_ || sock_fd_ < 0) {
            return -1;
        }

        if (protocol_ == Protocol::UDP) {
            // UDP需要目标地址
            struct sockaddr_in dest_addr;
            memset(&dest_addr, 0, sizeof(dest_addr));
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(port_);
            inet_pton(AF_INET, address_.c_str(), &dest_addr.sin_addr);

            ssize_t sent = sendto(sock_fd_, &frame, FRAME_SIZE, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (sent != FRAME_SIZE) {
                LOG_ERROR("UDP send failed");
                return -1;
            }
        }
        else {
            // TCP/WebSocket
            ssize_t sent = ::send(sock_fd_, &frame, FRAME_SIZE, 0);
            if (sent != FRAME_SIZE) {
                LOG_ERROR("TCP send failed");
                return -1;
            }
        }

        return 0;
    }

    int receive(ProtocolFrame &frame, int timeout_ms = 1000) override
    {
        if (!connected_ || sock_fd_ < 0) {
            return -1;
        }

        // 设置接收超时
#ifdef ESP_PLATFORM
        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif

        ssize_t received = 0;

        if (protocol_ == Protocol::UDP) {
            struct sockaddr_in src_addr;
            socklen_t addr_len = sizeof(src_addr);

            received = recvfrom(sock_fd_, &frame, FRAME_SIZE, 0, (struct sockaddr *)&src_addr, &addr_len);
        }
        else {
            received = ::recv(sock_fd_, &frame, FRAME_SIZE, 0);
        }

        if (received == FRAME_SIZE) {
            if (validate_frame(&frame) == 0) {
                return 0;
            }
        }

        return -1;
    }

    int start_async_receive(DataReceivedCallback callback) override
    {
        if (!connected_) {
            return -1;
        }

        data_callback_ = callback;
        async_running_ = true;

        // 在实际应用中，这里应该启动一个接收线程
        // 为了简化，这里只设置回调

        LOG_INFO("Network async receive started");
        return 0;
    }

    void stop_async_receive() override
    {
        async_running_ = false;
    }

    void set_error_callback(ErrorCallback callback) override
    {
        error_callback_ = callback;
    }

    bool is_connected() const override
    {
        return connected_;
    }

    void close() override
    {
        if (sock_fd_ >= 0) {
#ifdef ESP_PLATFORM
            lwip_close(sock_fd_);
#else
            ::close(sock_fd_);
#endif
            sock_fd_ = -1;
        }
        connected_ = false;
        LOG_INFO("Network transport closed");
    }
};
#ifndef TRANSPORT_INTERFACE_H
#define TRANSPORT_INTERFACE_H

#include "protocol_common.h"

#ifdef __cplusplus

    #include <functional>
    #include <memory>

// 数据接收回调类型
using DataReceivedCallback = std::function<void(const ProtocolFrame &)>;
using ErrorCallback = std::function<void(int error_code, const char *message)>;

class TransportInterface
{
public:
    virtual ~TransportInterface() = default;

    // 初始化传输接口
    virtual int initialize() = 0;

    // 发送数据
    virtual int send(const ProtocolFrame &frame) = 0;

    // 接收数据（阻塞）
    virtual int receive(ProtocolFrame &frame, int timeout_ms = 1000) = 0;

    // 开始异步接收
    virtual int start_async_receive(DataReceivedCallback callback) = 0;

    // 停止异步接收
    virtual void stop_async_receive() = 0;

    // 设置错误回调
    virtual void set_error_callback(ErrorCallback callback) = 0;

    // 获取连接状态
    virtual bool is_connected() const = 0;

    // 关闭连接
    virtual void close() = 0;
};

// 创建帧的辅助函数
ProtocolFrame create_protocol_frame(const SensorData &data, uint32_t seq = 0);

#endif // __cplusplus

#endif // TRANSPORT_INTERFACE_H
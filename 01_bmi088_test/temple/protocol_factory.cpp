#include "transport_interface.h"
#include <memory>

ProtocolFrame create_protocol_frame(const SensorData &data, uint32_t seq)
{
    ProtocolFrame frame;

    frame.magic = PROTOCOL_MAGIC;
    frame.version = PROTOCOL_VERSION;
    frame.type = 0x01; // 传感器数据类型
    frame.length = sizeof(SensorData);
    frame.seq = seq;
    frame.data = data;

    // 计算CRC（不包括CRC字段）
    size_t data_length = sizeof(ProtocolFrame) - sizeof(uint16_t);
    frame.crc16 = calculate_crc16((const uint8_t *)&frame, data_length);

    return frame;
}

// 工厂函数：创建UART传输
std::unique_ptr<TransportInterface> create_uart_transport(const std::string &port, int baudrate = 115200)
{

    return std::make_unique<UartTransport>(port, baudrate);
}

// 工厂函数：创建网络传输
std::unique_ptr<TransportInterface> create_network_transport(NetworkTransport::Protocol protocol,
                                                             const std::string &address, int port)
{

    return std::make_unique<NetworkTransport>(protocol, address, port);
}
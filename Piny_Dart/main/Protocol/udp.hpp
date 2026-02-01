#ifndef __UDP_HPP__
#define __UDP_HPP__

class udp
{
public:
    
    udp(const int &port);
    ~udp();
    void send(const char *data, const int &len);
    void recv(char *data, const int &len);

private:
    int port;
};

#endif
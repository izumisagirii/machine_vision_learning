#ifndef HITCRT_ROBOMASTERS_SERIAL_BASE_H_
#define HITCRT_ROBOMASTERS_SERIAL_BASE_H_

#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
#include <iostream>

namespace hitcrt
{

class SerialBase
{
public:
    typedef std::shared_ptr<SerialBase> Ptr;
    SerialBase();
    SerialBase(std::string str, int baud_rate);
    SerialBase(std::string local_ip, int local_port,std::string remote_ip,int remote_port);
    void init(std::string local_ip, int local_port,std::string remote_ip,int remote_port);
    void init(std::string str, int baud_rate);
    ~SerialBase();
    bool send(unsigned char* ch, size_t length);
    bool receive(unsigned char* buff, size_t& length,int buffsize);
    bool is_init=false;
private:
    bool checkDevice();
    std::string filename;
    int faild_num=0;
    bool use_serial;
    struct timeval timer;
    std::shared_ptr<boost::asio::io_service> io;
    std::shared_ptr<boost::asio::serial_port> port;
    std::shared_ptr<boost::asio::ip::udp::socket> socket;
    std::shared_ptr<boost::asio::ip::udp::endpoint> remote_endpoint;

};


}

#endif // HITCRT_ROBOMASTERS_SERIAL_BASE_H_

#include "serialbase.h"
#include <unistd.h>
#include <boost/filesystem.hpp>
namespace hitcrt
{
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 默认构造函数，后续需要调用init初始化网口或串口
* @param
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    SerialBase::SerialBase(){
        io.reset(new boost::asio::io_service);
    }

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 构造函数，串口
 * @param str:串口文件名
     * baud_rate：波特率
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
SerialBase::SerialBase(std::string str, int baud_rate)
{
    init(str,baud_rate);
}
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 初始化串口
 * @param str:串口文件名
     * baud_rate：波特率
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
void SerialBase::init(std::string str, int baud_rate){
    if(is_init){
        return;
    }
    filename=str;
    try {
        ///初始化串口
        use_serial= true;
        sleep(1);
        port.reset(new boost::asio::serial_port(*io, str.c_str()));
        port->set_option(boost::asio::serial_port::baud_rate(baud_rate));
        port->set_option(boost::asio::serial_port::flow_control());
        port->set_option(boost::asio::serial_port::parity());
        port->set_option(boost::asio::serial_port::stop_bits());
        port->set_option(boost::asio::serial_port::character_size(8));
        is_init= true;
    }catch (boost::exception &e){
        std::cerr<<"串口初始化失败！"<<std::endl;
    }
}

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 构造函数，网口，使用UDP协议，好处是只管发送数据不需要确定
     * 对方收到数据，如果使用TCP协议则一方未链接会报错，因为此协议需要
     * 确定对方收到数据
 * @param local_ip:本地ip
     * local_port:本地端口
     * remote_ip：远程ip
     * remote_port:远程端口
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
SerialBase::SerialBase(std::string local_ip, int local_port,std::string remote_ip,int remote_port){
    init(local_ip,local_port,remote_ip,remote_port);
}
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 初始化网口
 * @param local_ip:本地ip
     * local_port:本地端口
     * remote_ip：远程ip
     * remote_port:远程端口
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
void SerialBase::init(std::string local_ip, int local_port,std::string remote_ip,int remote_port){
    if(is_init){
        return;
    }
    try{
        use_serial= false;///使用网口
        sleep(1);
        ///初始化socket
        socket.reset(new boost::asio::ip::udp::socket(*io, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(local_ip.c_str()), local_port)));
        ///初始化远程ip和端口
        remote_endpoint.reset(new boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(remote_ip.c_str()),remote_port));
        is_init=true;
    }catch (boost::exception &e){
        std::cerr<<"网口初始化失败!"<<std::endl;
    }
}
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 析构函数
 * @param
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
SerialBase::~SerialBase(){}

bool SerialBase::checkDevice(){
    boost::filesystem::path mpath(filename);
    boost::system::error_code error;
    return boost::filesystem::exists(mpath,error);
}
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 发送数据
 * @param ch:发送的数据
     * length:数据长度
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
bool SerialBase::send(unsigned char* ch, size_t length)
{
    if(use_serial){
        try {
            ///向串口写length字节的数据
            boost::asio::write(*port, boost::asio::buffer(ch, length));
            faild_num=0;
        }catch (boost::exception &e){
            faild_num++;
            if(faild_num>2){
                if(!checkDevice()){
                    is_init= false;
                    faild_num=0;
                }
            }
            std::cerr<<"serial write error!"<<std::endl;
            return false;
        }

    } else{
        try {
            ///向网口写length字节的数据
            socket->send_to(boost::asio::buffer(ch, length),*remote_endpoint);
        }catch (boost::exception &e){
            std::cerr<<"ethernet write error!"<<std::endl;
            return false;
        }

    }
    return true;

}

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 接收数据
 * @param buff:接收的数据
     * length：接收数据的长度
     * buffsize：读取的buff的长度
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
bool SerialBase::receive(unsigned char* buff, size_t& length,int buffsize)
{
    boost::system::error_code err;
    if(use_serial){
        try {
            ///从串口读取buffsize字节的数据
            length = port->read_some(boost::asio::buffer(buff, buffsize), err);
        }catch (boost::exception &e){
            std::cerr<<"serial receive error!"<<std::endl;
            return false;
        }
    } else{
        try {
            ///从远程网口端口读取buffsize字节的数据
            length=socket->receive_from(boost::asio::buffer(buff, buffsize),*remote_endpoint);
        }catch (boost::exception &e){
            std::cerr<<"ethernet receive error!"<<std::endl;
            return false;
        }

    }
    return true;
}

}

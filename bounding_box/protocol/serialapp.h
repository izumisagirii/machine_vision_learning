#ifndef HITCRT_ROBOMASTERS_SERIALAPP_H_
#define HITCRT_ROBOMASTERS_SERIALAPP_H_

#include <boost/thread/mutex.hpp>
#include <string>
#include "serialbase.h"


namespace hitcrt
{
class SerialApp
{
public:
    typedef std::shared_ptr<SerialApp> Ptr;
    bool isok=false;
    union Float2uchar
    {
        float fl;
        unsigned char ch[4];
    };
    /***************************1 start********************/
    /**接收flag和发送flag,至少需要1个flag，不同flag可以用来区分不同
     * 数据**/
    enum  SEND_FLAG
    {
        COLUMN,
        ARROW,
        ODOMETRY
    };

    enum  RECEIVE_FLAG
    {
        RED,
        BLUE,
        UNKNOWN
    };
    /***************************1 end********************/

    SerialApp(const std::string local_ip,int local_port, const std::string remote_ip,int remote_port);
    SerialApp(const std::string str, int baudrate=115200);
    ~SerialApp();
    void init(const std::string local_ip,int local_port, const std::string remote_ip,int remote_port);
    void init(const std::string str="auto", int baudrate=115200);
    bool decode(unsigned char* buff, size_t& received_length, std::vector<float>& num, SerialApp::RECEIVE_FLAG& receive_flag);
    bool decode_num(unsigned char buff, float& num);
    bool send(SEND_FLAG send_flag, std::vector<float> num = std::vector<float>());
    bool receive(SerialApp::RECEIVE_FLAG& receive_flag, std::vector<float>& num);

private:
    std::string serial_name="auto",local_ip,remote_ip;
    int baud_rat=115200,local_port,remote_port;
    struct timeval timer;
    double firsttime;
    bool use_serial= true;
    bool send_statu,receive_statu;
    SerialBase::Ptr m_serialBase;
    std::vector<std::string> device_list;
    unsigned char FIRST_ONE;
    unsigned char FIRST_TWO;
    unsigned char LAST_TWO;
    unsigned char LAST_ONE;
    SerialApp();
    void findDevice(std::vector<std::string>& files);
/// 线程锁，很重要！！！！！！
/// 避免多个线程同时发送数据
    boost::mutex m_send_mutex,m_receive_mutex,init_mutex;




};


}


#endif // HITCRT_ROBOMASTERS_SERIALAPP_H_

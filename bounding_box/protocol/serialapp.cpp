/********************************************************
串口和UDP网口通信接口

协议为 55 00 ID NUM NUM*float 00 AA
比如 功能ID为 03
发送数据为1个float 则协议为
55 00 03 01 xx xx xx xx 00 AA

该程序应用了boost asio 库

使用时修改SerialApp头文件和源文件中共四个地方即可，这四个地方在代码中
特别标注出来了的
*********************************************************/

#include <dirent.h>
#include "serialapp.h"

/***************************2 start********************/
/**接收float的最大长度，最大为255，因为一个char有8位，
 * 2^8=256，发送长度不用设置但也要保证发送数据长度少于255**/
#define MAX_RECEIVE_FLOAT_LENGTH 10///修改为每次接收的float的最大个数
/***************************2 end********************/

const int RECEIVE_BUFF_LENGTH=MAX_RECEIVE_FLOAT_LENGTH*4+6;
namespace hitcrt
{
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 查找电脑上所有串口设备
 * @param files:找到的所有串口设备
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
    void SerialApp::findDevice(std::vector<std::string>& files){
        DIR* pDir;
        struct dirent* ptr;
        files.clear();
        if(!(pDir=opendir("/dev/"))){///打开/dev/目录
            return ;
        }
        const std::string path0="/dev/";
        std::string subFile;
        while ((ptr=readdir(pDir))!=0){
            subFile=ptr->d_name;
            auto rt=subFile.find("ttyUSB");
            if(rt != std::string::npos){///查找以/dev/ttyUSB开头的文件
                files.emplace_back(path0+subFile);
            }
        }
        if(!files.empty()){
            std::cout<<"find serial:"<<std::endl;
        } else{///没有找到任何串口文件
            return ;
        }
        for(auto device:files){///列出所以串口文件
            std::cout<<device<<std::endl;
        }
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 默认构造函数，后续需要手动调用init初始化网口或串口，可以
     * 自己控制初始化失败时立即退出还是一直循环初始化直到成功,这样的
     * 好处时没有串口或者网络未连接时代码不会退出，其它线程可以继续跑
* @param
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    SerialApp::SerialApp(){
        if(MAX_RECEIVE_FLOAT_LENGTH>255){
            std::cerr<<"数据长度不能超过255(uchar类型溢出)"<<std::endl;
            exit(0);
        }
        FIRST_ONE = 0x55;///第一位
        FIRST_TWO = 0x00;///第二位
        LAST_ONE = 0x00;  ///倒数第二
        LAST_TWO = 0xAA;  ///倒数第一
        m_serialBase.reset(new SerialBase());
    }

    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 构造函数，网口
* @param local_ip:本地ip
     * local_port:本地端口
     * remote_ip:远程ip
     * remote_port:远程端口
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    SerialApp::SerialApp(const std::string local_ip,int local_port, const std::string remote_ip,int remote_port):SerialApp(){
        std::cout<<"******************init ethernet***********************"<<std::endl;
        init(local_ip,local_port,remote_ip,remote_port);
        if(!isok){
            std::cerr<<"初始化失败"<<std::endl;
//            exit(1);
        }
    }
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 初始化网口，成功时会将isok设置为true
 * @param local_ip:本地ip
     * local_port:本地端口
     * remote_ip:远程ip
     * remote_port:远程端口
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
    void SerialApp::init(const std::string local_ip,int local_port, const std::string remote_ip,int remote_port){
        init_mutex.lock();///上锁防止多个线程同时初始化
        if(isok){
            init_mutex.unlock();
            return;
        }
        this->local_ip=local_ip;
        this->local_port=local_port;
        this->remote_ip=remote_ip;
        this->remote_port=remote_port;
        use_serial= false;
        m_serialBase->init(local_ip,local_port,remote_ip,remote_port);
        if(m_serialBase->is_init){
            isok=true;
            std::cout<<"succeed init ethernet!"<<std::endl;
        }
        gettimeofday(&timer,NULL);
        firsttime=timer.tv_sec*1e3+timer.tv_usec*1e-3;
        init_mutex.unlock();
    }

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 构造函数，串口,初始化失败时会结束程序
 * @param str:串口文件名，为auto时自动查找电脑链接的第一个串口
     * baudrate：波特率
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
SerialApp::SerialApp(const std::string str, int baudrate):SerialApp()
{
   std::cout<<"******************init serial***********************"<<std::endl;
   init(str,baudrate);
    if(!isok){
        std::cerr<<"初始化失败"<<std::endl;
//        exit(1);
    }
}

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 初始化，串口，成功时会将isok设置为true
 * @param str:串口文件名，为auto时自动查找电脑链接的第一个串口
     * baudrate：波特率
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
    void SerialApp::init(const std::string str, int baudrate){
        init_mutex.lock();///上锁防止多个线程同时初始化
        if(isok){
            init_mutex.unlock();
            return;
        }
        this->serial_name=str;
        this->baud_rat=baudrate;
        use_serial=true;
        findDevice(device_list);///查找串口设备
        if(device_list.empty()){///没有串口设备
            isok=false;
            init_mutex.unlock();
            return ;
        }
        if(str!="auto"){
            bool find=false;
            for(auto dev_find:device_list){///查找指定串口设备
                if(dev_find==str){
                    find= true;
                    break;
                }
            }
            if(find){///找到指定的串口设备
                sleep(1);///休眠是考虑代码运行中插入串口给其初始化时间
                m_serialBase->init(str.c_str(), baudrate);///串口基层，负责收发具体数据
            } else{///没找到指定设备，自动选择第一个串口
                std::cerr<<"can not find device:"<<str<<std::endl;
                std::cerr<<"open "<<device_list[0]<<" instead!"<<std::endl;
                sleep(1);///休眠是考虑代码运行中插入串口给其初始化时间
                m_serialBase->init(device_list[0].c_str(), baudrate);
            }

        } else{///自动选择串口设备
            sleep(1);///休眠是考虑代码运行中插入串口给其初始化时间
            m_serialBase->init(device_list[0].c_str(), baudrate);///串口基层，负责收发具体数据
        }

        gettimeofday(&timer,NULL);
        firsttime=timer.tv_sec*1e3+timer.tv_usec*1e-3;
        if(m_serialBase->is_init){
            isok=true;
            std::cout<<"succeed init serial!"<<std::endl;
        }
        init_mutex.unlock();
    }


    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 析构函数
 * @param
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */

SerialApp::~SerialApp(){}
    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 发送数据
 * @param send_flag:发送flag
     * num:发送的数据
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
bool SerialApp::send(SEND_FLAG send_flag, std::vector<float> num)
{
    if(!isok){///初始化未成功
        std::cerr<<"serial init failed!"<<std::endl;
        if(use_serial){
            init(this->serial_name,this->baud_rat);
        }else{
            init(this->local_ip,this->local_port,this->remote_ip,this->remote_port);
        }
        return false;
    }
    if(num.size()>255){
        std::cerr<<"发送数据过多,超过一次能发送的最大数量255！"<<std::endl;
        exit(1);
    }

    const int send_length = num.size()*4+6;
    unsigned char buff[send_length];

    buff[0] = FIRST_ONE;///第一位
    buff[1] = FIRST_TWO;///第二位

    /***************************3 start********************/
    ////不同发送id对应的flag///
    ///第三位，发送flag
    if(SerialApp::SEND_FLAG::COLUMN == send_flag)
        buff[2] = 0x01;
    else if(SerialApp::SEND_FLAG::ARROW == send_flag)
        buff[2] = 0x02;
    else if(SerialApp::SEND_FLAG::ODOMETRY == send_flag)
        buff[2] = 0x03;
    /***************************3 end********************/

    ///第四位，数据长度(float的个数)，最大为255，所以最多一次发送255个数
    buff[3] = static_cast<unsigned char>(num.size());

    for(int i = 0;i < num.size();i++)
    {///数据位，将1个float(4个字节，32位)转换为4个char（1个字节8位）
    ///这里用到联合体进行转换，联合体
    /// 表示的不同类型共用内存空间，使用可
    /// 以方便的将float转换为4个char
        Float2uchar temp;
        temp.fl = num[i];
        for(int j = 0;j < 4;j++)
            buff[i*4+4+j] = temp.ch[j];
    }

    buff[num.size()*4+4] = LAST_ONE;///倒数第二位
    buff[num.size()*4+4+1] = LAST_TWO;///倒数第一位

    m_send_mutex.lock();///防止多个线程同时发数产生混乱
    send_statu=m_serialBase->send(buff, send_length);///发送
    m_send_mutex.unlock();///解锁
    if(!m_serialBase->is_init){///串口突然断开，需要重新初始化，只有这个发数的函数能检测，接数的线程试过不行，因为接数是阻塞操作，串口断开也不会返回
        std::cerr<<"设备断开，尝试重新连接"<<std::endl;
        isok=false;
        return false;
    }
    if(!send_statu){
        return false;
    }
    return true;
}

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 接收数据
 * @param receive_flag:接收flag
     * num:接收的数据
 * @return
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
bool SerialApp::receive(SerialApp::RECEIVE_FLAG& receive_flag, std::vector<float>& num)
{
    if(!isok){///初始化未成功
        std::cerr<<"serial init failed!"<<std::endl;
        if(use_serial){
            init(this->serial_name,this->baud_rat);
        }else{
            init(this->local_ip,this->local_port,this->remote_ip,this->remote_port);
        }
        return false;
    }
    int zeronum=0;
    while(1)
    {
        // std::cerr << "\033[31mStart receiving data!\033[0m" << std::endl;
        unsigned char buff[RECEIVE_BUFF_LENGTH];
        size_t received_length;
        m_receive_mutex.lock();///上锁防止多个线程同时接收
        receive_statu=m_serialBase->receive(buff, received_length,RECEIVE_BUFF_LENGTH);///接收数据
        m_receive_mutex.unlock();
        if(!receive_statu){
            return false;
        }
        if(decode(buff, received_length, num, receive_flag))///解码
        {
            return true;
        }
    }
}

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 接收数据解码
 * @param buff:接收的原始数据
     * received_length：原始数据长度
     * num:解码得到的数据
     * receive_flag：接收到的标志
 * @return bool:解码是否成功
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
bool SerialApp::decode(unsigned char* buff, size_t& received_length, std::vector<float>& num, SerialApp::RECEIVE_FLAG& receive_flag)
{
    static int flag = 1;        ///接收状态机
    static RECEIVE_FLAG temp_receive_flag;  ///接收标志量
    static std::vector<float> vec_num;     ///接收数据
    static int temp_num_length;
    static bool return_flag;

    return_flag = false;
    for(size_t i = 0 ;i < received_length;i++)
    {
        // std::cerr << "\033[31mThe loop is opening!\033[0m" << std::endl;
//        std::cout<<std::hex<<(int)buff[i]<<std::endl;
        switch (flag)
        {
            case 1:     ///第一位，报头1
                // std::cout<<"case1: "<<std::hex<<(int)buff[i]<<std::endl;
                vec_num.clear();
                if(FIRST_ONE == buff[i]){
                    flag = 2;
                }else{
                    flag = 1;
                }
                break;
            case 2:     ///第二位，报头2
                // std::cout<<"case2: "<<std::hex<<(int)buff[i]<<std::endl;
                if(FIRST_TWO == buff[i])
                    flag = 3;
                else
                    flag = 1;
                break;
            case 3:     ///第三位，id
                // std::cout<<"case3: "<<std::hex<<(int)buff[i]<<std::endl;
                /***************************4 start********************/
                ///不同的接收id对应的flag///
                if(0x00== buff[i])
                {
                    temp_receive_flag = SerialApp::RECEIVE_FLAG::RED;
                    flag = 4;
                }else if(0x01== buff[i]){
                    temp_receive_flag = SerialApp::RECEIVE_FLAG::BLUE;
                    flag = 4;
                }
                else if(0x02== buff[i]){
                    temp_receive_flag = SerialApp::RECEIVE_FLAG::UNKNOWN;
                    flag = 4;
                }
                else{
                    flag = 1;
                }
                /***************************4 end********************/
                break;
            case 4:     ///第四位，浮点数长度
                // std::cout<<"case4:"<<(int)buff[i]<<std::endl;
                temp_num_length = static_cast<int>(buff[i]);
                if(temp_num_length < 0 || temp_num_length > MAX_RECEIVE_FLOAT_LENGTH)
                    flag = 1;
                else if(temp_num_length == 0)
                    flag = 6;
                else
                    flag = 5;
                break;
            case 5:///数据位
                // std::cout<<"case5:"<<(int)buff[i]<<std::endl;
                float temp;
                if(decode_num(buff[i], temp))
                    vec_num.push_back(temp);
                if(vec_num.size() == temp_num_length)
                    flag = 6;
                break;
            case 6:///倒数第二位
                // std::cout<<"case6: "<<std::hex<<(int)buff[i]<<std::endl;
                if(LAST_ONE == buff[i])
                    flag = 7;
                else
                    flag = 1;
                break;
            case 7:///倒数第一位
                // std::cout<<"case7: "<<std::hex<<(int)buff[i]<<std::endl;
                if(LAST_TWO == buff[i])
                {
                    receive_flag = temp_receive_flag;
                    num = vec_num;
                    return_flag = true;
                }
                flag = 1;
                break;
        }
    }
    return return_flag;

}

    /**
 * Copyright (C) 2020, HITCRT_VISION, all rights reserved.
 * @brief 解码一个float类型数据，也是利用联合体对float和char进行变换
 * @param buff:接收的原始char数据
     * num:解码得到的float数据
 * @return bool:解码是否成功
 * @author 黎林
 * -phone:18846140245;qq:1430244438
 */
bool SerialApp::decode_num(unsigned char buff, float& num)
{
    static Float2uchar temp_num;
    static int pos = 0;
    temp_num.ch[pos] = buff;
    pos++;
    if(pos == 4)
    {
        num = temp_num.fl;
        pos = 0;
        return true;
    }
    return false;
}



}

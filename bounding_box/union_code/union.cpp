#include "union.h"
/**
 * @brief center类构造函数，初始化相机和串口的位置
 * @param baudrate 串口收发的波特率
 * @param isRelease 是否输出抓图图像
 * @param isKinect 选择使用的相机是kinect还是xtion
 * @param openSerial 是否开启串口
 * @param flag 串口接受的flag
 **/
hitcrt::center::center(int baudrate, bool isRelease, bool isKinect, bool openSerial, RECEIVE_FLAG flag) : camera(isKinect), SerialApp("auto", baudrate)
{
    this->is_release = isRelease;
    this->flag = flag;
    this->is_kinect = isKinect;
    this->open_serial = openSerial;
    // R0 rotate the camera coordinate to basic robot coordinate(let their zs become parallel)
    this->R0 = std::make_shared<cv::Mat>((cv::Mat_<float>(3, 3) << 1, 0, 0,
                                          0, 0, 1,
                                          0, -1, 0));
    this->R = std::make_shared<cv::Mat>((cv::Mat_<float>(3, 3) << 1, 0, 0,
                                         0, 1, 0,
                                         0, 0, 1));
    this->t = std::make_shared<cv::Mat>((cv::Mat_<float>(3, 1) << 0, 0, 0));
    // this->t0 = (cv::Mat_<float>(3, 1) << 0.135, 0.3203, 0.549);
    this->t0 = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    this->stop = false;
    this->init_finished = false;
}
/**
 * @brief 像素坐标系坐标点转机器人坐标系
 * @param pixel_point 传入的像素坐标点，以cv::Mat形式传入
 * @param depth 获得的该点深度信息
 * @return 是否转换成功
 **/
bool hitcrt::center::get_robot_coordinate_point(cv::Mat &pixel_point, float depth)
{
    // std::cout << "depth in function:" << depth << std::endl;
    if(pixel_point.rows == 3)
    {
        // use intrinsic matrix to calculate the camrea coordinate point
        pixel_point = (*get_color_intrinsic()).inv() * pixel_point;
        pixel_point *= depth;
        // use extrinsic matrix to calculate the robot coordinate point
        pixel_point = ((*get_R0()) * (*get_R())) * pixel_point;
        pixel_point += *get_t();
        return true;
    }
    else if(pixel_point.rows == 2)
    {
        cv::Mat temp_point = (cv::Mat_<float>(3, 1) << pixel_point.at<float>(0, 0), pixel_point.at<float>(1, 0), 1);
        // use intrinsic matrix to calculate the camrea coordinate point
        temp_point = (*get_color_intrinsic()).inv() * temp_point;
        temp_point *= depth;
        // use extrinsic matrix to calculate the robot coordinate point
        temp_point = ((*get_R0()) * (*get_R())) * temp_point;
        temp_point += *get_t();
        return true;
    }
    else
    {
        std::cerr << "In func get_robot_coordinate_point():\nThe input row number is wrong!" << std::endl;
        return false;
    }
}
/**
 * @brief 创建线程过程
 * @return 无
 **/
//receive flag: 0	receive data: 0	1
void hitcrt::center::init()
{
    grab_image = boost::thread([this] { thread_image(); });
    if(open_serial)
        grab_serial = boost::thread([this] { thread_serial(); });
}
/**
 * @brief 关闭线程过程
 * @return 无
 * @attention 之后可能会写成析构函数形式
 **/
// for the threads to be freed before the process
void hitcrt::center::shutdown()
{
    stop = true;
    if(grab_image.joinable())
        grab_image.join();
    if(open_serial)
    {
        if(grab_serial.joinable())
            grab_serial.join();
    }
}
/**
 * @brief 图像线程，接收到stop的信息后就停止运行
 * @return 无
 **/
void hitcrt::center::thread_image()
{
    if (!is_release)
    {
        cv::namedWindow("color");
        cv::namedWindow("depth");
    }
    do
    {
        camera.grab();
        // 这里不能直接使用rgb_img，因为有可能得到的是nullptr，在其他地方调用时可能会带来段错误的麻烦
        auto _rgb_img = camera.getImageRGB();
        auto _depth_img = camera.getImageDepth();
        if (_rgb_img != nullptr && _depth_img != nullptr)
        {
            rgb_img_mu.lock();
            rgb_img = _rgb_img;
            rgb_img_mu.unlock();
            depth_img_mu.lock();
            depth_img = _depth_img;
            depth_img_mu.unlock();
        }
        else
        {
            std::cerr << "\033[31mThe image was lost!\033[0m" << std::endl;
        }
        if(!init_finished)
            init_finished = true;
        if(!is_release)
        {
            cv::imshow("color", *rgb_img);
            cv::imshow("depth", *depth_img);
            cv::waitKey(1);
        }
        else
            usleep(1000);
    }while(!stop);
    cv::destroyAllWindows();
    std::cerr << "end of read image thread" << std::endl;
}
/**
 * @brief 串口线程，接收到stop的信息后就停止运行
 * @return 无
 **/
void hitcrt::center::thread_serial()
{
    // std::cerr << "\033[31mThe serial is open!\033[0m" << std::endl;
    do
    {
        data.clear();
        if(!stop)
        {
            if(!this->receive(flag, data))
            {
                usleep(1000);
                continue;
            }
        }
        R.reset();
        t.reset();
        // data的存储顺序为：x坐标、y坐标(mm)、机器人的偏航角(0.1°)、相机偏航角(0.1°)
        // 将t转换为以m为单位
        t_mu.lock();
        // the receving coordinate is a little bit different, its x is on the opposite side compared with ours.
        t = std::make_shared<cv::Mat>((cv::Mat_<float>(3, 1) << -data[0] / 1000.0, data[1] / 1000.0, 0));
        *t += t0;
        t_mu.unlock();
        // 由于机器人只有yaw角的变化，故计算的R矩阵只需要一个欧拉角yaw即可
        // 这里计算的外参矩阵是相对于最初始状态下的相机位姿，由于相机目前不转，所以data[3]实际上恒为0
        temp_yaw = ((data[2] + data[3]) * 10 / 180) * M_PI;
        R_mu.lock();
        R = std::make_shared<cv::Mat>((cv::Mat_<float>(3, 3) << cos(temp_yaw), -sin(temp_yaw), 0,
                                       sin(temp_yaw), cos(temp_yaw), 0,
                                       0, 0, 1));
        R_mu.unlock();
        // 数据格式：flag表示我方是红方还是蓝方，data表示传输的数据
        std::cout << "the information is:"
                  << "flag:" << flag << "\tdata:"
                  << "(" << data[0] << ", " << data[1] << ")\n"
                  << "robot yaw:" << data[2] << "\tcamera yaw:" << data[3] << std::endl;
    } while(!stop);
    std::cerr << "end of serial" << std::endl;
}
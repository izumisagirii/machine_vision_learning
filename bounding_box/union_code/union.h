#ifndef __UNION_H__
#define __UNION_H__
#include <iostream>
#include <cmath>
#include <vector>
#include <mutex>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
// #include "../RGBD_driver/rgbd_camera.h"
#include "rgbd_camera.h"
#include "Kinectv3.h"
#include "serialapp.h"
namespace hitcrt
{
// The depth image is aligned by default, you can modify the code to avoid alignment.
class camera
{
public:
    camera(bool isKinect)
    {
        this->is_kinect = isKinect;
        if(isKinect)
        {
            kinect = std::make_shared<hitcrt::Kinectv3>();
            xtion = nullptr;
        }
        else
        {
            xtion = std::make_shared<hitcrt::RGBDcamera>(RGBDcamera::ONI,"/home/hitcrt/Downloads/DR.ONI",RGBDcamera::Xtion1);
            kinect = nullptr;
        }
    }
    void grab()
    {
        if(is_kinect)
            kinect->grab();
        else
            xtion->grab();
    }
    std::shared_ptr<cv::Mat> getImageRGB()
    {
        if(is_kinect)
            return kinect->getColorImage();
        else
            return xtion->getImageRGB();
    }
    std::shared_ptr<cv::Mat> getAlignedRGB()
    {
        if(is_kinect)
            return kinect->getAlignedColorImage();
        else
        {
            std::cerr << "This camera doesn't have aligned color image!" << std::endl;
            return nullptr;
        }
    }
    std::shared_ptr<cv::Mat> getImageDepth()
    {
        if(is_kinect)
            return kinect->getDepthImage();
        else
            return xtion->getImageDepth();
    }
    std::shared_ptr<cv::Mat> getIrImage()
    {
        if(is_kinect)
            return kinect->getIrImage();
        else
            return xtion->getImageIR();
    }
    auto getPointCloud()
    {
        if(is_kinect)
            return kinect->getPointcloud();
        else
            return xtion->getPointcloud();
    }
    std::shared_ptr<cv::Mat> getColorIntrinsic()
    {
        if(is_kinect)
            return std::make_shared<cv::Mat>(kinect->color_intrin);
        else
        // xtion's intrinsic is from the RGBD_driver
            return std::make_shared<cv::Mat>((cv::Mat_<float>(3,3)
             << 570.3422241210938, 0, 314.5,
                 0, 570.3422241210938, 235.5,
                 0, 0, 1.0));
    }
private:
    bool is_kinect;
    std::shared_ptr<hitcrt::Kinectv3> kinect;
    std::shared_ptr<hitcrt::RGBDcamera> xtion;
};

class center: public SerialApp
{
public:
    center(int baudrate=460800, bool isRelease=false, bool isKinect=true, bool openSerial=true, RECEIVE_FLAG flag=RECEIVE_FLAG::UNKNOWN);
    const std::shared_ptr<cv::Mat> get_rgb_img()
    {
        while(!init_finished)
            usleep(10);
        std::lock_guard<std::mutex> lock(rgb_img_mu);
        return rgb_img;
    }
    const std::shared_ptr<cv::Mat> get_depth_img()
    {
        while(!init_finished)
            usleep(10);
        std::lock_guard<std::mutex> lock(depth_img_mu);
        return depth_img;
    }
    const std::shared_ptr<cv::Mat> get_R()
    {
        while(!init_finished)
            usleep(10);
        std::lock_guard<std::mutex> lock(R_mu);
        return R;
    }
    const std::shared_ptr<cv::Mat> get_R0()
    {
        return R0;
    }
    const std::shared_ptr<cv::Mat> get_t()
    {
        while(!init_finished)
            usleep(10);
        std::lock_guard<std::mutex> lock(t_mu);
        return t;
    }
    const std::shared_ptr<cv::Mat> get_color_intrinsic()
    {
        return camera.getColorIntrinsic();
    }
    auto get_point_cloud()
    {
        while(!init_finished)
            usleep(10);
        return camera.getPointCloud();
    }
    bool get_robot_coordinate_point(cv::Mat& pixel_point, float depth);
    void init();
    void shutdown();
    bool stop;
private:
    float temp_yaw;
    bool open_serial;
    bool is_release;
    bool init_finished;
    bool is_kinect;
    boost::thread grab_image;
    boost::thread grab_serial;
    std::shared_ptr<cv::Mat> rgb_img;
    std::shared_ptr<cv::Mat> depth_img;
    // 外参、坐标
    // use the shared_ptr to avoid data .
    std::shared_ptr<cv::Mat> R, R0;
    std::shared_ptr<cv::Mat> t;
    cv::Mat t0;
    std::vector<float> data;
    std::mutex R_mu;
    std::mutex t_mu;
    std::mutex rgb_img_mu;
    std::mutex depth_img_mu;
    hitcrt::SerialApp::RECEIVE_FLAG flag;
    // 相机
    hitcrt::camera camera;
    void thread_image();
    void thread_serial();
};

}

#endif // __UNION_H__

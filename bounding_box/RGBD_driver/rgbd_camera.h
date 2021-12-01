// Copyright HITCRT.
// Created by Starry on 19/4/17.
// Interface class for RGBD device with openni2.

#ifndef HITCRTVISION_RGBD_H
#define HITCRTVISION_RGBD_H

#include <opencv2/opencv.hpp>
#include <openni2/OpenNI.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include <sys/types.h>                      // 下面四个头文件是linux系统编程特有的
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <linux/videodev2.h>                // 操作摄像头设备
#include <dirent.h>
#include <vector>
#include <cstring>
#include <boost/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hitcrt{

    class RGBDcamera {
    public:
        typedef pcl::PointXYZ Point;
        typedef pcl::PointXYZRGB Cpoint;
        typedef pcl::PointCloud<Point> Cloud;
        typedef pcl::PointCloud<Cpoint> Ccloud;
        typedef Cloud::Ptr Pcloud;
        typedef Ccloud::Ptr Pccloud;
      typedef std::shared_ptr<RGBDcamera> Ptr;
      typedef std::vector<Point,Eigen::aligned_allocator<Point>> PointVec;///Eigen的容器必须用Eigen::aligned_allocator<T>
      typedef enum device {
        Kinect,
        Xtion1,
        Xtion2,
        ASTRA,
        ONI,
        Any
      }Device;
      RGBDcamera(Device rgbd=RGBDcamera::Any,const char* filename = NULL,Device oni_dev=RGBDcamera::Kinect,bool isregi=true,bool grab_ir=false);
      ~RGBDcamera();

      void grab();
      std::shared_ptr<cv::Mat> getImageDepth(bool destroy=true);
      std::shared_ptr<cv::Mat> getImageRGB(bool destroy=true);
      std::shared_ptr<cv::Mat> getImageIR(bool destroy=true);
      Pcloud getPointcloud(Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),bool destroy=true,int numthread=2);
      Pccloud getCPointcloud(Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),bool destroy=true,int numthread=2);
      Pcloud getPointcloud(std::vector<cv::Point3f> input,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
      Pcloud getPointcloud(PointVec input,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
      bool get3dPoint(cv::Point3f input,Point &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
      bool get3dPoint(Point input,Point &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
      bool get2dPoint(Point input,cv::Point2f &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
      bool get2dPoint(cv::Point3f input,cv::Point2f &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
      bool get2dPoints(PointVec input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
      bool get2dPoints(std::vector<cv::Point3f> input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
      bool get2dPoints(Cloud input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
      bool get2dPoints(Pcloud input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
      bool record(const char* filename="video.ONI");
      void stopRecord();
      void showdevice();
      RGBDcamera::Device getDeviceType();
      int getFrameNum();
    private:
      openni::Status getStatus();
      void closecamera();
      bool enumuvcDevices(std::vector<std::string>& files);
      bool findAstrRGBDevice(char* vender,std::string& name);
      void m_astragrab();
      int XTION_HEIGHT=480;////xtion深度图还可以640x480不过点云量太大很卡,彩色图只能320x240
      int XTION_WIDTH=640;
      int KINECT_HEIGHT=424;
      int KINECT_WIDTH=512;
      int ASTRA_HEIGHT=240;////astra可以640x480不过点云量太大很卡
      int ASTRA_WIDTH=320;
      float camera_cx,camera_cy,camera_fx,camera_fy;
      RGBDcamera::Device m_device;
      cv::VideoCapture astra_rgb;
      std::shared_ptr<cv::Mat> m_ImageDepth;
      std::shared_ptr<cv::Mat> m_ImageDepth4cloud;
      std::shared_ptr<cv::Mat> m_ImageRGB;
      std::shared_ptr<cv::Mat> m_ImageRGB4cloud;
      std::shared_ptr<cv::Mat> m_AstraRGB;
      std::shared_ptr<cv::Mat> m_ImageIR;
      bool isok=true;
      bool grab_ir;
      const char* m_targetFile;
      int resolution_width;
      int resolution_height;
      std::string vendor = "";
      cv::Mat grabframe,saveframe;
      boost::shared_mutex astra_mutex,depth_mutex,ir_mutex,color_mutex,cloud_mutex;
      boost::thread m_astragrabThread;
      openni::Recorder mRecorder;
      openni::Status rc;
      openni::Device camera;
      openni::Array<openni::DeviceInfo> aDeviceList;
      openni::VideoFrameRef frameDepth;
      openni::VideoFrameRef frameColor;
      openni::VideoFrameRef frameIR;
      openni::VideoStream m_streamDepth;
      openni::VideoStream m_streamColor;
      openni::VideoStream m_streamIR;
      openni::PlaybackControl* playback;
    };


}

#endif //HITCRTVISION_RGBD_H

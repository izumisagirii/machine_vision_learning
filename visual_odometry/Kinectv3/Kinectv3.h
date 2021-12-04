//
// Created by l on 2020/2/25.
//

#ifndef KINECT_AZURE_KINECTV3_H
#define KINECT_AZURE_KINECTV3_H

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <k4arecord/record.hpp>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread.hpp>
namespace hitcrt {
    class Kinectv3 {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW///因为类成员变量有Eigen的对象，需要使用Eigen对齐方式
        typedef pcl::PointXYZ Point;
        typedef pcl::PointCloud<Point> Cloud;
        typedef Cloud::Ptr Pcloud;
        typedef Cloud::ConstPtr ConstPcloud;
        typedef pcl::PointXYZRGB ColorPoint;
        typedef pcl::PointCloud<ColorPoint> ColorCloud;
        typedef ColorCloud::Ptr PColorCloud;
        typedef std::vector<Point,Eigen::aligned_allocator<Point>> PointVec;///Eigen的容器必须用Eigen::aligned_allocator<T>
        enum depth_mode{
            NFOV_BIND,//大小:320*288;视角:75*65;距离:0.5-5.46;帧率:0,5,15,30
            NFOV_UBIND,//大小:640*576;视角:75*65;距离:0.5-3.86;帧率:0,5,15,30
            WFOV_BIND,//大小:512*512;视角:120*120;距离:0.25-2.88;帧率:0,5,15,30
            WFOV_UBIND,//大小:1024*1024;视角:120*120;距离:0.25-2.21;帧率:0,5,15
        };
        enum color_resolution{
            RES_720P,//1280*720,fps:0,5,15,30
            RES_1080P,//1920*1080,fps:0,5,15,30
            RES_1440P,//2560*1440,fps:0,5,15,30
            RES_1536P,//2048*1536,fps:0,5,15,30
            RES_2160P,//3840*2160,fps:0,5,15,30
            RES_3072P,//4096*3072,fps:0,5,15
        };
        static uint32_t getDeviceNum(){
            return k4a::device::get_installed_count();
        };
        Kinectv3(std::string filepath,bool start_imu=false);
        Kinectv3(int id=0,bool start_imu=false,std::string videofile="",depth_mode d_mode=NFOV_BIND,color_resolution c_res=RES_720P,int maxfailnum=2);
        ~Kinectv3()= default;
        void init();
        std::string grab(int timeout=100);
        std::string sample(int timeout=100);
        std::shared_ptr<cv::Mat> getAlignedColorImage(bool destroy=true);
        std::shared_ptr<cv::Mat> getColorImage(bool destroy=true);
        std::shared_ptr<cv::Mat> getDepthImage(bool destroy=true);
        std::shared_ptr<cv::Mat> getIrImage(bool destroy=true);
        void getCalibParam();
        PColorCloud getColorCloud(Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        Pcloud getPointcloud(Point range_min=Point(-100000,-100000,-100000),Point range_max=Point(100000,100000,100000),Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        Pcloud getPointcloud(cv::Mat input,Point range_min=Point(-100000,-100000,-100000),Point range_max=Point(100000,100000,100000),Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        Pcloud getPointcloud(std::vector<cv::Point3f> input,Point range_min=Point(-100000,-100000,-100000),Point range_max=Point(100000,100000,100000),Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        Pcloud getPointcloud(PointVec input,Point range_min=Point(-100000,-100000,-100000),Point range_max=Point(100000,100000,100000),Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        bool get3dPoint(cv::Point3f input,Point &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
        bool get3dPoint(Point input,Point &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
        bool get2dPoint(cv::Point3f input,cv::Point2f &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
        bool get2dPoint(Point input,cv::Point2f &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity());
        bool get2dPoints(PointVec input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        bool get2dPoints(std::vector<cv::Point3f> input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        bool get2dPoints(Cloud input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        bool get2dPoints(Pcloud input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans=Eigen::Matrix4f::Identity(),int numthread=2);
        uint64_t getAccTime(){
            boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
            return acc_time;
        }//us
        uint64_t getGyroTime(){
            boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
            return gyro_time;
        }
        uint64_t getColorTime(){
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return  color_time;
        }//ns
        uint64_t getDepthTime(){
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return  depth_time;
        }
        uint64_t getIrTime(){
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return  ir_time;
        }
        std::shared_ptr<Eigen::Vector3f> getAcc(bool destroy=true){
            if(destroy){
                boost::unique_lock<boost::shared_mutex> lock(imu_mutex);
                auto temp=acc;
                acc.reset();
                return temp;
            }else{
                boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
                return acc;
            }
        }
        std::shared_ptr<Eigen::Vector3f> getGyro(bool destroy=true){
            if(destroy){
                boost::unique_lock<boost::shared_mutex> lock(imu_mutex);
                auto temp=gyro;
                gyro.reset();
                return temp;
            }else{
                boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
                return gyro;
            }
        }
        std::string getSerialnumber();
        void record(std::string filename);
        bool checkFile(std::string filename);
        bool isinit=false;
        cv::Mat depth_intrin;
        cv::Mat depth_dist;
        cv::Mat color_intrin;
        cv::Mat color_dist;
        cv::Mat color_extrin;
        cv::Mat depth_extrin;
    private:
        boost::shared_mutex image_mutex;
        boost::shared_mutex imu_mutex;
        boost::shared_mutex imagecloud_mutex;
        boost::mutex init_mutex;
        int failednum=0;
        int mmaxfailnum;
        int mid=0;
        std::string mvideofile="";
        std::string videopath="";
        bool isvideo=false;
        bool start_imu;
        int min_depth=50;//mm
        std::shared_ptr<k4a::device> device;
        k4a::calibration calib;
        k4a::transformation transform;
        std::shared_ptr<k4a::record> recorder;
        k4a::playback playback;
        uint64_t acc_time=0,gyro_time=0;
        std::shared_ptr<Eigen::Vector3f> acc,gyro;
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        std::shared_ptr<cv::Mat> cvcolor,cvdepth,cvir,cvaligned,depth4cloud,color4cloud;
        uint64_t color_time,depth_time,ir_time;
    };

}
#endif //KINECT_AZURE_KINECTV3_H

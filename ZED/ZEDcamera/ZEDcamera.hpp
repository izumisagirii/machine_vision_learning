#ifndef HITCRTVISION_ZEDCAMERA_H
#define HITCRTVISION_ZEDCAMERA_H
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <boost/thread.hpp>
namespace hitcrt{
    class ZEDcamera{
    public:
        typedef pcl::PointXYZRGB Point;
        typedef pcl::PointCloud<Point> Cloud;
        typedef Cloud::Ptr Pcloud;
        enum IMAGE_TYPE{
            LEFT,
            RIGHT,
            DEPTH
        };
        enum RESOLUTION{
            R_720,
            R_1080,
            R_2k,
            R_VGA//640*480
        };
        ZEDcamera(std::string device="",ZEDcamera::RESOLUTION resolution=ZEDcamera::R_VGA);
        ~ZEDcamera(){
            closeCamera();
        };
        void init();
        /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取相机序列号
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
        int getSerialNumber(){ return caminfo.serial_number;}
        /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取相机参数
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
        sl::InitParameters getCameraParam(){ return camera_params;}
        bool grab();
        /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取抓图时间戳
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
        uint64 getGrabTimestamp(){
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return grab_timstamp;
        }
        /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取姿态时间戳
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
        uint64 getPoseTimestamp(){
            boost::shared_lock<boost::shared_mutex> lock(pose_mutex);
            return  pose_timestamp;
        }
        /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取IMU时间戳
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
        uint64 getIMUTimestamp(){
            boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
            return imu_timestamp;
        }
        bool grabPose();
        bool grabIMU();
        std::shared_ptr<cv::Mat> getLImage(bool destroy=true);
        std::shared_ptr<cv::Mat> getRImage(bool destroy=true);
        std::shared_ptr<cv::Mat> getDImage(bool destroy=true);
        std::shared_ptr<cv::Mat> getTrans(bool destroy=true);
        std::shared_ptr<cv::Mat> getRotate(bool destroy=true);
        std::shared_ptr<cv::Mat> getAcc(bool destroy=true);
        std::shared_ptr<cv::Mat> getOrien(bool destroy=true);
        Pcloud getPointCloud(bool destroy=true);
        bool enableRecord(char* file,sl::SVO_COMPRESSION_MODE mode=sl::SVO_COMPRESSION_MODE::H264);
        void stopRecord();
        void closeCamera();
        void getCalibParams(cv::Mat& T_2_L,cv::Mat& intrinL,cv::Mat& distL,cv::Mat& intrinR,cv::Mat& distR);

    private:
        boost::shared_mutex image_mutex,pose_mutex,imu_mutex,init_mutex;
        boost::thread record;
        int failnum=0;
        bool is_recording;
        sl::Camera camera;
        sl::CameraInformation caminfo;
        sl::InitParameters camera_params;
        sl::RecordingParameters recording_params;
        sl::ERROR_CODE statu_code=sl::ERROR_CODE::SUCCESS;
        sl::Mat left_image,right_image,depth_image,cloud_img;
        std::shared_ptr<cv::Mat> m_left_img,m_right_img,m_depth_img;
        sl::Pose camera_pose;
        uint64 pose_timestamp,grab_timstamp,imu_timestamp;
        std::shared_ptr<cv::Mat> m_cameraTrans,m_cameraRotate;
        std::shared_ptr<cv::Mat> m_orientation,m_acceleration;
        sl::SensorsData sensor_data;
        Pcloud m_cloud;
        cv::Mat slMat2cvMat(sl::Mat& input);
        bool slPcl2Pcl(const sl::Mat& data_cloud,Pcloud pcl_cloud);
        void record_s();
        inline float convertColor(float colorIn);
    };
}
#endif
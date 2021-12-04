#include "ZEDcamera.hpp"
namespace hitcrt{
/**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 构造函数
*@param
* device:默认为空，打开相机,若不为空则打开对应视频svo文件
* resolution:分辨率默认720p,可以设置为720p,1080p,和2k
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    ZEDcamera::ZEDcamera(std::string device,ZEDcamera::RESOLUTION resolution) {
        camera_params.camera_fps=100;
        if(!device.empty()){
            camera_params.input.setFromSVOFile(device.c_str());
        }
        switch (resolution){
            case ZEDcamera::R_720:
                camera_params.camera_resolution=sl::RESOLUTION::HD720;
                break;
            case ZEDcamera::R_1080:
                camera_params.camera_resolution=sl::RESOLUTION::HD1080;
                break;
            case ZEDcamera::R_2k:
                camera_params.camera_resolution=sl::RESOLUTION::HD2K;
                break;
            case ZEDcamera::R_VGA:
                camera_params.camera_resolution=sl::RESOLUTION::VGA;
                break;
            default:
                camera_params.camera_resolution=sl::RESOLUTION::HD720;
        }
        camera_params.depth_mode=sl::DEPTH_MODE::PERFORMANCE;
        camera_params.coordinate_units=sl::UNIT::MILLIMETER;
        init();
    }
    void ZEDcamera::init(){
        init_mutex.lock();
        if(camera.isOpened()){
            init_mutex.unlock();
            return;
        }
        statu_code=camera.open(camera_params);
        if(statu_code!=sl::ERROR_CODE::SUCCESS){
            std::cerr<<"打开相机失败！"<<std::endl;
            camera.close();
        }
        caminfo=camera.getCameraInformation();
        init_mutex.unlock();
    }
/**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 抓一次图
*@param
*@return bool抓图是否成功
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    bool ZEDcamera::grab(){
        if(!camera.isOpened()){
           init();
            sleep(1);
            return false;
        }
        statu_code=camera.grab();
        if (statu_code == sl::ERROR_CODE::SUCCESS){
            camera.retrieveImage(left_image,sl::VIEW::LEFT);
            camera.retrieveImage(right_image,sl::VIEW::RIGHT);
            camera.retrieveMeasure(depth_image,sl::MEASURE::DEPTH);
            camera.retrieveMeasure(cloud_img,sl::MEASURE::XYZRGBA);
            failnum=0;
        } else{
            failnum++;
            std::cout<<"grab failed!"<<std::endl;
            if(failnum>2){
                camera.close();
            }
            
            return false;
        }
        auto time=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE);
        //auto time=camera_pose.timestamp;
        cv::Mat cv_left_img=slMat2cvMat(left_image);
        cv::Mat cv_right_img=slMat2cvMat(right_image);
        cv::Mat cv_depth_img=slMat2cvMat(depth_image);
        Pcloud tempcloud(new Cloud);
        pcl::PointXYZRGB p;
        tempcloud->points.resize(sl::getResolution(camera_params.camera_resolution).area(),p);
        //std::cout<<sl::getResolution(camera_params.camera_resolution).area()<<cloud_img.getHeight()<<cloud_img.getWidth()<<std::endl;
        slPcl2Pcl(cloud_img,tempcloud);
        boost::unique_lock<boost::shared_mutex> lock(image_mutex);
        m_left_img=std::make_shared<cv::Mat>(cv_left_img);
        m_right_img=std::make_shared<cv::Mat>(cv_right_img);
        m_depth_img=std::make_shared<cv::Mat>(cv_depth_img);
        m_cloud=tempcloud;
        grab_timstamp=time;
        return true;
    }
/**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取相机姿态
*@param
* _trans:当前相机相对于初始时刻的平移向量
* _rotate:当前相机相当于初始时刻的旋转矩阵
*@return 是否获取成功
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    bool ZEDcamera::grabPose(){
        if(!camera.isOpened()){
            init();
            sleep(1);
            return false;
        }
        sl::PositionalTrackingParameters tracking_parameters;
        if(!camera.isPositionalTrackingEnabled()){
            statu_code = camera.enablePositionalTracking(tracking_parameters);
            if(statu_code!=sl::ERROR_CODE::SUCCESS){
                std::cout<<"无法获取位姿!"<<std::endl;
                return false;
            }
        }
        camera.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
        sl::Translation trans=camera_pose.getTranslation();
        sl::Rotation rotation=camera_pose.getRotationMatrix();
        cv::Mat temp_trans=(cv::Mat_<double>(3,1)<<trans.x,trans.y,trans.z);
        cv::Mat temp_rotate=(cv::Mat_<double>(3,3)<<rotation.r00,rotation.r01,rotation.r02,
                rotation.r10,rotation.r11,rotation.r12,
                rotation.r20,rotation.r21,rotation.r22);
        {
            boost::unique_lock<boost::shared_mutex> lock(pose_mutex);
            m_cameraTrans=std::make_shared<cv::Mat>(temp_trans);
            m_cameraRotate=std::make_shared<cv::Mat>(temp_rotate);
            pose_timestamp=camera_pose.timestamp;
        }
       // std::cout<<camera_pose.getEulerAngles().x*180/CV_PI<<" "<<camera_pose.getEulerAngles().y*180/CV_PI<<" "<<camera_pose.getEulerAngles().z*180/CV_PI<<std::endl;
        return true;
    }

    std::shared_ptr<cv::Mat> ZEDcamera::getTrans(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(pose_mutex);
            auto temp=m_cameraTrans;
            m_cameraTrans.reset();
            return temp;
        }else{
            boost::shared_lock<boost::shared_mutex> lock(pose_mutex);
            return m_cameraTrans;
        }
    }
    std::shared_ptr<cv::Mat> ZEDcamera::getRotate(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(pose_mutex);
            auto temp=m_cameraRotate;
            m_cameraRotate.reset();
            return temp;
        }else{
            boost::shared_lock<boost::shared_mutex> lock(pose_mutex);
            return m_cameraRotate;
        }
    }
    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取相机IMU数据,只有ZED_M有IMU,队里的ZED没有
*@param
* orientation:陀螺仪返回值
* acceleration:加速度计返回值
*@return 是否获取成功
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    bool ZEDcamera::grabIMU(){
        if(!camera.isOpened()){
            init();
            sleep(1);
            return false;
        }
        bool haveIMU = (camera.getCameraInformation().camera_model == sl::MODEL::ZED_M);
        if(!haveIMU){
            std::cout<<"此相机没有IMU!"<<std::endl;
            return false;
        }
        camera.getSensorsData(sensor_data, sl::TIME_REFERENCE::IMAGE);
        sl::Orientation ori=camera_pose.getOrientation();
        cv::Mat orientation=(cv::Mat_<double>(4,1)<<ori.ox,ori.oy,ori.oz,ori.ow);
        cv::Mat acceleration=(cv::Mat_<double>(3,1)<<sensor_data.imu.linear_acceleration.x,
                sensor_data.imu.linear_acceleration.y, sensor_data.imu.linear_acceleration.z);
        {
            boost::unique_lock<boost::shared_mutex> lock(imu_mutex);
            imu_timestamp=sensor_data.imu.timestamp;
            m_orientation=std::make_shared<cv::Mat>(orientation);
            m_acceleration=std::make_shared<cv::Mat>(acceleration);
        }
        return true;
    }
    std::shared_ptr<cv::Mat> ZEDcamera::getAcc(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(imu_mutex);
            auto temp=m_acceleration;
            m_acceleration.reset();
            return temp;
        }else{
            boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
            return m_acceleration;
        }
    }
    std::shared_ptr<cv::Mat> ZEDcamera::getOrien(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(imu_mutex);
            auto temp=m_orientation;
            m_orientation.reset();
            return temp;
        }else{
            boost::shared_lock<boost::shared_mutex> lock(imu_mutex);
            return m_orientation;
        }
    }
    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 关闭相机
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    void ZEDcamera::closeCamera(){
        camera.close();
    }
    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 打开录视频功能，2021：对新的接口进行修改，以往的录制有关简介可能不适用
*@param
* file:视频文件路径(*.svo)
* mode:模式，不知道是什么
* cyh：模式有h264（基本支持），h265（只有pascal显卡支持），显卡不支持就lossless，api参考我的修改
*@return 是否成功
* @author
* -黎林 phone:18846140245 QQ:1430244438
* —崔永恒
*/
    bool ZEDcamera::enableRecord(char* file,sl::SVO_COMPRESSION_MODE mode){
        recording_params.compression_mode = mode;
        recording_params.video_filename = file;
        statu_code=camera.enableRecording(recording_params);
        if (statu_code != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Recording initialization error. " << toString(statu_code) << std::endl;
            if (statu_code == sl::ERROR_CODE::SVO_RECORDING_ERROR)
                std::cout << " Note : This error mostly comes from a wrong path or missing writing permissions." << std::endl;
            if (statu_code == sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION)
                std::cout << " Note : This error mostly comes from a non-compatible graphic card. If you are using HEVC compression (H265), please note that most of the graphic card below pascal architecture will not support it. Prefer to use AVCHD compression which is supported on most of NVIDIA graphic cards" << std::endl;
            return false;
        }
        return true;
        record = boost::thread(std::bind(&ZEDcamera::record_s,this));
    }
    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 独立的录制线程，调用stoprecord终止
*@param
*@return 是否成功
* @author
* -崔永恒
*/
    void ZEDcamera::record_s(){
        while(camera.isOpened() && is_recording){
            camera.grab();
        }
        camera.disableRecording();
    }
    std::shared_ptr<cv::Mat> ZEDcamera::getLImage(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=m_left_img;
            m_left_img.reset();
            return temp;
        } else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return m_left_img;
        }
    }
    std::shared_ptr<cv::Mat> ZEDcamera::getRImage(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=m_right_img;
            m_right_img.reset();
            return temp;
        } else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return m_right_img;
        }
    }
    std::shared_ptr<cv::Mat> ZEDcamera::getDImage(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=m_depth_img;
            m_depth_img.reset();
            return temp;
        } else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return m_depth_img;
        }
    }
    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 获取点云
*@param
*cloud：点云
*@return 是否成功
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    ZEDcamera::Pcloud ZEDcamera::getPointCloud(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=m_cloud;
            m_cloud.reset();
            return temp;
        } else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return m_cloud;
        }
    }
    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief zed mat转opencv mat
*@param
*input zed mat
*@return opencv mat
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    cv::Mat ZEDcamera::slMat2cvMat(sl::Mat& input) {
        // Mapping between MAT_TYPE and CV_TYPE
        int cv_type = -1;
        switch (input.getDataType()) {
            case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
            case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
            case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
            case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
            case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
            case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
            case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
            case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
            default: break;
        }

        // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
        // cv::Mat and sl::Mat will share a single memory structure
        return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU)).clone();
    }

    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief zed 点云转pcl点云
*@param
*data_cloud： zed 点云
* pcl_cloud：pcl点云
*@return 是否成功
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    bool ZEDcamera::slPcl2Pcl(const sl::Mat& data_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud){
        float *p_data_cloud = data_cloud.getPtr<float>();
        int index = 0;
        // Check and adjust points for PCL format
        for (auto &it : pcl_cloud->points) {
            float X = p_data_cloud[index];
            if (!isValidMeasure(X)) // Checking if it's a valid point
                it.x = it.y = it.z = it.rgb = 0;
            else {
                it.x = p_data_cloud[index];
                it.y = p_data_cloud[index + 1];
                it.z = p_data_cloud[index + 2];
                it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
            }
            index += 4;
        }
        return true;
    }

    /**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief zed color转pcl color
*@param
*data_cloud： zed color
*@return pclcolor
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
    inline float ZEDcamera::convertColor(float colorIn) {
        uint32_t color_uint = *(uint32_t *) & colorIn;
        unsigned char *color_uchar = (unsigned char *) &color_uint;
        color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
        return *reinterpret_cast<float *> (&color_uint);
    }

    void ZEDcamera::getCalibParams(cv::Mat& T_2_L,cv::Mat& intrinL,cv::Mat& distL,cv::Mat& intrinR,cv::Mat& distR){
        sl::CalibrationParameters calibparams=caminfo.calibration_parameters;
        sl::Rotation R;
        sl::float3 t=calibparams.T;
        R.setRotationVector(calibparams.R);
        T_2_L=(cv::Mat_<double >(4,4)<<R.r00,R.r01,R.r02,t.x,
                R.r10,R.r11,R.r12,t.y,
                R.r20,R.r21,R.r22,t.z,
                0,0,0,1);
         sl::CameraParameters l_param=calibparams.left_cam,
         r_param=calibparams.right_cam;
         intrinL=(cv::Mat_<double>(3,3)<<l_param.fx,0,l_param.cx,
                 0,l_param.fy,l_param.cy,
                 0,0,1);
        intrinR=(cv::Mat_<double>(3,3)<<r_param.fx,0,r_param.cx,
                0,r_param.fy,r_param.cy,
                0,0,1);
        distL=(cv::Mat_<double>(1,5)<<l_param.disto[0],l_param.disto[1],l_param.disto[2],l_param.disto[3],l_param.disto[4]);
        distR=(cv::Mat_<double>(1,5)<<r_param.disto[0],r_param.disto[1],r_param.disto[2],r_param.disto[3],r_param.disto[4]);
    }

    void ZEDcamera::stopRecord() {
        is_recording = false;
        record.join();
    }

}
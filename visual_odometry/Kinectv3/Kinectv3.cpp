//
// Created by l on 2020/2/25.
//

#include "Kinectv3.h"
namespace hitcrt{
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 构造函数,打开相机
* @param id:打开第id个相机,只使用一个相机时为0
*@param start_imu:是否开启imu
*@param videofile:录制视频的路径，非法路径表示不录制视频，视频文件应以.mkv结尾
*@param d_mode:深度模式,深度相机的模式,具体解释见官方文档
*@param c_res:彩色图分辨率,具体解释见官方文档
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/

    Kinectv3::Kinectv3(int id,bool start_imu,std::string videofile,depth_mode d_mode,color_resolution c_res,int maxfailnum){
        switch (d_mode){///设置深度相机模式
            case NFOV_BIND://大小:320*288;视角:75*65;距离:0.5-5.46;帧率:0,5,15,30
                config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
                config.camera_fps = K4A_FRAMES_PER_SECOND_30;
                break;
            case NFOV_UBIND://大小:640*576;视角:75*65;距离:0.5-3.86;帧率:0,5,15,30
                config.depth_mode=K4A_DEPTH_MODE_NFOV_UNBINNED;
                config.camera_fps = K4A_FRAMES_PER_SECOND_30;
                break;
            case WFOV_BIND://大小:512*512;视角:120*120;距离:0.25-2.88;帧率:0,5,15,30
                config.depth_mode=K4A_DEPTH_MODE_WFOV_2X2BINNED;
                config.camera_fps = K4A_FRAMES_PER_SECOND_30;
                break;
            case WFOV_UBIND://大小:1024*1024;视角:120*120;距离:0.25-2.21;帧率:0,5,15
                config.depth_mode=K4A_DEPTH_MODE_WFOV_UNBINNED;
                config.camera_fps = K4A_FRAMES_PER_SECOND_15;
                break;
            default:
                config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
                config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        }
        switch (c_res){///设置彩色相机分辨率
            case RES_720P://1280*720,fps:0,5,15,30
                config.color_resolution=K4A_COLOR_RESOLUTION_720P;
                break;
            case RES_1080P://1920*1080,fps:0,5,15,30
                config.color_resolution=K4A_COLOR_RESOLUTION_1080P;
                break;
            case RES_1440P://2560*1440,fps:0,5,15,30
                config.color_resolution=K4A_COLOR_RESOLUTION_1440P;
                break;
            case RES_1536P://2048*1536,fps:0,5,15,30
                config.color_resolution=K4A_COLOR_RESOLUTION_1536P;
                break;
            case RES_2160P://3840*2160,fps:0,5,15,30
                config.color_resolution=K4A_COLOR_RESOLUTION_2160P;
                break;
            case RES_3072P://4096*3072,fps:0,5,15
                config.color_resolution=K4A_COLOR_RESOLUTION_3072P;
                config.camera_fps = K4A_FRAMES_PER_SECOND_15;
                break;
            default:
                config.color_resolution=K4A_COLOR_RESOLUTION_720P;
        }
        config.color_format=K4A_IMAGE_FORMAT_COLOR_MJPG;///不要动
        mmaxfailnum=maxfailnum;
        this->start_imu=start_imu;
        mid=id;
        mvideofile=videofile;
        init();
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 构造函数，打开视频(.mkv文件)
*@param filepath:文件路径
     * @param start_imu:是否含有imu数据
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    Kinectv3::Kinectv3(std::string filepath,bool start_imu){
        isvideo=true;
        this->videopath=filepath;
        this->start_imu=start_imu;
        init();
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 初始化函数
* @param id:打开第id个相机,只使用一个相机时为0
*@param start_imu:是否开启imu
*@param videofile:录制视频的路径，非法路径表示不录制视频，视频文件应以.mkv结尾
*@param d_mode:深度模式,深度相机的模式,具体解释见官方文档
*@param c_res:彩色图分辨率,具体解释见官方文档
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    void Kinectv3::init(){
       init_mutex.lock();
        if (isinit) {
            init_mutex.unlock();
            return;
        }
       if(!isvideo) {
           uint32_t num = k4a::device::get_installed_count();///获取连接设备数
           if (num < mid + 1) {///没有指定id的相机
               std::cerr << "没有id为" << mid << "的相机!" << std::endl;
               init_mutex.unlock();
               return;
           }
           try {
               device = std::make_shared<k4a::device>(k4a::device::open(mid));///打开设备
           } catch (k4a::error e) {
               std::cout << "打开设备失败:" << e.what() << std::endl;
               init_mutex.unlock();
               return;
           }
           try {
               device->start_cameras(&config);///开启相机
           } catch (k4a::error e) {
               std::cout << "开启相机失败:" << e.what() << std::endl;
               init_mutex.unlock();
               return;
           }
           if (start_imu) {
               try {
                   device->start_imu();///开启imu
               } catch (k4a::error e) {
                   std::cerr << "开启imu失败:" << e.what() << std::endl;
               }
           }
           try {
               calib = device->get_calibration(config.depth_mode, config.color_resolution);///获取相机内外参
           } catch (k4a::error e) {
               std::cerr << "获取相机参数失败:" << e.what() << std::endl;
               init_mutex.unlock();
               return;
           }
           isinit = true;
           transform = k4a::transformation(calib);///获取各坐标系(深度，彩色，陀螺仪，加速度计)间的变换
           getCalibParam();
           if (checkFile(mvideofile)) {
               record(mvideofile);///开启视频录制
           }
       } else{
           if(!checkFile(videopath)){
               std::cerr<<"文件名不合法!"<<std::endl;
               init_mutex.unlock();
               return ;
           }
           try {
               playback=k4a::playback::open(videopath.c_str());///打开视频
           }catch(k4a::error e){
               std::cerr<<"打开视频失败:"<<e.what()<<std::endl;
               init_mutex.unlock();
               return ;
           }
           try {
               calib=playback.get_calibration();///获取相机内外参
           }catch (k4a::error e){
               std::cerr<<"get calib faild:"<<e.what()<<std::endl;
               init_mutex.unlock();
               return ;
           }
           isinit=true;
           transform=k4a::transformation(calib);///获取各坐标系(深度，彩色，陀螺仪，加速度计)间的变换
           getCalibParam();
       }
        init_mutex.unlock();
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 检查视频文件路径是否合法(.mkv文件)
* @param filename:文件路径
* @return bool:是否合法
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::checkFile(std::string filename){
        if(filename.empty()||filename.size()==0){
            return false;
        }
        auto result=filename.find(".mkv");///以.mkv结尾
        if(result==std::string::npos){
            std::cerr<<"视频文件名不合法！"<<std::endl;
            return false;
        } else{
            return true;
        }
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 获取相机(彩色和深度)内外参,外参以深度相机为原点,但好像彩色
     * 相机外参给的其实是外参的逆,这些参数都保存在类的公共成员变量中
* @param
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    void Kinectv3::getCalibParam(){
        if(!isinit){
            std::cout<<"还未初始化"<<std::endl;
            init();
            return;
        }
        auto temp=calib.depth_camera_calibration.intrinsics.parameters.param;///彩色相机内参
        depth_intrin=(cv::Mat_<float >(3,3)<<temp.fx,0,temp.cx,
                                                    0,temp.fy,temp.cy,
                                                    0,0,1);
        depth_dist=(cv::Mat_<float >(1,8)<<temp.k1,temp.k2,temp.p1,temp.p2,temp.k3,temp.k4,temp.k5,temp.k6);///畸变
        auto rot=calib.depth_camera_calibration.extrinsics.rotation;///彩色相机外参，旋转
        auto tra=calib.depth_camera_calibration.extrinsics.translation;///彩色相机外参，平移
        depth_extrin=(cv::Mat_<float>(4,4)<<rot[0],rot[1],rot[2],tra[0],
                                                         rot[3],rot[4],rot[5],tra[1],
                                                         rot[6],rot[7],rot[8],tra[2],
                                                         0,0,0,1);///因为坐标系以深度相机为参考所以这个矩阵为单位阵
        std::cout<<"depth extrin\n"<<depth_extrin<<std::endl;
        std::cout<<"depth intrin:\n"<<depth_intrin<<std::endl;
        std::cout<<"depth distor:\n"<<depth_dist<<std::endl;
        temp=calib.color_camera_calibration.intrinsics.parameters.param;///深度相机内参
        color_intrin=(cv::Mat_<float >(3,3)<<temp.fx,0,temp.cx,
                0,temp.fy,temp.cy,
                0,0,1);
        color_dist=(cv::Mat_<float >(1,8)<<temp.k1,temp.k2,temp.p1,temp.p2,temp.k3,temp.k4,temp.k5,temp.k6);
        rot=calib.color_camera_calibration.extrinsics.rotation;
        tra=calib.color_camera_calibration.extrinsics.translation;
        color_extrin=(cv::Mat_<float>(4,4)<<rot[0],rot[1],rot[2],tra[0],
                rot[3],rot[4],rot[5],tra[1],
                rot[6],rot[7],rot[8],tra[2],
                0,0,0,1);///这个矩阵实际上好像是彩色相机坐标系到深度相机坐标系变换矩阵的逆，也就是好像实际上是彩色彩色相机外参的逆，不知道为什么
        std::cout<<"color extrin\n"<<color_extrin<<std::endl;
        std::cout<<"color intrin:\n"<<color_intrin<<std::endl;
        std::cout<<"color dist:\n"<<color_dist<<std::endl;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 获取相机序列号
* @param
* @return std::string :相机序列号
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::string Kinectv3::getSerialnumber(){
        if(isvideo){
            std::cerr<<"视频没有序列号!"<<std::endl;
            return "";
        }
        if(!isinit){
            std::cout<<"还未初始化"<<std::endl;
            return "";
        }
        try{
            return device->get_serialnum();
        }catch (k4a::error e){
            std::cout<<"获取序列号失败:"<<e.what()<<std::endl;
        }
        return "";
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 抓图
* @param timeout：超时时间
* @return string：结果状态
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::string Kinectv3::grab(int timeout){
        if(!isinit){
            //std::cout<<"还未初始化"<<std::endl;
            init();
            sleep(1);
            return "init";
        }
        k4a::capture capture;
        try {
            bool result=false;
            if(isvideo){
                result=playback.get_next_capture(&capture);///视频
                if(!result){
                    //std::cerr<<"视频结束!"<<std::endl;
                    return "eof";
                }
            } else{
                result=device->get_capture(&capture,std::chrono::milliseconds(timeout));///相机
                if(!result){
                   // std::cerr<<"抓图超时!"<<std::endl;
                   failednum++;
                   if(failednum>=mmaxfailnum){///连续5次抓图失败判断设备是不是断开
                       uint32_t num=k4a::device::get_installed_count();///获取连接设备数
                       if(num<mid+1){///没有指定id的相机
                           std::cerr<<"设备断开,尝试重新连接"<<std::endl;
                           recorder.reset();
                           isinit=false;
                           failednum=0;
                           return "disconnect";
                       }
                   }
                    return "timeout";
                }
            }
            k4a::image mcolor=capture.get_color_image();
            k4a::image mdepth=capture.get_depth_image();
            k4a::image mir=capture.get_ir_image();
            k4a::image mbgr;
            bool covsucceed=true;
            if(mcolor.get_height_pixels()>=720&&mcolor.get_width_pixels()>=1080){
                try {
                    mcolor.cov2bgr(mbgr);///转换为bgr格式
                }catch (k4a::error e){
                    std::cerr<<"jpg转bgra失败!"<<std::endl;
                    covsucceed=false;
                }
            } else{
                covsucceed=false;
            }
            cv::Mat tempcolor;
            if(covsucceed){
                tempcolor=cv::Mat(mbgr.get_height_pixels(),mbgr.get_width_pixels(),CV_8UC4,mbgr.get_buffer()).clone();
            }
            cv::Mat tempdepth=cv::Mat(mdepth.get_height_pixels(),mdepth.get_width_pixels(),CV_16UC1,mdepth.get_buffer()).clone();
            cv::Mat tempir=cv::Mat(mir.get_height_pixels(),mir.get_width_pixels(),CV_16UC1,mir.get_buffer()).clone();
            k4a::image aligned_color;
            cv::Mat tempaligned;
            bool aligne_succeed=true;
            if(covsucceed){
                try {
                    aligned_color=transform.color_image_to_depth_camera(mdepth,mbgr);
                    tempaligned=cv::Mat(aligned_color.get_height_pixels(),aligned_color.get_width_pixels(),CV_8UC4,aligned_color.get_buffer()).clone();
                }catch (k4a::error e){
                    std::cerr<<"彩色图对齐失败!"<<std::endl;
                    aligne_succeed=false;
                }
            } else{
                aligne_succeed=false;
            }

            {
                boost::unique_lock<boost::shared_mutex> lock(image_mutex);
                if(covsucceed){
                    cvcolor=std::make_shared<cv::Mat>(tempcolor);
                } else{
                    cvcolor.reset();
                }
                cvdepth=std::make_shared<cv::Mat>(tempdepth);
                cvir=std::make_shared<cv::Mat>(tempir);
                if(aligne_succeed){
                    cvaligned=std::make_shared<cv::Mat>(tempaligned);
                } else{
                    cvaligned.reset();
                }
                color_time=mcolor.get_device_timestamp().count();
                depth_time=mdepth.get_device_timestamp().count();
                ir_time=mir.get_device_timestamp().count();
            }
            {
                boost::unique_lock<boost::shared_mutex> lock(imagecloud_mutex);
                depth4cloud=cvdepth;
                color4cloud=cvaligned;
            }
        }catch (k4a::error e){
            failednum++;
            if(!isvideo&&failednum>=mmaxfailnum){
                uint32_t num=k4a::device::get_installed_count();///获取连接设备数
                if(num<mid+1){///没有指定id的相机
                    std::cerr<<"设备断开，尝试重新连接"<<std::endl;
                    recorder.reset();
                    isinit=false;
                    failednum=0;
                    return "disconnect";
                }
            }
           // std::cout<<"grab failed:"<<e.what()<<std::endl;
            return "failed";
        }
        if(recorder!=NULL){///是否录制视频
            try {
                recorder->write_capture(capture.handle());
            }catch (k4a::error e){
                std::cout<<"写入视频帧失败:"<<e.what()<<std::endl;
            }
        }
        failednum=0;
        return "succeed";
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 采集一次imu数据
* @param timeout:超时时间
* @return string：结果状态
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::string Kinectv3::sample(int timeout){
        if(!isinit){
            //std::cout<<"还未初始化"<<std::endl;
            init();
            sleep(1);
            return "init";
        }
        if(!start_imu){
            //std::cout<<"未开启imu"<<std::endl;
            return "stop";
        }
        k4a_imu_sample_t imu_sample;
        try {
            bool result=false;
            if(isvideo){
                result=playback.get_next_imu_sample(&imu_sample);///视频
                if(!result){
                    return "eof";
                }
            } else{
                result=device->get_imu_sample(&imu_sample,std::chrono::milliseconds(timeout));///相机
                if(!result){
                    return "timeout";
                }
            }
            Eigen::Vector3f tempacc,tempgyro;
            tempacc<<imu_sample.acc_sample.xyz.x,imu_sample.acc_sample.xyz.y,imu_sample.acc_sample.xyz.z;///加速度
            tempgyro<<imu_sample.gyro_sample.xyz.x,imu_sample.gyro_sample.xyz.y,imu_sample.gyro_sample.xyz.z;///角加速度
            {
                boost::unique_lock<boost::shared_mutex> lock(imu_mutex);
                acc=std::make_shared<Eigen::Vector3f>(tempacc);
                gyro=std::make_shared<Eigen::Vector3f>(tempgyro);
                acc_time=imu_sample.acc_timestamp_usec;///加速度采样时间
                gyro_time=imu_sample.gyro_timestamp_usec;///角加速度采样时间
            }

        }catch (k4a::error e){
           // std::cerr<<"sample failed:"<<e.what()<<std::endl;
            return "failed";
        }
        if(recorder!=NULL){///是否录制视频
            try {
                recorder->write_imu(imu_sample);
            }catch (k4a::error e){
                std::cout<<"写入imu帧失败:"<<e.what()<<std::endl;
            }
        }
        return "succeed";
    }

    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 获取彩色图
*@param destroy:获取图像后是否销毁cvcolor,建议销毁，以避免使用同一张图
* @return std::shared_ptr<cv::Mat>:图像
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::shared_ptr<cv::Mat> Kinectv3::getColorImage(bool destroy){
        if(destroy){///销毁
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=cvcolor;
            cvcolor.reset();
            return temp;
        } else{//不销毁
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return cvcolor;
        }

    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 获取与深度图对齐后的彩色图
*@param destroy:获取图像后是否销毁cvcolor,建议销毁，以避免使用同一张图
* @return std::shared_ptr<cv::Mat>:图像
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::shared_ptr<cv::Mat> Kinectv3::getAlignedColorImage(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=cvaligned;
            cvaligned.reset();
            return temp;
        }else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return cvaligned;
        }
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 获取深度图
*@param destroy:获取图像后是否销毁cvcolor,建议销毁，以避免使用同一张图
* @return std::shared_ptr<cv::Mat>:图像
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::shared_ptr<cv::Mat> Kinectv3::getDepthImage(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=cvdepth;
            cvdepth.reset();
            return temp;
        } else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return cvdepth;
        }
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 获取ir图
*@param destroy:获取图像后是否销毁cvcolor,建议销毁，以避免使用同一张图
* @return std::shared_ptr<cv::Mat>:图像
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    std::shared_ptr<cv::Mat> Kinectv3::getIrImage(bool destroy){
        if(destroy){
            boost::unique_lock<boost::shared_mutex> lock(image_mutex);
            auto temp=cvir;
            cvir.reset();
            return temp;
        }else{
            boost::shared_lock<boost::shared_mutex> lock(image_mutex);
            return cvir;
        }

    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批像素点到三维坐标系（点云）,trans为单位阵时转换到相机坐标系，
     * 如果要转换到其他坐标系（例如世界坐标系），需将trans设为相机坐标系到
     * 该坐标系的变换矩阵，注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：深度图
*@param range_min:x,y,z的最小值,不在范围内的点将被设置为(0,0,0)
*@param range_max:x,y,z的最大值,不在范围内的点将被设置为(0,0,0)
*@param trans:相机坐标系到目标坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return 点云,单位m,注意其中（0,0,0）为异常值，需要自行去除
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    Kinectv3::Pcloud Kinectv3::getPointcloud(std::vector<cv::Point3f> input,Point range_min,Point range_max,Eigen::Matrix4f trans,int numthread){
       if(!isinit||input.empty()){
           return NULL;
       }
        Kinectv3::Pcloud output(new Kinectv3::Cloud);
        output->points.resize(input.size(),Point(0,0,0));///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for(int i=0;i<input.size();++i){
            if(input[i].z<min_depth){
                continue;
            }
            Point temp;
            bool flag=get3dPoint(input[i],temp,trans);
            if(flag&&temp.x>=range_min.x&&temp.x<=range_max.x&&temp.y>=range_min.y&&temp.y<=range_max.y&&temp.z>=range_min.z&&temp.z<=range_max.z){
                {
//#if _OPENMP///线程锁，因为上锁会降低速度，所以预先给output分配的空间，避免使用push_back(),就不用上锁了
//#pragma omp critical
//#endif
                    output->points[i]=temp;
                }
            }
        }
        return output;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批像素点到三维坐标系（点云）,trans为单位阵时转换到相机坐标系，
     * 如果要转换到其他坐标系（例如世界坐标系），需将trans设为相机坐标系到
     * 该坐标系的变换矩阵，注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：深度图
*@param range_min:x,y,z的最小值,不在范围内的点将被设置为(0,0,0)
*@param range_max:x,y,z的最大值,不在范围内的点将被设置为(0,0,0)
*@param trans:相机坐标系到目标坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return 点云,单位m,注意其中（0,0,0）为异常值，需要自行去除
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    Kinectv3::Pcloud Kinectv3::getPointcloud(PointVec input,Point range_min,Point range_max,Eigen::Matrix4f trans,int numthread){
        if(!isinit||input.empty()){
            return NULL;
        }
        Kinectv3::Pcloud output(new Kinectv3::Cloud);
        output->points.resize(input.size(),Point(0,0,0));///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for(int i=0;i<input.size();++i){
            if(input[i].z<min_depth){
                continue;
            }
            Point temp;
            bool flag=get3dPoint(input[i],temp,trans);
            if(flag&&temp.x>=range_min.x&&temp.x<=range_max.x&&temp.y>=range_min.y&&temp.y<=range_max.y&&temp.z>=range_min.z&&temp.z<=range_max.z){
                {
//#if _OPENMP///线程锁，因为上锁会降低速度，所以预先给output分配的空间，避免使用push_back(),就不用上锁了
//#pragma omp critical
//#endif
                    output->points[i]=temp;
                }
            }
        }
        return output;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批像素点到三维坐标系（点云）,trans为单位阵时转换到相机坐标系，
     * 如果要转换到其他坐标系（例如世界坐标系），需将trans设为相机坐标系到
     * 该坐标系的变换矩阵，注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：深度图
*@param range_min:x,y,z的最小值,不在范围内的点将被设置为(0,0,0)
*@param range_max:x,y,z的最大值,不在范围内的点将被设置为(0,0,0)
*@param trans:相机坐标系到目标坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return 点云,单位m,注意其中（0,0,0）为异常值，需要自行去除
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    Kinectv3::Pcloud Kinectv3::getPointcloud(cv::Mat input,Point range_min,Point range_max,Eigen::Matrix4f trans,int numthread){
        if(!isinit||input.empty()){
            return NULL;
        }
        Kinectv3::Pcloud output(new Kinectv3::Cloud);
        output->points.resize(input.cols*input.rows,Point(0,0,0));///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for(int i=0;i<input.rows;++i){
            for(int j=0;j<input.cols;++j){
                float depth=input.at<short>(i,j);
                if(depth<min_depth){
                    continue;
                }
                Point temp;
                bool flag=get3dPoint(Point(j,i,depth),temp,trans);
                if(flag&&temp.x>=range_min.x&&temp.x<=range_max.x&&temp.y>=range_min.y&&temp.y<=range_max.y&&temp.z>=range_min.z&&temp.z<=range_max.z){
                    {
//#if _OPENMP///线程锁，因为上锁会降低速度，所以预先给output分配的空间，避免使用push_back(),就不用上锁了
//#pragma omp critical
//#endif
                        output->points[i*input.cols+j]=temp;
                    }
                }
            }
        }
        return output;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换深度图到三维坐标系（点云）,trans为单位阵时转换到相机坐标系，
     * 如果要转换到其他坐标系（例如世界坐标系），需将trans设为相机坐标系到
     * 该坐标系的变换矩阵，注意代码里面所有相机坐标系都是以中间的深度传感器为标准
*@param range_min:x,y,z的最小值,不在范围内的点将被设置为(0,0,0)
*@param range_max:x,y,z的最大值,不在范围内的点将被设置为(0,0,0)
*@param trans:相机坐标系到目标坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return 点云,单位m,注意其中（0,0,0）为异常值，需要自行去除
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    Kinectv3::Pcloud Kinectv3::getPointcloud(Point range_min,Point range_max,Eigen::Matrix4f trans,int numthread){
        std::shared_ptr<cv::Mat> input;
        {
            boost::shared_lock<boost::shared_mutex> lock(imagecloud_mutex);
            if(depth4cloud==NULL||depth4cloud->empty()){
                return NULL;
            }
            input=depth4cloud;
        }
        return getPointcloud(*input,range_min,range_max,trans,numthread);
    }

    Kinectv3::PColorCloud Kinectv3::getColorCloud(Eigen::Matrix4f trans,int numthread){
        std::shared_ptr<cv::Mat> inputdepth,inputcolor;
        {
            boost::shared_lock<boost::shared_mutex> lock(imagecloud_mutex);
            if(depth4cloud==NULL||color4cloud==NULL||depth4cloud->empty()||color4cloud->empty()){
                return NULL;
            }
            inputdepth=depth4cloud;
            inputcolor=color4cloud;
        }
        Kinectv3::PColorCloud output(new Kinectv3::ColorCloud);
        output->points.resize(inputdepth->cols*inputdepth->rows,ColorPoint(0,0,0));///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for(int i=0;i<inputdepth->rows;++i){
            for(int j=0;j<inputdepth->cols;++j){
                float depth=inputdepth->at<short>(i,j);
                if(depth<min_depth){
                    continue;
                }
                Point temp;
                bool flag=get3dPoint(Point(j,i,depth),temp,trans);
                if(flag){
                    ColorPoint tempcolor;
                    tempcolor.x=temp.x;
                    tempcolor.y=temp.y;
                    tempcolor.z=temp.z;
                    tempcolor.r=(int)inputcolor->at<cv::Vec3b>(i,j)[2];
                    tempcolor.g=(int)inputcolor->at<cv::Vec3b>(i,j)[1];
                    tempcolor.b=(int)inputcolor->at<cv::Vec3b>(i,j)[0];
                    output->points[i*inputdepth->cols+j]=tempcolor;
                }
            }
        }

        return output;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一个像素点到三维坐标系,trans为单位阵时转换到相机坐标系，
     * 如果要转换到其他坐标系（例如世界坐标系），需将trans设为相机坐标系到
     * 该坐标系的变换矩阵，注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input.x和input.y代表像素坐标,input.z代表深度值
*@param output:输出三维坐标,单位m
*@param trans:相机坐标系到目标坐标系的变换矩阵
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get3dPoint(cv::Point3f input,Point &output,Eigen::Matrix4f trans){
        return  get3dPoint(Point(input.x,input.y,input.z),output,trans);
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一个像素点到三维坐标系,trans为单位阵时转换到相机坐标系，
     * 如果要转换到其他坐标系（例如世界坐标系），需将trans设为相机坐标系到
     * 该坐标系的变换矩阵，注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input.x和input.y代表像素坐标,input.z代表深度值
*@param output:输出三维坐标,单位m
*@param trans:相机坐标系到目标坐标系的变换矩阵
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get3dPoint(Point input,Point &output,Eigen::Matrix4f trans){
        if(!isinit){
            std::cerr<<"还未初始化"<<std::endl;
            return false;
        }
        k4a_float2_t p_origin;
        k4a_float3_t p_final;
        p_origin.xy.x=input.x;
        p_origin.xy.y=input.y;
        try{///计算像素点对应的三维坐标
            calib.convert_2d_to_3d(p_origin,input.z,K4A_CALIBRATION_TYPE_DEPTH,K4A_CALIBRATION_TYPE_DEPTH,&p_final);
        }catch (k4a::error e){
            std::cout<<"cov 2d to 3d failed:"<<e.what()<<std::endl;
            return false;
        }
        Eigen::Vector4f  temp;
        temp<<p_final.xyz.x,p_final.xyz.y,p_final.xyz.z,1;
        temp=trans*temp;///从相机坐标系变换到目标坐标系中
        output.x=temp[0]/1000.0;///转换为m
        output.y=temp[1]/1000.0;
        output.z=temp[2]/1000.0;
        return true;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一个三维坐标到像素(深度图)坐标，trans为单位阵时，三维
     * 坐标是相机坐标系下的点，如果要将其他坐标系（例如世界坐标系）
     * 转换到像素坐标系，需将trans设为该坐标系到相机坐标系的转换矩阵，
     * 注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input三维坐标,单位m
*@param output:输出的像素坐标，注意（0,0）为无效值
*@param trans:三维点的坐标系到相机坐标系的变换矩阵
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get2dPoint(cv::Point3f input,cv::Point2f &output,Eigen::Matrix4f trans){
        if(!isinit){
            std::cerr<<"还未初始化"<<std::endl;
            return false;
        }
        k4a_float3_t p_origin;
        k4a_float2_t p_final;
        Eigen::Vector4f temp;
        temp<<input.x,input.y,input.z,1;
        temp=trans*temp;///变换到相机坐标系中
        p_origin.xyz.x=temp[0]*1000;///转换为mm好像没有必要
        p_origin.xyz.y=temp[1]*1000;
        p_origin.xyz.z=temp[2]*1000;
        try{///计算三维坐标对应的像素坐标
            calib.convert_3d_to_2d(p_origin,K4A_CALIBRATION_TYPE_DEPTH,K4A_CALIBRATION_TYPE_DEPTH,&p_final);
        }catch (k4a::error e){
            std::cout<<"cov 3d to 2d failed:"<<e.what()<<std::endl;
            return false;
        }
        output.x=p_final.xy.x;
        output.y=p_final.xy.y;
        return true;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一个三维坐标到像素(深度图)坐标，trans为单位阵时，三维
     * 坐标是相机坐标系下的点，如果要将其他坐标系（例如世界坐标系）
     * 转换到像素坐标系，需将trans设为该坐标系到相机坐标系的转换矩阵，
     * 注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input三维坐标,单位m
*@param output:输出的像素坐标，注意（0,0）为无效值
*@param trans:三维点坐标系到相机坐标系的变换矩阵
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get2dPoint(Point input,cv::Point2f &output,Eigen::Matrix4f trans){
        return get2dPoint(cv::Point3f(input.x,input.y,input.z),output, trans);
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批三维坐标到像素(深度图)坐标，trans为单位阵时，三维
 * 坐标是相机坐标系下的点，如果要将其他坐标系（例如世界坐标系）
 * 转换到像素坐标系，需将trans设为该坐标系到相机坐标系的转换矩阵，
 * 注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input三维坐标,单位m
*@param output:输出的像素坐标，注意里面的（0,0）为无效值
*@param trans:三维点坐标系到相机坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get2dPoints(PointVec input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans,int numthread){
        if(!isinit){
            std::cerr<<"还未初始化"<<std::endl;
            return false;
        }
        if(input.empty()){
            std::cerr<<"输入3d点集为空！"<<std::endl;
            return false;
        }
        bool result=false;
        output.resize(input.size(),cv::Point2f(0,0));///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for(int i=0;i< input.size();++i){
            cv::Point2f temp;
            bool flag=get2dPoint(input[i],temp,trans);
            if(flag){
                output[i]=temp;
                result=true;
            }
        }
        return result;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批三维坐标到像素(深度图)坐标，trans为单位阵时，三维
* 坐标是相机坐标系下的点，如果要将其他坐标系（例如世界坐标系）
* 转换到像素坐标系，需将trans设为该坐标系到相机坐标系的转换矩阵，
* 注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input三维坐标,单位m
*@param output:输出的像素坐标，注意里面的（0,0）为无效值
*@param trans:三维点坐标系到相机坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get2dPoints(std::vector<cv::Point3f> input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans,int numthread){
        if(!isinit||input.empty()){
            std::cerr<<"还未初始化"<<std::endl;
            return false;
        }
        if(input.empty()){
            std::cerr<<"输入3d点集为空！"<<std::endl;
            return false;
        }
        bool result=false;
        output.resize(input.size(),cv::Point2f(0,0));///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for(int i=0;i< input.size();++i){
            cv::Point2f temp;
            bool flag=get2dPoint(input[i],temp,trans);
            if(flag){
                output[i]=temp;
                result=true;
            }
        }
        return result;
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批三维坐标到像素(深度图)坐标，trans为单位阵时，三维
* 坐标是相机坐标系下的点，如果要将其他坐标系（例如世界坐标系）
* 转换到像素坐标系，需将trans设为该坐标系到相机坐标系的转换矩阵，
* 注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input三维坐标,单位m
*@param output:输出的像素坐标，注意里面的（0,0）为无效值
*@param trans:三维点坐标系到相机坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get2dPoints(Cloud input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans,int numthread){
        return get2dPoints(input.points,output,trans,numthread);
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 转换一批三维坐标到像素(深度图)坐标，trans为单位阵时，三维
* 坐标是相机坐标系下的点，如果要将其他坐标系（例如世界坐标系）
* 转换到像素坐标系，需将trans设为该坐标系到相机坐标系的转换矩阵，
* 注意代码里面所有相机坐标系都是以中间的深度传感器为标准
* @param input：input三维坐标,单位m
*@param output:输出的像素坐标，注意里面的（0,0）为无效值
*@param trans:三维点坐标系到相机坐标系的变换矩阵
*@param numthread:线程数，默认为2，可以加快运行速度，但太大了反而降低速度
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    bool Kinectv3::get2dPoints(Pcloud input,std::vector<cv::Point2f> &output,Eigen::Matrix4f trans,int numthread){
        return get2dPoints(input->points,output,trans,numthread);
    }
    /**
* Copyright (C) 2020, HITCRT_VISION, all rights reserved.
* @brief 开启视频录制
* @param filename:视频路径
* @return
* @author 黎林
* -phone:18846140245;qq:1430244438
*/
    void Kinectv3::record(std::string filename){
        if(!isinit){
            std::cerr<<"还未初始化"<<std::endl;
            init();
            sleep(1);
            return;
        }
        if(recorder!=NULL){
            try {///初始化视频文件
                recorder=std::make_shared<k4a::record>(k4a::record::create(filename.c_str(),device->handle(),config,start_imu));
                std::cout<<"录制视频:"<<filename<<std::endl;
            }catch (k4a::error e){
                std::cout<<"初始化recorder失败:"<<e.what()<<std::endl;
            }
        }
    }
}

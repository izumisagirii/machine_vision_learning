#include "rgbd_camera.h"

namespace hitcrt
{
    /**
 * Copyright(C) 2018,HITCRT_VISION,all rights reserved
 * @brief 构造函数
 * @param rgbd:设备类型Kinect,Xtion1,Xtion2,ONI,Any分别代表kinect2,xtion1,xtion2,和ONI视频,Any会打开任意相机
 * @param target:设备类型为ONI文件时使用,代表文件路径,
 * @param oni_dev:打开ONI文件时使用这个参数表示录制视频的设备，从而选择合适相机参数由于计算点云
 * @param isregi:kinect有效，false代表不对齐彩色图和深度图，此时彩色图大小为1080*1920，深度图为424*512，否则都为424*512
 * 注意不对齐时录制的ONI文件可能打不开
 * @param grab_ir:是否开启ir流，kinect2或者kinect2的oni文件可以开启，xtion1和其oni文件开启会出问题
 * @return
 * @author
 * -黎林(修改) phone:18846140245 QQ:1430244438
 */
    RGBDcamera::RGBDcamera(Device rgbd, const char *target, Device oni_dev, bool isregi, bool _grab_ir)
    {
        m_device = rgbd;
        m_targetFile = target;
        grab_ir = _grab_ir;
        openni::OpenNI::initialize();
        rc = openni::STATUS_OK;
        if (rgbd == RGBDcamera::ONI)
        {
            if (oni_dev == RGBDcamera::Kinect)
            {
                camera_cx = 256.162;
                camera_cy = 213.28;
                camera_fx = 364.485;
                camera_fy = 364.485;
            }
            else if (oni_dev == RGBDcamera::Xtion1)
            {
                camera_cx = 160.5912;
                camera_cy = 120.4792;
                camera_fx = 253.0589;
                camera_fy = 254.1649;
            }
            else if (oni_dev == RGBDcamera::Xtion2)
            {
                camera_cx = 160.5912;
                camera_cy = 120.4792;
                camera_fx = 253.0589;
                camera_fy = 254.1649;
            }
            else if (oni_dev == RGBDcamera::ASTRA)
            {
                camera_cx = 164.2771;
                camera_cy = 124.3905;
                camera_fx = 289.2989;
                camera_fy = 290.3885;
            }
            rc = camera.open(m_targetFile);
            if (rc != openni::STATUS_OK)
            {
                std::cerr << "无法打开ONI文件" << std::endl;
                closecamera();
                exit(-1);
            }
            else
            {
                std::cout << "成功打开ONI文件：" << target << std::endl;
            }
            playback = camera.getPlaybackControl();
            playback->setSpeed(1.0);
            playback->setRepeatEnabled(true);
            vendor = "ONIFile";
        }
        else
        {
            openni::OpenNI::enumerateDevices(&aDeviceList);
            if (aDeviceList.getSize() < 1)
            {
                std::cout << "没有设备连接！" << std::endl;
                closecamera();
                exit(0);
            }
            showdevice();
            std::string uri = "";
            if (rgbd == RGBDcamera::Kinect)
            {
                vendor = "Microsoft";
            }
            else if (rgbd == RGBDcamera::Xtion1)
            {
                vendor = "PrimeSense";
            }
            else if (rgbd == RGBDcamera::Xtion2)
            {
                vendor = "ASUS";
            }
            else if (rgbd == RGBDcamera::ASTRA)
            {
                vendor = "Orbbec";
            }
            if (rgbd != RGBDcamera::Any)
            {
                for (int i = 0; i < aDeviceList.getSize(); ++i)
                {
                    const openni::DeviceInfo &rDevInfo = aDeviceList[i];
                    //std::cout << "Vendor: " << rDevInfo.getVendor() << " " << vendor << std::endl;
                    if (vendor == rDevInfo.getVendor())
                    {
                        uri = rDevInfo.getUri();
                        break;
                    }
                }
                if (uri.empty())
                {
                    if (vendor == "Microsoft")
                    {
                        std::cout << "没有kinect连接！" << std::endl;
                    }
                    else if (vendor == "PrimeSense")
                    {
                        std::cout << "没有Xtion1连接！" << std::endl;
                    }
                    else if (vendor == "ASUS")
                    {
                        std::cout << "没有Xtion2连接！" << std::endl;
                    }
                    else if (vendor == "Orbbec")
                    {
                        std::cout << "没有Astra连接！" << std::endl;
                    }
                    closecamera();
                    exit(0);
                }
                rc = camera.open(uri.c_str());
            }
            else
            {
                rc = camera.open(openni::ANY_DEVICE);
                if (rc != openni::STATUS_OK)
                {
                    for (int i = 0; i < aDeviceList.getSize(); ++i)
                    {
                        const openni::DeviceInfo &rDevInfo = aDeviceList[i];
                        uri = rDevInfo.getUri();
                        rc = camera.open(uri.c_str());
                        if (rc == openni::STATUS_OK)
                        {
                            break;
                        }
                    }
                }
                if (rc != openni::STATUS_OK)
                {
                    std::cerr << "can not open any device!" << std::endl;
                    closecamera();
                    exit(-1);
                }
                vendor = camera.getDeviceInfo().getVendor();
            }
            if(vendor == "Microsoft"){
                camera_cx = 256.162;
                camera_cy = 213.28;
                camera_fx = 364.485;
                camera_fy = 364.485;
            }else if(vendor == "PrimeSense"){
                camera_cx = 160.5912;
                camera_cy = 120.4792;
                camera_fx = 253.0589;
                camera_fy = 254.1649;
            }else if(vendor == "ASUS"){
                camera_cx = 160.5912;
                camera_cy = 120.4792;
                camera_fx = 253.0589;
                camera_fy = 254.1649;
            }else{
                camera_cx = 164.2771;
                camera_cy = 124.3905;
                camera_fx = 289.2989;
                camera_fy = 290.3885;
            }
            if (rc != openni::STATUS_OK)
            {
                std::cerr << "无法打开设备" << std::endl;
                closecamera();
                exit(-1);
            }
            if (vendor == "Microsoft")
            {
                resolution_width = KINECT_WIDTH;
                resolution_height = KINECT_HEIGHT;
            }
            else if (vendor == "PrimeSense")
            {
                resolution_width = XTION_WIDTH;
                resolution_height = XTION_HEIGHT;
            }
            else if (vendor == "ASUS")
            {
                resolution_width = XTION_WIDTH;
                resolution_height = XTION_HEIGHT;
            }
            else if (vendor == "Orbbec")
            {
                resolution_width = ASTRA_WIDTH;
                resolution_height = ASTRA_HEIGHT;
            }
        }

        if (camera.hasSensor(openni::SENSOR_DEPTH))
        {
            rc = m_streamDepth.create(camera, openni::SENSOR_DEPTH);
            if (vendor == "Microsoft")
            {
                m_streamDepth.setMirroringEnabled(true);
            }
            else
            {
                m_streamDepth.setMirroringEnabled(false);
            }

            if (rc == openni::STATUS_OK)
            {
                if (vendor != "ONIFile")
                {
                    openni::VideoMode mModeDepth;
                    mModeDepth.setResolution(resolution_width, resolution_height);
                    mModeDepth.setFps(30);
                    mModeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
                    m_streamDepth.setVideoMode(mModeDepth);
                }
                rc = m_streamDepth.start();
                if (rc != openni::STATUS_OK)
                {
                    std::cerr << "无法打开深度数据流：" << openni::OpenNI::getExtendedError() << std::endl;
                    closecamera();
                    exit(-1);
                }
            }
            else
            {
                std::cerr << "无法创建深度数据流：" << openni::OpenNI::getExtendedError() << std::endl;
                closecamera();
                exit(-1);
            }
            if (!m_streamDepth.isValid())
            {
                std::cerr << "深度数据流不合法" << std::endl;
                closecamera();
                exit(-1);
            }
            std::cout << "成功开启深度流" << std::endl;
        }

        if (vendor != "Orbbec")
        {
            if (camera.hasSensor(openni::SENSOR_COLOR))
            {
                rc = m_streamColor.create(camera, openni::SENSOR_COLOR);
                if (vendor == "Microsoft")
                {
                    m_streamColor.setMirroringEnabled(true);
                }
                else
                {
                    m_streamColor.setMirroringEnabled(false);
                }
                if (rc == openni::STATUS_OK)
                {
                    if (vendor != "ONIFile")
                    {
                        openni::VideoMode mModeColor;
                        mModeColor.setFps(30);
                        if (vendor == "Microsoft" && !isregi)
                        {
                            mModeColor.setResolution(1920, 1080);
                        }
                        mModeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
                        m_streamColor.setVideoMode(mModeColor);
                    }
                    rc = m_streamColor.start();
                    if (rc != openni::STATUS_OK)
                    {
                        std::cerr << "无法打开彩色图像数据流：" << openni::OpenNI::getExtendedError() << std::endl;
                        closecamera();
                        exit(-1);
                    }
                }
                else
                {
                    std::cerr << "无法创建彩色图像数据流：" << openni::OpenNI::getExtendedError() << std::endl;
                }
                if (!m_streamColor.isValid())
                {
                    std::cerr << "彩色数据流不合法" << std::endl;
                    closecamera();
                    exit(-1);
                }
                std::cout << "成功开启彩色流" << std::endl;
            }
        }
        else
        {
            std::string filename;
            findAstrRGBDevice("Astra Pro HD Camera: Astra Pro ", filename);
            astra_rgb.open(filename);
            if (!astra_rgb.isOpened())
            {
                std::cerr << "Astra color camera open failed!" << std::endl;
                closecamera();
                exit(-1);
            }
            astra_rgb.set(cv::CAP_PROP_FRAME_WIDTH, resolution_width);
            astra_rgb.set(cv::CAP_PROP_FRAME_HEIGHT, resolution_height);
            m_astragrabThread = boost::thread(boost::bind(&RGBDcamera::m_astragrab, this));
        }

        if ((vendor == "Microsoft" || vendor == "ONIFile") && grab_ir)
        {
            rc = m_streamIR.create(camera, openni::SENSOR_IR);
            m_streamIR.setMirroringEnabled(true);
            if (rc == openni::STATUS_OK)
            {
                if (vendor != "ONIFile")
                {
                    openni::VideoMode mModeIR;
                    mModeIR.setFps(30);
                    mModeIR.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
                    mModeIR.setResolution(resolution_width, resolution_height);
                    m_streamIR.setVideoMode(mModeIR);
                }
                rc = m_streamIR.start();
                if (rc != openni::STATUS_OK)
                {
                    std::cout << "无法打开ir数据流" << std::endl;
                    m_streamIR.destroy();
                }
            }
            else
            {
                std::cout << "无法创建ir数据流" << std::endl;
                m_streamIR.destroy();
            }
            if (!m_streamIR.isValid())
            {
                std::cerr << "IR数据流不合法" << std::endl;
                //closecamera();
                //exit(-1);
            }
            else
            {
                std::cout << "成功开启IR流" << std::endl;
            }
        }

        if (vendor == "ONIFile")
        {
            if (getFrameNum() < 1)
            {
                std::cerr << "ONI文件 " << target << " 为空或损坏" << std::endl;
                exit(1);
            }
        }

        //彩色图与深度图对齐
        if (isregi)
        {
            if (camera.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
            {
                camera.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            }
        }
    }

    void RGBDcamera::m_astragrab()
    {
        while (isok)
        {
            cv::Mat img;
            astra_rgb >> img;
            if (!img.empty())
            {
                boost::unique_lock<boost::shared_mutex> writelock(astra_mutex);
                m_AstraRGB = std::make_shared<cv::Mat>(img);
            }
        }
    }
    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 显示连接的所有可用设备
   *@param
   *@return
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    void RGBDcamera::showdevice()
    {
        std::cout << "电脑上连接着 " << aDeviceList.getSize() << " 个体感设备." << std::endl;
        for (int i = 0; i < aDeviceList.getSize(); ++i)
        {
            std::cout << "设备 " << i << std::endl;
            const openni::DeviceInfo &rDevInfo = aDeviceList[i];
            std::cout << "设备名： " << rDevInfo.getName() << std::endl;
            std::cout << "设备Id： " << rDevInfo.getUsbProductId() << std::endl;
            std::cout << "供应商名： " << rDevInfo.getVendor() << std::endl;
            std::cout << "供应商Id: " << rDevInfo.getUsbVendorId() << std::endl;
            std::cout << "设备URI: " << rDevInfo.getUri() << std::endl;
        }
    }
    void RGBDcamera::grab()
    {
        cv::Mat depth, color, ir;
        std::shared_ptr<cv::Mat> ctemp, dtemp;
        if (camera.hasSensor(openni::SENSOR_DEPTH) && m_streamDepth.isValid())
        {
            rc = m_streamDepth.readFrame(&frameDepth);
            if (rc == openni::STATUS_OK)
            {
                depth = cv::Mat(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void *)frameDepth.getData()).clone();
                {
                    boost::unique_lock<boost::shared_mutex> lock(depth_mutex);
                    m_ImageDepth = std::make_shared<cv::Mat>(depth);
                    dtemp = m_ImageDepth;
                }
            }
        }
        if (grab_ir && camera.hasSensor(openni::SENSOR_IR) && m_streamIR.isValid())
        {
            rc = m_streamIR.readFrame(&frameIR);
            if (rc == openni::STATUS_OK)
            {
                ir = cv::Mat(frameIR.getHeight(), frameIR.getWidth(), CV_16UC1, (void *)frameIR.getData()).clone();
                {
                    boost::unique_lock<boost::shared_mutex> lock(ir_mutex);
                    m_ImageIR = std::make_shared<cv::Mat>(ir);
                }
            }
        }
        if (vendor == "Orbbec")
        {
            {
                boost::shared_lock<boost::shared_mutex> writelock(astra_mutex);
                ctemp = m_AstraRGB;
            }
            {
                boost::unique_lock<boost::shared_mutex> lock(color_mutex);
                m_ImageRGB = ctemp;
            }
        }
        else
        {
            if (camera.hasSensor(openni::SENSOR_COLOR) && m_streamColor.isValid())
            {
                rc = m_streamColor.readFrame(&frameColor);
                if (rc == openni::STATUS_OK)
                {
                    color = cv::Mat(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void *)frameColor.getData()).clone();
                    // 首先将RGB格式转换为BGR格式
                    // std::cout<<m_ImageRGB.size()<<" "<<frameColor.getHeight()<<std::endl;
                    cvtColor(color, color, cv::COLOR_BGR2RGB);
                    {
                        boost::unique_lock<boost::shared_mutex> lock(color_mutex);
                        m_ImageRGB = std::make_shared<cv::Mat>(color);
                        ctemp = m_ImageRGB;
                    }
                }
            }
        }

        {
            boost::unique_lock<boost::shared_mutex> lock(cloud_mutex);
            m_ImageDepth4cloud = dtemp;
            m_ImageRGB4cloud = ctemp;
        }
    }

    std::shared_ptr<cv::Mat> RGBDcamera::getImageDepth(bool destroy)
    {
        if (destroy)
        {
            boost::unique_lock<boost::shared_mutex> lock(depth_mutex);
            auto temp = m_ImageDepth;
            m_ImageDepth.reset();
            return temp;
        }
        else
        {
            boost::shared_lock<boost::shared_mutex> lock(depth_mutex);
            return m_ImageDepth;
        }
    }
    std::shared_ptr<cv::Mat> RGBDcamera::getImageRGB(bool destroy)
    {
        if (destroy)
        {
            boost::unique_lock<boost::shared_mutex> lock(color_mutex);
            auto temp = m_ImageRGB;
            m_ImageRGB.reset();
            return temp;
        }
        else
        {
            boost::shared_lock<boost::shared_mutex> lock(color_mutex);
            return m_ImageRGB;
        }
    }
    std::shared_ptr<cv::Mat> RGBDcamera::getImageIR(bool destroy)
    {
        if (destroy)
        {
            boost::unique_lock<boost::shared_mutex> lock(ir_mutex);
            auto temp = m_ImageIR;
            m_ImageIR.reset();
            return temp;
        }
        else
        {
            boost::shared_lock<boost::shared_mutex> lock(ir_mutex);
            return m_ImageIR;
        }
    }

    RGBDcamera::Pcloud RGBDcamera::getPointcloud(Eigen::Matrix4f trans, bool destroy, int numthread)
    {
        RGBDcamera::Pcloud cloud(new RGBDcamera::Cloud);
        std::shared_ptr<cv::Mat> temp;
        if (destroy)
        {
            boost::unique_lock<boost::shared_mutex> lock(cloud_mutex);
            temp = m_ImageDepth4cloud;
            m_ImageDepth4cloud.reset();
        }
        else
        {
            boost::shared_lock<boost::shared_mutex> lock(cloud_mutex);
            temp = m_ImageDepth4cloud;
        }
        if (temp == NULL || temp->empty())
        {
            return NULL;
        }
        cloud->points.resize(temp->rows * temp->cols, RGBDcamera::Point(0, 0, 0));
#if _OPENMP ///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for (int i = 0; i < temp->rows; ++i)
        {
            for (int j = 0; j < temp->cols; ++j)
            {
                float depthv = 1.0 * temp->at<short>(i, j);
                if (depthv < 1)
                    continue;
                pcl::PointXYZ in(j, i, depthv), out;
                get3dPoint(in, out, trans);
                cloud->points[i * temp->cols + j] = out;
            }
        }
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = false;
        return cloud;
    }

    RGBDcamera::Pccloud RGBDcamera::getCPointcloud(Eigen::Matrix4f trans, bool destroy, int numthread)
    {
        RGBDcamera::Pccloud cloud(new RGBDcamera::Ccloud);
        std::shared_ptr<cv::Mat> temp, tempcolor;
        if (destroy)
        {
            boost::unique_lock<boost::shared_mutex> lock(cloud_mutex);
            temp = m_ImageDepth4cloud;
            tempcolor = m_ImageRGB4cloud;
            m_ImageDepth4cloud.reset();
            m_ImageRGB4cloud.reset();
        }
        else
        {
            boost::shared_lock<boost::shared_mutex> lock(cloud_mutex);
            temp = m_ImageDepth4cloud;
            tempcolor = m_ImageRGB4cloud;
        }
        if (temp == NULL || tempcolor == NULL || temp->empty() || tempcolor->empty())
        {
            return NULL;
        }
        cloud->points.resize(temp->rows * temp->cols, RGBDcamera::Cpoint(0, 0, 0));
#if _OPENMP ///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for (int i = 0; i < temp->rows; ++i)
        {
            for (int j = 0; j < temp->cols; ++j)
            {
                float depthv = 1.0 * temp->at<short>(i, j);
                if (depthv < 1)
                    continue;
                RGBDcamera::Cpoint p;
                RGBDcamera::Point in(j, i, depthv), out;
                get3dPoint(in, out, trans);
                p.x = out.x;
                p.y = out.y;
                p.z = out.z;
                p.r = tempcolor->at<cv::Vec3b>(i, j)[2];
                p.g = tempcolor->at<cv::Vec3b>(i, j)[1];
                p.b = tempcolor->at<cv::Vec3b>(i, j)[0];
                cloud->points[i * temp->cols + j] = p;
            }
        }
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = false;
        return cloud;
    }

    RGBDcamera::Pcloud RGBDcamera::getPointcloud(PointVec input, Eigen::Matrix4f trans, int numthread)
    {
        if (input.empty())
        {
            return NULL;
        }
        RGBDcamera::Pcloud output(new RGBDcamera::Cloud);
        output->points.resize(input.size(), Point(0, 0, 0)); ///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP                                                  ///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for (int i = 0; i < input.size(); ++i)
        {
            if (input[i].z < 1)
            {
                continue;
            }
            Point temp;
            get3dPoint(input[i], temp, trans);
            output->points[i] = temp;
        }
        return output;
    }
    RGBDcamera::Pcloud RGBDcamera::getPointcloud(std::vector<cv::Point3f> input, Eigen::Matrix4f trans, int numthread)
    {
        if (input.empty())
        {
            return NULL;
        }
        RGBDcamera::Pcloud output(new RGBDcamera::Cloud);
        output->points.resize(input.size(), Point(0, 0, 0)); ///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP                                                  ///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for (int i = 0; i < input.size(); ++i)
        {
            if (input[i].z < 1)
            {
                continue;
            }
            Point temp;
            bool flag = get3dPoint(input[i], temp, trans);
            output->points[i] = temp;
        }
        return output;
    }

    bool RGBDcamera::get3dPoint(RGBDcamera::Point input, RGBDcamera::Point &output, Eigen::Matrix4f trans)
    {
        Eigen::Vector4f temp;
        temp << (input.x - camera_cx) * input.z / camera_fx, (input.y - camera_cy) * input.z / camera_fy, input.z, 1.0;
        temp = trans * temp;         ///从相机坐标系变换到目标坐标系中
        output.x = temp[0] / 1000.0; ///转换为m
        output.y = temp[1] / 1000.0;
        output.z = temp[2] / 1000.0;
        return true;
    }
    bool RGBDcamera::get3dPoint(cv::Point3f input, RGBDcamera::Point &output, Eigen::Matrix4f trans)
    {
        return get3dPoint(Point(input.x, input.y, input.z), output, trans);
    }

    bool RGBDcamera::get2dPoint(Point input, cv::Point2f &output, Eigen::Matrix4f trans)
    {
        Eigen::Vector4f temp;
        temp << input.x, input.y, input.z, 1;
        temp = trans * temp;
        if (fabs(temp[0]) < 1e-6)
        {
            return false;
        }
        output.x = temp[0] * camera_fx / temp[2] + camera_cx;
        output.y = temp[1] * camera_fy / temp[2] + camera_cy;
    }
    bool RGBDcamera::get2dPoint(cv::Point3f input, cv::Point2f &output, Eigen::Matrix4f trans)
    {
        return get2dPoint(Point(input.x, input.y, input.z), output, trans);
    }

    bool RGBDcamera::get2dPoints(PointVec input, std::vector<cv::Point2f> &output, Eigen::Matrix4f trans, int numthread)
    {
        if (input.empty())
        {
            std::cerr << "输入3d点集为空！" << std::endl;
            return false;
        }
        bool result = false;
        output.clear();
        output.resize(input.size(), cv::Point2f(0, 0)); ///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP                                             ///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for (int i = 0; i < input.size(); ++i)
        {
            cv::Point2f temp;
            bool flag = get2dPoint(input[i], temp, trans);
            if (flag)
            {
                output[i] = temp;
                result = true;
            }
        }
        return result;
    }

    bool RGBDcamera::get2dPoints(std::vector<cv::Point3f> input, std::vector<cv::Point2f> &output, Eigen::Matrix4f trans, int numthread)
    {
        if (input.empty())
        {
            std::cerr << "输入3d点集为空！" << std::endl;
            return false;
        }
        bool result = false;
        output.clear();
        output.resize(input.size(), cv::Point2f(0, 0)); ///预先分配内存避免使用push_back(),不然需要上锁
#if _OPENMP                                             ///openmp加速
#pragma omp parallel for num_threads(numthread)
#endif
        for (int i = 0; i < input.size(); ++i)
        {
            cv::Point2f temp;
            bool flag = get2dPoint(input[i], temp, trans);
            if (flag)
            {
                output[i] = temp;
                result = true;
            }
        }
        return result;
    }

    bool RGBDcamera::get2dPoints(Cloud input, std::vector<cv::Point2f> &output, Eigen::Matrix4f trans, int numthread)
    {
        return get2dPoints(input.points, output, trans, numthread);
    }

    bool RGBDcamera::get2dPoints(Pcloud input, std::vector<cv::Point2f> &output, Eigen::Matrix4f trans, int numthread)
    {
        return get2dPoints(input->points, output, trans, numthread);
    }
    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 获取相机状态
   *@param
   *@return 状态
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    openni::Status RGBDcamera::getStatus()
    {
        return rc;
    }

    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 析构函数
   *@param
   *@return
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    RGBDcamera::~RGBDcamera()
    {
        isok = false;
        closecamera();
    }

    void RGBDcamera::closecamera()
    {
        stopRecord();
        m_streamDepth.destroy();
        m_streamColor.destroy();
        m_streamIR.destroy();
        camera.close();
        openni::OpenNI::shutdown();
    }

    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 获取视频帧数
   *@param
   *@return 打开文件时代表帧数，打开相机时返回-1
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    int RGBDcamera::getFrameNum()
    {
        if (m_device == ONI)
            return playback->getNumberOfFrames(m_streamDepth);
        else
            return -1;
    }

    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 开启录视频功能
   *@param
   *@return
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    bool RGBDcamera::record(const char *filename)
    {
        if (m_targetFile != NULL)
        {
            if (strcmp(filename, m_targetFile) == 0)
            {
                std::cerr << "录制文件与打开文件不能相同" << std::endl;
                return false;
            }
        }

        if (mRecorder.create(filename) != openni::STATUS_OK)
        {
            std::cerr << "Can't create Recorder: "
                      << openni::OpenNI::getExtendedError() << std::endl;
            return false;
        }
        int faild_num = 0;
        if (camera.hasSensor(openni::SENSOR_COLOR))
        {
            if (mRecorder.attach(m_streamColor) != openni::STATUS_OK)
            {
                std::cerr << "Can't Recorde color: "
                          << openni::OpenNI::getExtendedError() << std::endl;
                //return false;
                faild_num++;
            }
        }
        if (camera.hasSensor(openni::SENSOR_DEPTH))
        {
            if (mRecorder.attach(m_streamDepth) != openni::STATUS_OK)
            {
                std::cerr << "Can't  Recorde depth: "
                          << openni::OpenNI::getExtendedError() << std::endl;
                //return false;
                faild_num++;
            }
        }
        if (camera.hasSensor(openni::SENSOR_IR))
        {
            if (mRecorder.attach(m_streamIR) != openni::STATUS_OK)
            {
                std::cerr << "Can't Recorde ir: "
                          << openni::OpenNI::getExtendedError() << std::endl;
                //return false;
                faild_num++;
            }
        }
        if (faild_num >= 3)
        {
            std::cerr << "can't record any stream" << std::endl;
            return false;
        }
        if (mRecorder.isValid())
        {
            if (mRecorder.start() == openni::STATUS_OK)
            {
                std::cout << " START RECORDING...... " << std::endl;
            }
            else
            {
                std::cout << "OH! NO!....." << std::endl;
                return false;
            }
        }
        else
        {
            std::cout << "recorder is invalid!" << std::endl;
            return false;
        }
        return true;
    }

    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 停止录视频
   *@param
   *@return
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    void RGBDcamera::stopRecord()
    {
        mRecorder.stop();
        mRecorder.destroy();
        //std::cout<<"STOPED RECORD"<<std::endl;
    }
    /**
   * Copyright(C) 2018,HITCRT_VISION,all rights reserved
   * @brief 获取相机类型，Kinect,Xtion1,Xtion2,ONI,Any分别代表kinect2,xtion1,xtion2,和ONI视频,Any代表未知情况
   *@param
   *@return 相机类型
   * @author
   * -黎林(修改) phone:18846140245 QQ:1430244438
   */
    RGBDcamera::Device RGBDcamera::getDeviceType()
    {
        if (vendor == "Microsoft")
        {
            std::cout << "open kinect" << std::endl;
            return RGBDcamera::Device::Kinect;
        }
        else if (vendor == "PrimeSense")
        {
            std::cout << "open Xtion1" << std::endl;
            return RGBDcamera::Device::Xtion1;
        }
        else if (vendor == "ASUS")
        {
            std::cout << "open Xtion2" << std::endl;
            return RGBDcamera::Device::Xtion2;
        }
        else if (vendor == "ONIFile")
        {
            std::cout << "open oni file" << std::endl;
            return RGBDcamera::Device::ONI;
        }
        else if (vendor == "Orbbec")
        {
            std::cout << "open Astra" << std::endl;
            return RGBDcamera::ASTRA;
        }
        else
        {
            std::cout << "open any" << std::endl;
            return RGBDcamera::Device::Any;
        }
    }

    bool RGBDcamera::enumuvcDevices(std::vector<std::string> &files)
    {
        DIR *pDir;
        struct dirent *ptr;
        if (!(pDir = opendir("/dev/")))
        {
            return false;
        }
        const std::string path0 = "/dev/";
        std::string subFile;
        while ((ptr = readdir(pDir)) != 0)
        {
            subFile = ptr->d_name;
            if (subFile.substr(0, subFile.size() - 1) == "video")
            {
                files.emplace_back(path0 + subFile);
            }
        }
        if (!files.empty())
        {
            std::cout << "find camera:" << std::endl;
        }
        else
        {
            std::cout << "no camera find!" << std::endl;
            return false;
        }
        for (auto device : files)
        {
            std::cout << device << std::endl;
        }
        return true;
    }

    bool RGBDcamera::findAstrRGBDevice(char *vender, std::string &name)
    {
        std::vector<std::string> videos;
        enumuvcDevices(videos);
        for (auto videofile : videos)
        {
            int ret, i, j;
            int fd;

            /* 打开摄像头设备 */
            fd = open(videofile.c_str(), O_RDWR); // 注意查看摄像头设备名
            if (-1 == fd)
            {
                perror(videofile.c_str());
                return -1;
            }

            /* 查询打开的设备是否属于摄像头：设备video不一定是摄像头*/
            struct v4l2_capability cap;
            ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
            if (-1 == ret)
            {
                perror("ioctl VIDIOC_QUERYCAP");
                close(fd);
            }
            if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
            {
                /* 如果为摄像头设备则打印摄像头驱动名字 */
                printf("Driver Name: %s\n", cap.driver);
                printf("card Name: %s\n", cap.card);
                if (strcmp(reinterpret_cast<const char *>(cap.card), vender) == 0)
                {
                    name = videofile;
                    return true;
                }
            }
            else
            {
                printf("open file is not video\n");
                close(fd);
                return -2;
            }
            close(fd);
        }
    }

} // namespace hitcrt

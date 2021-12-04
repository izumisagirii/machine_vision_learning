//
// Created by l on 2019/10/15.
//
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include "ZEDcamera.hpp"
/**
* Copyright(C) 2018,HITCRT_VISION,all rights reserved
* @brief 例程
*@param
*@return
* @author
* -黎林 phone:18846140245 QQ:1430244438
*/
int main(){//
    hitcrt::ZEDcamera camera("",hitcrt::ZEDcamera::R_720);
    //hitcrt::ZEDcamera camera("../data/test_detect1.svo",hitcrt::ZEDcamera::R_720);///打开视频文件
    struct timeval time_t;
    gettimeofday(&time_t,NULL);
    double time_start=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    int count=0;
    bool issave=false;
    cv::Mat T_2_l,intrinxl,distl,intrinxr,distr;
    camera.getCalibParams(T_2_l,intrinxl,distl,intrinxr,distr);
    std::cout<<"T_2_l:"<<std::endl<<T_2_l<<std::endl;
    std::cout<<"intrinxl:"<<std::endl<<intrinxl<<std::endl;
    std::cout<<"distol:"<<std::endl<<distl<<std::endl;
    std::cout<<"intrinxr:"<<std::endl<<intrinxr<<std::endl;
    std::cout<<"distor:"<<std::endl<<distr<<std::endl;
    cv::namedWindow("left",cv::WINDOW_NORMAL);
    cv::namedWindow("right",cv::WINDOW_NORMAL);
    cv::namedWindow("depth",cv::WINDOW_NORMAL);
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    camera.enableRecord("../data/test_detect1.svo",sl::SVO_COMPRESSION_MODE::LOSSLESS);///开启录视频
    //cyh
    //默认设置成了H264，但是由于显卡太拉我貌似我只有lossless能跑。录视频可以注释掉
    while (!viewer.wasStopped()){
        camera.grab();
            //camera.record();///录一帧视频
        //}
        auto color_l=camera.getLImage();
        auto color_r=camera.getRImage();
        auto depth=camera.getDImage();
        auto cloud=camera.getPointCloud();
    //    auto grab_time=camera.getGrabTimestamp();

        camera.grabPose();
        auto cam_t=camera.getTrans();
        auto cam_R=camera.getRotate();
    //    auto pose_time=camera.getPoseTimestamp();

        ///ZED没有IMU,ZEDmini有IMU
    //    camera.grabIMU();
    //    auto cam_o=camera.getOrien();
   //     auto cam_a=camera.getAcc();
   //     auto imu_time=camera.getIMUTimestamp();
        if(color_l==NULL||color_l->empty()){
            std::cout<<"aaaaa"<<std::endl;
        }
        if(color_l!=NULL&&!color_l->empty()){
            cv::imshow("left",*color_l);
        }
        if(color_r!=NULL&&!color_r->empty()){
            cv::imshow("right",*color_r);
        }
        if(depth!=NULL&&!depth->empty()){
            count++;
            gettimeofday(&time_t,NULL);
            double time_end=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
            //std::cout<<"t:"<<cam_t<<std::endl<<"R:"<<cam_R<<std::endl<<"time:"<<pose_time<<std::endl;
            std::cout<<"fps:"<<1000*count/(time_end-time_start)<<std::endl;

            cv::Mat depth_show;
            depth->convertTo(depth_show,CV_8UC1);
            cv::imshow("depth",depth_show);
        }
        if(cloud!=NULL&&!cloud->empty()){
            viewer.showCloud (cloud);
        }
        char key=cv::waitKey(1);
        if(key==27||key=='q'){
            break;
        } else if(key=='s'){
            issave= true;
        } else if(key=='e'){
            issave=false;
        }
        camera.stopRecord();
    }

}

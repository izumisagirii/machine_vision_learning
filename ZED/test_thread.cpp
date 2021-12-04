#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include "ZEDcamera.hpp"

hitcrt::ZEDcamera camera("",hitcrt::ZEDcamera::R_VGA);
bool isok=true;
void grab(){
    while(isok){
        camera.grab();
        camera.grabPose();
        // camera.grabIMU();
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));//抓图不要太频繁会失败降低帧率
    }
    isok=false;
    std::cout<<"grab thread finish!"<<std::endl;
}
void process(){
    struct timeval time_t;
    gettimeofday(&time_t,NULL);
    double time_start=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    int count=0;
     pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    while(isok&&!viewer.wasStopped()){
        auto color_l=camera.getLImage();
        auto color_r=camera.getRImage();
        auto depth=camera.getDImage();
        auto cloud=camera.getPointCloud();
        auto grab_time=camera.getGrabTimestamp();
        auto cam_t=camera.getTrans();
        auto cam_R=camera.getRotate();
        auto pose_time=camera.getPoseTimestamp();
        auto cam_o=camera.getOrien();
        auto cam_a=camera.getAcc();
        auto imu_time=camera.getIMUTimestamp();
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
        }
        // boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    isok=false;
    std::cout<<"process thread finish!"<<std::endl;
}
int main(){
    boost::thread grab_thread(grab);
    boost::thread process_thread(process);
    grab_thread.join();
    process_thread.join();
    return 0;
}
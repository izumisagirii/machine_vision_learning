// #include <chrono>
// chrono是目前用于计时的一个库，测试中暂时未用到
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "box.h"

using namespace hitcrt;
/**
 * Author:姚益祁
 * Modified date:2021/1/253
 * Phone:18676436991
 * QQ:1210364094
 * Note: 如果有任何不懂的代码，可以直接通过注释的形式push上去，我会及时看到并修改
 **/
int main()
{
    // center c;
    center c(460800, true, false, false);
    //相对路径，记得改文件名
    std::string names_file   = "../yolo_bounding_box/data/arrow.names";
    std::string cfg_file     = "../yolo_bounding_box/data/arrow.cfg";
    std::string weights_file = "../yolo_bounding_box/data/arrow.weights";
    std::string video_file   = "/home/hitcrt/Downloads/4.mp4";
    std::string yaml_file = "../yolo_bounding_box/yaml/config.yaml";
    // cv::VideoCapture video(video_file);
    // cv::connectedComponentsWithStats()
    // 使用bbox_to_points的例子，容器数组的大小根据YOLO学习中names的多少来判断
    // box bounding_box(video_file, yaml_file, names_file, cfg_file, weights_file);
    box bounding_box(c, yaml_file, names_file, cfg_file, weights_file);
    bounding_box.setup(SETUP_FLAG::RELEASE_DETECTOR);
    // sleep(25);
    bounding_box.shutdown();
    return 0;
}
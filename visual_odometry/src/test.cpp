#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "Kinectv3.h"
#include "odometry.h"
hitcrt::Kinectv3 cam(0, false, "");
int main()
{
    std::shared_ptr<cv::Mat> depth, aligned_color, color;
    // Eigen::Matrix3d K, R;
    // Eigen::Vector3d t;
    // K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
    // R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    // t << 0, 0, 0;
    // cv::Mat K = (cv::Mat_<double>(3,3)<<520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    // odometry test(K);
    // cv::Mat p1 = cv::imread("/home/ethan/Documents/slam/slambook2/ch7/1.png");
    // cv::Mat d1 = cv::imread("/home/ethan/Documents/slam/slambook2/ch7/1_depth.png", cv::IMREAD_UNCHANGED);
    // cv::Mat p2 = cv::imread("/home/ethan/Documents/slam/slambook2/ch7/2.png");
    // cv::Mat d2 = cv::imread("/home/ethan/Documents/slam/slambook2/ch7/2_depth.png", cv::IMREAD_UNCHANGED);
    // test.solve(p1, d1);
    // test.solve(p2, d2);
    // test.print_info();
    odometry test(cam.color_intrin);
    cv::namedWindow("depth");
    cv::namedWindow("color");

    std::string result;
    char key = '\0';
    while(key != 'q' || key != 27)
    {
        result = cam.grab();
        if(result == "eof")
            break;
        color = cam.getColorImage();
        aligned_color = cam.getAlignedColorImage();
        depth = cam.getDepthImage();
        
        if(color!=NULL&&!color->empty()
            &&aligned_color!=NULL&&!aligned_color->empty()
            &&depth!=NULL&&!depth->empty())
        {
            // cv::normalize(*aligned_color, *aligned_color, 0, 256*256, cv::NORM_MINMAX);
            cv::imshow("color", *color);
            cv::imshow("depth", *depth);
            test.solve(*color, *depth);
            test.print_info();
        }
        key = cv::waitKey(1);
    }
    return 0;
}
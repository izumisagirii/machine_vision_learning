#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
//#include <sophus/common.hpp>
#include <opencv2/opencv.hpp>

typedef Eigen::Matrix<double, 8, 1> Vector8f;
class odometry
{
public:
    odometry(cv::Mat depth_K, cv::Mat init_R=cv::Mat::eye(3,3,CV_32FC1),
             cv::Mat init_t=cv::Mat::zeros(3,1,CV_32FC1)) : K(depth_K), R(init_R), t(init_t){}
    odometry(Eigen::Matrix3d depth_K, Eigen::Matrix3d init_R,
             Eigen::Vector3d init_t);
    // 注意，传入的彩色图要用aligned_color_img
    bool solve(cv::Mat aligned_color_img, cv::Mat depth_img);
    void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2);
    void print_info();
    cv::Mat get_R()const {return R;}
    cv::Mat get_t()const {return t;}
private:
    cv::Mat K;
    cv::Mat R, t;
    cv::Mat aligned_color_img_0, depth_img_0;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> temp_matches;
    std::vector<cv::DMatch> matches;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<cv::Point3d> pts_1, pts_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    cv::Point3d center_pts1, center_pts2;
    cv::Point2d temp_p1, temp_p2;
    Eigen::Matrix3d W, U, V, R_;
    Eigen::Vector3d t_;
};
#endif // __ODOMETRY_H__
/**
 * Author:姚益祁
 * Modified date:2020/12/06
 * Phone:18676436991
 * QQ:1210364094
 * Note: 如果有任何不懂的代码，可以直接通过注释的形式push上去，我会及时看到并修改
 **/
#ifndef YOLO_BOUNDING_BOX_H
#define YOLO_BOUNDING_BOX_H

#define OPENCV 4.1.1
// 一定要定义这个OPENCV，否则在darknet中的接口将无法使用！
// 定义根据你的Opencv版本来，Opencv3和Opencv4均可使用本代码
// #include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#include "yolo_v2_class.hpp" // darknet中的C++API
#include "union.h"
#define LOWEST_PROB 0.6
//#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSIO_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
// 实现自动识别opencv版本的方法，注意要导入opencv2/core/version.hpp头文件
//#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
// 将opencv_world4.4.0.lib静态库加入（Windows平台）
// 如果你的windows中没有opencv_world.lib，则为很多其他的lib文件，实际上结果是等价的
// 实际上在CMakeLists.txt中写好了就没有必要在这里加入这些代码
namespace hitcrt
{
/**
 * 注释了的所有代码都是作废代码，这些都是尝试用5帧以内的信息判断当前帧是否返回了正确的点的失败尝试
 **/
// 定义了一个缓冲区数据类型，用于存储5帧以内的中心点信息，中心点的距离移动太大则判定为异常点
// template<typename T>
// struct buff_data{
//     T data;
//     int index;
//     bool is_filled = false;
// };
/**
 * @brief 用于启动setup函数的flag，默认为DEBUG_DETECTOR
 * 二者的区别在于是否在终端输出相关信息，以及是否使用多线程加速
 **/
enum SETUP_FLAG
{
    DEBUG_DETECTOR,
    // 当选择RELEASE_DETECTOR时，则输出frame没有效果，只保留了数值的运算
    RELEASE_DETECTOR
};
/**
 * @brief box类实现了绘制boundingbox和直接将得到的bbox_t转为points的操作 \n
 * 继承了Detector类，实现了对Detector的进一步封装，如果有任何bug需要完善，请QQ联系
 * QQ 1210364094
 * @author 姚益祁
 */
class box : public Detector
{
public:
    box(hitcrt::center& center, const std::string yaml, const std::string names, const std::string cfg, const std::string weights);
    box(const std::string video_file, const std::string yaml, const std::string names, const std::string cfg, const std::string weights);
    void setup(int flag);
    void wait_key();
    void debug_runing();
    void release_runing();
    void shutdown();
    void box_detect();
    void draw_boxes();
    void arrow_detect();
    bool angle_detect();
    bool arrow_shaft(cv::Mat mask);
    inline unsigned short PixelCounter(cv::Mat& src);
    std::vector<bbox_t> &get_bbox(void) { return bounded_boxes; }
    cv::Point2f get_center_point(bbox_t &curr_box)
    {
        return cv::Point2f(curr_box.x + curr_box.w / 2, curr_box.y + curr_box.h / 2);
    }
    std::string get_curr_name(bbox_t &curr_box)
    {
        if (bounded_boxes.empty())
        {
            return "null";
        }
        return names[curr_box.obj_id];
    }
    void print_info();

private:
    unsigned short curr_resolution[2] = {640, 480};
    float feather_param[3], head_param[3], shaft_param[3];
    std::vector<cv::Point2f> center_points;
    // the two edge points of a shaft
    std::vector<cv::Point2f> shaft_points;
    std::vector<float> robot_center_points;
    std::vector<float> robot_shaft_points;
    hitcrt::center* center;
    const cv::String video_file;
    // 这个变量存储的是该帧所有框选区域的信息
    std::vector<bbox_t> bounded_boxes;
    // 存储所有标签的名字
    std::vector<std::string> names;
    std::shared_ptr<cv::Mat> frame;
    // There is no worry aboud the alignment as the depth image has been aligned.
    std::shared_ptr<cv::Mat> depth_frame;
    boost::thread _thread;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat copied_frame;
    cv::Mat image_ROI;
    cv::Rect rect_ROI;
    cv::Mat temp_point;
    cv::Mat temp_point2;
    // 使用FAST特征点提取，阈值调整为30
    cv::Ptr<cv::FeatureDetector> feature_detector = cv::FastFeatureDetector::create(30);
    cv::FileStorage yaml_reader;
};
} // namespace hitcrt

#endif // YOLO_BOUNDING_BOX_H

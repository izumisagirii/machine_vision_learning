#ifndef __SLIDE_WINDOW_H__
#define __SLIDE_WINDOW_H__
#include <algorithm>
#include <opencv2/opencv.hpp>
namespace hitcrt
{
class PointNode
{
public:
    PointNode()
    {
        Data = {0,0};
        Next = nullptr;
    }
    // 必须拷贝，防止外部data被释放
    PointNode(const cv::Point2f data, PointNode* next=nullptr): Data(data), Next(next){}
    cv::Point2f Data;
    PointNode* Next;
};
class slideWindow
{
private:
    int length, max_length;
    float min_threshold, max_threshold, temp_distance, max_distance;
    PointNode *left, *right, *p, *q;
    // 返回两点之间距离
    float diff_distance(const cv::Point2f& input1, const cv::Point2f& input2);
    inline void calc_temp_distance(const cv::Point2f& input);
    void append(const cv::Point2f& input);
    void slide(const cv::Point2f& input);
public:
    slideWindow();
    ~slideWindow();
    // 返回值用于判断是否发现桶动
    bool move(const cv::Point2f& input);
};
}
#endif
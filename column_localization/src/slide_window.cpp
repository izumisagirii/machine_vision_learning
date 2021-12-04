#include "slide_window.h"
using namespace hitcrt;
slideWindow::slideWindow()
{
    left = nullptr, right = nullptr;
    // 暂时设置滑动窗口大小为10
    length = 0, max_length = 6;
    min_threshold = 50, max_threshold = 140*M_PI, temp_distance = 0.0, max_distance = 0.0;
}
slideWindow::~slideWindow()
{
    p = left;
    while(p != nullptr)
    {
        q = p->Next;
        delete p;
        p = q;
    }
}
// 返回两点之间距离
float slideWindow::diff_distance(const cv::Point2f& input1, const cv::Point2f& input2)
{
    return std::sqrt(std::pow(input1.x-input2.x,2) + std::pow(input1.y-input2.y,2));
}

inline void slideWindow::calc_temp_distance(const cv::Point2f& input)
{
    p = left;
    max_distance = 0;
    while(p != nullptr)
    {
        temp_distance = diff_distance(p->Data, input);
        if(temp_distance > max_distance)
            max_distance = temp_distance;
        p = p->Next;
    }
}
void slideWindow::append(const cv::Point2f& input)
{
    if(length == 0)
    {
        left = right = new PointNode(input);
    }
    else
    {
        q = new PointNode(input);
        right->Next = q;
        right = q;
    }
    length++;
    return;
}
void slideWindow::slide(const cv::Point2f& input)
{
    p = left;
    left = left->Next;
    delete p;
    q = new PointNode(input);
    right->Next = q;
    right = q;
    return;
}
// 返回值用于判断是否发现桶动
bool slideWindow::move(const cv::Point2f& input)
{
    // 最初状态，没有任何数据在内部
    if(length == 0)
    {
        append(input);
        return false;
    }
    calc_temp_distance(input);
    if(length == max_length)
    {
        slide(input);
    }
    else if(length < max_length)
    {
        append(input);
    }
    // 大于最大阈值表明当前机器人在运动
    if(max_distance > min_threshold && max_distance < max_threshold)
        return true;
    else
        return false;
}
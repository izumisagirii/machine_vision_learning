//
// Created by hitcrt on 2020/9/16.
//

#ifndef MVS_MVS_H
#define MVS_MVS_H

#include "MvCameraControl.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cstdio>
#include <cstring>
#include <iostream>

using namespace std;
using namespace cv;

class MVS {
public:
    MVS();

    bool grab(Mat &img);

    bool open(unsigned int nIndex, unsigned int width, unsigned int height);

    bool close();

    bool get_param(const string &param);

    bool set_param(const string &param, unsigned int p1, float p2);

    int param_int;
    float param_float;
    Mat srcImage;
private:
    unsigned int _width, _height;

    static int RGB2BGR(unsigned char *pRgbData, unsigned int nWidth, unsigned int nHeight);

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    int nRet = MV_OK;
    void *handle = nullptr;
    MVCC_FLOATVALUE fps;
    MVCC_INTVALUE brightness;
    MVCC_FLOATVALUE paramfloat;
    MVCC_INTVALUE paramint;
    MVCC_INTVALUE stParam;
    MV_FRAME_OUT stOutFrame = {nullptr};
    unsigned int g_nPayloadSize = 0;
};

#endif //MVS_MVS_H

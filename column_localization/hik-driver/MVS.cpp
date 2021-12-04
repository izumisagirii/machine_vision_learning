#include "MvCameraControl.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cstdio>
#include "MVS.h"
#include <cstring>
#include <iostream>

using namespace std;
using namespace cv;

MVS::MVS() {}

int MVS::RGB2BGR(unsigned char *pRgbData, unsigned int nWidth, unsigned int nHeight) {
    if (nullptr == pRgbData) {
        return MV_E_PARAMETER;
    }

    for (unsigned int j = 0; j < nHeight; j++) {
        for (unsigned int i = 0; i < nWidth; i++) {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }

    return MV_OK;
}

bool MVS::open(unsigned int nIndex, unsigned int width, unsigned int height) {
    _width = width;
    _height = height;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nIndex >= stDeviceList.nDeviceNum) {
        printf("Input error!\n");
        return false;
    }
    // Select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        printf("Create Handle fail! nRet [0x%x]\n", nRet);
        return false;
    }
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        printf("Open Device fail! nRet [0x%x]\n", nRet);
        return false;
    }
    // Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0) {
            nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
            if (nRet != MV_OK) {
                printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                return false;
            }
        } else {
            printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            return false;
        }
    }

    // Set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
        printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
        return false;
    }
    // Set PixelFormat
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    if (MV_OK != nRet) {
        printf("Set PixelFormat fail! nRet [0x%x]\n", nRet);
        return false;
    }
    // Set gain
    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 2);
    // Set white balance
    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 1);
    if (MV_OK != nRet) {
        printf("Set White Balance fail! nRet [0x%x]\n", nRet);
        return false;
    }
    // Get payload size
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return false;
    }
    printf("height is %d \n", height);
    nRet = MV_CC_SetHeight(handle, height);
    if (MV_OK != nRet) {
        printf("Set Height fail! nRet [0x%x]\n", nRet);
        return false;
    }
    nRet = MV_CC_SetWidth(handle, width);
    if (MV_OK != nRet) {
        printf("Set Width fail! nRet [0x%x]\n", nRet);
        return false;
    }
    nRet = MV_CC_SetImageNodeNum(handle, 1);
    if (MV_OK != nRet) {
        printf("Set ImageNodeNum fail! nRet [0x%x]\n", nRet);
        return false;
    }
    g_nPayloadSize = stParam.nCurValue;
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        return false;
    }
    return true;
}

bool MVS::close() {
    // Stop Grabbing
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
        printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
        return false;
    }
    // Close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
        printf("ClosDevice fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // Destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
        printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
        if (handle != nullptr) {
            MV_CC_DestroyHandle(handle);
            handle = nullptr;
        }
    }
    return true;
}

bool MVS::get_param(const string &param) {
    int nRet1 = MV_OK;
    if (param == "brightness") {
        nRet1 = MV_CC_GetBrightness(handle, &paramint);
        if (MV_OK != nRet1) {
            printf("Get Brightness fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Brightness is %d \n", paramint.nCurValue);
        }
        return true;
    }
    if (param == "blacklevel") {
        nRet1 = MV_CC_GetIntValue(handle, "blacklevel", &paramint);
        if (MV_OK != nRet1) {
            printf("Get Blacklevel fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Blacklevel is %d \n", paramint.nCurValue);
        }
        return true;
    }
    if (param == "exposure") {
        nRet1 = MV_CC_GetFloatValue(handle, "ExposureTime", &paramfloat);
        if (MV_OK != nRet1) {
            printf("Get Exposure Time fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Exposure Time is %f \n", paramfloat.fCurValue);
        }
        return true;
    }
    if (param == "gamma") {
        nRet1 = MV_CC_GetGamma(handle, &paramfloat);
        if (MV_OK != nRet1) {
            printf("Get Gamma fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Gamma is %f \n", paramfloat.fCurValue);
        }
        return true;
    }
    if (param == "saturation") {
        nRet1 = MV_CC_GetSaturation(handle, &paramint);
        if (MV_OK != nRet1) {
            printf("Get Saturation fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Saturation is %d \n", paramint.nCurValue);
        }
        return true;
    }
    if (param == "width") {
        nRet1 = MV_CC_GetWidth(handle, &paramint);
        if (MV_OK != nRet1) {
            printf("Get Width fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Width is %d \n", paramint.nCurValue);
            printf("nint is %d \n", paramint.nInc);
        }
        return true;
    }
    if (param == "height") {
        nRet1 = MV_CC_GetHeight(handle, &paramint);
        if (MV_OK != nRet1) {
            printf("Get Height fail! nRet [0x%x]\n", nRet1);
            return false;
        } else {
            printf("Height is %d \n", paramint.nCurValue);
            printf("nint is %d \n", paramint.nInc);
        }
        return true;
    }
}

bool MVS::set_param(const string &param, unsigned int p1, float p2) {
    int nRet1 = MV_OK;
    if (param == "brightness") {
        nRet1 = MV_CC_SetBrightness(handle, p1);
        if (MV_OK != nRet1) {
            printf("Set Brightness fail! nRet [0x%x]\n", nRet1);
            return false;
        }
        return true;
    }
    if (param == "exposure") {
        nRet1 = MV_CC_SetFloatValue(handle, "ExposureTime", p2);
        if (MV_OK != nRet1) {
            printf("Set Exposure Time fail! nRet [0x%x]\n", nRet1);
            return false;
        }
        return true;
    }
    if (param == "gamma") {
        nRet1 = MV_CC_SetGamma(handle, p2);
        if (MV_OK != nRet1) {
            printf("Set Gamma fail! nRet [0x%x]\n", nRet1);
            return false;
        }
        return true;
    }
    if (param == "height") {
        nRet = MV_CC_SetHeight(handle, p1);
        if (MV_OK != nRet) {
            printf("Set Height fail! nRet [0x%x]\n", nRet);
            return false;
        }
        return true;
    }
    if (param == "saturation") {
        nRet1 = MV_CC_SetSaturation(handle, p1);
        if (MV_OK != nRet1) {
            printf("Set saturation fail! nRet [0x%x]\n", nRet1);
            return false;
        }
        return true;
    }
    if (param == "AutoBalance") {
        nRet1 = MV_CC_SetBalanceWhiteAuto(handle, 2);
        if (MV_OK != nRet1) {
            printf("Set AutoBalance fail! nRet [0x%x]\n", nRet1);
            return false;
        }
        return true;
    }
    return false;
}

bool MVS::grab(Mat &img) {
    //MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    //memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
//    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSize));
//    if (pData == NULL)
//    {
//        printf("Allocate memory failed.\n");
//        return false;
//    }
    //unsigned char *pLastImage = nullptr;
    nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
    if (nRet != MV_OK) {
        printf("Get Image Buffer fail! nRet [0x%x]\n", nRet);
        return false;
    } else {
        // printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
        //        stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum);
    }

//    else
//    {
//        printf("No data[0x%x]\n", nRet);
//    }
//    nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 1000);
//    if (nRet == MV_OK)
//    {
////        printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
////               stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
//    }
//    else
//    {
//        printf("No data[0x%x]\n", nRet);
//        free(pData);
//        pData = NULL;
//        return false;
//    }

    // convert to Mat
    RGB2BGR(stOutFrame.pBufAddr, _width, _height);
    srcImage = cv::Mat(_height, _width, CV_8UC3, stOutFrame.pBufAddr);
    nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
    if (nRet != MV_OK) {
        printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        return false;
    }
    if (nullptr != srcImage.data) {
//        printf("OpenCV format convert finished.\n");
        img = srcImage.clone();
//        free(pData);
//        pData = NULL;
    } else {
        printf("OpenCV format convert failed.\n");
        return false;
    }
    return true;
}
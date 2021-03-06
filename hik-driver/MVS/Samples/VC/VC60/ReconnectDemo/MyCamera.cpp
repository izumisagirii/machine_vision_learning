#include "stdafx.h"
#include "MyCamera.h"
#include <string.h>

CMyCamera::CMyCamera()
{
    m_hDevHandle        = NULL;
}

CMyCamera::~CMyCamera()
{
    if (m_hDevHandle)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle    = NULL;
    }
}


int CMyCamera::EnumDevices(MV_CC_DEVICE_INFO_LIST* pstDevList)
{
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, pstDevList);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    return MV_OK;
}

// ch:打开设备 | en:Open Device
int     CMyCamera::Open(MV_CC_DEVICE_INFO* pstDeviceInfo)
{
    if (NULL == pstDeviceInfo)
    {
        return MV_E_PARAMETER;
    }

    int nRet  = MV_CC_CreateHandle(&m_hDevHandle, pstDeviceInfo);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    nRet = MV_CC_OpenDevice(m_hDevHandle, MV_ACCESS_Exclusive, 0);
    if (MV_OK != nRet)
    {
        MV_CC_DestroyHandle(&m_hDevHandle);
        return nRet;
    }

    return MV_OK;
}


// ch:关闭设备 | en:Close Device
int     CMyCamera::Close()
{
    if (NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }
    MV_CC_CloseDevice(m_hDevHandle);
    return MV_CC_DestroyHandle(m_hDevHandle);
}


// ch:开启抓图 | en:Start Grabbing
int     CMyCamera::StartGrabbing()
{
    if (NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    return MV_CC_StartGrabbing(m_hDevHandle);
}


// ch:停止抓图 | en:Stop Grabbing
int     CMyCamera::StopGrabbing()
{
    if (NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    return MV_CC_StopGrabbing(m_hDevHandle);
}

int     CMyCamera::GetOneFrameTimeout(unsigned char* pData, unsigned int* pnDataLen, unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec)
{
    if (   NULL == m_hDevHandle 
        || NULL == pData 
        || NULL == pnDataLen 
        || NULL == pFrameInfo)
    {
        return MV_E_PARAMETER;
    }

    *pnDataLen  = 0;

    int nRet = MV_CC_GetOneFrameTimeout(m_hDevHandle, pData, nDataSize, pFrameInfo, nMsec);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    *pnDataLen = pFrameInfo->nFrameLen;

    return MV_OK;
}


// ch:设置显示窗口句柄 | en:Set Display Window Handle
int     CMyCamera::Display(void* hWnd)
{
    if (NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    return MV_CC_Display(m_hDevHandle, hWnd);
}


int     CMyCamera::SaveImage(MV_SAVE_IMAGE_PARAM_EX* pstParam)
{
    if (NULL == pstParam)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SaveImageEx2(m_hDevHandle, pstParam);
}

// ch:注册图像数据回调 | en:Register Image Data CallBack
int     CMyCamera::RegisterImageCallBack(void(__stdcall* cbOutput)(unsigned char * pData, 
                                                                MV_FRAME_OUT_INFO_EX* pFrameInfo, 
                                                                void* pUser),
                                         void* pUser)
{
    if (NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    return MV_CC_RegisterImageCallBackEx(m_hDevHandle, cbOutput, pUser);
}


// ch:注册消息异常回调 | en:Register Message Exception CallBack
int     CMyCamera::RegisterExceptionCallBack(void(__stdcall* cbException)(unsigned int nMsgType, 
                                                                            void* pUser),
                                             void* pUser)
{
    if (NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    return MV_CC_RegisterExceptionCallBack(m_hDevHandle, cbException, pUser);
}


// ch:获取Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Int type parameters, such as Width and Height, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::GetIntValue(IN const char* strKey, OUT unsigned int *pnValue)
{
    if (NULL == m_hDevHandle || NULL == strKey || NULL == pnValue)
    {
        return MV_E_PARAMETER;
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    int nRet = MV_CC_GetIntValue(m_hDevHandle, strKey, &stParam);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    *pnValue = stParam.nCurValue;

    return MV_OK;
}


// ch:设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Set Int type parameters, such as Width and Height, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::SetIntValue(IN const char* strKey, IN unsigned int nValue)
{
    if (NULL == m_hDevHandle || NULL == strKey)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SetIntValue(m_hDevHandle, strKey, nValue);
}


// ch:获取Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Float type parameters, such as ExposureTime and Gain, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::GetFloatValue(IN const char* strKey, OUT float *pfValue)
{
    if (NULL == m_hDevHandle || NULL == strKey || NULL == pfValue)
    {
        return MV_E_PARAMETER;
    }

    MVCC_FLOATVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_FLOATVALUE));
    int nRet = MV_CC_GetFloatValue(m_hDevHandle, strKey, &stParam);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    *pfValue = stParam.fCurValue;

    return MV_OK;
}


// ch:设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Set Float type parameters, such as ExposureTime and Gain, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::SetFloatValue(IN const char* strKey, IN float fValue)
{
    if (NULL == m_hDevHandle || NULL == strKey)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SetFloatValue(m_hDevHandle, strKey, fValue);
}


// ch:获取Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Enum type parameters, such as PixelFormat, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::GetEnumValue(IN const char* strKey, OUT unsigned int *pnValue)
{
    if (NULL == m_hDevHandle || NULL == strKey || NULL == pnValue)
    {
        return MV_E_PARAMETER;
    }

    MVCC_ENUMVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_ENUMVALUE));
    int nRet = MV_CC_GetEnumValue(m_hDevHandle, strKey, &stParam);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    *pnValue = stParam.nCurValue;

    return MV_OK;
}


// ch:设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Set Enum type parameters, such as PixelFormat, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::SetEnumValue(IN const char* strKey, IN unsigned int nValue)
{
    if (NULL == m_hDevHandle || NULL == strKey)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SetEnumValue(m_hDevHandle, strKey, nValue);
}


// ch:获取Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Bool type parameters, such as ReverseX, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::GetBoolValue(IN const char* strKey, OUT bool *pbValue)
{
    if (NULL == m_hDevHandle || NULL == strKey || NULL == pbValue)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_GetBoolValue(m_hDevHandle, strKey, pbValue);
}


// ch:设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Set Bool type parameters, such as ReverseX, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::SetBoolValue(IN const char* strKey, IN bool bValue)
{
    if (NULL == m_hDevHandle || NULL == strKey)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SetBoolValue(m_hDevHandle, strKey, bValue);
}


// ch:获取String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
// en:Get String type parameters, such as DeviceUserID, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::GetStringValue(IN const char* strKey, IN OUT char* strValue, IN unsigned int nSize)
{
    if (NULL == m_hDevHandle || NULL == strKey || NULL == strValue)
    {
        return MV_E_PARAMETER;
    }

    MVCC_STRINGVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_STRINGVALUE));
    int nRet = MV_CC_GetStringValue(m_hDevHandle, strKey, &stParam);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    strcpy(strValue, stParam.chCurValue);

    return MV_OK;
}


// ch:设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
// en:Set String type parameters, such as DeviceUserID, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::SetStringValue(IN const char* strKey, IN const char* strValue)
{
    if (NULL == m_hDevHandle || NULL == strKey)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SetStringValue(m_hDevHandle, strKey, strValue);
}


// ch:执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Execute Command once, such as UserSetSave, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int     CMyCamera::CommandExecute(IN const char* strKey)
{
    if (NULL == m_hDevHandle || NULL == strKey)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_SetCommandValue(m_hDevHandle, strKey);
}

int CMyCamera::GetOptimalPacketSize()
{
    return MV_CC_GetOptimalPacketSize(m_hDevHandle);
}


int     CMyCamera::GetAllMatchInfo(MV_ALL_MATCH_INFO* pstInfo)
{
    if (NULL == m_hDevHandle || NULL == pstInfo)
    {
        return MV_E_PARAMETER;
    }
	
    int nRet = MV_CC_GetAllMatchInfo(m_hDevHandle,pstInfo);
    if (MV_OK != nRet)
    {
        return nRet;
    }
	
    return MV_OK;
}
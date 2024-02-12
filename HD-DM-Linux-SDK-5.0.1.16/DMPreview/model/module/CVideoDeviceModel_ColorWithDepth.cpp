#include "CVideoDeviceModel_ColorWithDepth.h"
#include "CVideoDeviceController.h"
#include "CImageDataModel.h"

CVideoDeviceModel_ColorWithDepth::CVideoDeviceModel_ColorWithDepth(DEVSELINFO *pDeviceSelfInfo):
CVideoDeviceModel(pDeviceSelfInfo)
{

}

int CVideoDeviceModel_ColorWithDepth::Init()
{
    InitColorStreamMask();
    return CVideoDeviceModel::Init();
}

int CVideoDeviceModel_ColorWithDepth::InitColorStreamMask()
{
    std::vector<int> colorIndex = GetColorIndexListInCombineStream();
    std::vector<int> depthIndex = GetDepthIndexListInCombineStream();

    for (int nIndex : colorIndex){
        m_colorStreamMask[nIndex] = COLOR_STEAM_MASK_COLOR;
    }

    for (int nIndex : depthIndex){
        if (0 == m_colorStreamMask.count(nIndex)){
            m_colorStreamMask[nIndex] = COLOR_STEAM_MASK_DEPTH;
        }else{
            m_colorStreamMask[nIndex] |= COLOR_STEAM_MASK_DEPTH;
        }
    }

    return APC_OK;
}


int CVideoDeviceModel_ColorWithDepth::GetDepthIndexFromCombineStream(int nStreamIndex)
{
    if (0 == m_colorStreamMask.count(nStreamIndex)) return EOF;

    if (0 == (m_colorStreamMask[nStreamIndex] & COLOR_STEAM_MASK_DEPTH)) return EOF;

    std::vector<APC_STREAM_INFO> colorStreamInfoList = GetStreamInfoList(STREAM_COLOR);
    std::vector<APC_STREAM_INFO> depthStreamInfoList = GetStreamInfoList(STREAM_DEPTH);

    int nMatchWidth = colorStreamInfoList[nStreamIndex].nWidth;
    if (m_colorStreamMask[nStreamIndex] & COLOR_STEAM_MASK_COLOR){
        nMatchWidth /= 2;
    }

    int nMatchHeight = colorStreamInfoList[nStreamIndex].nHeight;
    for (size_t i = 0 ; i < depthStreamInfoList.size() ; ++i){
        if(nMatchWidth == depthStreamInfoList[i].nWidth &&
           nMatchHeight == depthStreamInfoList[i].nHeight){
            return i;
        }
    }
    return EOF;
}

int CVideoDeviceModel_ColorWithDepth::GetColorIndexFromCombineStream(int nStreamIndex)
{
    if(0 == m_colorStreamMask.count(nStreamIndex)) return EOF;

    if(0 == (m_colorStreamMask[nStreamIndex] & COLOR_STEAM_MASK_COLOR)) return EOF;

    return nStreamIndex;
}

int CVideoDeviceModel_ColorWithDepth::GetCombineStreamIndexFromDepth(int nDepthIndex)
{
    std::vector<APC_STREAM_INFO> colorStreamInfoList = GetStreamInfoList(STREAM_COLOR);
    std::vector<APC_STREAM_INFO> depthStreamInfoList = GetStreamInfoList(STREAM_DEPTH);
    for (size_t i = 0 ; i < colorStreamInfoList.size() ; ++i){
        if (depthStreamInfoList[nDepthIndex].nWidth == colorStreamInfoList[i].nWidth &&
            depthStreamInfoList[nDepthIndex].nHeight == colorStreamInfoList[i].nHeight){
            return i;
        }
    }
    return EOF;
}

int CVideoDeviceModel_ColorWithDepth::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    nDepthDataType = CVideoDeviceModel::TransformDepthDataType(nDepthDataType, bRectifyData);

    if (!m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR)){
        switch (nDepthDataType){
            case APC_DEPTH_DATA_11_BITS:
            case APC_DEPTH_DATA_11_BITS_RAW:
                return APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY;
            case APC_DEPTH_DATA_14_BITS:
            case APC_DEPTH_DATA_14_BITS_RAW:
                return  APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY;
        }
    }

    return nDepthDataType;
}

int CVideoDeviceModel_ColorWithDepth::PrepareOpenDevice()
{
    int ret = CVideoDeviceModel::PrepareOpenDevice();

    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);
    if (bColorStream && bDepthStream){
        m_colorWithDepthImageData.nWidth = m_imageData[STREAM_COLOR].nWidth;
        m_colorWithDepthImageData.nHeight = m_imageData[STREAM_COLOR].nHeight;
        m_colorWithDepthImageData.bMJPG = m_imageData[STREAM_COLOR].bMJPG;
        m_colorWithDepthImageData.depthDataType = m_imageData[STREAM_COLOR].depthDataType;
        m_colorWithDepthImageData.imageDataType = m_imageData[STREAM_COLOR].imageDataType;

        unsigned short nBytePerPixel = 2;
        unsigned int nBufferSize = m_colorWithDepthImageData.nWidth * m_colorWithDepthImageData.nHeight * nBytePerPixel;
        if (m_colorWithDepthImageData.imageBuffer.size() != nBufferSize){
            m_colorWithDepthImageData.imageBuffer.resize(nBufferSize);
        }
        memset(&m_colorWithDepthImageData.imageBuffer[0], 0, sizeof(nBufferSize));
    }

    return ret;
}

int CVideoDeviceModel_ColorWithDepth::UpdateFrameGrabberData(STREAM_TYPE streamType)
{
    if (STREAM_COLOR != streamType &&
        STREAM_DEPTH != streamType){
        return CVideoDeviceModel::UpdateFrameGrabberData(streamType);
    }

    bool bIsCombineMode = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR) &&
                          m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    if (bIsCombineMode || streamType == STREAM_COLOR){
        CVideoDeviceModel::UpdateFrameGrabberData(STREAM_COLOR);
    }

    if (bIsCombineMode || streamType == STREAM_DEPTH){
        CVideoDeviceModel::UpdateFrameGrabberData(STREAM_DEPTH);
    }

    return APC_OK;

}

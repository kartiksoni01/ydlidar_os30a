#include "eSPDI.h"
#include "DepthFusionHelper.h"
#include <string.h>

CDepthMergeWrapper::CDepthMergeWrapper(int depthCount, int width, int height,
    float focus, std::vector<float>& baselineDist)
    : m_depthCount(depthCount), m_width(width), m_height(height)
    , m_focus(focus), m_baselineDist(baselineDist)
{
    m_outDepth.resize(m_width * m_height);
    m_flag.resize(m_width * m_height);

    for (int i = 0; i < depthCount; ++i)
    {
        m_workingRangeNear.push_back(m_focus * m_baselineDist[i] / 255.0);
        m_workingRangeFar.push_back(m_focus * m_baselineDist[i] / 20.0);
        m_workingRangeFusion.push_back(0.0);
    }
    SetSDKHandle( NULL, NULL );
}

CDepthMergeWrapper::~CDepthMergeWrapper()
{

}

void CDepthMergeWrapper::SetSDKHandle( void *pHandleEYSD, PDEVSELINFO pDevSelInfo )
{
    m_pHandleEYSD = pHandleEYSD;
    m_pDevSelInfo    = pDevSelInfo;
}

// outDepth will be mapping to the disparity of inDepth[0]
// selectedIndex[i] == 255 means this pixel has no depth value in all inDepth
int CDepthMergeWrapper::DepthMerge(unsigned char** inDepth, unsigned char* outDepth, unsigned char* selectedIndex)
{
    DoFusion(inDepth, &m_outDepth[0], &m_flag[0], m_width, m_height, m_focus,
        &m_baselineDist[0], &m_workingRangeNear[0], &m_workingRangeFar[0], &m_workingRangeFusion[0],
        m_depthCount, true);// currently only support 11 bits depth

    // if for loop condition is "i < m_outDepth.size()", performance will be influenced
    // using vector's [] operator also influence the performance
    float* pOutInDouble = &m_outDepth[0];
    unsigned short* pOutDepth = (unsigned short*)outDepth;
    for (size_t i = 0, size = m_outDepth.size(); i < size; ++i)
    {
        // range of disparity is [0.0..256.0], x 8 to 11 bits for drawing (range [0..2047])
        pOutDepth[i] = (unsigned short)(pOutInDouble[i] * 8);
    }

    if (selectedIndex != nullptr)
    {
        memcpy(selectedIndex, &m_flag[0], m_flag.size());
    }

    return 0;
}

int CDepthMergeWrapper::DoFusion(unsigned char** pDepthBufList, float *pDepthMerge, unsigned char *pDepthMergeFlag,
    int nDWidth, int nDHeight, float fFocus, float * pBaseline, float * pWRNear, float * pWRFar,
    float * pWRFusion, int nMergeNum, bool bdepth2Byte11bit)
{
    return APC_DepthMerge( m_pHandleEYSD, m_pDevSelInfo, pDepthBufList, pDepthMerge, pDepthMergeFlag, nDWidth, nDHeight, fFocus, pBaseline,
                                pWRNear, pWRFar, pWRFusion, nMergeNum );
    //auto fusionFunc = depthMergeMBLbase;
    //switch (m_fusionMethod)
    //{
    //case MBLbase:
    //    fusionFunc = depthMergeMBLbase;
    //    break;
    //case MBRbaseV0:
    //    fusionFunc = depthMergeMBRbaseV0;
    //    break;
    //case MBRbaseV1:
    //    fusionFunc = depthMergeMBRbaseV1;
    //    break;
    //default:
    //    fusionFunc = depthMergeMBLbase;
    //    break;
    //}

    //return fusionFunc(pDepthBufList, pDepthMerge, pDepthMergeFlag, nDWidth, nDHeight, fFocus, pBaseline, pWRNear, pWRFar,
    //    pWRFusion, nMergeNum, bdepth2Byte11bit);
}


CDepthFusionHelper::CDepthFusionHelper(size_t depthCount, int width, int height,
    float focus, std::vector<float>& baselineDist, size_t queueSize,
    DepthFusionCallbackFn callbackFn, void* callbackParam)
    : m_width(width), m_height(height), m_depthSize(width * height * 2), m_callbackFn(callbackFn), m_callbackParam(callbackParam)
    , m_dmWrapper((int)depthCount, width, height, focus, baselineDist), m_swPostProcHandle(nullptr), m_postProcEnabled(false), m_colorData(queueSize)
{
    for (size_t i = 0; i < depthCount; ++i)
    {
        m_depthData.push_back(DataItemPool(queueSize));
    }

#ifndef ESPDI_EG
    //APC_CreateSwPostProc(11, &m_swPostProcHandle);
#endif

    m_threadStart = true;
    m_checkDepthReadyThread = new std::thread(CDepthFusionHelper::CheckDepthReadyThreadFn, this);
}

CDepthFusionHelper::~CDepthFusionHelper()
{
    m_threadStart = false;
    m_checkDepthReadyThread->join();
    delete m_checkDepthReadyThread;

#ifndef ESPDI_EG
    //APC_ReleaseSwPostProc(&m_swPostProcHandle);
#endif
}

void CDepthFusionHelper::UpdateColorData(int serialNumber, unsigned char* colorBuf, size_t colorSize)
{
    std::lock_guard<std::mutex> lock(m_colorDataMutex);
    m_colorData.UpdateData(serialNumber, colorBuf, colorSize);
}

void CDepthFusionHelper::UpdateDepthData(int depthId, int serialNumber, unsigned char* depthBuf, size_t depthSize)
{
    std::lock_guard<std::mutex> lock(m_depthDataMutex);

    if (depthId >= 0 && depthId < (int)m_depthData.size())
    {
        m_depthData[depthId].UpdateData(serialNumber, depthBuf, depthSize);
    }
}

void CDepthFusionHelper::EnablePostProc(bool enable)
{
    m_postProcEnabled = enable;
}

//bool CDepthFusionHelper::FindAndDropOutOfDateDepthItem(int targetSn, DataItemPool& depthPool) const
//{
//    while (!depthPool.dataItems.empty())
//    {
//        if (depthPool.dataItems.front().sn == targetSn)
//        {
//            return true;
//        }
//        else if (depthPool.dataItems.front().sn > targetSn)
//        {
//            return false;
//        }
//        else// depthPool.dataItems.front().sn < targetSn
//        {
//            depthPool.dataItems.pop();
//        }
//    }
//
//    return false;
//}

void CDepthFusionHelper::DropColorItem(int targetSn)
{
    std::lock_guard<std::mutex> lock(m_colorDataMutex);

    while (!m_colorData.dataItems.empty())
    {
        if (m_colorData.dataItems.front().sn > targetSn)
        {
            break;
        }

        m_colorData.dataItems.pop();
    }
}

bool CDepthFusionHelper::GetColorData(int targetSn, std::vector<unsigned char>& outBuf)
{
    std::lock_guard<std::mutex> lock(m_colorDataMutex);

    while (!m_colorData.dataItems.empty())
    {
        if (m_colorData.dataItems.front().sn == targetSn)
        {
            if (outBuf.size() != m_colorData.dataItems.front().data.size())
            {
                outBuf.resize(m_colorData.dataItems.front().data.size());
            }

            memcpy(&outBuf[0], &m_colorData.dataItems.front().data[0], m_colorData.dataItems.front().data.size());
            return true;
        }
        else if (m_colorData.dataItems.front().sn > targetSn)
        {
            break;
        }

        m_colorData.dataItems.pop();
    }

    return false;
}

void CDepthFusionHelper::CheckDepthReadyThreadFn(CDepthFusionHelper* pThis)
{
    std::vector<unsigned char*> inDepth(pThis->m_depthData.size());
    std::vector<unsigned char> outDepth(pThis->m_depthSize);
    std::vector<unsigned char> selectedIndex(pThis->m_width * pThis->m_height);
    std::vector<unsigned char> postProcColorBuf(pThis->m_depthSize);
    std::vector<unsigned char> postProcTempBuf(pThis->m_depthSize);

    auto CheckDepth = [ & ]()
    {
        for (size_t i = 0; i < pThis->m_depthData.size(); ++i)
        {
            if ( pThis->m_depthData[i].dataItems.empty() ) return false;

            inDepth[i] = &pThis->m_depthData[i].dataItems.front().data[0];
        }
        return true;
    };
    while (pThis->m_threadStart)
    {
        int fusionDepthSn = -1;
        {
            std::lock_guard<std::mutex> lock(pThis->m_depthDataMutex);

            DataItemPool& baseDepthPool = pThis->m_depthData[0];
            if (!baseDepthPool.dataItems.empty())
            {
                //bool doFusion = true;
                //bool baseItemOutOfDate = false;
                //for (size_t i = 1; i < pThis->m_depthData.size(); ++i)
                //{
                //    if (!pThis->FindAndDropOutOfDateDepthItem(baseDepthPool.dataItems.front().sn, pThis->m_depthData[i]))
                //    {// should search all to drop out of date items, so don't call "break"
                //        if (!pThis->m_depthData[i].dataItems.empty())
                //        {
                //            baseItemOutOfDate = true;
                //        }

                //        doFusion = false;
                //    }
                //}

                if ( CheckDepth() )
                {
                    //for (size_t i = 0; i < pThis->m_depthData.size(); ++i)
                    //{
                    //    inDepth[i] = &pThis->m_depthData[i].dataItems.front().data[0];
                    //}

                    pThis->m_dmWrapper.DepthMerge(&inDepth[0], &outDepth[0], &selectedIndex[0]);
                    fusionDepthSn = baseDepthPool.dataItems.front().sn;

#ifndef ESPDI_EG
                    //if (pThis->m_postProcEnabled &&
                    //    pThis->GetColorData(fusionDepthSn, postProcColorBuf))
                    //{
                    //    APC_DoSwPostProc(pThis->m_swPostProcHandle, &postProcColorBuf[0], false,
                    //        &outDepth[0], &postProcTempBuf[0], pThis->m_width, pThis->m_height);
                    //    memcpy(&outDepth[0], &postProcTempBuf[0], postProcTempBuf.size());
                    //}
#endif

                    pThis->DropColorItem(fusionDepthSn);
                    for (auto& depthPool : pThis->m_depthData)
                    {
                        depthPool.dataItems.pop();
                    }
                }
                //else if (baseItemOutOfDate)
                //{
                //    pThis->DropColorItem(baseDepthPool.dataItems.front().sn);
                //    baseDepthPool.dataItems.pop();
                //}
            }
        }

        if (fusionDepthSn != -1)// callback after unlock m_depthDataMutex to prevent deadlock
        {
            pThis->m_callbackFn(&outDepth[0], &selectedIndex[0], (int)pThis->m_depthSize, pThis->m_width, pThis->m_height,
                fusionDepthSn, pThis->m_callbackParam);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void CDepthFusionHelper::SetSDKHandle( void *pHandleEYSD, PDEVSELINFO pDevSelInfo )
{
    m_dmWrapper.SetSDKHandle( pHandleEYSD, pDevSelInfo );
}

CDepthFusionHelper::DataItem::DataItem(int serialNumber, unsigned char* depthBuf, size_t depthSize)
    : sn(serialNumber)
{
    if (data.size() != depthSize)
    {
        data.resize(depthSize);
    }

    memcpy(&data[0], depthBuf, depthSize);
}

void CDepthFusionHelper::DataItemPool::UpdateData(int serialNumber, unsigned char* dataBuf, size_t dataSize)
{
    if (dataItems.size() >= maxSize)
    {
        dataItems.pop();
    }

    dataItems.push(DataItem(serialNumber, dataBuf, dataSize));
}

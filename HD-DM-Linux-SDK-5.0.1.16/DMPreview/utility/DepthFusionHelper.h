#pragma once
#include <thread>
#include <queue>
#include <mutex>

typedef void (*DepthFusionCallbackFn)(unsigned char* depthBuf, unsigned char* selectedIndex, int depthSize, int width, int height, int serialNumber, void* pParam);

//struct DepthMergeMethod
//{
//    enum Value
//    {
//        MBLbase = 0,
//        MBRbaseV0 = 1,
//        MBRbaseV1 = 2
//    };
//};

class CDepthMergeWrapper
{
public:
    CDepthMergeWrapper(int depthCount, int width, int height,
        float focus, std::vector<float>& baselineDist);
    virtual ~CDepthMergeWrapper();

    int DepthMerge(unsigned char** inDepth, unsigned char* outDepth, unsigned char* selectedIndex = nullptr);
    void SetSDKHandle( void *pHandleEYSD, PDEVSELINFO pDevSelInfo );

private:
    int m_depthCount;
    int m_width;
    int m_height;
    float m_focus;
    void * m_pHandleEYSD;
    PDEVSELINFO m_pDevSelInfo;
    std::vector<float> m_outDepth;
    std::vector<unsigned char> m_flag;
    std::vector<float> m_baselineDist;
    std::vector<float> m_workingRangeNear;
    std::vector<float> m_workingRangeFar;
    std::vector<float> m_workingRangeFusion;

private:
    int DoFusion(unsigned char** pDepthBufList, float *pDepthMerge, unsigned char *pDepthMergeFlag,
        int nDWidth, int nDHeight, float fFocus, float * pBaseline, float * pWRNear, float * pWRFar,
        float * pWRFusion, int nMergeNum, bool bdepth2Byte11bit);
};


class CDepthFusionHelper
{
public:
    CDepthFusionHelper(size_t depthCount, int width, int height,
        float focus, std::vector<float>& baselineDist, size_t queueSize,
        DepthFusionCallbackFn callbackFn, void* callbackParam);
    virtual ~CDepthFusionHelper();

    void UpdateColorData(int serialNumber, unsigned char* colorBuf, size_t colorSize);// yuy2
    void UpdateDepthData(int depthId, int serialNumber, unsigned char* depthBuf, size_t depthSize);// id is 0-base
    void EnablePostProc(bool enable);
    void SetSDKHandle( void *pHandleEYSD, PDEVSELINFO pDevSelInfo );

private:
    class DataItem
    {
    public:
        DataItem(int serialNumber, unsigned char* depthBuf, size_t depthSize);

        int sn;
        std::vector<unsigned char> data;
    };

    class DataItemPool
    {
    public:
        DataItemPool(size_t poolSize) : maxSize(poolSize) {}
        void UpdateData(int serialNumber, unsigned char* dataBuf, size_t dataSize);

        std::queue<DataItem> dataItems;
        size_t maxSize;
    };

private:
    int m_width;
    int m_height;
    size_t m_depthSize;
    DepthFusionCallbackFn m_callbackFn;
    void* m_callbackParam;
    bool m_threadStart;
    std::thread* m_checkDepthReadyThread;
    std::mutex m_depthDataMutex;
    std::vector<DataItemPool> m_depthData;
    CDepthMergeWrapper m_dmWrapper;
    std::mutex m_colorDataMutex;
    DataItemPool m_colorData;
    void* m_swPostProcHandle;
    bool m_postProcEnabled;

private:
    static void CheckDepthReadyThreadFn(CDepthFusionHelper* pThis);
    //bool FindAndDropOutOfDateDepthItem(int targetSn, DataItemPool& depthPool) const;
    void DropColorItem(int targetSn);
    bool GetColorData(int targetSn, std::vector<unsigned char>& outBuf);
};

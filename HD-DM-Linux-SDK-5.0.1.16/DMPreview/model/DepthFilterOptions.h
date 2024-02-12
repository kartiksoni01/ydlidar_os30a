#ifndef DEPTHFILTEROPTIONS_H
#define DEPTHFILTEROPTIONS_H

class CVideoDeviceController;
class DepthFilterOptions
{
public:
    enum STATE{
        CUSTOM = 0,
        MIN,
        FULL,
        NONE
    };

    template <typename TYPE>
    struct Attribute{
         TYPE value;
         bool bLock;
    };

public:
    DepthFilterOptions(CVideoDeviceController *pVideoDeviceController);
    ~DepthFilterOptions() = default;

    void SetDefaultValue();

    void SetState(STATE state);
    STATE GetState(){ return m_state; }

    void EnableDepthFilter(bool bEnable){ m_bDoDepthFilter.value = bEnable; }
    bool IsDepthFilter(){ return m_bDoDepthFilter.value; }

    void EnableSubSample(bool bEnable){ m_bSubSample.value = bEnable; }
    bool IsSubSample(){ return m_bSubSample.value; }

    void EnableEdgePreServingFilter(bool bEnable){ m_bEdgePreServingFilter.value = bEnable; }
    bool IsEdgePreServingFilter(){ return m_bEdgePreServingFilter.value; }

    void EnableHoleFill(bool bEnable){ m_bHoleFill.value = bEnable; }
    bool IsHoleFill(){ return m_bHoleFill.value; }

    void EnableTempleFilter(bool bEnable){ m_bTempleFilter.value = bEnable; }
    bool IsTempleFilter(){ return m_bTempleFilter.value; }

    void EnableFlyingDepthCancellation(bool bEnable){ m_bFlyingDepthCancellation.value = bEnable; }
    bool IsFlyingDepthCancellation(){ return m_bFlyingDepthCancellation.value; }

    //Common
    void SetBytesPerPixel(int nBytesPerPixel){ m_nBytesPerPixel.value = nBytesPerPixel; }
    int GetBytesPerPixel(){ return m_nBytesPerPixel.value;}

    //SubSample
    void SetSubSampleMode(int nMode);
    int GetSubSampleMode(){ return m_nSubSampleMode.value;}

    void SetSubSampleFactor(int nSubSampleFactor){ m_nSubSampleFactor.value = nSubSampleFactor; }
    int GetSubSampleFactor(){ return m_nSubSampleFactor.value;}

    //EdgePreServingFilter
    void SetType(int nType){ m_nType.value = nType; }
    int GetType(){ return m_nType.value;}

    void SetEdgeLevel(int nEdgeLevel){ m_nEdgeLevel.value = nEdgeLevel; }
    int GetEdgeLevel(){ return m_nEdgeLevel.value;}

    void SetSigma(float fSigma){ m_fSigma.value = fSigma; }
    float GetSigma(){ return m_fSigma.value;}

    void SetLumda(float fLumda){ m_fLumda.value = fLumda; }
    float GetLumda(){ return m_fLumda.value;}

    //HoleFill
    void SetKernelSize(int nKernelSize){ m_nKernelSize.value = nKernelSize; }
    int GetKernelSize(){ return m_nKernelSize.value;}

    void SetLevel(int nLevel){ m_nLevel.value = nLevel; }
    int GetLevel(){ return m_nLevel.value;}

    void SetHorizontal(bool bHorizontal){ m_bHorizontal.value = bHorizontal; }
    bool IsHorizontal(){ return m_bHorizontal.value;}

    //TemporalFilter
    void SetAlpha(float fAlpha){ m_fAlpha.value = fAlpha; }
    float GetAlpha(){ return m_fAlpha.value;}

    void SetHistory(int nHistory){ m_nHistory.value = nHistory; }
    int GetHistory(){ return m_nHistory.value;}

    void SetDepthEnableLock(bool bLock){ m_bDoDepthFilter.bLock = bLock; }
    bool IsDepthEnableLock(){ return m_bDoDepthFilter.bLock; }

    void SetFlyingDepthCancellationLock(bool bLock){ m_bFlyingDepthCancellation.bLock = bLock; }
    bool IsFlyingDepthCancellationLock(){ return m_bFlyingDepthCancellation.bLock; }

private:

    CVideoDeviceController *m_pVideoDeviceController;

    STATE m_state;

    Attribute<bool> m_bDoDepthFilter;

    Attribute<bool> m_bSubSample;
    Attribute<bool> m_bEdgePreServingFilter;
    Attribute<bool> m_bHoleFill;
    Attribute<bool> m_bTempleFilter;
    Attribute<bool> m_bFlyingDepthCancellation;

    //Common
    Attribute<int> m_nBytesPerPixel;

    //SubSample
    Attribute<int> m_nSubSampleMode;
    Attribute<int> m_nSubSampleFactor;

    //EdgePreServingFilter
    Attribute<int> m_nType;
    Attribute<int> m_nEdgeLevel;
    Attribute<float> m_fSigma;
    Attribute<float> m_fLumda;

    //HoleFill
    Attribute<int> m_nKernelSize;
    Attribute<int> m_nLevel;
    Attribute<bool> m_bHorizontal;

    //TemporalFilter
    Attribute<float> m_fAlpha;
    Attribute<int> m_nHistory;
};

#endif // DEPTHFILTEROPTIONS_H

#include "DepthFilterOptions.h"
#include "CVideoDeviceController.h"
#include <string.h>

DepthFilterOptions::DepthFilterOptions(CVideoDeviceController *pVideoDeviceController):
m_pVideoDeviceController(pVideoDeviceController),
m_state(NONE)
{
    EnableDepthFilter(false);
}

void DepthFilterOptions::SetDefaultValue()
{
    EnableSubSample(false);
    EnableEdgePreServingFilter(false);
    EnableFlyingDepthCancellation(false);
    EnableHoleFill(false);
    EnableTempleFilter(false);

    SetDepthEnableLock(false);
    SetFlyingDepthCancellationLock(false);

    SetSubSampleMode(0);
    SetSubSampleFactor(3);

    SetLumda(0.1f);
    SetSigma(0.015f);
    SetEdgeLevel(1);

    SetHorizontal(false);
    SetLevel(1);

    SetHistory(3);
    SetAlpha(0.4f);
}

void DepthFilterOptions::SetState(STATE state)
{
    if (state == m_state) return;

    m_state = state;

    switch (m_state){
        case CUSTOM: break;
        case MIN:
        case FULL:
            SetDefaultValue();
            EnableSubSample(true);
            EnableEdgePreServingFilter(true);
            EnableHoleFill(true);
            SetHorizontal(true);
            EnableTempleFilter(true);
            EnableFlyingDepthCancellation(FULL == m_state);
            break;
        default: break;
    }
}

void DepthFilterOptions::SetSubSampleMode(int nMode)
{
    if (m_nSubSampleMode.value == nMode) return;

    m_nSubSampleMode.value = nMode;

    switch (m_nSubSampleMode.value) {
        case 0: SetSubSampleFactor(3); break;
        case 1: SetSubSampleFactor(4); break;
        default: break;
    }
}

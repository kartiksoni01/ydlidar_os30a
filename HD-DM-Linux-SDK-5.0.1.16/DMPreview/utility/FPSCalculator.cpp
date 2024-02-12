#include "FPSCalculator.h"

FPSCalculator::FPSCalculator():
m_fFPS(0.0f)
{

}

void FPSCalculator::clock()
{
    // update fps
    if (m_receiveTimeStorage.size() > MAX_RECEIVE_TIME_LIMIT) m_receiveTimeStorage.pop_front();

    m_receiveTimeStorage.push_back(QTime::currentTime());
    if (!m_receiveTimeStorage.empty())
        m_fFPS = ((m_receiveTimeStorage.size() - 1) * 1000.0f) / m_receiveTimeStorage.front().msecsTo(m_receiveTimeStorage.back());
}

float FPSCalculator::GetFPS()
{
    return m_fFPS;
}

#ifndef FPSCALCULATOR_H
#define FPSCALCULATOR_H

#include <deque>
#include <QTime>

#define MAX_RECEIVE_TIME_LIMIT (61)

class FPSCalculator
{
public:
    FPSCalculator();

    void clock();
    float GetFPS();

private:
    float m_fFPS;
    std::deque<QTime> m_receiveTimeStorage;
};

#endif // FPSCALCULATOR_H

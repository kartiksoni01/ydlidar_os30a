#ifndef CIMUCALIBRATION_H
#define CIMUCALIBRATION_H
#include "IMUData.h"
#include <vector>
#include <QVector3D>

struct GyroDatas{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    bool empty(){ return x.empty() || y.empty() || z.empty(); }
};

class CIMUCalibration
{
public:
    CIMUCalibration();
    virtual ~CIMUCalibration();

    virtual int DoCalib(IMUData& data);

private:

    void InitGyroData(QVector3D gyroData);
    void AddData(QVector3D gyroData);

    void FilterGyro(QVector3D &gyroData);

    double GetMedian(std::vector<double> datas);

    bool VerifyGyro(QVector3D &gyroData);

    void CalculateOffset(QVector3D gyroRawData);


private:
    GyroDatas m_gyroDatas;

    QVector3D m_gyroSum;
    QVector3D m_gyroStatic;
    QVector3D m_gyroOffset;
    QVector3D m_previousGyroOffset;

    bool m_bFlagHold;

    unsigned int m_nRecviceDataCount;
};

#endif // CIMUCALIBRATION_H

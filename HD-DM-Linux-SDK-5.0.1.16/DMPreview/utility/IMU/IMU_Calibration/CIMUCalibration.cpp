#include "CIMUCalibration.h"
#include <math.h>

#define DATA_SIZE 5
#define PLOT_DATA_COUNT 75

CIMUCalibration::CIMUCalibration():
m_nRecviceDataCount(0),
m_bFlagHold(false)
{
}

CIMUCalibration::~CIMUCalibration()
{

}

int CIMUCalibration::DoCalib(IMUData& data)
{
    QVector3D gyroData(data._gyroScopeX, data._gyroScopeY, data._gyroScopeZ);

    if (m_gyroDatas.empty()) InitGyroData(gyroData);

    FilterGyro(gyroData);

    if (!VerifyGyro(gyroData)) return -1;

    ++m_nRecviceDataCount;
    m_gyroSum += gyroData;
    gyroData -= m_gyroStatic;

    if (PLOT_DATA_COUNT == m_nRecviceDataCount){
        CalculateOffset({data._gyroScopeX, data._gyroScopeY, data._gyroScopeZ});
        m_gyroSum = QVector3D(0, 0, 0);
        m_nRecviceDataCount = 0;
    }

    data._gyroScopeX = gyroData.x();
    data._gyroScopeY = gyroData.y();
    data._gyroScopeZ = gyroData.z();

    return 0;
}

void CIMUCalibration::InitGyroData(QVector3D gyroData)
{
    m_gyroDatas.x.resize(DATA_SIZE, gyroData.x());
    m_gyroDatas.y.resize(DATA_SIZE, gyroData.y());
    m_gyroDatas.z.resize(DATA_SIZE, gyroData.z());
}

void CIMUCalibration::AddData(QVector3D gyroData)
{
    m_gyroDatas.x.pop_back();
    m_gyroDatas.x.insert(m_gyroDatas.x.begin(), gyroData.x());

    m_gyroDatas.y.pop_back();
    m_gyroDatas.y.insert(m_gyroDatas.y.begin(), gyroData.y());

    m_gyroDatas.z.pop_back();
    m_gyroDatas.z.insert(m_gyroDatas.z.begin(), gyroData.z());
}

void CIMUCalibration::FilterGyro(QVector3D &gyroData)
{
    AddData(gyroData);

    gyroData.setX(GetMedian(m_gyroDatas.x));
    gyroData.setY(GetMedian(m_gyroDatas.y));
    gyroData.setZ(GetMedian(m_gyroDatas.z));
}

double CIMUCalibration::GetMedian(std::vector<double> datas)
{
    int mid = int(datas.size())/2;
    std::partial_sort(datas.begin(), datas.begin()+mid+1, datas.end());
    return datas.at(mid);
}

bool CIMUCalibration::VerifyGyro(QVector3D &gyroData)
{
    return fabs(gyroData.x()) < 990 && fabs(gyroData.y()) < 990 && fabs(gyroData.z()) < 990;
}

#define TH_OFFSET_DEVIATION_GYRO 0.15
#define TH_OFFSET_GYRO 0.8
#define TH_OFFSET_GYRO_D2 (TH_OFFSET_GYRO / 2)

void CIMUCalibration::CalculateOffset(QVector3D gyroRawData)
{
    m_gyroOffset = m_gyroSum / PLOT_DATA_COUNT;

    auto CalculateOffset = [&](double offset, double prevOffset, double staticOffset, double gyroZ, bool bZAxis) -> double{
        double diffStatic = fabs(staticOffset - offset);
        double diifPrev = fabs(prevOffset - offset);

        if (
             ( diffStatic < TH_OFFSET_DEVIATION_GYRO && offset < TH_OFFSET_GYRO ) ||
             ( diifPrev < TH_OFFSET_DEVIATION_GYRO && offset < TH_OFFSET_GYRO ) ||
             ( offset < staticOffset && staticOffset > TH_OFFSET_GYRO_D2)
           ){
            staticOffset = offset; // Update offset

            if (bZAxis){
                m_bFlagHold = false;
            }
        }else{
            if (bZAxis){
                if (!m_bFlagHold &&
                    fabs(staticOffset * 16) < abs(gyroZ)){ // disable offset compensation during motion
                    staticOffset /= 2;
                }
                m_bFlagHold = false;
            }
        }

        return staticOffset;
    };

    m_gyroStatic.setX(CalculateOffset(m_gyroOffset.x(), m_previousGyroOffset.x(), m_gyroStatic.x(), gyroRawData.z(), false));
    m_previousGyroOffset.setX(m_gyroOffset.x());

    m_gyroStatic.setY(CalculateOffset(m_gyroOffset.y(), m_previousGyroOffset.y(), m_gyroStatic.y(), gyroRawData.z(), false));
    m_previousGyroOffset.setY(m_gyroOffset.y());

    m_gyroStatic.setZ(CalculateOffset(m_gyroOffset.z(), m_previousGyroOffset.z(), m_gyroStatic.z(), gyroRawData.z(), true));
    m_previousGyroOffset.setZ(m_gyroOffset.z());

}

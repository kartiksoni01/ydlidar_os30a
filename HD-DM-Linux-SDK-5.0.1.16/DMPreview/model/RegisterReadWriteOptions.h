#ifndef REGISTERREADWRITEOPTIONS_H
#define REGISTERREADWRITEOPTIONS_H
#include "eSPDI_def.h"
#include <QTime>

#define REGISTER_REQUEST_MAX_COUNT 8
class RegisterReadWriteOptions
{
public:
    enum TYPE{
        IC2,
        ASIC,
        FW,
        TYPE_NONE
    };

public:
    RegisterReadWriteOptions();

    TYPE GetType(){ return m_registerType; }
    void SetType(TYPE type)
    {
        m_registerType = type;
    }

    int GetSlaveID(){ return m_nSlaveID; }
    void SetSlaveID(int id){ m_nSlaveID = id; }

    unsigned short GetAddressSize(){ return m_nAddressSize; }
    void SetAddressSize(unsigned short nSize){ m_nAddressSize = nSize; }

    unsigned short GetValueSize(){ return m_nValueSize; }
    void SetValueSize(unsigned short nSize){ m_nValueSize = nSize; }

    SENSORMODE_INFO GetSensorMode(){ return m_sensorMode; }
    void SetSensorMode(SENSORMODE_INFO sensorMode){ m_sensorMode = sensorMode; }

    int GetRequestAddress(int nIndex){ return m_requestAddess[nIndex]; }
    void SetRequestAddress(int nIndex, int nAddress){ m_requestAddess[nIndex] = nAddress; }

    int GetRequestValue(int nIndex){ return m_requestValue[nIndex]; }
    void SetRequestValue(int nIndex, int nValue){ m_requestValue[nIndex] = nValue; }

    bool IsPerodicRead(){ return m_bPeriodicRead; }
    void EnablePerodicRead(bool bEnable){ m_bPeriodicRead = bEnable; }

    int GetPeriodTimeMs(){ return m_nPeriodTimeMs; }
    void SetPeriodTimeMs(int nMs){ m_nPeriodTimeMs = nMs;}

    bool IsSaveLog(){ return m_bSaveLog; }
    void EnableSaveLog(bool bEnable){ m_bSaveLog = bEnable; }

private:
    TYPE m_registerType = IC2;
    int m_nSlaveID = EOF;
    unsigned short m_nAddressSize = FG_Address_1Byte;
    unsigned short m_nValueSize = FG_Value_1Byte;
    SENSORMODE_INFO m_sensorMode = SENSOR_A;
    int m_requestAddess[REGISTER_REQUEST_MAX_COUNT];
    int m_requestValue[REGISTER_REQUEST_MAX_COUNT];

    bool m_bPeriodicRead = false;
    int  m_nPeriodTimeMs = 1000;
    bool m_bSaveLog = false;    
};

#endif // REGISTERREADWRITEOPTIONS_H

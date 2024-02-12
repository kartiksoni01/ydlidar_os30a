#include "CIMUModel.h"
#include "eSPDI_def.h"
#include <QThread>
#include <vector>
#include "CVideoDeviceModel.h"
#include <map>
#include "CEYSDDeviceManager.h"
#include "CIMUDeviceManager.h"

unsigned char CIMUModel::GenerateIMUNumber()
{
    static unsigned char nIMUNumberCounter = 1;
    nIMUNumberCounter = (nIMUNumberCounter + 1) % UCHAR_MAX;
    return nIMUNumberCounter;
}

unsigned char CIMUModel::GetIMUNumber(QString serialNumber)
{
    static std::map<QString, unsigned char> imuNumberMap;

    if (0 == imuNumberMap.count(serialNumber)){
        imuNumberMap[serialNumber] = GenerateIMUNumber();
    }
    return imuNumberMap[serialNumber];
}

const char GET_MODULE_NAME_0[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_MODULE_NAME_1[8] = { 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_MODULE_NAME_2[8] = { 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_MODULE_NAME_3[8] = { 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

const char GET_FW_VERSION_0[8] = { 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_1[8] = { 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_2[8] = { 0x00, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_3[8] = { 0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_4[8] = { 0x00, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_5[8] = { 0x00, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_6[8] = { 0x00, 0x0A, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_FW_VERSION_7[8] = { 0x00, 0x0B, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };

const char READ_OUTPUT_STATUS[8] = { 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char DISABLE_OUTPUT[8] = { 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char ENABLE_OUTPUT[8] = { 0x00, 0x11, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };

const char READ_OUTPUT_FORMAT[8] = { 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const char SET_OUTPUT_FORMAT_1[8] = { 0x00, 0x12, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
const char SET_OUTPUT_FORMAT_2[8] = { 0x00, 0x12, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00 };
const char SET_OUTPUT_FORMAT_3[8] = { 0x00, 0x12, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
const char SET_OUTPUT_FORMAT_4[8] = { 0x00, 0x12, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00 };
const char SET_OUTPUT_FORMAT_5[8] = { 0x00, 0x12, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00 };

const char CHECK_CALIBRATING_STATUS[8] = { 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char START_CALIBRATION[8] = { 0x00, 0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char READ_CALIBRATED[8] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const char GET_SERIAL_NUMBER_0[8] = { 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_SERIAL_NUMBER_1[8] = { 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_SERIAL_NUMBER_2[8] = { 0x00, 0x14, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_SERIAL_NUMBER_3[8] = { 0x00, 0x14, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
const char GET_SERIAL_NUMBER_4[8] = { 0x00, 0x14, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };

const char CHECK_FLASH_WRITING_STATUS[8] = { 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const char START_WRITE_FLASH[8] = { 0x00, 0x1B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

CIMUModel::CIMUModel(INFO info, CVideoDeviceModel *pVideoDeviceModel):
m_imuType(info.axis),
m_pHandle(nullptr),
m_nCurrentIMUFormat(RAW_DATA_WITHOUT_OFFSET),
m_pLogFile(nullptr),
m_pVideoDeviceModel(pVideoDeviceModel),
m_bIsSyncWithCamera(false),
m_nSyncIndex(0)
{
    if (m_pVideoDeviceModel) Init(info);
}

CIMUModel::~CIMUModel()
{
}

int CIMUModel::Init(INFO info)
{
    if (m_pHandle){
        m_pHandle = nullptr;
    }

    m_bIsSyncWithCamera = false;

    std::vector<hid_device *> hidDeviceList;

    auto EnumerateIMU = [&](std::vector<hid_device *> &deviceList) -> int {

#if 1
        deviceList = CIMUDeviceManager::GetInstance()->GetDeviceList(info.nVID, info.nPID);
#else
        deviceList.clear();

        hid_device_info *deviceInfo = hid_enumerate(info.nVID, info.nPID);

        hid_device_info *headInfo = deviceInfo;
        while (deviceInfo){
            hid_device *device = hid_open_path(deviceInfo->path);            
            if (device){
                hid_set_nonblocking(device, true);
                deviceList.push_back(device);
            }
            deviceInfo = deviceInfo->next;
        }
        if (headInfo) hid_free_enumeration(headInfo);
#endif

        return deviceList.size();
    };

    auto IsIMUConntectdWithCamera = [&](hid_device *device) -> bool{

        m_pHandle = device;

        bool IsConnected = false;

        switch (m_imuType){
            case IMU_6_AXIS:
            {
                QString sFWSerialNumber = m_pVideoDeviceModel->GetDeviceInformation()[0].sSerialNumber.c_str();
                QString sIMUSerialNumber = GetSerialNumber();
                IsConnected = !QString::compare(sFWSerialNumber, sIMUSerialNumber);
                break;
            }
            case IMU_9_AXIS:
            {
                int moduleID = GetModuleID();
                if (0 == moduleID) break;
                IMUData data;
                ReadIMUData(data, false);
                IsConnected = (data._hour == moduleID);
                break;
            }
            default: break;
        }

        m_pHandle = nullptr;

        return IsConnected;
    };

    auto TryConnectIMUWithCamera = [&](){

        if ((size_t)m_nSyncIndex >= hidDeviceList.size()){
            m_nSyncIndex = m_nSyncIndex % hidDeviceList.size();
        }

        m_pHandle = hidDeviceList[m_nSyncIndex];
        if (IMU_6_AXIS == m_imuType) {
            QString sFWSerialNumber = m_pVideoDeviceModel->GetDeviceInformation()[0].sSerialNumber.c_str();
            SetSerialNumber(sFWSerialNumber);
            m_bIsSyncWithCamera = true;
        }
        return;
    };

    auto InitializeIMUWithCamera = [&](){

        for (hid_device *device : hidDeviceList){
            if (IsIMUConntectdWithCamera(device)) {
                m_pHandle = device;
                m_bIsSyncWithCamera = true;
                break;
            }
        }

        if (!m_pHandle){
            TryConnectIMUWithCamera();
        }
    };

    if(IMU_UNKNOWN == m_imuType){
        return APC_NotSupport;
    }

    if (IMU_9_AXIS == m_imuType) {
        SetModuleID((unsigned char)GenerateIMUNumber());
    }

    int nDeviceCount = EnumerateIMU(hidDeviceList);
    if (0 == nDeviceCount) return APC_NoDevice;

    InitializeIMUWithCamera();

    ReadDataOutputFormat();

    return APC_OK;
}

void CIMUModel::ReadDataOutputFormat()
{
    SetFeatureDATA_Item setFeatureData = { &READ_OUTPUT_FORMAT[0], (sizeof(READ_OUTPUT_FORMAT) / sizeof(READ_OUTPUT_FORMAT[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    //memcpy(pImuDataOutoutFormat, &status[0], setFeatureData.nDataLength);
    m_nCurrentIMUFormat = status[0] != 0 ? (DATA_FORMAT)status[0] : RAW_DATA_WITHOUT_OFFSET;
}

void CIMUModel::EnableDataOutout(bool bIsEnbale)
{
    SetFeatureDATA_Item setFeatureData;

    if (bIsEnbale) {
        setFeatureData = { &ENABLE_OUTPUT[0], (sizeof(ENABLE_OUTPUT) / sizeof(ENABLE_OUTPUT[0])) };
    }
    else {
        setFeatureData = { &DISABLE_OUTPUT[0], (sizeof(DISABLE_OUTPUT) / sizeof(DISABLE_OUTPUT[0])) };
    }

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
}

int CIMUModel::SelectDataFormat(DATA_FORMAT format)
{
    SetFeatureDATA_Item setFeatureData;

    switch (format)
    {
    case RAW_DATA_WITHOUT_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_1[0], (sizeof(SET_OUTPUT_FORMAT_1) / sizeof(SET_OUTPUT_FORMAT_1[0])) };
        break;

    case RAW_DATA_WITH_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_2[0], (sizeof(SET_OUTPUT_FORMAT_2) / sizeof(SET_OUTPUT_FORMAT_2[0])) };
        break;

    case OFFSET_DATA:
        setFeatureData = { &SET_OUTPUT_FORMAT_3[0], (sizeof(SET_OUTPUT_FORMAT_3) / sizeof(SET_OUTPUT_FORMAT_3[0])) };
        break;

    case DMP_DATA_WITHOT_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_4[0], (sizeof(SET_OUTPUT_FORMAT_4) / sizeof(SET_OUTPUT_FORMAT_4[0])) };
        break;

    case DMP_DATA_WITH_OFFSET:
        setFeatureData = { &SET_OUTPUT_FORMAT_5[0], (sizeof(SET_OUTPUT_FORMAT_5) / sizeof(SET_OUTPUT_FORMAT_5[0])) };
        break;

    default:
        return APC_NotSupport;
    }

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    m_nCurrentIMUFormat = format;
    return APC_OK;
}

void CIMUModel::GetFeatureReport(char* pData, size_t data_lenght)
{
    if (m_pHandle) {
        unsigned char *pBuf = nullptr;
        pBuf = (unsigned char *)calloc(data_lenght + 1, sizeof(unsigned char));
        pBuf[0] = { 0x0 };
        hid_get_feature_report(m_pHandle, pBuf, data_lenght + 1);
        memcpy(pData, pBuf + 1, data_lenght);
        free( pBuf );
    }
}

void CIMUModel::SendFeatureReport(const char* pData, size_t data_lenght)
{
    if (m_pHandle) {
        unsigned char *pBuf = nullptr;
        pBuf = (unsigned char *)calloc(data_lenght + 1, sizeof(unsigned char));
        pBuf[0] = { 0x0 };
        memcpy(pBuf + 1, pData, data_lenght);
        hid_send_feature_report(m_pHandle, pBuf, data_lenght + 1);
        free( pBuf );
    }
}

QString CIMUModel::GetFWVersion()
{
    SetFeatureDATA_Item setFeatureData[8] = {
        { &GET_FW_VERSION_0[0], (sizeof(GET_FW_VERSION_0) / sizeof(GET_FW_VERSION_0[0])) },
        { &GET_FW_VERSION_1[0], (sizeof(GET_FW_VERSION_1) / sizeof(GET_FW_VERSION_1[0])) },
        { &GET_FW_VERSION_2[0], (sizeof(GET_FW_VERSION_2) / sizeof(GET_FW_VERSION_2[0])) },
        { &GET_FW_VERSION_3[0], (sizeof(GET_FW_VERSION_3) / sizeof(GET_FW_VERSION_3[0])) },
        { &GET_FW_VERSION_4[0], (sizeof(GET_FW_VERSION_4) / sizeof(GET_FW_VERSION_4[0])) },
        { &GET_FW_VERSION_5[0], (sizeof(GET_FW_VERSION_5) / sizeof(GET_FW_VERSION_5[0])) },
        { &GET_FW_VERSION_6[0], (sizeof(GET_FW_VERSION_6) / sizeof(GET_FW_VERSION_6[0])) },
        { &GET_FW_VERSION_7[0], (sizeof(GET_FW_VERSION_7) / sizeof(GET_FW_VERSION_7[0])) }
    };

    char fwVersion[256] = { 0 };
    char* pBuf = &fwVersion[0];
    WORD count = 0;

    for (int i = 0; i < 8; i++) {
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
        GetFeatureReport(pBuf, 8);
        pBuf += 8;
        count += 8;
    }

    QString sFWVersion;
    sFWVersion.sprintf("%s", fwVersion);
    return sFWVersion;
}

QString CIMUModel::GetModuleName()
{
    SetFeatureDATA_Item setFeatureData[4] = {
        { &GET_MODULE_NAME_0[0], (sizeof(GET_MODULE_NAME_0) / sizeof(GET_MODULE_NAME_0[0])) },
        { &GET_MODULE_NAME_1[0], (sizeof(GET_MODULE_NAME_1) / sizeof(GET_MODULE_NAME_1[0])) },
        { &GET_MODULE_NAME_2[0], (sizeof(GET_MODULE_NAME_2) / sizeof(GET_MODULE_NAME_2[0])) },
        { &GET_MODULE_NAME_3[0], (sizeof(GET_MODULE_NAME_3) / sizeof(GET_MODULE_NAME_3[0])) }
    };

    char moduleName[256] = { 0 };
    WORD count = 0;

    for (int i = 0; i < 4; i++) {
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
        GetFeatureReport(&moduleName[count], 8);
        count += 8;
    }

    QString sModuleName;
    sModuleName.sprintf("%s", moduleName);
    return sModuleName;
}

QString CIMUModel::GetStatus()
{
    SetFeatureDATA_Item setFeatureData = { &READ_OUTPUT_STATUS[0], (sizeof(READ_OUTPUT_STATUS) / sizeof(READ_OUTPUT_STATUS[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    return (0 == status[0]) ? "Disable" : "Enable";
}

QString CIMUModel::GetSerialNumber()
{
    if (IMU_6_AXIS != m_imuType) return "";

    SetFeatureDATA_Item setFeatureData[5] = {
        { &GET_SERIAL_NUMBER_0[0], (sizeof(GET_SERIAL_NUMBER_0) / sizeof(GET_SERIAL_NUMBER_0[0])) },
        { &GET_SERIAL_NUMBER_1[0], (sizeof(GET_SERIAL_NUMBER_1) / sizeof(GET_SERIAL_NUMBER_1[0])) },
        { &GET_SERIAL_NUMBER_2[0], (sizeof(GET_SERIAL_NUMBER_2) / sizeof(GET_SERIAL_NUMBER_2[0])) },
        { &GET_SERIAL_NUMBER_3[0], (sizeof(GET_SERIAL_NUMBER_3) / sizeof(GET_SERIAL_NUMBER_3[0])) },
        { &GET_SERIAL_NUMBER_4[0], (sizeof(GET_SERIAL_NUMBER_4) / sizeof(GET_SERIAL_NUMBER_4[0])) }
    };

    char serialNumber[256] = { 0 };
    WORD count = 0;
    QString sSerialNumber;
    for (int i = 0; i < 5; i++) {
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
        GetFeatureReport(&serialNumber[count], 8);
        int j = (0 == i) ? 2 : 0;
        for (; j < 8 ; j+=2){
            QChar unicode = QChar(serialNumber[count + j] | (serialNumber[count + j + 1]) << 8);
            if (QChar('\0') == unicode) break;
            sSerialNumber += unicode;
        }

        count += 8;
    }

    return sSerialNumber;
}

void CIMUModel::SetSerialNumber(QString sSerialNumber)
{
    if (IMU_6_AXIS != m_imuType) return;

    char SET_SERIAL_NUMBER[6][8] = { {0x00, 0x15, 0x24, 0x03, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
                                     {0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }};


    SetFeatureDATA_Item setFeatureData[6] = {
        { &SET_SERIAL_NUMBER[0][0], (sizeof(SET_SERIAL_NUMBER[0]) / sizeof(SET_SERIAL_NUMBER[0][0])) },
        { &SET_SERIAL_NUMBER[1][0], (sizeof(SET_SERIAL_NUMBER[1]) / sizeof(SET_SERIAL_NUMBER[1][0])) },
        { &SET_SERIAL_NUMBER[2][0], (sizeof(SET_SERIAL_NUMBER[2]) / sizeof(SET_SERIAL_NUMBER[2][0])) },
        { &SET_SERIAL_NUMBER[3][0], (sizeof(SET_SERIAL_NUMBER[3]) / sizeof(SET_SERIAL_NUMBER[3][0])) },
        { &SET_SERIAL_NUMBER[4][0], (sizeof(SET_SERIAL_NUMBER[4]) / sizeof(SET_SERIAL_NUMBER[4][0])) },
        { &SET_SERIAL_NUMBER[5][0], (sizeof(SET_SERIAL_NUMBER[5]) / sizeof(SET_SERIAL_NUMBER[5][0])) }
    };

    QChar *pSerialNumber = sSerialNumber.data();
    for (int i = 0; i < 6; i++) {
        int j = (0 == i) ? 4 : 2;
        for(; j < 8; j += 2){
            if (*pSerialNumber != QChar('\0')) {
                unsigned short unicode = pSerialNumber->unicode();
                SET_SERIAL_NUMBER[i][j] = unicode & 0xff;
                SET_SERIAL_NUMBER[i][j + 1] = (unicode & 0xff00) >> 8;
                pSerialNumber++;
            }
        }
        SendFeatureReport(setFeatureData[i].pData, setFeatureData[i].nDataLength);
    }

    SetFeatureDATA_Item checkFlashWritingStatue = { &CHECK_FLASH_WRITING_STATUS[0], (sizeof(CHECK_FLASH_WRITING_STATUS) / sizeof(CHECK_FLASH_WRITING_STATUS[0])) };
    SendFeatureReport(checkFlashWritingStatue.pData, checkFlashWritingStatue.nDataLength);
    char status[8] = { 0 };
    GetFeatureReport(&status[0], checkFlashWritingStatue.nDataLength);

    char nRetryCount = 10;
    while (0 != status[0] && --nRetryCount >= 0) {
        QThread::msleep(100);
        SendFeatureReport(checkFlashWritingStatue.pData, checkFlashWritingStatue.nDataLength);
        GetFeatureReport(&status[0], checkFlashWritingStatue.nDataLength);
    }

    SetFeatureDATA_Item writeSerialToFlash = { &START_WRITE_FLASH[0], (sizeof(START_WRITE_FLASH) / sizeof(START_WRITE_FLASH[0])) };
    SendFeatureReport(writeSerialToFlash.pData, writeSerialToFlash.nDataLength);
}

int CIMUModel::GetModuleID()
{
    if (IMU_9_AXIS != m_imuType || !m_pVideoDeviceModel) return -1;

    unsigned short value;

    int ret = APC_GetHWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_pVideoDeviceModel->GetDeviceSelInfo()[0],
                                    0xf306, &value,
                                    FG_Address_2Byte | FG_Value_1Byte);

    if (APC_OK != ret) return -1;

    return value;
}

void CIMUModel::SetModuleID(unsigned char nID)
{
    if (IMU_9_AXIS != m_imuType) return;

    APC_SetHWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                          m_pVideoDeviceModel->GetDeviceSelInfo()[0],
                          0xf306, nID,
                          FG_Address_2Byte | FG_Value_1Byte);
}

int CIMUModel::ReadIMUData(IMUData &imuData, bool bSync)
{
    if(!m_pHandle){
        return APC_NullPtr;
    }

    unsigned char imuRawData[256] = {0};
    int ret = hid_read(m_pHandle, imuRawData, sizeof(imuRawData));
    if (ret > 0){
        int nIMUDataByte = GetIMUDataOutputByte(m_nCurrentIMUFormat);

        if(m_pLogFile){
            for (int i = 0; i < nIMUDataByte; i++) {
                fprintf(m_pLogFile, "%02x ", imuRawData[i]);
            }
            fprintf(m_pLogFile, "\n");
            fflush(m_pLogFile);
        }

        if (IMU_9_AXIS == m_imuType){
            imuData.parsePacket_Quaternion(imuRawData);
        }else if (27 == nIMUDataByte){
            imuData.parsePacket(imuRawData, OFFSET_DATA != m_nCurrentIMUFormat);
        }else if (58 == nIMUDataByte){
            imuData.parsePacket_DMP(imuRawData);
        }

        if (bSync &&
            !m_bIsSyncWithCamera &&
            IMU_9_AXIS == m_imuType &&
            CVideoDeviceModel::STREAMING == m_pVideoDeviceModel->GetState()){
            int nMoudleId = GetModuleID();
            if (imuData._hour == nMoudleId){
                m_bIsSyncWithCamera = true;
            }else{
                m_nSyncIndex++;
                for (INFO info : m_pVideoDeviceModel->GetIMUInfo()){
                    Init(info);
                    if (m_pHandle) break;
                }
                QThread::sleep(1);
            }
        }

    }

    return ret;
}

int CIMUModel::GetIMUDataOutputByte(DATA_FORMAT format)
{
    switch(format){
        case RAW_DATA_WITHOUT_OFFSET:
        case RAW_DATA_WITH_OFFSET:
        case OFFSET_DATA:
            return 27;
        case DMP_DATA_WITHOT_OFFSET:
        case DMP_DATA_WITH_OFFSET:
            return 58;
        default: return 0;
    }
}

std::vector<CIMUModel::DATA_FORMAT> CIMUModel::GetSupportDataFormat()
{
    switch (m_imuType){
        case IMU_6_AXIS:
        return {
            RAW_DATA_WITHOUT_OFFSET,
            RAW_DATA_WITH_OFFSET,
            OFFSET_DATA
        };
        case IMU_9_AXIS:
        default:
        return {};
    }
}

void CIMUModel::CheckCalibratingStatus(char *pCalibratingStatus)
{
    SetFeatureDATA_Item setFeatureData = { &CHECK_CALIBRATING_STATUS[0], (sizeof(CHECK_CALIBRATING_STATUS) / sizeof(CHECK_CALIBRATING_STATUS[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    *pCalibratingStatus = status[0];
}

void CIMUModel::StartCalibration()
{
    SetFeatureDATA_Item setFeatureData = { &START_CALIBRATION[0], (sizeof(START_CALIBRATION) / sizeof(START_CALIBRATION[0])) };
    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
}

void CIMUModel::ReadCalibrated(char *pCalibrated)
{
    SetFeatureDATA_Item setFeatureData = { &READ_CALIBRATED[0], (sizeof(READ_CALIBRATED) / sizeof(READ_CALIBRATED[0])) };

    char status[8] = { 0 };

    SendFeatureReport(setFeatureData.pData, setFeatureData.nDataLength);
    GetFeatureReport(&status[0], setFeatureData.nDataLength);

    *pCalibrated = status[0];
}

#include "CVideoDeviceModel.h"
#include "eSPDI.h"
#include "CEYSDDeviceManager.h"
#include "CVideoDeviceController.h"
#include "CThreadWorkerManage.h"
#include "CTaskInfoManager.h"
#include "PlyFilter.h"
#include "RegisterSettings.h"
#include "CImageDataModel.h"
#include <libudev.h>
#include <unistd.h>
#include "CFrameSyncManager.h"

CVideoDeviceModel::CVideoDeviceModel(DEVSELINFO *pDeviceSelfInfo):
m_state(CLOSED),
m_nIRMin(0),
m_nIRMax(0),
m_nIRValue(0),
m_usbPortType(USB_PORT_TYPE_UNKNOW),
m_depthDataType(APC_DEPTH_DATA_DEFAULT),
m_pVideoDeviceController(nullptr),
m_pFrameGrabber(nullptr),
m_coldResetTask(nullptr),
m_pIMUModel(nullptr),
m_nLastInterLeaveColorSerial(0),
m_nLastInterLeaveDepthSerial(0),
m_serialNumberType(FRAME_COUNT)
{
    DEVSELINFO *pDevSelfInfo = new DEVSELINFO;
    pDevSelfInfo->index = pDeviceSelfInfo->index;
    m_deviceSelInfo.push_back(std::move(pDevSelfInfo));

    memset(m_nColdResetThresholdMs, 0, sizeof(m_nColdResetThresholdMs));
    memset(m_nLastestSuccessTime, 0, sizeof(m_nLastestSuccessTime));
}

CVideoDeviceModel::~CVideoDeviceModel()
{
    for (DEVSELINFO *pSelfInfo : m_deviceSelInfo){
        delete pSelfInfo;
    }

    if(m_pFrameGrabber){
        delete m_pFrameGrabber;
    }

    if(m_pIMUModel) delete m_pIMUModel;
}

void CVideoDeviceModel::AdjustDeviceSelfInfo(DEVSELINFO *pDeviceSelfInfo)
{
    m_deviceSelInfo[0]->index = pDeviceSelfInfo->index;
    Init();
}

bool CVideoDeviceModel::EqualModel(CVideoDeviceModel *pModel)
{
    std::vector<DeviceInfo> selfInfo = GetDeviceInformation();
    std::vector<DeviceInfo> modelInfo = pModel->GetDeviceInformation();

    if (selfInfo.empty() || modelInfo.empty()) return false;

    if (selfInfo.size() != modelInfo.size()) return false;

    for(size_t i = 0 ; i < selfInfo.size() ; ++i){
        DeviceInfo self = selfInfo[i];
        DeviceInfo another = modelInfo[i];

        if (self.deviceInfomation.nChipID != another.deviceInfomation.nChipID) return false;
        if (self.deviceInfomation.wPID != another.deviceInfomation.wPID) return false;
        if (self.deviceInfomation.wVID != another.deviceInfomation.wVID) return false;

        if (self.sFWVersion.compare(another.sFWVersion)) return false;
        if (self.sSerialNumber.compare(another.sSerialNumber)) return false;
        if (self.sBusInfo.compare(another.sBusInfo)) return false;
    }

    return true;
}

int CVideoDeviceModel::Init()
{
    Reset();
    InitDeviceSelInfo();
    InitDeviceInformation();
    InitStreamInfoList();
#if !defined(_MIPI_CAMERA_)
    InitUsbType();
#endif
    InitCameraproperty();
    InitIMU();
    Update();
    if (m_pVideoDeviceController) m_pVideoDeviceController->Init();
    return DataVerification();
}

int CVideoDeviceModel::Reset()
{
    for(size_t i = 1 ; i < m_deviceSelInfo.size() ; ++i){
        delete m_deviceSelInfo[i];
    }

    m_deviceSelInfo.resize(1);

    m_deviceInfo.clear();

    for(int i = 0 ; i < STREAM_TYPE_COUNT ; ++i){
        m_streamInfo[i].resize(MAX_STREAM_INFO_COUNT, {0, 0, false});
    }

    for(CCameraPropertyModel *pCameraPropertyModel : m_cameraPropertyModel){
        delete pCameraPropertyModel;
    }
    m_cameraPropertyModel.clear();

    return APC_OK;
}

int CVideoDeviceModel::Update()
{
    UpdateIR();
    UpdateDepthDataType();
    UpdateZDTable();
    for(CCameraPropertyModel *pCameraPropertyModel : m_cameraPropertyModel){
        pCameraPropertyModel->Update();
    }
    return APC_OK;
}

int CVideoDeviceModel::DataVerification()
{
    for (DeviceInfo deviceInfo : m_deviceInfo){
        ERROR_HANDLE(/* deviceInfo.deviceInfomation.nChipID != 0 && */ // 8063 did not give chipId temporarily
                     deviceInfo.deviceInfomation.wPID != 0 &&
                     deviceInfo.deviceInfomation.wVID != 0 &&
                     deviceInfo.deviceInfomation.strDevName != 0 &&
                     deviceInfo.sSerialNumber.length() != 0 &&
                     deviceInfo.sSerialNumber.length() != 0, "Get device information failed!!\n");
    }

    for (int i = 0 ; i < STREAM_TYPE_COUNT ; ++i){
        if (IsStreamSupport( (STREAM_TYPE)i )){
            ERROR_HANDLE(!m_streamInfo[i].empty(), "Get stream information failed!!\n");
        }
    }
#if !defined(_MIPI_CAMERA_)
    ERROR_HANDLE(true == (USB_PORT_TYPE_UNKNOW != m_usbPortType), "Get usb port type failed!!\n");
#endif
    return APC_OK;
}

void CVideoDeviceModel::ChangeState(STATE state)
{
    if(RECONNECTING == state && RECONNECTING != m_state){
        m_restoreState = m_state;
        if (STREAMING == m_restoreState){
            StopStreaming(true);
        }
    }else if(RECONNECTED == state){
        state = m_restoreState;
        if (STREAMING == m_restoreState){
            StartStreaming();
        }
    }

    m_state = state;
}

int CVideoDeviceModel::InitDeviceSelInfo()
{
    return APC_OK;
}

std::vector<DEVSELINFO *> CVideoDeviceModel::GetDeviceSelInfo() const
{
    return m_deviceSelInfo;
}

int CVideoDeviceModel::InitDeviceInformation()
{
    m_deviceInfo.push_back(GetDeviceInformation(m_deviceSelInfo[0]));
    return APC_OK;
}

std::vector<CVideoDeviceModel::DeviceInfo> CVideoDeviceModel::GetDeviceInformation() const
{
    return m_deviceInfo;
}

CVideoDeviceModel::DeviceInfo CVideoDeviceModel::GetDeviceInformation(DEVSELINFO *pDeviceSelfInfo)
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();

    DeviceInfo deviceInfo;
    memset(&deviceInfo, 0, sizeof(DeviceInfo));
    if(!pDeviceSelfInfo) return deviceInfo;

    int ret;
    RETRY_APC_API(ret, APC_GetDeviceInfo(pEYSDI, pDeviceSelfInfo, &deviceInfo.deviceInfomation));

    char pFWVersion[256] = {0};
    int  nFWLength;
    RETRY_APC_API(ret, APC_GetFwVersion(pEYSDI, pDeviceSelfInfo, pFWVersion, 256, &nFWLength));
    if (APC_OK == ret){
        deviceInfo.sFWVersion = pFWVersion;
    }


    unsigned char pSerialNumber[256] = {0};
    int  nSerialNumberLength;
    RETRY_APC_API(ret, APC_GetSerialNumber(pEYSDI, pDeviceSelfInfo, pSerialNumber, 256, &nSerialNumberLength));
    if (APC_OK == ret){
        if(nSerialNumberLength<0)
            nSerialNumberLength = 0;
        deviceInfo.sSerialNumber.resize( nSerialNumberLength / 2 );
        for (int i = 0 ; i < nSerialNumberLength / 2 ; ++i)
            deviceInfo.sSerialNumber[i] = pSerialNumber[i * 2 + 1] * 256 + pSerialNumber[i * 2];
    }

    char pBusInfo[256] = {0};
    int  nBusInfoLength;
#if !defined(_MIPI_CAMERA_)
    RETRY_APC_API(ret, APC_GetBusInfo(pEYSDI, pDeviceSelfInfo, pBusInfo, &nBusInfoLength));
    if (APC_OK == ret){
        deviceInfo.sBusInfo = pBusInfo;
    }
#endif
    auto getModelName = [=](char *pDevV4lPath, char *pModelNameResult){
        FILE* f;
        char buffer[128];
        int i = 0;

        f = fopen(pDevV4lPath, "r" );
        if (!f) {
            printf("Could not open %s\n", pDevV4lPath);
            return -1;
        }

        fgets(buffer, sizeof(buffer), f);
        do {
            pModelNameResult[i] = buffer[i];
            i++;
        } while(buffer[i] != '\0');
        i--;
        pModelNameResult[i] = '\0';
        fclose( f );

        return 0;
    };

    char pDevBuf[128] = {0};
    char pDevBuf_v4l[128] = {0};
    char pModelName[128] = {0};
    strcpy(pDevBuf, &deviceInfo.deviceInfomation.strDevName[strlen("/dev/")]);
    sprintf(pDevBuf_v4l, "/sys/class/video4linux/%s/name", pDevBuf);
    getModelName(pDevBuf_v4l, pModelName);
    deviceInfo.sModelName = pModelName;

    return deviceInfo;
}

int  CVideoDeviceModel::InitUsbType()
{
    int ret;
    RETRY_APC_API(ret, APC_GetDevicePortType(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                   m_deviceSelInfo[0],
                                                   &m_usbPortType));
    if (APC_OK != ret){
        struct udev *udev;
        struct udev_enumerate *enumerate;
        struct udev_list_entry *devices, *dev_list_entry;
        struct udev_device *dev;

        udev = udev_new();

        enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, "video4linux");
        udev_enumerate_scan_devices(enumerate);
        devices = udev_enumerate_get_list_entry(enumerate);

        char devPath[256] = {0};
        strcpy(devPath, &m_deviceInfo[0].deviceInfomation.strDevName[strlen("/dev/")]);
        udev_list_entry_foreach(dev_list_entry, devices) {
            const char *path = udev_list_entry_get_name(dev_list_entry);
            if (strcmp(&path[strlen(path) - strlen(devPath)], devPath)) continue;

            dev = udev_device_new_from_syspath(udev, path);
            const char *speed = udev_device_get_sysattr_value(dev, "speed");
            struct udev_device *nodeDev = dev;
            while (!speed){
                struct udev_device *parentDev = udev_device_get_parent(dev);
                if (!parentDev) break;
                dev = parentDev;
                speed = udev_device_get_sysattr_value(dev, "speed");
            }

            if (speed){
                if (!strcmp(speed, "5000")){
                    m_usbPortType = USB_PORT_TYPE_3_0;
                }else if (!strcmp(speed, "480")){
                    m_usbPortType = USB_PORT_TYPE_2_0;
                }

                ret = APC_OK;
            }
            udev_device_unref(nodeDev);
            break;
        }
        udev_enumerate_unref(enumerate);
        udev_unref(udev);
    }

    return ret;
}

USB_PORT_TYPE CVideoDeviceModel::GetUsbType()
{
    return m_usbPortType;
}

bool CVideoDeviceModel::IsStreamSupport(STREAM_TYPE type)
{
    switch (type){
        case STREAM_COLOR:
        case STREAM_DEPTH:
            return !GetStreamInfoList(type).empty();
        default:
            return false;
    }
}

int CVideoDeviceModel::InitStreamInfoList()
{
    int ret;
    RETRY_APC_API(ret, APC_GetDeviceResolutionList(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                         m_deviceSelInfo[0],
                                                         MAX_STREAM_INFO_COUNT, &m_streamInfo[STREAM_COLOR][0],
                                                         MAX_STREAM_INFO_COUNT, &m_streamInfo[STREAM_DEPTH][0]));

    if (APC_OK != ret) return ret;

    auto it = m_streamInfo[STREAM_COLOR].begin();
    for ( ; it != m_streamInfo[STREAM_COLOR].end() ; ++it){
        if (0 == (*it).nWidth){
            break;
        }
    }
    m_streamInfo[STREAM_COLOR].erase(it, m_streamInfo[STREAM_COLOR].end());
    m_streamInfo[STREAM_COLOR].shrink_to_fit();

    it = m_streamInfo[STREAM_DEPTH].begin();
    for ( ; it != m_streamInfo[STREAM_DEPTH].end() ; ++it){
        if (0 == (*it).nWidth){
            break;
        }
    }
    m_streamInfo[STREAM_DEPTH].erase(it, m_streamInfo[STREAM_DEPTH].end());
    m_streamInfo[STREAM_DEPTH].shrink_to_fit();

    return APC_OK;
}

std::vector<APC_STREAM_INFO> CVideoDeviceModel::GetStreamInfoList(STREAM_TYPE type)
{
    return m_streamInfo[type];
}

int CVideoDeviceModel::InitCameraproperty()
{
    AddCameraPropertyModels();

    for(CCameraPropertyModel *pCameraPropertyModel : m_cameraPropertyModel){
        pCameraPropertyModel->Init();
    }

    return APC_OK;
}

int CVideoDeviceModel::SetPlumAR0330(bool bEnable)
{
    unsigned short nValue = 0;
    if(APC_OK != APC_GetFWRegister(CEYSDDeviceManager::GetInstance()->GetInstance(),
                                           m_deviceSelInfo[0],
                                           0xF3,
                                           &nValue,
                                           FG_Address_1Byte | FG_Value_1Byte)){
        return APC_WRITE_REG_FAIL;
    }

    nValue |= 0x11;
    nValue &= bEnable ? 0x10 : 0x01;
    if(APC_OK != APC_SetFWRegister(CEYSDDeviceManager::GetInstance()->GetInstance(),
                                           m_deviceSelInfo[0],
                                           0xF3,
                                           nValue,
                                           FG_Address_1Byte | FG_Value_1Byte)){
        return APC_WRITE_REG_FAIL;
    }

    return APC_OK;
}

int CVideoDeviceModel::AddCameraPropertyModels()
{
    CCameraPropertyModel *pNewCameraPropertyModel = new CCameraPropertyModel("Color", this, m_deviceSelInfo[0]);
    m_cameraPropertyModel.push_back(std::move(pNewCameraPropertyModel));

    return APC_OK;
}

std::vector<CCameraPropertyModel *> CVideoDeviceModel::GetCameraproperty()
{
    return m_cameraPropertyModel;
}

#define DEFAULT_IR_MAX 6
#define EXTEND_IR_MAX 15

int CVideoDeviceModel::SetIRValue(unsigned short nValue)
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret;

    if (nValue != 0) {
        RETRY_APC_API(ret, APC_SetIRMode(pEYSDI, m_deviceSelInfo[0], 0x3f)); // 6 bits on for opening both 6 ir
        RETRY_APC_API(ret, APC_SetCurrentIRValue(pEYSDI, m_deviceSelInfo[0], nValue));
    } else {
        RETRY_APC_API(ret, APC_SetCurrentIRValue(pEYSDI, m_deviceSelInfo[0], nValue));
    }

    UpdateIR();

    return APC_OK;
}

int CVideoDeviceModel::ExtendIR(bool bEnable)
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();

    int ret;
    bool bDisableIR = (0 == m_nIRValue);
    if (bDisableIR){
        RETRY_APC_API(ret, APC_SetIRMode(pEYSDI, m_deviceSelInfo[0], 0x03)); // 2 bits on for opening both 2 ir
    }

    RETRY_APC_API(ret, APC_SetIRMaxValue(pEYSDI, m_deviceSelInfo[0], bEnable ? EXTEND_IR_MAX : DEFAULT_IR_MAX));

    if (bDisableIR){
        RETRY_APC_API(ret, APC_SetIRMode(pEYSDI, m_deviceSelInfo[0], 0x00);); // turn off ir
    }

    if (ret != APC_OK) return ret;

    ret = UpdateIR();

    return ret;
}

int CVideoDeviceModel::SetSerialNumberType(SERIAL_NUMBER_TYPE type)
{
    int ret;
    RETRY_APC_API(ret, APC_SetControlCounterMode(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                       m_deviceSelInfo[0],
                                                       (SERIAL_COUNT == type) ? 0x1 : 0x0));

    unsigned char result;
    APC_GetControlCounterMode(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                  m_deviceSelInfo[0],
                                  &result);
    m_serialNumberType = (1 == result) ? SERIAL_COUNT : FRAME_COUNT;

    return ret;
}

int CVideoDeviceModel::UpdateIR()
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();
    int ret;
    ret = APC_GetFWRegister(pEYSDI, m_deviceSelInfo[0],
                                0xE2, &m_nIRMax,
                                FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret) return ret;

    ret = APC_GetFWRegister(pEYSDI, m_deviceSelInfo[0],
                                0xE1, &m_nIRMin,
                                FG_Address_1Byte | FG_Value_1Byte);
    if (APC_OK != ret) return ret;

    ret = APC_GetCurrentIRValue(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[0], &m_nIRValue);
    if (APC_OK != ret) return ret;

    return APC_OK;
}

int CVideoDeviceModel::GetIRRange(unsigned short &nMin, unsigned short &nMax)
{
    nMin = m_nIRMin;
    nMax = m_nIRMax;
    return APC_OK;
}

unsigned short CVideoDeviceModel::GetIRValue()
{
    return m_nIRValue;
}

bool CVideoDeviceModel::IsIRExtended()
{
    return EXTEND_IR_MAX == m_nIRMax;
}

int CVideoDeviceModel::NormalizeDepthDataType(int &nDepthDataType)
{
    if (nDepthDataType >= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)
    {
        nDepthDataType -= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }
    else if (nDepthDataType >= APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)
    {
        nDepthDataType -= APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET;
    }
    else if (nDepthDataType >= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET)
    {
        nDepthDataType -= APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET;
    }

    return APC_OK;
}

int CVideoDeviceModel::TransformDepthDataType(int nDepthDataType, bool bRectifyData)
{
    NormalizeDepthDataType(nDepthDataType);

    switch (nDepthDataType){
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_RAW:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS : APC_DEPTH_DATA_8_BITS_RAW; break;
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_8_BITS_x80_RAW:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_8_BITS_x80 : APC_DEPTH_DATA_8_BITS_x80_RAW; break;
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_RAW:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_11_BITS : APC_DEPTH_DATA_11_BITS_RAW;  break;
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_RAW:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_14_BITS : APC_DEPTH_DATA_14_BITS_RAW;  break;
        case APC_DEPTH_DATA_OFF_RAW:
        case APC_DEPTH_DATA_OFF_RECTIFY:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_OFF_RECTIFY : APC_DEPTH_DATA_OFF_RAW;  break;
        default:
            nDepthDataType = bRectifyData ? APC_DEPTH_DATA_DEFAULT : APC_DEPTH_DATA_OFF_RECTIFY;  break;
    }

    if (IsInterleaveMode()) nDepthDataType += APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET;

    return nDepthDataType;
}

int CVideoDeviceModel::SetDepthDataType(int nDepthDataType)
{
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();

    int ret;
    for (DEVSELINFO *devSelInfo : m_deviceSelInfo){
        ret = APC_SetDepthDataType(pEYSDI, devSelInfo, nDepthDataType);
        if (ret != APC_OK) return ret;
        UpdateDepthDataType();
    }

    return ret;
}

int CVideoDeviceModel::TransformDepthDataType(bool bRectifyData)
{
    return TransformDepthDataType(GetDepthDataType(), bRectifyData);
}

int CVideoDeviceModel::TransformDepthDataType(int nDepthDataType)
{
    return TransformDepthDataType(nDepthDataType, IsRectifyData());
}

int CVideoDeviceModel::UpdateDepthDataType()
{
    int ret;
    RETRY_APC_API(ret, APC_GetDepthDataType(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                  m_deviceSelInfo[0], &m_depthDataType));

    if (APC_OK != ret) return ret;
    return APC_OK;
}

unsigned short CVideoDeviceModel::GetDepthDataType()
{
    return m_depthDataType;
}

bool CVideoDeviceModel::IsRectifyData()
{
    int depthDataType = GetDepthDataType();
    NormalizeDepthDataType(depthDataType);

    switch (depthDataType){
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_OFF_RECTIFY:
            return true;
        default:
            return false;
    }
}

APCImageType::Value CVideoDeviceModel::GetDepthImageType()
{
    int depthDataType = GetDepthDataType();
    NormalizeDepthDataType(depthDataType);

    switch (depthDataType){
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_RAW:
            return APCImageType::DEPTH_8BITS;
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_8_BITS_x80_RAW:
            return APCImageType::DEPTH_8BITS_0x80;
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_RAW:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
            return APCImageType::DEPTH_11BITS;
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_RAW:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
            return APCImageType::DEPTH_14BITS;
        default:
            return APCImageType::IMAGE_UNKNOWN;
    }
}

bool CVideoDeviceModel::IsHWPP()
{
    unsigned short nHWPP;
    int ret = APC_GetHWRegister(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[0],
                                    0xf424,
                                    &nHWPP,
                                    FG_Address_2Byte | FG_Value_1Byte);

    if (ret != APC_OK) return false;

    return (nHWPP & 0x30) == 0;
}

int CVideoDeviceModel::SetHWPP(bool bEnable)
{
    unsigned short value = 0;
    void *pEYSDI = CEYSDDeviceManager::GetInstance()->GetEYSD();

    for(size_t i = 0 ; i < m_deviceSelInfo.size() ; ++i){
        value = 0;
        APC_GetHWRegister(pEYSDI, m_deviceSelInfo[i],
                              0xf424, &value,
                              FG_Address_2Byte | FG_Value_1Byte);
        value &= 0x0F;
        value |= (bEnable ? 0x40 : 0x70);
        APC_SetHWRegister(pEYSDI, m_deviceSelInfo[i],
                              0xf424, value,
                              FG_Address_2Byte | FG_Value_1Byte);

        if(0 == i && m_deviceSelInfo.size() > 1){
            APC_SetSensorRegister(pEYSDI, m_deviceSelInfo[i],
                                      0xC2, 0x9024, value,
                                      FG_Address_2Byte | FG_Value_1Byte,
                                      SENSOR_A);
        }
    }

    return APC_OK;
}

int CVideoDeviceModel::UpdateZDTable()
{
    if (!m_pVideoDeviceController) return APC_NullPtr;
    if (GetCameraFocus() > 0.0) return APC_OK;

    ZDTABLEINFO zdTableInfo;

    zdTableInfo.nDataType = APC_DEPTH_DATA_11_BITS;
    memset(m_zdTableInfo.ZDTable, 0, sizeof(m_zdTableInfo.ZDTable));
    if (m_deviceInfo[0].deviceInfomation.nDevType == PUMA) {
        m_zdTableInfo.nTableSize = APC_ZD_TABLE_FILE_SIZE_11_BITS;
    } else {
        m_zdTableInfo.nTableSize = APC_ZD_TABLE_FILE_SIZE_8_BITS;
    }

    zdTableInfo.nIndex = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
    AdjustZDTableIndex(zdTableInfo.nIndex);

    int nActualLength = 0;
    int ret;
    RETRY_APC_API(ret, APC_GetZDTable(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                            m_deviceSelInfo[0],
                                            m_zdTableInfo.ZDTable, m_zdTableInfo.nTableSize,
                                            &nActualLength, &zdTableInfo));

    if (APC_OK != ret) return ret;

    m_zdTableInfo.ZDTable[0] = 0;
    m_zdTableInfo.ZDTable[1] = 0;
    m_zdTableInfo.nZNear = USHRT_MAX;
    m_zdTableInfo.nZFar = 0;

    unsigned short nZValue;
    for( int i = 0 ; i < nActualLength / 2 ; ++i) {

        nZValue = (((unsigned short)m_zdTableInfo.ZDTable[i * 2]) << 8) + m_zdTableInfo.ZDTable[i * 2 + 1];
        if (nZValue){
            m_zdTableInfo.nZNear = std::min(m_zdTableInfo.nZNear, nZValue);
            m_zdTableInfo.nZFar = std::max(m_zdTableInfo.nZFar, nZValue);
        }
    }

    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
    m_pVideoDeviceController->GetPreviewOptions()->SetZRange(m_zdTableInfo.nZNear, nZFar);
    m_pVideoDeviceController->AdjustZRange();

    return APC_OK;
}

CVideoDeviceModel::ZDTableInfo *CVideoDeviceModel::GetZDTableInfo()
{
    return &m_zdTableInfo;
}
#define DEBUG_RECTIFY_LOG (0)
int CVideoDeviceModel::GetRectifyLogData(int nDevIndex, int nRectifyLogIndex, eSPCtrl_RectLogData *pRectifyLogData, STREAM_TYPE depthType)
{
    if (!pRectifyLogData) return APC_NullPtr;

    if (nDevIndex >= (int)m_deviceInfo.size()) return APC_NullPtr;

    int ret;
    RETRY_APC_API(ret, APC_GetRectifyMatLogData(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                      m_deviceSelInfo[nDevIndex],
                                                      pRectifyLogData,
                                                      nRectifyLogIndex));
    if (DEBUG_RECTIFY_LOG) {
        fprintf(stderr, "rectify main CamMat2 %f %f %f %f %f %f %f %f %f \n",
                pRectifyLogData->CamMat2[0],
                pRectifyLogData->CamMat2[1],
                pRectifyLogData->CamMat2[2],
                pRectifyLogData->CamMat2[3],
                pRectifyLogData->CamMat2[4],
                pRectifyLogData->CamMat2[5],
                pRectifyLogData->CamMat2[6],
                pRectifyLogData->CamMat2[7],
                pRectifyLogData->CamMat2[8]
        );
        fprintf(stderr, "rectify main RotaMat %f %f %f %f %f %f %f %f %f \n",
                pRectifyLogData->RotaMat[0],
                pRectifyLogData->RotaMat[1],
                pRectifyLogData->RotaMat[2],
                pRectifyLogData->RotaMat[3],
                pRectifyLogData->RotaMat[4],
                pRectifyLogData->RotaMat[5],
                pRectifyLogData->RotaMat[6],
                pRectifyLogData->RotaMat[7],
                pRectifyLogData->RotaMat[8]
        );

        fprintf(stderr, "rectify main TranMat %f %f %f \n",
                pRectifyLogData->TranMat[0],
                pRectifyLogData->TranMat[1],
                pRectifyLogData->TranMat[2]
        );
    }

    return ret;
}

bool CVideoDeviceModel::IsStreamAvailable()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    return bColorStream || bDepthStream;
}

int CVideoDeviceModel::StartStreaming()
{
    if (!m_pVideoDeviceController->GetControlView()) return APC_NullPtr;
    if (!IsStreamAvailable()) return APC_NullPtr;

    m_mapSerialCountLast.clear();
    m_mapSerialCountDiff.clear();
    m_mapSerialCount2FrameCount.clear();

    m_coldResetTask = nullptr;

    PrepareOpenDevice();

    if (m_pVideoDeviceController->GetPreviewOptions()->IsPointCloudViewer()){
        StartFrameGrabber();
    }

    if (m_pVideoDeviceController->GetPreviewOptions()->IsIMUSyncWithFrame()){
        m_pVideoDeviceController->StartIMUSyncWithFrame();
    }

    PreparePointCloudInfo();

    int ret = OpenDevice();
    if (APC_OK != ret) {
        StopStreaming();
        return ret;
    }

    ChangeState(STREAMING);

    StartStreamingTask();

    return ret;
}

int CVideoDeviceModel::PrepareOpenDevice()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    SetIRValue(m_pVideoDeviceController->GetPreviewOptions()->GetIRLevel());
    if (bDepthStream){
        SetDepthDataType(TransformDepthDataType(GetDepthDataType()));
    }else{
        SetDepthDataType(TransformDepthDataType(APC_DEPTH_DATA_OFF_RAW));
    }

    m_imageData[STREAM_COLOR].nWidth = 0;
    m_imageData[STREAM_COLOR].nHeight = 0;
    m_imageData[STREAM_COLOR].bMJPG = false;
    m_imageData[STREAM_DEPTH].nWidth = 0;
    m_imageData[STREAM_DEPTH].nHeight = 0;

    if (bColorStream){
        std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(STREAM_COLOR);
        int index = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_COLOR);
        m_imageData[STREAM_COLOR].nWidth = streamInfo[index].nWidth;
        m_imageData[STREAM_COLOR].nHeight  = streamInfo[index].nHeight;
        m_imageData[STREAM_COLOR].bMJPG = streamInfo[index].bFormatMJPG;
        m_imageData[STREAM_COLOR].depthDataType = GetDepthDataType();
        m_imageData[STREAM_COLOR].imageDataType = m_imageData[STREAM_COLOR].bMJPG ?
                                                  APCImageType::COLOR_MJPG :
                                                  APCImageType::COLOR_YUY2;

        unsigned short nBytePerPixel = 2;
        unsigned int nBufferSize = m_imageData[STREAM_COLOR].nWidth * m_imageData[STREAM_COLOR].nHeight * nBytePerPixel;
        if (m_imageData[STREAM_COLOR].imageBuffer.size() != nBufferSize){
            m_imageData[STREAM_COLOR].imageBuffer.resize(nBufferSize);
        }
        memset(&m_imageData[STREAM_COLOR].imageBuffer[0], 0, sizeof(nBufferSize));
    }

    if (bDepthStream){
        std::vector<APC_STREAM_INFO> streamInfo = GetStreamInfoList(STREAM_DEPTH);
        int index = m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH);
        m_imageData[STREAM_DEPTH].nWidth = streamInfo[index].nWidth;
        m_imageData[STREAM_DEPTH].nHeight = streamInfo[index].nHeight;
        m_imageData[STREAM_DEPTH].depthDataType = GetDepthDataType();
        m_imageData[STREAM_DEPTH].imageDataType = GetDepthImageType();

        unsigned short nBytePerPixel = 2;
        unsigned int nBufferSize = m_imageData[STREAM_DEPTH].nWidth * m_imageData[STREAM_DEPTH].nHeight * nBytePerPixel;
        if (m_imageData[STREAM_DEPTH].imageBuffer.size() != nBufferSize){
            m_imageData[STREAM_DEPTH].imageBuffer.resize(nBufferSize);
        }
        memset(&m_imageData[STREAM_DEPTH].imageBuffer[0], 0, sizeof(nBufferSize));
    }

    if (InterleaveModeSupport() &&
        APC_OK != SetInterleaveModeEnable(IsInterleaveMode())){
        CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::VIDEO_INTERLEAVE_MODE, this);
        CThreadWorkerManage::GetInstance()->AddTask(pInfo);
    }

    return APC_OK;
}

CVideoDeviceModel::ImageData &CVideoDeviceModel::GetColorImageData()
{
    return m_imageData[STREAM_COLOR];
}

CVideoDeviceModel::ImageData &CVideoDeviceModel::GetDepthImageData()
{
    return m_imageData[STREAM_DEPTH];
}

CImageDataModel *CVideoDeviceModel::GetPreivewImageDataModel(STREAM_TYPE streamType)
{
    if (!m_pVideoDeviceController || !m_pVideoDeviceController->GetControlView()) return nullptr;

    return m_pVideoDeviceController->GetControlView()->GetPreviewImageData(streamType);
}

int CVideoDeviceModel::StartFrameGrabber()
{
    if(m_pFrameGrabber) {
        delete m_pFrameGrabber;
    }

    int nMaxPoolSize = 1;
    m_pFrameGrabber = new FrameGrabber(nMaxPoolSize, CVideoDeviceModel::FrameGrabberCallbackFn, this);
    return APC_OK;

}

int CVideoDeviceModel::PreparePointCloudInfo()
{
    GetRectifyLogData(0, m_pVideoDeviceController->GetPreviewOptions()->GetStreamIndex(STREAM_DEPTH), &m_rectifyLogData);
    GetPointCloudInfo(&m_rectifyLogData, m_pointCloudInfo, GetColorImageData(), GetDepthImageData());
    return APC_OK;
}

int CVideoDeviceModel::GetPointCloudInfo(eSPCtrl_RectLogData *pRectifyLogData, PointCloudInfo &pointCloudInfo,
                                         ImageData colorImageData, ImageData depthImageData)
{
    memset(&pointCloudInfo, 0, sizeof(PointCloudInfo));

    pointCloudInfo.wDepthType = depthImageData.depthDataType;

    const float ratio_Mat = (float)depthImageData.nHeight / pRectifyLogData->OutImgHeight;
    const float baseline  = 1.0f / pRectifyLogData->ReProjectMat[14];
    const float diff      = pRectifyLogData->ReProjectMat[15] * ratio_Mat;

    pointCloudInfo.centerX = -1.0f * pRectifyLogData->ReProjectMat[3] * ratio_Mat;
    pointCloudInfo.centerY = -1.0f * pRectifyLogData->ReProjectMat[7] * ratio_Mat;
    pointCloudInfo.focalLength = pRectifyLogData->ReProjectMat[11] * ratio_Mat;

    switch (depthImageData.imageDataType){
        case APCImageType::DEPTH_14BITS: pointCloudInfo.disparity_len = 0; break;
        case APCImageType::DEPTH_11BITS:
        {
            pointCloudInfo.disparity_len = 2048;
            for(int i = 0 ; i < pointCloudInfo.disparity_len ; ++i){
                pointCloudInfo.disparityToW[i] = ( i * ratio_Mat / 8.0f ) / baseline + diff;
            }
            break;
        }
        default:
            pointCloudInfo.disparity_len = 256;
            for(int i = 0 ; i < pointCloudInfo.disparity_len ; ++i){
                pointCloudInfo.disparityToW[i] = (i * ratio_Mat) / baseline + diff;
            }
            break;
    }

    return APC_OK;
}

int CVideoDeviceModel::OpenDevice()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    int nFPS = bColorStream ? m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_COLOR) :
               bDepthStream ? m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_DEPTH) :
               0;
#if !defined(_MIPI_CAMERA_)
    int ret = APC_OpenDevice2(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                  m_deviceSelInfo[0],
                                  m_imageData[STREAM_COLOR].nWidth, m_imageData[STREAM_COLOR].nHeight, m_imageData[STREAM_COLOR].bMJPG,
                                  m_imageData[STREAM_DEPTH].nWidth, m_imageData[STREAM_DEPTH].nHeight,
                                  DEPTH_IMG_NON_TRANSFER,
                                  true, nullptr,
                                  &nFPS,
                                  IMAGE_SN_SYNC);
#else
    int ret = APC_OpenDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                  m_deviceSelInfo[0],
                                  m_imageData[STREAM_COLOR].nWidth, m_imageData[STREAM_COLOR].nHeight, m_imageData[STREAM_COLOR].bMJPG,
                                  m_imageData[STREAM_DEPTH].nWidth, m_imageData[STREAM_DEPTH].nHeight,
                                  DEPTH_IMG_NON_TRANSFER,
                                  true, nullptr,
                                  &nFPS,
                                  IMAGE_SN_SYNC);
#endif
    if(APC_OK == ret){
        if(bColorStream){
            m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_COLOR, nFPS);
            if(bDepthStream) m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_DEPTH, nFPS);
        }else{
            m_pVideoDeviceController->GetPreviewOptions()->SetStreamFPS(STREAM_DEPTH, nFPS);
        }
    }

    return ret;
}

int CVideoDeviceModel::StartStreamingTask()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

#if !defined(_MIPI_CAMERA_)
    if (bColorStream){
        CreateStreamTask(STREAM_COLOR);
    }

    if (bDepthStream){
        auto AdjustRealDepthWidth = [](int &nWidth, APCImageType::Value type){
            if (APCImageType::DEPTH_8BITS == type){
                nWidth *= 2;
            }
        };
        AdjustRealDepthWidth(m_imageData[STREAM_DEPTH].nWidth, m_imageData[STREAM_DEPTH].imageDataType);

        CreateStreamTask(STREAM_DEPTH);
    }
#else
    if (bColorStream && bDepthStream){
        CreateStreamTask(STREAM_BOTH);
    } else if (bColorStream){
        CreateStreamTask(STREAM_COLOR);
    }
#endif

    if (InterleaveModeSupport()){

        if (IsInterleaveMode()){
            m_bPrevLowLightValue = m_cameraPropertyModel[0]->GetCameraProperty(CCameraPropertyModel::LOW_LIGHT_COMPENSATION).nValue;
            m_cameraPropertyModel[0]->SetCameraPropertyValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, 0);
            m_cameraPropertyModel[0]->SetCameraPropertySupport(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, false);
        }
    }


    return APC_OK;
}

int CVideoDeviceModel::StopStreaming(bool bRestart)
{
    ChangeState(OPENED);

    if (m_pVideoDeviceController->GetPreviewOptions()->IsIMUSyncWithFrame()){
        m_pVideoDeviceController->StopIMUSyncWithFrame();
    }

    StopStreamingTask();
    if (m_pVideoDeviceController->GetPreviewOptions()->IsPointCloudViewer()){
        StopFrameGrabber();
    }

    if (!bRestart) ClosePreviewView();

    SetIRValue(0);

    CloseDevice();

    return APC_OK;
}

int CVideoDeviceModel::StopStreamingTask()
{
    for (CTaskInfo *pTask : m_taskInfoStorage){
        CThreadWorkerManage::GetInstance()->RemoveTask(pTask);
    }

    m_taskInfoStorage.clear();

    if (InterleaveModeSupport()){

        if(IsInterleaveMode()){
            m_cameraPropertyModel[0]->SetCameraPropertySupport(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, true);
            m_cameraPropertyModel[0]->SetCameraPropertyValue(CCameraPropertyModel::LOW_LIGHT_COMPENSATION, m_bPrevLowLightValue);
        }

        SetInterleaveModeEnable(false);
    }

    return APC_OK;
}

int CVideoDeviceModel::CloseDevice()
{
    return APC_CloseDevice(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                  m_deviceSelInfo[0]);
}

int CVideoDeviceModel::ClosePreviewView()
{
    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    if (bColorStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_COLOR);
    }

    if (bDepthStream){
        m_pVideoDeviceController->GetControlView()->ClosePreviewView(STREAM_DEPTH);
    }

    if (m_pVideoDeviceController->GetPreviewOptions()->IsPointCloudViewer()){
        m_pVideoDeviceController->GetControlView()->ClosePointCloud();
    }

    return APC_OK;
}

int CVideoDeviceModel::StopFrameGrabber()
{
    if(m_pFrameGrabber) {
        delete m_pFrameGrabber;
        m_pFrameGrabber = nullptr;
    }
    return APC_OK;
}

int CVideoDeviceModel::AdjustRegister()
{
    if(GetState() != STREAMING) return APC_OK;

    if (!IsHWPP()){
        SetHWPP(true);
    }
    int ret =  RegisterSettings::DM_Quality_Register_Setting(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                             m_deviceSelInfo[0],
                                                             m_deviceInfo[0].deviceInfomation.wPID);

    m_pVideoDeviceController->GetControlView()->UpdateUI();

    return ret;
}

int CVideoDeviceModel::ConfigDepthFilter()
{
    if (!m_pVideoDeviceController) return APC_OK;

    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();

    pDepthFilterOptions->EnableDepthFilter(false);
    pDepthFilterOptions->SetState(DepthFilterOptions::MIN);

    return APC_OK;
}

int CVideoDeviceModel::DoImageGrabber(CTaskInfo::TYPE type)
{
    STREAM_TYPE streamType;
    switch (type){
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR:
            streamType = STREAM_COLOR;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_SLAVE:
            streamType = STREAM_COLOR_SLAVE;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH:
            streamType = STREAM_DEPTH;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_60mm:
            streamType = STREAM_DEPTH_60mm;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_150mm:
            streamType = STREAM_DEPTH_150mm;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR:
            streamType = STREAM_KOLOR;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR_SLAVE:
            streamType = STREAM_KOLOR_SLAVE;
            break;
        case CTaskInfo::GRABBER_VIDEO_IMAGE_TRACK:
            streamType = STREAM_TRACK;
            break;
        //+[Thermal device]
        case CTaskInfo::GRABBER_VIDEO_IMAGE_THERMAL:
            streamType = STREAM_THERMAL;
            break;
        //-[Thermal device]
        case CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_WITH_DEPTH:
            streamType = STREAM_BOTH;
            break;
        default:
            return APC_NotSupport;
    }

    int ret = GetImage(streamType);
    HandleGetImageResult(streamType, ret);
    if(APC_OK == ret && m_pVideoDeviceController->GetPreviewOptions()->IsPointCloudViewer() &&
       m_pFrameGrabber){
        UpdateFrameGrabberData(streamType);
    }

    return ret;
}

int CVideoDeviceModel::GetImage(STREAM_TYPE type)
{
    int ret;
    switch (type){
        case STREAM_COLOR:
#if !defined(_MIPI_CAMERA_)
            ret = GetColorImage();
#else
			ret = Get2Image(type);
#endif
            break;
        case STREAM_DEPTH:
            ret = GetDepthImage();
            break;
        case STREAM_BOTH:
            ret = Get2Image(type);
            break;
        default:
            return APC_NotSupport;
    }

    return ret;
}

#if 0
int max_calc_frame_count = 30;
FILE *color_fp = NULL;
#endif
int CVideoDeviceModel::Get2Image(STREAM_TYPE type)
{
    QMutexLocker locker(&m_streamMutex[STREAM_COLOR]);
    unsigned long int nColorImageSize = 0, nDepthImageSize = 0;
    int nSerial = EOF;

    int ret = APC_Get2Image(CEYSDDeviceManager::GetInstance()->GetEYSD(),
							m_deviceSelInfo[0],
                            &m_imageData[STREAM_COLOR].imageBuffer[0],
                            &m_imageData[STREAM_DEPTH].imageBuffer[0],
                            &nColorImageSize,
                            &nDepthImageSize,
                            &nSerial,
                            &nSerial,
                            m_imageData[STREAM_COLOR].depthDataType);
    if (APC_OK != ret) return ret;
#if 0
    if (max_calc_frame_count == 30) {
        color_fp = fopen("DMPreview.yuv", "wb");
        fseek(color_fp, 0, SEEK_SET);
    }
    if (max_calc_frame_count > 0) {
        fwrite((uint8_t*)&m_imageData[STREAM_COLOR].imageBuffer[0], sizeof(uint8_t), nColorImageSize, color_fp);
        max_calc_frame_count--;
    } else if (max_calc_frame_count == 0) {
        fclose(color_fp);
        max_calc_frame_count--;
    }
#endif

    ret = ProcessImage(STREAM_COLOR, nColorImageSize, nSerial);
    if (APC_OK != ret) return ret;

    if (type == STREAM_BOTH) { 
        ret = ProcessImage(STREAM_DEPTH, nDepthImageSize, nSerial);
        if (APC_OK != ret) return ret;
    }

    return ret;
}

int CVideoDeviceModel::GetColorImage()
{
    QMutexLocker locker(&m_streamMutex[STREAM_COLOR]);
    unsigned long int nImageSize = 0;
    int nSerial = EOF;
#if 1
    int ret = APC_GetColorImage(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[0],
                                    &m_imageData[STREAM_COLOR].imageBuffer[0],
                                    &nImageSize,
                                    &nSerial,
                                    m_imageData[STREAM_COLOR].depthDataType);

    if (APC_OK != ret)  {
        if (ret == APC_DEVICE_TIMEOUT) {
            /*
                When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
            */
            printf("[%s][%d]Getting image is timeout!!\n",  __func__, __LINE__);
            usleep(1 * 1000);
        }
        return ret;
    }
#else
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    static int64_t prv_tv_sec = 0;
    static int64_t prv_tv_usec = 0;
    static uint64_t frame_rate_count = 0;
    unsigned int max_calc_frame_count = 50;

    int ret = APC_GetColorImageWithTimestamp(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                             m_deviceSelInfo[0],
                                             &m_imageData[STREAM_COLOR].imageBuffer[0],
                                             &nImageSize,
                                             &nSerial,
                                             m_imageData[STREAM_COLOR].depthDataType,
                                             &cur_tv_sec, &cur_tv_usec);
    if (APC_OK != ret)  {
        if (ret == APC_DEVICE_TIMEOUT) {
            /*
                When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
            */
            printf("[%s][%d]Getting image is timeout!!\n",  __func__, __LINE__);
            usleep(1 * 1000);
        }
        return ret;
    }

    if (frame_rate_count == 0) {
        prv_tv_sec  = cur_tv_sec;
        prv_tv_usec = cur_tv_usec;
    }

    if (frame_rate_count == (max_calc_frame_count -1)) {
        float fltotal_time = 0.0;
        fltotal_time = ((cur_tv_sec - prv_tv_sec)*1000000+cur_tv_usec)-prv_tv_usec;
        printf("[%s] %lu usec per %ufs (fps = %6f)\n", __func__,
            (unsigned long)fltotal_time, max_calc_frame_count,(1000000*max_calc_frame_count)/fltotal_time);
        frame_rate_count = 0;
    } else {
        frame_rate_count++;
    }
#endif

    return ProcessImage(STREAM_COLOR, nImageSize, nSerial);
}

int CVideoDeviceModel::FirstSuccessGetImageCallback(STREAM_TYPE type)
{
    QMutexLocker locker(&m_streamMutex[type]);
    if (STREAM_DEPTH == type ||
        STREAM_DEPTH_30mm == type){
        CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::VIDEO_QUALITY_REGISTER_SETTING, this);
        CThreadWorkerManage::GetInstance()->AddTask(pInfo);
    }

    return APC_OK;
}

int CVideoDeviceModel::GetDepthImage()
{
    QMutexLocker locker(&m_streamMutex[STREAM_DEPTH]);
    unsigned long int nImageSize = 0;
    int nSerial = EOF;
#if 1
    int ret = APC_GetDepthImage(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                    m_deviceSelInfo[0],
                                    &m_imageData[STREAM_DEPTH].imageBuffer[0],
                                    &nImageSize,
                                    &nSerial,
                                    m_imageData[STREAM_DEPTH].depthDataType);

    if (APC_OK != ret)  {
        if (ret == APC_DEVICE_TIMEOUT) {
            /*
                When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
            */
            printf("[%s][%d]Getting image is timeout!!\n",  __func__, __LINE__);
            usleep(1 * 1000);
        }
        return ret;
    }
#else
    int64_t cur_tv_sec = 0;
    int64_t cur_tv_usec = 0;
    static int64_t prv_tv_sec = 0;
    static int64_t prv_tv_usec = 0;
    static uint64_t frame_rate_count = 0;
    unsigned int max_calc_frame_count = 50;

    int ret = APC_GetDepthImageWithTimestamp(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                            m_deviceSelInfo[0],
                                            &m_imageData[STREAM_DEPTH].imageBuffer[0],
                                            &nImageSize,
                                            &nSerial,
                                            m_imageData[STREAM_DEPTH].depthDataType,
                                            &cur_tv_sec, &cur_tv_usec);
    if (APC_OK != ret)  {
        if (ret == APC_DEVICE_TIMEOUT) {
            /*
                When return the APC_DEVICE_TIMEOUT from SDK, this mean the device may busy state.
                So, we must let process entry the sleep state. Otherwise, the CPU usage will very high.
            */
            printf("[%s][%d]Getting image is timeout!!\n",  __func__, __LINE__);
            usleep(1 * 1000);
        }
        return ret;
    }

    if (frame_rate_count == 0) {
        prv_tv_sec  = cur_tv_sec;
        prv_tv_usec = cur_tv_usec;
    }

    if (frame_rate_count == (max_calc_frame_count -1)) {
        float fltotal_time = 0.0;
        fltotal_time = ((cur_tv_sec - prv_tv_sec)*1000000+cur_tv_usec)-prv_tv_usec;
        printf("[%s] %lu usec per %ufs (fps = %6f)\n", __func__,
               (unsigned long)fltotal_time, max_calc_frame_count,(1000000*max_calc_frame_count)/fltotal_time);
        frame_rate_count = 0;
    } else {
        frame_rate_count++;
    }
#endif

    return ProcessImage(STREAM_DEPTH, nImageSize, nSerial);
}

int CVideoDeviceModel::HandleGetImageResult(STREAM_TYPE type, int getImageResult)
{
    if (STREAMING != GetState()) return getImageResult;

    if (APC_OK == getImageResult){
        if(GetColdResetThresholdMs(type) == FirstOpenDeviceColdeRestThresholdMs()){
            FirstSuccessGetImageCallback(type);
            SetColdResetThresholdMs(type, OpenDeviceColdeRestThresholdMs());
        }

        SetLastestSuccessTime(type, QTime::currentTime());

    }else{
        QTime lastestSuccessTime = GetLastestSuccessTime(type);
        if (lastestSuccessTime.msecsTo(QTime::currentTime()) > GetColdResetThresholdMs(type) ||
            lastestSuccessTime.msecsTo(QTime::currentTime()) < 0){
            static QMutex mutex;
            QMutexLocker locker(&mutex);
            if(!m_coldResetTask){
                SetColdResetStartTime(QTime::currentTime());
                m_coldResetTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::VIDEO_COLD_RESET, this);
                CThreadWorkerManage::GetInstance()->AddTask(m_coldResetTask);
            }
        }
    }

    return getImageResult;
}

int CVideoDeviceModel::ProcessImageCallback(STREAM_TYPE streamType,
                                            int nImageSize, int nSerialNumber)
{
    if (IsInterleaveMode()){
        if (STREAM_COLOR == streamType){
            if (1 == nSerialNumber % 2) return APC_OK;
            if (m_nLastInterLeaveColorSerial + 2 != nSerialNumber){
                m_nLastInterLeaveColorSerial = nSerialNumber;
                return APC_OK;
            }
            m_nLastInterLeaveColorSerial = nSerialNumber;
        }

        if (STREAM_DEPTH == streamType){
            if (0 == nSerialNumber % 2) return APC_OK;
            if (m_nLastInterLeaveDepthSerial + 2 != nSerialNumber){
                m_nLastInterLeaveDepthSerial = nSerialNumber;
                return APC_OK;
            }
            m_nLastInterLeaveDepthSerial = nSerialNumber;
        }
    }

    CEYSDUIView *pView = m_pVideoDeviceController->GetControlView();
    if (!pView) return APC_OK;

    if (IsIMUSyncWithFrame()){
        return CFrameSyncManager::GetInstance()->SyncImageCallback(this,
                                                            m_imageData[streamType].imageDataType, streamType,
                                                            &m_imageData[streamType].imageBuffer[0],
                                                            nImageSize,
                                                            m_imageData[streamType].nWidth, m_imageData[streamType].nHeight,
                                                            nSerialNumber, nullptr);
    }else{
        return pView->ImageCallback(m_imageData[streamType].imageDataType, streamType,
                                    &m_imageData[streamType].imageBuffer[0],
                                    nImageSize,
                                    m_imageData[streamType].nWidth, m_imageData[streamType].nHeight,
                                    nSerialNumber, nullptr);
    }
}

int CVideoDeviceModel::ProcessImage(STREAM_TYPE streamType,
                                    int nImageSize, int nSerialNumber)
{
    SerialCountToFrameCount(streamType ,nSerialNumber);
    return ProcessImageCallback(streamType, nImageSize, nSerialNumber);
}

int CVideoDeviceModel::UpdateFrameGrabberData(STREAM_TYPE streamType)
{
    const bool bIsDpethOutput = PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_DEPTH == m_pVideoDeviceController->GetPreviewOptions()->GetPointCloudViewOutputFormat();
    const bool bIsRGBStreamEnabled = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR) ||
            m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_KOLOR);

    switch(streamType){
        case STREAM_COLOR:
        case STREAM_KOLOR:
        {
            CImageDataModel *pColorImageData = GetPreivewImageDataModel(streamType);
            if (!pColorImageData) return APC_NullPtr;

            QMutexLocker locker(&pColorImageData->GetDataMutex());

            BYTE *pBuffer = &pColorImageData->GetRGBData()[0];
            int nBytesPerPixelColor = 3;
            if (bIsDpethOutput){
                CImageDataModel *pDepthImageData = GetPreivewImageDataModel(STREAM_DEPTH);
                if (pDepthImageData &&
                    pDepthImageData->GetWidth() > 0 && pDepthImageData->GetHeight() > 0){
                    if (pDepthImageData->GetWidth() == pColorImageData->GetWidth() &&
                        pDepthImageData->GetHeight() == pColorImageData->GetHeight()){
                        pBuffer = &pDepthImageData->GetRGBData()[0];
                    }else{
                        if (m_imageData[STREAM_RESERVED].imageBuffer.size() != pColorImageData->GetWidth() * pColorImageData->GetHeight() * 3){
                            m_imageData[STREAM_RESERVED].imageBuffer.resize(pColorImageData->GetWidth() * pColorImageData->GetHeight() * 3);
                        }
                        PlyWriter::resampleImage(pDepthImageData->GetWidth(), pDepthImageData->GetHeight(), &pDepthImageData->GetRGBData()[0],
                                                 pColorImageData->GetWidth(), pColorImageData->GetHeight(), &m_imageData[STREAM_RESERVED].imageBuffer[0],
                                                 nBytesPerPixelColor);
                        pBuffer = &m_imageData[STREAM_RESERVED].imageBuffer[0];
                    }
                }
            }

            m_pFrameGrabber->SetFrameFormat(FrameGrabber::FRAME_POOL_INDEX_COLOR,
                                            pColorImageData->GetWidth(), pColorImageData->GetHeight(),
                                            nBytesPerPixelColor);
            m_pFrameGrabber->UpdateFrameData(FrameGrabber::FRAME_POOL_INDEX_COLOR,
                                             pColorImageData->GetSerialNumber(),
                                             pBuffer,
                                             pColorImageData->GetRGBData().size());
            break;
        }
        case STREAM_DEPTH:
        {
            CImageDataModel *pDepthData = GetPreivewImageDataModel(STREAM_DEPTH);
            if (!pDepthData) return APC_NullPtr;

            int nBytesPerPixelDepth = 2;
            if(APCImageType::DEPTH_8BITS == m_imageData[STREAM_DEPTH].imageDataType) nBytesPerPixelDepth = 1;
            m_pFrameGrabber->SetFrameFormat(FrameGrabber::FRAME_POOL_INDEX_DEPTH,
                                            pDepthData->GetWidth(), pDepthData->GetHeight(),
                                            nBytesPerPixelDepth);
            if (bIsDpethOutput && !bIsRGBStreamEnabled) { // Only Depth without doing case STREAM_COLOR
                m_pFrameGrabber->SetFrameFormat(FrameGrabber::FRAME_POOL_INDEX_COLOR,
                                                pDepthData->GetWidth(), pDepthData->GetHeight(),
                                                3);
                m_pFrameGrabber->UpdateFrameData(FrameGrabber::FRAME_POOL_INDEX_COLOR,
                                                 pDepthData->GetSerialNumber(),
                                                 &pDepthData->GetRGBData()[0],
                                                 pDepthData->GetRGBData().size());
            }
            m_pFrameGrabber->UpdateFrameData(FrameGrabber::FRAME_POOL_INDEX_DEPTH,
                                             pDepthData->GetSerialNumber(),
                                             &pDepthData->GetRawData()[0],
                                             pDepthData->GetRawData().size());
            break;
        }
        default: return APC_NotSupport;
    }

    return APC_OK;
}

int CVideoDeviceModel::CreateStreamTask(STREAM_TYPE type)
{
    SetColdResetThresholdMs(type, FirstOpenDeviceColdeRestThresholdMs());
    SetLastestSuccessTime(type, QTime::currentTime());

    CTaskInfo::TYPE taskType;
    switch (type){
        case STREAM_COLOR: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR; break;
        case STREAM_COLOR_SLAVE: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_SLAVE; break;
        case STREAM_DEPTH: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH; break;
        case STREAM_DEPTH_60mm: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_60mm; break;
        case STREAM_DEPTH_150mm: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_DEPTH_150mm; break;
        case STREAM_KOLOR: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR; break;
        case STREAM_KOLOR_SLAVE: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_KOLOR_SLAVE; break;
        case STREAM_TRACK: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_TRACK; break;
        //+[Thermal device]
        case STREAM_THERMAL: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_THERMAL; break;
        //-[Thermal device]
        case STREAM_BOTH: taskType = CTaskInfo::GRABBER_VIDEO_IMAGE_COLOR_WITH_DEPTH; break;
        default: return APC_NotSupport;
    }

    CTaskInfo *pInfo = CTaskInfoManager::GetInstance()->RequestTaskInfo(taskType, this);
    CThreadWorkerManage::GetInstance()->AddTask(pInfo);
    m_taskInfoStorage.push_back(std::move(pInfo));

    return APC_OK;
}

void CVideoDeviceModel::SerialCountToFrameCount(STREAM_TYPE streamType, int &nSerialNumber)
{
    QMutexLocker locker(&m_serialCountMutex);

    auto CheckSerialCountRecycle = [ & ] ( const STREAM_TYPE streamType , const int nSerialCount)
    {
        if (!m_mapSerialCountLast.count( streamType )) return;
        if ( nSerialCount < m_mapSerialCountLast[ streamType ] ) m_mapSerialCountLast[ streamType ] = m_mapSerialCountLast[ streamType ] - 65535;
    };

    auto ResetSerialCount = [ & ] ()
    {
        m_mapSerialCountLast.clear();
        m_mapSerialCountDiff.clear();
        m_mapSerialCount2FrameCount.clear();
    };

    if ( 0 == m_mapSerialCountDiff.count( streamType ))
    {
        if ( 1 == nSerialNumber ) // frame-count
        {
            m_mapSerialCountDiff[ streamType ] = 1;

            return;
        }
        else if ( 0 != m_mapSerialCountLast.count( streamType ))
        {
            CheckSerialCountRecycle( streamType , nSerialNumber);

            const int iDiff = nSerialNumber - m_mapSerialCountLast[ streamType ];

            if ( iDiff ) {
                m_mapSerialCountDiff[ streamType ] = nSerialNumber - m_mapSerialCountLast[ streamType ];
            }
        }else {
            m_mapSerialCount2FrameCount[ streamType ] = 0;
        }
    }

    if (m_mapSerialCountDiff.count( streamType ) > 0){

        if ( 1 == m_mapSerialCountDiff[ streamType ] ) // frame-count
        {
            return;
        }
        else // serial-count
        {
            CheckSerialCountRecycle( streamType, nSerialNumber );

            m_mapSerialCount2FrameCount[ streamType ]++;
            int nDiff = (nSerialNumber - m_mapSerialCountLast[ streamType ]) - m_mapSerialCountDiff[ streamType ];
            if ( nDiff >= SERIAL_THRESHOLD  ||  nDiff <= -SERIAL_THRESHOLD) { // reset count if count not match
                nSerialNumber = m_mapSerialCount2FrameCount[ streamType ];
                ResetSerialCount();
                return;
            }
        }
    }

    m_mapSerialCountLast[ streamType ] = nSerialNumber;

    if (m_mapSerialCount2FrameCount.count(streamType) > 0){
        nSerialNumber = m_mapSerialCount2FrameCount[ streamType ];
    }

}

std::vector<CloudPoint> CVideoDeviceModel::GeneratePointCloud(
        STREAM_TYPE depthType,
        std::vector<BYTE> &depthData, unsigned short nDepthWidth, unsigned short nDepthHeight,
        std::vector<BYTE> &colorData, unsigned short nColorWidth, unsigned short nColorHeight,
        bool bEnableFilter)
{
    APCImageType::Value depthImageType = APCImageType::DepthDataTypeToDepthImageType(GetDepthDataType());

    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);

    bool bUsePlyFilter = bEnableFilter &&
                         m_pVideoDeviceController->GetPreviewOptions()->IsPlyFilter() &&
                         PlyFilterSupprot();

    std::vector<float> imgFloatBufOut;
    if (bUsePlyFilter){
        if(APC_OK != PlyFilterTransform(depthData, colorData,
                                            nDepthWidth, nDepthHeight,
                                            nColorWidth, nColorHeight,
                                            imgFloatBufOut, GetRectifyLogData(depthType),
                                            depthImageType)){
            return {};
        }
    }

    return GeneratePointCloud(depthData, colorData,
                              nDepthWidth, nDepthHeight,
                              nColorWidth, nColorHeight,
                              GetRectifyLogData(depthType),
                              depthImageType,
                              nZNear, nZFar,
                              bUsePlyFilter, imgFloatBufOut);
}

std::vector<CloudPoint> CVideoDeviceModel::GeneratePointCloud(std::vector<unsigned char> &depthData,
                                                              std::vector<unsigned char> &colorData,
                                                              unsigned short nDepthWidth,
                                                              unsigned short nDepthHeight,
                                                              unsigned short nColorWidth,
                                                              unsigned short nColorHeight,
                                                              eSPCtrl_RectLogData rectifyLogData,
                                                              APCImageType::Value depthImageType,
                                                              int nZNear, int nZFar,
                                                              bool bUsePlyFilter, std::vector<float> imgFloatBufOut)
{
    std::vector<CloudPoint> cloudPoints;
    if(bUsePlyFilter && !imgFloatBufOut.empty()){
        PlyWriter::EYSDFrameTo3D_PlyFilterFloat(nDepthWidth, nDepthHeight,
                                                 imgFloatBufOut,
                                                 nColorWidth, nColorHeight,
                                                 colorData,
                                                 &rectifyLogData, depthImageType,
                                                 cloudPoints,
                                                 true, nZNear, nZFar,
                                                 true, false, 1.0f);
    }else{
        PlyWriter::EYSDFrameTo3D(nDepthWidth, nDepthHeight, depthData,
                                  nColorWidth, nColorHeight, colorData,
                                  &rectifyLogData, depthImageType,
                                  cloudPoints,
                                  true, nZNear, nZFar,
                                  true, false, 1.0f);
    }

    return cloudPoints;
}

int CVideoDeviceModel::PlyFilterTransform(std::vector<unsigned char> &depthData,
                                          std::vector<unsigned char> &colorData,
                                          unsigned short &nDepthWidth,
                                          unsigned short &nDepthHeight,
                                          unsigned short &nColorWidth,
                                          unsigned short &nColorHeight,
                                          std::vector<float> &imgFloatBufOut,
                                          eSPCtrl_RectLogData &rectifyLogData,
                                          APCImageType::Value depthImageType)
{
    float ratio = (float)rectifyLogData.OutImgHeight / nDepthHeight;
    if (ratio != 1.0f) {
        int resampleWidthDepth = nDepthWidth * ratio;
        int resampleHeightDepth = nDepthHeight * ratio;

        int bufSize = resampleWidthDepth * resampleHeightDepth * 2;
        std::vector<unsigned char> dArrayResized(bufSize);
        if ( depthImageType == APCImageType::DEPTH_8BITS )
            PlyWriter::MonoBilinearFineScaler( &depthData[0], &dArrayResized[0], nDepthWidth, nDepthHeight, resampleWidthDepth, resampleHeightDepth, 1);
        else
            PlyWriter::MonoBilinearFineScaler_short( (unsigned short*)&depthData[0], (unsigned short*)&dArrayResized[0], nDepthWidth, nDepthHeight, resampleWidthDepth, resampleHeightDepth, 1 );

        depthData.resize(bufSize);
        depthData.assign(dArrayResized.begin(), dArrayResized.end());

        nDepthWidth = resampleWidthDepth;
        nDepthHeight = resampleHeightDepth;
    }

    switch (depthImageType){
        case APCImageType::DEPTH_8BITS:
        {
            //D8 TO D11 IMAGE +
            std::vector< BYTE > bufDepthTmpout;
            bufDepthTmpout.resize( depthData.size() * 2 );

            WORD* pDepthOut = ( WORD* )bufDepthTmpout.data();

            for ( size_t i = 0; i != depthData.size(); i++ )
            {
                pDepthOut[ i ] = ( ( WORD )depthData[ i ] ) << 3;
            }

            //D8 TO D11 IMAGE -
            PlyFilter::CF_FILTER(bufDepthTmpout, colorData,
                                 nDepthWidth, nDepthHeight,
                                 nColorWidth, nColorHeight,
                                 imgFloatBufOut,
                                 &rectifyLogData);
            break;
        }
        case APCImageType::DEPTH_11BITS:
        {
            PlyFilter::UnavailableDisparityCancellation(depthData, nDepthWidth, nDepthHeight, 16383);
            PlyFilter::CF_FILTER(depthData, colorData,
                                 nDepthWidth, nDepthHeight,
                                 nColorWidth, nColorHeight,
                                 imgFloatBufOut,
                                 &rectifyLogData);
            break;
        }
        case APCImageType::DEPTH_14BITS:
        {
            PlyFilter::CF_FILTER_Z14(depthData, colorData,
                                     nDepthWidth, nDepthHeight,
                                     nColorWidth, nColorHeight,
                                     imgFloatBufOut,
                                     &rectifyLogData);
            break;
        }
        default:
            break;
    }

    if (imgFloatBufOut.empty()){
        return APC_NullPtr;
    }

    return APC_OK;
}

void CVideoDeviceModel::FrameGrabberCallbackFn(std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
                                               std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
                                               int serialNumber, void* pParam)
{
    CVideoDeviceModel *pModel = static_cast<CVideoDeviceModel *>(pParam);
    pModel->ProcessFrameGrabberCallback(bufDepth, widthDepth, heightDepth,
                                        bufColor, widthColor, heightColor,
                                        serialNumber);
}

int CVideoDeviceModel::ProcessFrameGrabberCallback(std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
                                                   std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
                                                   int serialNumber)
{


    size_t nPointCloudSize = widthDepth * heightDepth * 3;
    if (m_pointCloudDepth.size() != nPointCloudSize) m_pointCloudDepth.resize(nPointCloudSize);
    else                                             std::fill(m_pointCloudDepth.begin(), m_pointCloudDepth.end(), 0.0f);

    if (m_pointCloudColor.size() != nPointCloudSize) m_pointCloudColor.resize(nPointCloudSize);
    else                                             std::fill(m_pointCloudColor.begin(), m_pointCloudColor.end(), 0);

    FrameGrabberDataTransform(bufDepth, widthDepth, heightDepth,
                              bufColor, widthColor, heightColor,
                              serialNumber);

    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
    float fZNear = (nZNear * 1.0f) > 0 ? nZNear : 0.1f;
    float fZFar = (nZFar * 1.0f) > 0? nZFar : 1000.0f;
    unsigned char *pImgColor = bufColor.empty() ? nullptr : &bufColor[0];
    if(APC_OK == APC_GetPointCloud(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                           m_deviceSelInfo[0],
                                           pImgColor, widthColor, heightColor,
                                           &bufDepth[0], widthDepth, heightDepth,
                                           &m_pointCloudInfo,
                                           &m_pointCloudColor[0],
                                           &m_pointCloudDepth[0],
                                           fZNear, fZFar)){
        m_pVideoDeviceController->GetControlView()->PointCloudCallback(m_pointCloudDepth, m_pointCloudColor);
    }
    return APC_OK;
}

int CVideoDeviceModel::InitIMU()
{
    if (!IMUSupport()) return APC_NotSupport;

    if (m_pIMUModel) delete m_pIMUModel;

    for (CIMUModel::INFO info : GetIMUInfo()){
        m_pIMUModel = new CIMUModel(info, this);

        if (m_pIMUModel->IsValid()) break;

        delete m_pIMUModel;
        m_pIMUModel = nullptr;
    }

    if (!m_pIMUModel) {
        m_pIMUModel = new CIMUModel({0, 0, CIMUModel::IMU_UNKNOWN},
                                    this);
    }

    return APC_OK;
}

void CVideoDeviceModel::SetIMUSyncWithFrame(bool bSync)
{
    if (!m_pVideoDeviceController) return;
    if (bSync == m_pVideoDeviceController->GetPreviewOptions()->IsIMUSyncWithFrame()) return;

    if (bSync){
        m_pVideoDeviceController->StartIMUSyncWithFrame();
    }else{
        m_pVideoDeviceController->StopIMUSyncWithFrame();
    }

    m_pVideoDeviceController->GetPreviewOptions()->SetIMUSyncWithFrame(bSync);
}

bool CVideoDeviceModel::IsIMUSyncWithFrame()
{
    if (!IMUSupport()) return false;
    if (!GetIMUModel()) return false;
    if (!m_pVideoDeviceController) return false;

    return m_pVideoDeviceController->GetPreviewOptions()->IsIMUSyncWithFrame();
}

bool CVideoDeviceModel::IsInterleaveMode()
{
    if (!InterleaveModeSupport()) return false;

    bool bIsInterLeave = false;

    bool bColorStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_COLOR);
    bool bDepthStream = m_pVideoDeviceController->GetPreviewOptions()->IsStreamEnable(STREAM_DEPTH);

    if (!bColorStream || !bDepthStream) {
        return bIsInterLeave;
    }

    int nFPS = m_pVideoDeviceController->GetPreviewOptions()->GetStreamFPS(STREAM_COLOR);

    for (int nInterleaveFPS : GetInterleaveModeFPS()){
        if (nInterleaveFPS == nFPS){
            bIsInterLeave = true;
            break;
        }
    }

    return bIsInterLeave;
}

int CVideoDeviceModel::AdjustInterleaveModeState()
{
    if (!InterleaveModeSupport()) return APC_NotSupport;
    if(STREAMING != GetState()) return APC_OK;

    return SetInterleaveModeEnable(IsInterleaveMode());
}

std::vector<int> CVideoDeviceModel::GetInterleaveModeFPS()
{
    if (!InterleaveModeSupport()) return {};

    ModeConfig::MODE_CONFIG modeConfig = m_pVideoDeviceController->GetModeConfigOptions()->GetCurrentModeInfo();

    return { modeConfig.iInterLeaveModeFPS };
}

int CVideoDeviceModel::SetInterleaveModeEnable(bool bEnable)
{
    if (!InterleaveModeSupport()) return APC_NotSupport;

    int ret;
    RETRY_APC_API(ret, APC_EnableInterleave(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                  m_deviceSelInfo[0],
                                                  bEnable));

    return ret;
}

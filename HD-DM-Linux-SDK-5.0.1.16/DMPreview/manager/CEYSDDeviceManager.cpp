#include "CEYSDDeviceManager.h"
#include "CVideoDeviceModel.h"
#include "CVideoDeviceModelFactory.h"
#include "eSPDI.h"
#include <QDBusInterface>
#include "CIMUDeviceManager.h"

CEYSDDeviceManager::CEYSDDeviceManager():
m_pEYSD(nullptr),
m_bEnableSDKLog(false)
{
    QDBusMessage m = QDBusMessage::createMethodCall("org.freedesktop.login1",
                                                    "/org/freedesktop/login1",
                                                    "org.freedesktop.login1.Manager",
                                                    "Inhibit");

    QList<QVariant> args;
    args.append("sleep:shutdown");
    args.append("DMPreview");
    args.append("Inhibit sleep:shutdown avoid nvidia driver issue.");
    args.append("block");
    m.setArguments(args);
    QDBusMessage response = QDBusConnection::systemBus().call(m);
    QList<QVariant> list = response.arguments();
    if (!list.empty()){
        static QVariant lock = list.first();
    }
}

CEYSDDeviceManager::~CEYSDDeviceManager()
{
    if (m_pEYSD){
        APC_Release(&m_pEYSD);
        m_pEYSD = nullptr;
    }

    CVideoDeviceModelFactory::ReleaseModels(m_videoDeviceModels);
}

void CEYSDDeviceManager::EnableSDKLog(bool bEnable)
{
    if(m_bEnableSDKLog == bEnable) return;

    m_bEnableSDKLog = bEnable;
    Reconnect();
}

int CEYSDDeviceManager::Reconnect()
{        
    for (CVideoDeviceModel *pModel : m_videoDeviceModels){
        pModel->ChangeState(CVideoDeviceModel::RECONNECTING);
    }

    std::vector<CVideoDeviceModel *> findDevices;
    InitEYSD();

    CIMUDeviceManager::GetInstance()->Clear();
    CIMUDeviceManager::GetInstance()->Init();

    FindDevices(findDevices);

    if (findDevices.size() != m_videoDeviceModels.size()) return APC_Init_Fail;

    for (CVideoDeviceModel *pTargetModel : m_videoDeviceModels){
        CVideoDeviceModel *pFindModel = nullptr;
        for (CVideoDeviceModel *pModel : findDevices){
            if (pTargetModel->EqualModel(pModel)){
                pFindModel = pModel;
                break;
            }
        }

        if (!pFindModel) {
            CVideoDeviceModelFactory::ReleaseModels(findDevices);
            return APC_Init_Fail;
        }

        pTargetModel->AdjustDeviceSelfInfo(pFindModel->GetDeviceSelInfo()[0]);
    }

    for (CVideoDeviceModel *pModel : m_videoDeviceModels){
        pModel->ChangeState(CVideoDeviceModel::RECONNECTED);
    }

    CVideoDeviceModelFactory::ReleaseModels(findDevices);

    return APC_OK;
}

int CEYSDDeviceManager::UpdateDevice()
{
    return FindDevices(m_videoDeviceModels);
}

std::vector<CVideoDeviceModel *> CEYSDDeviceManager::GetDeviceModels()
{
    return m_videoDeviceModels;
}

int CEYSDDeviceManager::InitEYSD()
{
    if (m_pEYSD){
        APC_Release(&m_pEYSD);
        m_pEYSD = nullptr;
    }

    return APC_Init(&m_pEYSD, m_bEnableSDKLog);
}

int CEYSDDeviceManager::FindDevices(std::vector<CVideoDeviceModel *> &devices)
{
    if (!m_pEYSD){
        if (APC_OK != InitEYSD()){
            return APC_Init_Fail;
        }
    }

    devices.clear();
    int nDevCount = APC_GetDeviceNumber(m_pEYSD);
    for (int i = 0 ; i < nDevCount ; ++i){
        DEVSELINFO devSelInfo;
        devSelInfo.index = i;
        CVideoDeviceModel *pDeviceModel = CVideoDeviceModelFactory::CreateVideoDeviceModel(&devSelInfo);
        if (nullptr != pDeviceModel){
            devices.push_back(std::move(pDeviceModel));
        }
    }

    return APC_OK;
}

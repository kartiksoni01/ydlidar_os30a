#include "CIMUDeviceManager.h"

CIMUDeviceManager::CIMUDeviceManager()
{
    Init();
}

CIMUDeviceManager::~CIMUDeviceManager()
{
    Clear();
}

void CIMUDeviceManager::Clear()
{
    for (auto it = m_hidDeviceMap.begin() ; it != m_hidDeviceMap.end() ; ++it){
        for (hid_device *device : (*it).second){
            hid_close(device);
        }
    }

    m_hidDeviceMap.clear();
    hid_exit();
}

void CIMUDeviceManager::Init()
{
    hid_init();
}

std::vector<hid_device *> CIMUDeviceManager::GetDeviceList(unsigned short nVID, unsigned short nPID)
{
    std::pair<unsigned short, unsigned short> info(nVID, nPID);

    if (0 == m_hidDeviceMap.count(info)){
        hid_device_info *deviceInfo = hid_enumerate(nVID, nPID);
        hid_device_info *headInfo = deviceInfo;
        while (deviceInfo){
            hid_device *device = hid_open_path(deviceInfo->path);
            if (device){
                hid_set_nonblocking(device, true);
                m_hidDeviceMap[info].push_back(std::move(device));
            }
            deviceInfo = deviceInfo->next;
        }
        if (headInfo) hid_free_enumeration(headInfo);
    }

    return m_hidDeviceMap[info];
}

#ifndef CIMUDEVICEMANAGER_H
#define CIMUDEVICEMANAGER_H
#include <vector>
#include <map>
#include "hidapi/hidapi.h"

class CIMUDeviceManager
{
public:
    static CIMUDeviceManager *GetInstance(){
        static CIMUDeviceManager *pInstance = nullptr;
        if (!pInstance){
            pInstance = new CIMUDeviceManager();
        }
        return pInstance;
    }

    std::vector<hid_device *> GetDeviceList(unsigned short nVID, unsigned short nPID);

    void Clear();
    void Init();
private:
    CIMUDeviceManager();
    ~CIMUDeviceManager();

private:
    std::map<std::pair<unsigned short, unsigned short>,
             std::vector<hid_device *>> m_hidDeviceMap;
};

#endif // CIMUDEVICEMANAGER_H

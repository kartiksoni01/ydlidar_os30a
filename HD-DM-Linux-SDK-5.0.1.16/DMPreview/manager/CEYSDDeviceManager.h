#ifndef CEYSDDeviceManager_H
#define CEYSDDeviceManager_H

#include <vector>

class CVideoDeviceModel;
class CEYSDDeviceManager
{
public:
    static CEYSDDeviceManager *GetInstance(){
        static CEYSDDeviceManager *pInstance = nullptr;
        if (!pInstance){
            pInstance = new CEYSDDeviceManager();
        }

        return pInstance;
    }

    bool IsSDKLog(){ return m_bEnableSDKLog; }
    void EnableSDKLog(bool bEnable);

    int Reconnect();
    int UpdateDevice();
    std::vector<CVideoDeviceModel *> GetDeviceModels();

    void *GetEYSD(){ return m_pEYSD; }
private:
    CEYSDDeviceManager();
    ~CEYSDDeviceManager();

    int InitEYSD();
    int FindDevices(std::vector<CVideoDeviceModel *> &devices);

private:
    void *m_pEYSD;
    bool m_bEnableSDKLog;
    std::vector<CVideoDeviceModel *> m_videoDeviceModels;
};

#endif // CEYSDDeviceManager_H

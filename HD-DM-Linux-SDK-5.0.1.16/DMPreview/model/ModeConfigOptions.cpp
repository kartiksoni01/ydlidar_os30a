#include "ModeConfigOptions.h"

ModeConfigOptions::ModeConfigOptions(USB_PORT_TYPE usbType, unsigned short nPID):
m_nCurrentIndex(EOF)
{    
    std::vector<ModeConfig::MODE_CONFIG> allConfigs = ModeConfig::GetModeConfig().GetModeConfigList(nPID);
    for(ModeConfig::MODE_CONFIG config : allConfigs ){
        if(config.iUSB_Type != usbType) continue;
        m_modeConfigs.push_back(config);
    }

    if(!m_modeConfigs.empty()){
        if(nPID == APC_PID_SANDRA && usbType == USB_PORT_TYPE_3_0 || nPID == APC_PID_HYPATIA2 && usbType == USB_PORT_TYPE_2_0){
            int specifyDefaultMode = 5;
            for (int loop = 0 ; loop < allConfigs.size(); loop ++)
            {
                if (allConfigs[loop].iMode == 5)
                {
                    m_nCurrentIndex = loop;
                    break;
                }
                else
                {
                    continue;
                }
            }
        }
        else
        {
            m_nCurrentIndex = 0;
        }
    }
}

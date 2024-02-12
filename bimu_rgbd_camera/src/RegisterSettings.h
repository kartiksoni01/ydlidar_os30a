#pragma once
#include <string>
#include "eSPDI.h"

class RegisterSettings {
	RegisterSettings();
public:
    static int FramesyncD0(void* hEYSD, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int Framesync(void* hEYSD, PDEVSELINFO pDevSelInfo, PDEVSELINFO pDevSelInfoEx,
                         int DepthWidth, int DepthHeight,
                         int ColorWidth, int ColorHeight,
                         bool bFormatMJPG, int Fps, const int nPid);
    static int FramesyncFor8054(void* hEYSD, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int FramesyncFor8040S(void* hEYSD, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int FrameSync8053_8059(void* hEYSD, PDEVSELINFO pDevSelInfo);
    static int FrameSync8053_8059_Clock( void* hEYSD, PDEVSELINFO pDevSelInfo );
    static int FrameSync8053_8059_Reset(void* hEYSD, PDEVSELINFO pDevSelInfo);
	static int ForEx8053Mode9(void* hEYSD, PDEVSELINFO pDevSelInfo);
    static int DM_Quality_Register_Setting(void* hEYSD, PDEVSELINFO pDevSelInfo, unsigned short wPID);
    static int DM_Quality_Register_Setting_For6cm(void* hEYSD, PDEVSELINFO pDevSelInfo);
    static int DM_Quality_Register_Setting_For15cm(void* hEYSD, PDEVSELINFO pDevSelInfo);
};

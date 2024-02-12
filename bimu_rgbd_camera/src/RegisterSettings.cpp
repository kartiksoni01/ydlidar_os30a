#include "RegisterSettings.h"
#include <unistd.h>
#include <iostream>
#include <fstream>
using namespace std;
extern std::string filePath;

RegisterSettings::RegisterSettings() {
}

int exchange(int data)
{
	int data_low;
	int data_high;
	data_low = (data & 0xff00) >> 8;
	data_high = (data & 0x00ff) << 8;
	int exchange_data = data_high + data_low;
	return exchange_data;
}

int RegisterSettings::FramesyncD0(void* hEYSD, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps)
{
	unsigned short value = 0;
    SENSORMODE_INFO SensorMode = SENSOR_BOTH;
    if (APC_GetFWRegister(hEYSD, pDevSelInfo, 0xf3, &value, FG_Address_2Byte | FG_Value_1Byte) != APC_OK)
	{
        printf("%s: %s\n", __func__, "Read Register, from PUMA,APC_READ_FW_REG_0xf3_FAIL");
		return -1;
	}
	else
	{
		if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf3, 0x0010, FG_Address_2Byte | FG_Value_1Byte) != APC_OK)
        {
            printf("%s: %s\n", __func__, "Switch to Sensor PID 141 VID 1E4E ,AR0330,APC_WRITE_FW_REG_0xf3_FAIL");
			return -1;
		}
	}
    if (APC_GetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, SensorMode) != APC_OK)
	{
        printf("%s: %s\n", __func__, "Read Register, from Sensor PID 141 VID 1E4E ,AR0330,APC_READ_Sensor_REG_0x30_FAIL");
	}
	else
	{
        printf("%s: 0x30, 0x301a,value1=%x\n", __func__, value);    /*0x5c00*/
		value = exchange(value);
        printf("%s: 0x30, 0x301a,value2=%x\n", __func__, value);    	/*0x005c*/
		BIT_CLEAR(value, 2);
        if (APC_SetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, SensorMode) != APC_OK)
		{
            printf("%s: %s\n", __func__, "APC_WRITE_REG_FAIL");
		}
		else
		{
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, SensorMode) != APC_OK)
			{
                printf("%s: %s\n", __func__, "Read Register, from Sensor PID 141 VID 1E4E ,AR0330,APC_READ_Sensor_REG_0x30_FAIL");
			}
			else
			{
				value = exchange(value);
                printf("%s: 0x30, 0x301a,value3=%x\n", __func__, value);	/*0x0058*/
			}
		}
		BIT_SET(value, 8);
        if (APC_SetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, SensorMode) != APC_OK)
		{
            printf("%s: %s\n", __func__, "APC_WRITE_REG_FAIL");
		}
		else
		{
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, SensorMode) != APC_OK)
			{
                printf("%s: %s\n", __func__, "Read Register, from Sensor PID 141 VID 1E4E ,AR0330,APC_READ_Sensor_REG_0x30_FAIL");
			}
			else
			{
				value = exchange(value);
                printf("%s: 0x30, 0x301a,value4=%x\n", __func__, value);	/*0x0158*/
			}
		}
	}
	value = 0;
	if (APC_GetFWRegister(hEYSD, pDevSelInfo, 0xf3, &value, FG_Address_2Byte | FG_Value_1Byte) != APC_OK)
	{
        printf("%s: %s\n", __func__, "Read Register, from PUMA,APC_WRITE_FW_REG_0xf3_FAIL");
		return -1;
	}
	if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf3, 0x0000, FG_Address_2Byte | FG_Value_1Byte) != APC_OK)
	{
        printf("%s: %s\n", __func__, "Switch to PUMA 139 VID 1E4E,APC_WRITE_FW_REG_0xf3_FAIL");
		return -1;
	}

	return 0;
}

int RegisterSettings::Framesync(void* hEYSD, PDEVSELINFO pDevSelInfo, PDEVSELINFO pDevSelInfo_Ex,
                                int DepthWidth, int DepthHeight,
                                int ColorWidth, int ColorHeight,
                                bool bFormatMJPG, int Fps, const int nPid)
{    
#if 1
    unsigned short value = 0;
    if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
    {
        printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
    }
    else
    {
        //printf("0x6c, 0x301a,value1=%x\n", value);	/*0x1c02*/
        value = exchange(value);
        //printf("0x6c, 0x301a,value2=%x\n", value);	/*0x021c*/
        BIT_CLEAR(value, 2);
        if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
        {
            printf("APC_WRITE_REG_FAIL\n");
        }
        else
        {
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
            {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            }
            else
            {
                value = exchange(value);
                //printf("0x6c, 0x301a,value3=%x\n", value);	/*0x0218*/
            }
        }
        usleep(100  * 1000);
        BIT_SET(value, 8);
        if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
        {
            printf("APC_WRITE_REG_FAIL\n");
        }
        else
        {
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
            {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            }
            else
            {
                value = exchange(value);
                //printf("0x6c, 0x301a,value4=%x\n", value);	/*0x0318*/
            }
        }
        value = 0;
        if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x3026, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
        {
            printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
        }
        else
        {
            //printf("0x6c, 0x3026,value5=%x\n", value);	/*0xfbff*/
            value = exchange(value);
            //printf("0x6c, 0x3026,value6=%x\n", value);	/*0xfffb*/
            BIT_CLEAR(value, 7);
            BIT_CLEAR(value, 9);
            BIT_SET(value, 8);
            usleep(100  * 1000);
            if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x3026, value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
            {
                printf("APC_WRITE_REG_FAIL\n");
            }
            else
            {
                if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x3026, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
                {
                    printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
                }
                else
                {
                    value = exchange(value);
                    //printf("0x6c, 0x3026,value7=%x\n", value);	/*0xfd7b*/
                }
            }
            value = 0;
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x315E, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
            {
                //printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL");
            }
            else
            {
                value = exchange(value);
                //printf("0x6c, 0x315E,value8=%x\n", value);	/*0x0*/
                BIT_SET(value, 0);
                usleep(100  * 1000);
                if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x315E, value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
                {
                    printf("APC_WRITE_REG_FAIL\n");
                }
                else
                {
                    if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x315E, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
                    {
                        printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
                    }
                    else
                    {
                        value = exchange(value);
                        //printf("0x6c, 0x315E,value9=%x\n", value);	/*0x1*/
                    }
                }
            }
            value = 0;
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
            {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            }
            else
            {
                //printf("0x6c, 0x301a,value6=%x\n", value);	/*0x1803*/
                value = exchange(value);
                //printf("0x6c, 0x301a,value10=%x\n", value);	/*0x0318*/
                BIT_SET(value, 2);
                usleep(100  * 1000);
                if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
                {
                    printf("APC_WRITE_REG_FAIL\n");
                }
                else
                {
                    if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, (SENSORMODE_INFO)2) != APC_OK)
                    {
                        printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
                    }
                    else
                    {
                        value = exchange(value);
                        //printf("0x6c, 0x301a,value11=%x\n", value);	/*0x031c*/
                    }
                }
            }
        }
    }
    if ( APC_GetFWRegister(hEYSD, pDevSelInfo, 0xf4, &value, FG_Address_1Byte | FG_Value_1Byte) != APC_OK ||
         APC_GetFWRegister(hEYSD, pDevSelInfo, 0xf5, &value, FG_Address_1Byte | FG_Value_1Byte) != APC_OK )
    {
        printf("Read Register, from PUMA,APC_WRITE_FW_REG_0xf4_FAIL\n");
        return -1;
    }
    switch ( nPid ) {
        case APC_PID_8054:  return FramesyncFor8054( hEYSD, pDevSelInfo, DepthWidth, DepthHeight, ColorWidth, ColorHeight, bFormatMJPG, Fps );
        case APC_PID_8040S: return FramesyncFor8040S( hEYSD, pDevSelInfo, DepthWidth, DepthHeight, ColorWidth, ColorHeight, bFormatMJPG, Fps );
    }
    return APC_NotSupport;
#else
#if 1
    unsigned short value = 0;
    //AR0330
    if (APC_GetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("Read Register, from Sensor,AR0130,APC_READ_Sensor_REG_0x30_FAIL\n");
        return -1;
    }
    printf("0x30, 0x301a,value1=%x\n", value);
    value = exchange(value);
    printf("0x30, 0x301a,value2=%x\n", value);
    BIT_CLEAR(value, 2);
    if (APC_SetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("APC_WRITE_REG_FAIL\n");
        return -1;
    }

    if (APC_GetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x30ce, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("Read Register, from Sensor AR0130,APC_READ_Sensor_REG_0x30_FAIL\n");
        return -1;
    }

    value = exchange(value);
    printf("0x30, 0x30ce,value3=%x\n", value);
    usleep(100  * 1000);
    BIT_SET(value, 0);
    if (APC_SetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x30ce, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("APC_WRITE_REG_FAIL\n");
        return -1;
    }

    if (APC_GetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("Read Register, from Sensor AR1310,APC_READ_Sensor_REG_0x30_FAIL\n");
        return -1;
    }
    value = exchange(value);
    printf("0x30, 0x301a,value4=%x\n", value);
    BIT_SET(value, 8);

    if (APC_SetSensorRegister(hEYSD, pDevSelInfo, 0x30, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("APC_WRITE_REG_FAIL\n");
        return -1;
    }
#endif
    // AR01335
    if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
        printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
    } else {
        printf("0x6c, 0x301a,value1=%x\n", value);	/*0x1c02*/
        value = exchange(value);
        printf("0x6c, 0x301a,value2=%x\n", value);	/*0x021c*/
        BIT_CLEAR(value, 2);
        if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
            printf("APC_WRITE_REG_FAIL\n");
        } else {
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            } else {
                value = exchange(value);
                printf("0x6c, 0x301a,value3=%x\n", value);	/*0x0218*/
            }
        }
        usleep(100  * 1000);
        BIT_SET(value, 8);
        if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
            printf("APC_WRITE_REG_FAIL\n");
        } else {
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            } else {
                value = exchange(value);
                printf("0x6c, 0x301a,value4=%x\n", value);	/*0x0318*/
            }
        }
        value = 0;
        if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x3026, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
            printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
        } else {
            printf("0x6c, 0x3026,value5=%x\n", value);	/*0xfbff*/
            value = exchange(value);
            printf("0x6c, 0x3026,value6=%x\n", value);	/*0xfffb*/
            BIT_CLEAR(value, 7);
            BIT_CLEAR(value, 9);
            BIT_SET(value, 8);
            usleep(100  * 1000);
            if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x3026, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                printf("APC_WRITE_REG_FAIL\n");
            } else {
                if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x3026, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                    printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
                } else {
                    value = exchange(value);
                    printf("0x6c, 0x3026,value7=%x\n", value);	/*0xfd7b*/
                }
            }
            value = 0;
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x315E, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            } else {
                value = exchange(value);
                printf("0x6c, 0x315E,value8=%x\n", value);	/*0x0*/
                BIT_SET(value, 0);
                usleep(100  * 1000);
                if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x315E, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                    printf("APC_WRITE_REG_FAIL\n");
                } else {
                    if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x315E, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                        printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
                    } else {
                        value = exchange(value);
                        printf("0x6c, 0x315E,value9=%x\n", value);	/*0x1*/
                    }
                }
            }
            value = 0;
            if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
            } else {
                printf("0x6c, 0x301a,value6=%x\n", value);	/*0x1803*/
                value = exchange(value);
                printf("0x6c, 0x301a,value10=%x\n", value);	/*0x0318*/
                BIT_SET(value, 2);
                usleep(100  * 1000);
                if (APC_SetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                    printf("APC_WRITE_REG_FAIL\n");
                } else {
                    if (APC_GetSensorRegister(hEYSD, pDevSelInfo_Ex, 0x6c, 0x301a, &value, FG_Address_2Byte | FG_Value_2Byte, 2) != APC_OK) {
                        printf("Read Register, from Sensor PID 143 VID1E4E ,AR1335,APC_READ_Sensor_REG_0x6c_FAIL\n");
                    } else {
                        value = exchange(value);
                        printf("0x6c, 0x301a,value11=%x\n", value);	/*0x031c*/
                    }
                }
            }
        }
    }
    if ( APC_GetFWRegister(hEYSD, pDevSelInfo_Ex, 0xf4, &value, FG_Address_1Byte | FG_Value_1Byte) != APC_OK ||
         APC_GetFWRegister(hEYSD, pDevSelInfo_Ex, 0xf5, &value, FG_Address_1Byte | FG_Value_1Byte) != APC_OK ) {
        printf("Read Register, from PUMA,APC_WRITE_FW_REG_0xf4_FAIL\n");
        return -1;
    }

    switch(nPid){
        case APC_PID_8054 :  return FramesyncFor8054( hEYSD, pDevSelInfo_Ex, DepthWidth, DepthHeight, ColorWidth, ColorHeight, bFormatMJPG, Fps );
        case APC_PID_8040S : return FramesyncFor8040S( hEYSD, pDevSelInfo_Ex, DepthWidth, DepthHeight, ColorWidth, ColorHeight, bFormatMJPG, Fps );
    }
    //case APC_PID_8054:  return FramesyncFor8054( hEYSD, pDevSelInfo_Ex, DepthWidth, DepthHeight, ColorWidth, ColorHeight, bFormatMJPG, Fps );
    return APC_NotSupport;
#endif
}

int RegisterSettings::FramesyncFor8054(void* hEYSD, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps)
{
    /*Check mode*/
    int mode = 0;
    if (DepthWidth == 1920 && DepthHeight == 1080) {
        if (ColorWidth == 3840 && ColorHeight == 2160 && bFormatMJPG)
            mode = 5;
        else if (ColorWidth == 4080 && ColorHeight == 3120 && bFormatMJPG)
            mode = 6;
        else if (ColorWidth == 3840 && ColorHeight == 2160 && !bFormatMJPG)
            mode = 7;
        else if (ColorWidth == 4208 && ColorHeight == 3120 && !bFormatMJPG)
            mode = 8;
        else if (ColorWidth == 1920 && ColorHeight == 1080 && bFormatMJPG)
            mode = 9;
    } else if (DepthWidth == 1080 && DepthHeight == 1440) {
        if (ColorWidth == 3840 && ColorHeight == 2160 && !bFormatMJPG)
            mode = 10;
        else if (ColorWidth == 3840 && ColorHeight == 2160 && bFormatMJPG)
            mode = 11;
    } else if (DepthWidth == 1280 && DepthHeight == 1280) {
        if (ColorWidth == 2560 && ColorHeight == 2560 && bFormatMJPG && Fps == 15)
            mode = 16;
        else if (ColorWidth == 2560 && ColorHeight == 2560 && !bFormatMJPG && Fps == 30)
            mode = 17;
        else if (ColorWidth == 2560 && ColorHeight == 2560 && !bFormatMJPG && Fps == 5)
            mode = 23;
    } else if (DepthWidth == 640 && DepthHeight == 640) {
        if (ColorWidth == 1280 && ColorHeight == 1280 && !bFormatMJPG)
            mode = 22;
    }
    printf("RegisterSettings::FramesyncFor8054 Mode: %d  set Frame-sync reg.\n", mode);
    /* TRIGGER */
    if (mode == 5 || mode == 7) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x004e, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0001, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 6 || mode == 8) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0075, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0001, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 9) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0014, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0014, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 10 || mode == 11) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0050, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0010, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 16) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x003e, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0001, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 17) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x005e, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0002, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 22) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0013, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x000c, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 23) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0050, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0010, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    }
    return APC_OK;
}

int RegisterSettings::FramesyncFor8040S(void* hEYSD, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps)
{
    int mode = 0;
    if (DepthWidth == 912 && DepthHeight == 1920) {
        if (ColorWidth == 3840 && ColorHeight == 1824 && Fps == 10)
            mode = 5;
        else if (ColorWidth == 3840 && ColorHeight == 1824 && Fps == 5)
            mode = 6;
        else if (ColorWidth == 2560 && ColorHeight == 1216 && Fps == 10)
            mode = 7;
        else if (ColorWidth == 2560 && ColorHeight == 1216 && Fps == 5)
            mode = 9;
    } else if (DepthWidth == 456 && DepthHeight == 960) {
        if (ColorWidth == 1920 && ColorHeight == 912 && Fps == 30)
            mode = 8;
    }
    printf("RegisterSettings::FramesyncFor8040S Mode: %d  set Frame-sync reg.\n", mode);
    if (mode == 5) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x005a, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0005, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 6) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x00b9, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0005, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 7) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0028, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x000A, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 8) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x001e, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0001, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    } else if (mode == 9) {
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf4, 0x0050, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
        if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xf5, 0x0014, FG_Address_1Byte | FG_Value_1Byte) != APC_OK)
            printf("APC_WRITE_REG_FAIL\n");
    }
    return APC_OK;
}

int RegisterSettings::FrameSync8053_8059(void* hEYSD, PDEVSELINFO pDevSelInfo)
{
    //if (APC_SetFWRegister(hEYSD, pDevSelInfo, 0xE4, 0x01, FG_Address_1Byte | FG_Value_1Byte)) { // turn on serial count
    //    printf("APC_WRITE_REG_FAIL\n");
    //    return APC_WRITE_REG_FAIL;
    //}

    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF07A, 0x01, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF07B, 0x00, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF07C, 0x00, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF07D, 0x00, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF01A, 0x80, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF041, 0x5c, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF045, 0x00, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    APC_SetHWRegister(hEYSD, pDevSelInfo, 0xF0FE, 0x03, FG_Address_2Byte | FG_Value_1Byte);
    usleep(1000 * 10);
    return 0;
}

int RegisterSettings::FrameSync8053_8059_Clock( void* hEYSD, PDEVSELINFO pDevSelInfo )
{
    if ( APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF07A, 0x01, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
         APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF07B, 0x00, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
         APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF07C, 0x00, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
         APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF07D, 0x00, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK )
    {
        WORD wF01A = NULL;
        WORD wF041 = NULL;
        WORD wF045 = NULL;
        WORD wF0FE = NULL;
        //WORD wE079 = NULL;

        if ( APC_GetHWRegister( hEYSD, pDevSelInfo, 0xF01A, &wF01A, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
             APC_GetHWRegister( hEYSD, pDevSelInfo, 0xF041, &wF041, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
             APC_GetHWRegister( hEYSD, pDevSelInfo, 0xF045, &wF045, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
             APC_GetHWRegister( hEYSD, pDevSelInfo, 0xF0FE, &wF0FE, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK /*&&
             APC_GetHWRegister( hEYSD, pDevSelInfo, 0xF079, &wE079, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK*/ )
        {
            wF01A &= 0xDF; // bit6 = 0
            wF041 |= 0x40; // bit6 = 1
            wF045 &= 0xDF; // bit6 = 0
            wF0FE &= 0xDF; // bit6 = 0
            //wE079 &= 0xFC; // bit0,1 = 0

            if ( APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF01A, wF01A, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
                 APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF041, wF041, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
                 APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF045, wF045, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK &&
                 APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF0FE, wF0FE, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK /*&&
                 APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF079, wE079, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK*/ )
            {
                //if ( APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF079, wE079 + 1, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK )
                //{
                //    return APC_OK; /* enable mudule-sync-success */
                //}
                return APC_OK;
            }
        }
    }
    printf("APC_WRITE_REG_FAIL\n");
    return APC_WRITE_REG_FAIL;
}

int RegisterSettings::FrameSync8053_8059_Reset(void* hEYSD, PDEVSELINFO pDevSelInfo)
{
    BYTE wE079 = NULL;

    if (APC_GetHWRegister( hEYSD, pDevSelInfo, 0xF079, (WORD *)&wE079, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK) {
        wE079 &= 0xFC; // bit0,1 = 0

        if (APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF079, wE079, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK) {
            //usleep(1000 * 1000);
            if (APC_SetHWRegister( hEYSD, pDevSelInfo, 0xF079, wE079 + 1, FG_Address_2Byte | FG_Value_1Byte ) == APC_OK) {
                return APC_OK; /* enable mudule-sync-success */
            }
        }
    }
    printf("%s: APC_WRITE_REG_FAIL\n", __func__);
    return APC_WRITE_REG_FAIL;
}

int RegisterSettings::ForEx8053Mode9(void* hEYSD, PDEVSELINFO pDevSelInfo)
{
	unsigned short value = 0x0;
	unsigned short address = 0xf0;
	if (APC_GetFWRegister(hEYSD, pDevSelInfo, address, &value, FG_Address_2Byte | FG_Value_1Byte) != APC_OK)
	{
        printf("%s: Read Register, READ FW REG %x FAIL\n", __func__, address);
		return -1;
	}

	value |= 0x0001;
	if (APC_SetFWRegister(hEYSD, pDevSelInfo, address, value, FG_Address_2Byte | FG_Value_1Byte) != APC_OK)
	{
        printf("%s: Read Register, WRITE FW REG %x FAIL\n", __func__, address);
		return -1;
	}
}

int RegisterSettings::DM_Quality_Register_Setting_For15cm(void* hEYSD, PDEVSELINFO pDevSelInfo)
{
    DEVINFORMATION devinfo;
    int nRet = APC_GetDeviceInfo(hEYSD, pDevSelInfo, &devinfo);
    if (nRet != APC_OK) {
        return nRet;
    }

    char *modelName = NULL;

    printf("devinfo.wPID = %x\n", devinfo.wPID);
    if (devinfo.wPID == 0x147) {
        modelName = "EX8038_BL15cm"; //M0
    } else {
        modelName = NULL;
        return -1;
    }

    char fileName[256] = {0};
    char tmp[255];
    int RegAddress, ValidDataRange, Data;
    std::string file_apth = filePath + "/../../../HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/" + modelName +"_DM_Quality_Register_Setting.cfg";
    sprintf(fileName, file_apth.c_str());
    // sprintf(fileName, "./../../../../HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/%s_DM_Quality_Register_Setting.cfg", modelName);

    ifstream in(fileName);
    if (!in) {
        printf("[%s][%d]Cannot open cfg file %s\n", __func__, __LINE__, fileName);
        return -1;
    }
    printf("[%s][%d]Successfully open cfg file %s\n", __func__, __LINE__, fileName);
    while (in) {
            in.getline(tmp, 255);  // delim defaults to '\n'
            if (in) {
                unsigned short RegValue;
                unsigned short NotValidDataRange;

                sscanf(tmp, "%x, %x, %x", &RegAddress, &ValidDataRange, &Data);

            APC_GetHWRegister(hEYSD, pDevSelInfo, RegAddress, &RegValue, FG_Address_2Byte | FG_Value_1Byte);

            NotValidDataRange = ~ValidDataRange;
            RegValue = RegValue & NotValidDataRange;
            RegValue |= Data;

            APC_SetHWRegister(hEYSD, pDevSelInfo, RegAddress, RegValue, FG_Address_2Byte | FG_Value_1Byte);

            //Sleep(5); // delay time, need fine tune in the feature

            ////////////////////

            unsigned short RegValueCheck = 0;
            APC_GetHWRegister(hEYSD, pDevSelInfo, RegAddress, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
            if (RegValue != RegValueCheck) {
                printf("Set config. to Register failed !!");
                return -1;
            }
        }
    }

    in.close();

    //AfxMessageBox(_T("Set config. to Register done !!"));
    return 0;
}


int RegisterSettings::DM_Quality_Register_Setting_For6cm(void* hEYSD, PDEVSELINFO pDevSelInfo)
{
    DEVINFORMATION devinfo;
    int nRet = APC_GetDeviceInfo(hEYSD, pDevSelInfo, &devinfo);
    if (nRet != APC_OK) {
        return nRet;
    }

    char *modelName = NULL;

    if (devinfo.wPID == 0x124) {
        modelName = "EX8038_BL6cm"; //M0
    } else {
        modelName = NULL;
        return -1;
    }

    char fileName[256] = {0};
    char tmp[255];
    int RegAddress, ValidDataRange, Data;
    std::string file_apth = filePath + "/../../../HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/" + modelName +"_DM_Quality_Register_Setting.cfg";
    sprintf(fileName, file_apth.c_str());
    // sprintf(fileName, "./../../../../HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/%s_DM_Quality_Register_Setting.cfg", modelName);

    ifstream in(fileName);
    if (!in) {
        printf("[%s][%d]Cannot open cfg file %s\n", __func__, __LINE__, fileName);
        return -1;
    }
    printf("[%s][%d]Successfully open cfg file %s\n", __func__, __LINE__, fileName);
    while (in) {
            in.getline(tmp, 255);  // delim defaults to '\n'
            if (in) {
                unsigned short RegValue;
                unsigned short NotValidDataRange;

                sscanf(tmp, "%x, %x, %x", &RegAddress, &ValidDataRange, &Data);
                SENSORMODE_INFO SensorMode = SENSOR_A;
                APC_GetSensorRegister(hEYSD, pDevSelInfo, 0xC2, RegAddress, &RegValue, FG_Address_2Byte | FG_Value_1Byte, SensorMode);

                NotValidDataRange = ~ValidDataRange;
                RegValue = RegValue & NotValidDataRange;
                RegValue |= Data;

                APC_SetSensorRegister(hEYSD, pDevSelInfo, 0xC2, RegAddress, RegValue, FG_Address_2Byte | FG_Value_1Byte, SensorMode);
                //Sleep(5); // delay time, need fine tune in the feature

                ////////////////////

                unsigned short RegValueCheck = 0;
                APC_GetSensorRegister(hEYSD, pDevSelInfo, 0xC2, RegAddress, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte, SensorMode);
                if (RegValue != RegValueCheck) {
                    printf("Set config. to Register failed !!");
                    return -1;
                }
            }
    }

    in.close();

    return 0;
}

int RegisterSettings::DM_Quality_Register_Setting(void* hEYSD, PDEVSELINFO pDevSelInfo, unsigned short wPID)
{
    char *modelName = NULL;
    unsigned int max_retry_count = 100;
    unsigned int retry_count = 0;
    int nRet = 0;
    

    switch (wPID) {
    case APC_PID_8036:
        modelName = "EX8036";
        break;
    case APC_PID_8037:
        modelName = "EX8037";
        break;
    case APC_PID_8038_M0:
        modelName = "EX8038_BL3cm"; //M0
        break;
    case APC_PID_8040S:
        modelName = "EX8040S";
        break;
    case APC_PID_8052:
        modelName = "EX8052";
        break;
    case APC_PID_8053:
        modelName = "EX8053";
        break;
    case APC_PID_8054:
        modelName = "EX8054";
        break;
    case APC_PID_HYPATIA:
        modelName = "HYPATIA";
        break;
    case APC_PID_HYPATIA2:
        modelName = "HYPATIA2";
        break;
    case APC_PID_8062:
        modelName = "EX8062";
        break;
    case APC_PID_8059:
        modelName = "YX8059";
        break;
    case APC_PID_SANDRA:
        modelName = "SANDRA";
        break;
    case APC_PID_NORA:
        modelName = "NORA";
        break;
    default:
        modelName = NULL;
        break;
    }

    char fileName[256] = {0};
    char tmp[255];
    int RegAddress, ValidDataRange, Data;
    std::string file_apth = filePath + "/../../../HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/" + modelName +"_DM_Quality_Register_Setting.cfg";
    sprintf(fileName, file_apth.c_str());
    // sprintf(fileName, "./../../../../HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/%s_DM_Quality_Register_Setting.cfg", modelName);

    ifstream in(fileName);

    if (!in) {
        printf("[%s][%d]Cannot open cfg file %s for (0x%04x)\n", __func__, __LINE__, fileName, wPID);
        return -1;
    }
    
    printf("[%s][%d]Successfully open cfg file %s for (0x%04x)\n", __func__, __LINE__, fileName, wPID);
    
    while (in) {
            in.getline(tmp, 255);  // delim defaults to '\n'
            if (in) {
                unsigned short RegValue;
                unsigned short NotValidDataRange;
                unsigned short RegValueCheck = 0;
                
                sscanf(tmp, "%x, %x, %x", &RegAddress, &ValidDataRange, &Data);

                APC_GetHWRegister(hEYSD, pDevSelInfo, RegAddress, &RegValue, FG_Address_2Byte | FG_Value_1Byte);

                NotValidDataRange = ~ValidDataRange;
                RegValue = RegValue & NotValidDataRange;
                RegValue |= Data;
retry_again:
                if (retry_count >= max_retry_count) {
                    printf("Set config. to Register failed !! [0x%04x] != [0x%04x] [0x%04x]\n", RegAddress, RegValue, RegValueCheck);
                    retry_count = 0;
                    continue;
                }
                
                APC_SetHWRegister(hEYSD, pDevSelInfo, RegAddress, RegValue, FG_Address_2Byte | FG_Value_1Byte);

                //Sleep(5); // delay time, need fine tune in the feature

                ////////////////////

                APC_GetHWRegister(hEYSD, pDevSelInfo, RegAddress, &RegValueCheck, FG_Address_2Byte | FG_Value_1Byte);
                if (RegValue != RegValueCheck) {
                    retry_count++;
                    usleep(5000);
                    goto retry_again;
                }
            }
    }

    in.close();

    return nRet;
}


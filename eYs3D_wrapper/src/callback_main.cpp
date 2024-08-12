/*
 * Copyright (C) 2022 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#include "EYS3DSystem.h"
#include "devices/CameraDevice.h"
#include "video/Frame.h"
#include "sensors/SensorData.h"
#include "debug.h"

#ifdef _WIN32
#  include "utils.h"
#  include <magic.h>
#else
#  include <unistd.h>
#endif

#define LOG_TAG "EYS3DSystem Test"
#define DURATION 30

#ifdef WIN32
typedef struct {
    int colorFormat;
    int colorWidth;
    int colorHeight;
    int fps;
    int depthWidth;
    int depthHeight;
    int videoMode;
} camera_open_config;
#endif

using namespace libeYs3D;

std::shared_ptr<libeYs3D::devices::CameraDevice> dDevice;

// using Callback = std::function<bool(const Frame* frame)>;
static bool color_image_callback(const libeYs3D::video::Frame* frame)    {
    char buffer[512];
    static int64_t count = 0ll;
    static int64_t time = 0ll;
    static int64_t transcodingTime = 0ll;
    static int64_t filteringTime = 0ll;
    //static int64_t lastSerialNumber = 0, c = -1;

#if 1
    //if(frame->serialNumber > 0XFFFC)
        LOG_INFO(LOG_TAG, "[# COLOR #] color_image_callback: S/N=%" PRIu32 "", frame->serialNumber);
        //if(frame->serialNumber - lastSerialNumber > 1)
        //	c++;
        //LOG_INFO(LOG_TAG, "Drop frame count = %d (%d->%d)", c, lastSerialNumber, frame->serialNumber);
        //lastSerialNumber = frame->serialNumber;
#endif
#if 0
    frame->toStringSimple(buffer, sizeof(buffer));
    LOG_INFO(LOG_TAG": color_image_callback", "Color image: %s", buffer); 
#endif

#if 1
    if((count++ % DURATION) == 0)    {
        if(count != 1)    {
            int64_t temp = 0ll;
            LOG_INFO(LOG_TAG, "Color image trancoding cost average: %" PRId64 " ms...",
                     (transcodingTime / 1000 / DURATION));

            temp = (frame->tsUs - time) / 1000 / DURATION;
            LOG_INFO(LOG_TAG, "Color image cost average: %" PRId64 " ms, %" PRId64 " fps...",
                     temp, ((int64_t)1000) / temp);    
        }

        time = frame->tsUs;
        transcodingTime = frame->rgbTranscodingTimeUs;
    } else    {
        transcodingTime += frame->rgbTranscodingTimeUs;
    }
#endif

    return true;
}

static bool depth_image_callback(const libeYs3D::video::Frame* frame)    {
    char buffer[1024];
    static int64_t count = 0ll;
    static int64_t time = 0ll;
    static int64_t transcodingTime = 0ll;
    static int64_t filteringTime = 0ll;
    
#if 1
    //if(frame->serialNumber > 0XFFFC)
        LOG_INFO(LOG_TAG, "[# DEPTH #] depth_image_callback: S/N=%" PRIu32 "", frame->serialNumber);
#endif
#if 0
    frame->toStringSimple(buffer, sizeof(buffer));
    LOG_INFO(LOG_TAG": depth_image_callback", "%s", buffer);
#endif
#if 1
    if((count++ % DURATION) == 0)    {
        if(count != 1)    {
            int64_t temp = 0ll;
            LOG_INFO(LOG_TAG, "Depth image trancoding cost average: %" PRId64 " ms...",
                     (transcodingTime / 1000 / DURATION));
            LOG_INFO(LOG_TAG, "Depth image filtering cost average: %" PRId64 " ms...",
                     (filteringTime / 1000 / DURATION));
            temp = (frame->tsUs - time) / 1000 / DURATION;
            LOG_INFO(LOG_TAG, "Depth image cost average: %" PRId64 " ms, %" PRId64 " fps...",
                     temp, ((int64_t)1000) / temp);    
        }
        
        time = frame->tsUs;
        transcodingTime = frame->rgbTranscodingTimeUs;
        filteringTime = frame->filteringTimeUs;
    } else    {
        transcodingTime += frame->rgbTranscodingTimeUs;
        filteringTime += frame->filteringTimeUs;
    }
#endif
#if 0

    std::unique_ptr<ModeConfigOptions> mModeConfigOptions;
    ModeConfig::MODE_CONFIG mode_config;    
    mModeConfigOptions = dDevice->getModeConfigOptions();
    mode_config = mModeConfigOptions->GetCurrentModeInfo();

    int d_depthWidth = mode_config.D_Resolution.Width;//D_Resolution.Width;
    int d_depthHeight = mode_config.D_Resolution.Height;
    int *zdTable = nullptr;
    int depth_size = d_depthWidth * d_depthHeight * 3;
    if(zdTable == nullptr)
    zdTable = new int [depth_size];

    memcpy(zdTable, frame->zdDepthVec.data(), frame->zdDepthVec.size() * sizeof(uint16_t));

#if(100)
    int x = 640/2;
    int y = 480/2;
    uint16_t DepthValue = 0;
    uint16_t ZValue = 0;
    int zdDepthVecValue = 0;
    int ZDTableValue = 0;
    int zdTableIndexValue = 0;
    
    DepthValue = frame->getDepth(x,y);
    ZValue = frame->getZValue(DepthValue);
    zdDepthVecValue = frame->zdDepthVec[DepthValue];
    ZDTableValue = frame->nZDTable[DepthValue];
    

    if(x >= 0 && x < d_depthWidth && y >= 0 && y < d_depthHeight){
        zdTableIndexValue = zdTable[y * d_depthWidth + x];
        }
    else
       {zdTableIndexValue = 0;} 
    
    printf("Get Depth: %d \n",DepthValue);
    printf("Get ZValue: %d \n",ZValue);
    printf("zdTable vector for depth index: %d \n", zdDepthVecValue);
    printf("camera zdTable for dpeth index: %d\n", ZDTableValue);
    printf("ZValue from zdDepthVec data: %d\n", zdTableIndexValue);

#endif
#endif
    return true;
}

static bool pc_frame_callback(const libeYs3D::video::PCFrame *pcFrame)    {
    char buffer[2048];
    static int64_t count = 0ll;
    static int64_t time = 0ll;
    static int64_t transcodingTime = 0ll;

#if 1
    //if(pcFrame->serialNumber > 0XFFFC)
        LOG_INFO(LOG_TAG, "[# PC #] pc_image_callback: S/N=%" PRIu32 "", pcFrame->serialNumber);
#endif

#if 0
    pcFrame->toStringSimple(buffer, sizeof(buffer));
    LOG_INFO(LOG_TAG": pc_frame_callback", "%s", buffer);
#endif

#if 0
    std::vector<CloudPoint> cloud;
    for (int i = 0; i < pcFrame->rgbDataVec.size(); i+=3) {
        if (pcFrame->xyzDataVec[i+2] >= 0) {
            CloudPoint point = {
                    pcFrame->xyzDataVec[i],pcFrame->xyzDataVec[i+1], pcFrame->xyzDataVec[i+2],
                    pcFrame->rgbDataVec[i], pcFrame->rgbDataVec[i+1], pcFrame->rgbDataVec[i+2]
            };
            cloud.push_back(point);
        }
    }
    std::string file = std::to_string(pcFrame->serialNumber);
    file.append(".ply");
    PlyWriter::writePly(cloud, file);
#endif
#if 0
    if((count++ % DURATION) == 0)    {
        if(count != 1)    {
            int64_t temp = 0ll;
            LOG_INFO(LOG_TAG, "PC image trancoding cost average: %" PRId64 " ms...",
                     (transcodingTime / 1000 / DURATION));
                     
            temp = (pcFrame->tsUs - time) / 1000 / DURATION;
            LOG_INFO(LOG_TAG, "PC image cost average: %" PRId64 " ms, %" PRId64 " fps...",
                     temp, ((int64_t)1000) / temp);
        }
        
        time = pcFrame->tsUs;
        transcodingTime = pcFrame->transcodingTime;
    } else    {
        transcodingTime += pcFrame->transcodingTime;
    }
#endif

    return true;
}

static bool imu_data_callback(const libeYs3D::sensors::SensorData *sensorData)    {
    char buffer[2048];
#if 1
    //if(sensorData->serialNumber > 0XFFFC)
        LOG_INFO(LOG_TAG, "[# IMU #] imu_data_callback: S/N=%" PRIu32 "", sensorData->serialNumber);
#endif
#if 0
    sensorData->toString(buffer, sizeof(buffer));
    LOG_INFO(LOG_TAG, "%s", buffer);
#endif

    return true;
}

int TransformDepthDataType(int depth_raw_data_type , int bRectifyMode , unsigned short wPID , int depthWidth , int depthHeight)
{
	switch (depth_raw_data_type){
		case 8:
			depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS;
			switch (depth_raw_data_type){
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_RAW:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_8_BITS:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_8_BITS_RAW:
					depth_raw_data_type = bRectifyMode ? libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS : libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_RAW; 
					break;
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_x80:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_x80_RAW:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_8_BITS_x80:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_8_BITS_x80_RAW:
					depth_raw_data_type = bRectifyMode ? libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_x80 : libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_x80_RAW; break;
			}
			break;
        case 11:
			depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS;
			switch (depth_raw_data_type){
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS_RAW:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS_COMBINED_RECTIFY:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_11_BITS:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_11_BITS_RAW:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_11_BITS_COMBINED_RECTIFY:
					depth_raw_data_type = bRectifyMode ? libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS : libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS_RAW;  break;
			}
			break;
		case 14:
			depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS;
			switch (depth_raw_data_type){
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS_RAW:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS_COMBINED_RECTIFY:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_14_BITS:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_14_BITS_RAW:
				case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_14_BITS_COMBINED_RECTIFY:
					depth_raw_data_type = bRectifyMode ? libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS : libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS_RAW;  break;
			}
			break;
		default:
			depth_raw_data_type = bRectifyMode ? libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_DEFAULT : libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RECTIFY;  break;
			/*case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RAW:
			case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RECTIFY:
			case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_OFF_RAW:
			case libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_ILM_OFF_RECTIFY:
				depth_raw_data_type = bRectifyMode ? libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RECTIFY : libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RAW;  break;*/
    }
    //##############only happen in device 8036, need to handle when Height = 360#####################
    if (depth_raw_data_type == libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS)
    {
		if((wPID == 0x0120 || wPID == 0x0137) && depthWidth == 640 && depthHeight == 360)
        {
			depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS;
        }
    }
    else if (depth_raw_data_type == libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS)
    {
        if((wPID == 0x0120 || wPID == 0x0137) && depthWidth == 640 && depthHeight == 360)
        {
            depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_14_BITS;
        }
    }
    //##############only happen in device 8036, need to handle when Height = 360#####################

    //if (isInterleaveModeEnabled()) mDepthFormat += libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_DATA_INTERLEAVE_MODE_OFFSET;
    return depth_raw_data_type;
}

int main(int argc, char** argv)    {

	static int c = 0;
start:
{  // <--- it is important to tell the compiler the scope of local memory.

#if 1
    LOG_INFO(LOG_TAG, "Starting EYS3DSystem...");
    //LOG_ERR(LOG_TAG, "This is error test");
    std::shared_ptr<EYS3DSystem> eYs3DSystem = std::make_shared<EYS3DSystem>(EYS3DSystem::COLOR_BYTE_ORDER::COLOR_BGR24);
    //std::shared_ptr<EYS3DSystem> eYs3DSystem = EYS3DSystem::getEYS3DSystemRGBOrder(EYS3DSystem::COLOR_BYTE_ORDER::COLOR_BGR24);
    if(0 == eYs3DSystem->getCameraDeviceCount())    {
        LOG_ERR(LOG_TAG, "NONE camera device found...");

#if 1
		sleep(3);
		goto start;
#else
        exit(-1);
#endif
	}

    std::shared_ptr<libeYs3D::devices::CameraDevice> device = eYs3DSystem->getCameraDevice(0);
    dDevice = device;
	CameraDeviceInfo mCameraDeviceInfo;
	mCameraDeviceInfo = device->getCameraDeviceInfo();
    if(!device)    {
        LOG_INFO(LOG_TAG, "Unable to find any camera devices...");
        exit(-1);
    }
#if 0
    unsigned short nFW_Value = 0;
    unsigned short nHW_Value = 0;
    unsigned short nSensor_Value = 0;
    LOG_INFO(LOG_TAG, "============START TEST REGISTER============");
    device->setFWRegister(0xe0,1);
    device->setHWRegister(0x424,10);
    //device->setSensorRegister(0x305E,2,SENSOR_BOTH,0x30);
    nFW_Value = device->getFWRegister(0xe0);
    nHW_Value = device->getHWRegister(0x424);
    //nSensor_Value = device->getSensorRegister(0x305e,SENSOR_BOTH,0x30);
    LOG_INFO(LOG_TAG, "FW Value:%d",nFW_Value);
    LOG_INFO(LOG_TAG, "HW Value:%d",nHW_Value);
    //LOG_INFO(LOG_TAG, "Sensor Value:%d",nSensor_Value);
    LOG_INFO(LOG_TAG, "============END TEST REGISTER============");
#endif

#if 0 //Depth Filter Options Test for Open CL
bool mIsdepthfilterEnable;
libeYs3D::devices::DepthFilterOptions Mmdepthfilteroptions = device->getDepthFilterOptions();
Mmdepthfilteroptions.enableEdgePreServingFilter(false);
mIsdepthfilterEnable = Mmdepthfilteroptions.isEdgePreServingFilterEnabled();
printf("========================Filter result: %d========================\n",mIsdepthfilterEnable); 

#endif

#ifdef WIN32
    std::unique_ptr<ModeConfigOptions> mModeConfigOptions;
	ModeConfig::MODE_CONFIG mode_config;
	std::vector< ModeConfig::MODE_CONFIG > m_modeConfigs;
	int depth_raw_data_type;
	camera_open_config config;
	bool haveColor = true;
	bool haveDepth = true;
	bool useInterleaveMode = false;
#endif

    int retry = 1;
    while(retry-- > 0)    {
        int ret = 0;
        LOG_INFO(LOG_TAG, "\n\nEnabling device stream...\n");
#ifdef WIN32
	int pif = 1;
	int i, mode_index;
	int depth_raw_index = 0, color_fps_index = 0, depth_fps_index = 0;
	mModeConfigOptions = device->getModeConfigOptions();
	m_modeConfigs = mModeConfigOptions->GetModes();
	printf("mode count = %d\n", m_modeConfigs.size());
	for(i = 0; i < m_modeConfigs.size(); i++)
		printf("[%d]", m_modeConfigs[i].iMode);
	printf("\n");
	for(i = m_modeConfigs.size() - 1; i >= 0; i--){
		mode_index = i;
		if(m_modeConfigs[mode_index].iMode == pif)
			break;
	}
	mode_index = (mode_index < 0 || mode_index >= m_modeConfigs.size()) ? 0 : mode_index;
	printf("selected PIF = %d\n", m_modeConfigs[mode_index].iMode);
	mModeConfigOptions->SelectCurrentIndex(m_modeConfigs[mode_index].iMode);
	mode_config = mModeConfigOptions->GetCurrentModeInfo();
	haveColor = (mode_config.L_Resolution.Width > 0) ? true : false;
	haveDepth = (mode_config.D_Resolution.Width > 0) ? true : false;
	printf("haveColor=%d, haveDepth=%d\n", haveColor, haveDepth);
	printf("iMode=%d, iUSB_Type=%d, iInterLeaveModeFPS=%d, bRectifyMode=%d\n"
		  , mode_config.iMode, mode_config.iUSB_Type, mode_config.iInterLeaveModeFPS, mode_config.bRectifyMode);
	printf("eDecodeType_L=%d, L_Resolution.Width=%d ,L_Resolution.Height=%d, D_Resolution.Width=%d, D_Resolution.Height=%d, vecDepthType=%d, vecColorFps=%d, vecDepthFps=%d\n"
	      , mode_config.eDecodeType_L, mode_config.L_Resolution.Width, mode_config.L_Resolution.Height, mode_config.D_Resolution.Width, mode_config.D_Resolution.Height
	      , (mode_config.vecDepthType.size() > 0) ? mode_config.vecDepthType.at(0) : 0, (haveColor) ? mode_config.vecColorFps.at(0) : 0, (haveDepth) ? mode_config.vecDepthFps.size() : 0);
	printf("mode_config.vecDepthType.size()=%d\n", mode_config.vecDepthType.size());
	config.depthHeight = mode_config.D_Resolution.Height;
	config.depthWidth = mode_config.D_Resolution.Width;
	if(mode_config.vecDepthType.size() == 0)
		depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RAW;
	else{
		depth_raw_index = (depth_raw_index < 0 || depth_raw_index >= mode_config.vecDepthType.size()) ? 0 : depth_raw_index;
		depth_raw_data_type = TransformDepthDataType(mode_config.vecDepthType.at(depth_raw_index),mode_config.bRectifyMode,mCameraDeviceInfo.devInfo.wPID,config.depthWidth,config.depthHeight);
	}
	
	config.colorFormat = mode_config.eDecodeType_L == ModeConfig::MODE_CONFIG::YUYV? libeYs3D::video::COLOR_RAW_DATA_YUY2 : libeYs3D::video::COLOR_RAW_DATA_MJPG;
	config.colorWidth = mode_config.L_Resolution.Width;
	config.colorHeight = mode_config.L_Resolution.Height;
	config.depthWidth = mode_config.D_Resolution.Width;
	config.depthHeight = mode_config.D_Resolution.Height;
	config.videoMode = depth_raw_data_type;
	printf("vecColorFps.size()=%d, vecDepthFps.size()=%d\n", mode_config.vecColorFps.size(), mode_config.vecDepthFps.size());
	printf("vecColorFps=");
	for(i = 0; i < mode_config.vecColorFps.size(); i++)
		printf("[%d]", mode_config.vecColorFps.at(i));
	printf("\n");
	if(haveColor){
		if (mode_config.iInterLeaveModeFPS > 0) {
			for ( i = mode_config.vecColorFps.size() - 1 ; i >= 0 ; i--){
				if (mode_config.vecColorFps.at(i) == mode_config.iInterLeaveModeFPS){
					color_fps_index = i;
					break;
				}
			}
		}
		color_fps_index = (color_fps_index < 0 || color_fps_index >= mode_config.vecColorFps.size()) ? 0 : color_fps_index;
		config.fps = mode_config.vecColorFps.at(color_fps_index);
	}
	else if(haveDepth){
		if (mode_config.iInterLeaveModeFPS > 0) {
			for ( i = mode_config.vecDepthFps.size() - 1 ; i >= 0 ; i--){
				if (mode_config.vecDepthFps.at(i) == mode_config.iInterLeaveModeFPS){
					depth_fps_index = i;
					break;
				}
			}
		}
		depth_fps_index = (depth_fps_index < 0 || depth_fps_index >= mode_config.vecDepthFps.size()) ? 0 : depth_fps_index;
		config.fps = mode_config.vecDepthFps.at(depth_fps_index);
	}
#if 0
	if(mode_config.iInterLeaveModeFPS == config.fps)
		useInterleaveMode = true;
	else
		useInterleaveMode = false;
#endif
	printf("vecDepthFps=");
	for(i = 0; i < mode_config.vecDepthFps.size(); i++)
		printf("[%d]", mode_config.vecDepthFps.at(i));
	printf("\n");

	printf("config: {%d, %d, %d, %d, %d, %d, %d} depth_raw_data_type=%d (%d, %d, %d, %d, %d)\n", config.colorFormat, config.colorWidth, 
		config.colorHeight, config.fps, config.depthWidth, config.depthHeight, config.videoMode, depth_raw_data_type, 
		libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS,
		libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
		libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS,
		libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS,
		libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_14_BITS);
	printf("config.videoMode = %d (%d bits)\n", config.videoMode, 
	         (config.videoMode == libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS) ? 8 : 
	         ((config.videoMode == libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS | libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS) ? 11 : 
	         ((config.videoMode == libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS | libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_14_BITS) ? 14 : 0)));

		ret = device->initStream((libeYs3D::video::COLOR_RAW_DATA_TYPE)config.colorFormat,
						         config.colorWidth, config.colorHeight, config.fps,
						         (libeYs3D::video::DEPTH_RAW_DATA_TYPE)config.videoMode,
						         config.depthWidth, config.depthHeight,
						         DEPTH_IMG_COLORFUL_TRANSFER,
						         IMAGE_SN_SYNC,
						         0, // rectifyLogIndex
						         color_image_callback,
						         depth_image_callback,
						         pc_frame_callback,
                                 imu_data_callback);
#if 0
		ret = device->initStream((libeYs3D::video::COLOR_RAW_DATA_TYPE)mode_config.eDecodeType_L,
						         mode_config.L_Resolution.Width, mode_config.L_Resolution.Height, mode_config.vecColorFps.at(0),
						         (libeYs3D::video::DEPTH_RAW_DATA_TYPE)depth_raw_data_type,
						         mode_config.D_Resolution.Width, mode_config.D_Resolution.Height,
						         (DEPTH_TRANSFER_CTRL)DEPTH_IMG_COLORFUL_TRANSFER,
						         IMAGE_SN_SYNC,
						         0, // rectifyLogIndex
						         color_image_callback,
						         depth_image_callback,
						         pc_frame_callback,
                                 imu_data_callback);
#endif
#else
#if 0 // USB3 with 640 X 480
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 640, 480, 90,
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 640, 480,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#if 1 // USB3
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 1280, 720, 60, /* or 24 */
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 1280, 720,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#if 0   // 8073 sample interleave L' + D
        // device->enableInterleaveMode(true); // Un-comment this to enable Interleave Mode.

        device->enableExtendIR(true);
        auto irProp = device->getIRProperty();
        irProp.setIRValue(96);
        device->setIRProperty(irProp);

        /* Spatial Filter Only Support D11 */
        PostProcessOptions& processOptions = device->getPostProcessOptions();
        processOptions.enable(true);
        processOptions.setSpatialOutlierThreshold(3);
        processOptions.setSpatialFilterKernelSize(7); // Should be odd number
        processOptions.setDecimationFactor(2);
        processOptions.setColorResizeFactor(0.5f);
        processOptions.enableColorPostProcess(true);

        device->setPostProcessOptions(processOptions);

        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 1104, 848, 30,
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 1104, 848,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 1, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 nullptr,
                                 nullptr);
#endif

#if 0   // 8036 scale down test
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 1280, 720, 30,
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS,
                                 640, 360,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 nullptr);
#endif
#if 0 // 8059 USB2 (Mode 9)
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_MJPG,
                                 1280, 720, 24,
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 640, 360,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#if 0 // 8052 USB2 (Mode 51)
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_MJPG,
                                 1280, 720, 30,
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_RAW,
                                 320, 480,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#if 0 // USB2
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_MJPG,
                                 1280, 720, 30, /* or 24 */
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 640, 360,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 nullptr, //depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#if 0 // depth only
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 0, 0, 60, /* or 24 */
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 1280, 720,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#if 0 // color only
        ret = device->initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 2560, 720, 60,
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::APC_DEPTH_DATA_OFF_RAW,
                                 0, 0,
//                                 DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0, // rectifyLogIndex
                                 color_image_callback,
                                 depth_image_callback,
                                 pc_frame_callback,
                                 imu_data_callback);
#endif
#endif

        if(ret != 0)    break;
                         
        device->enableStream();

#if 0   // Test PostProcessOptions enable disable on streaming
        sleep(10);
        processOptions.enable(false);
        device->setPostProcessOptions(processOptions);
        sleep(10);
        processOptions.enable(true);
        device->setPostProcessOptions(processOptions);
        sleep(5);
        processOptions.enableColorPostProcess(true);
        device->setPostProcessOptions(processOptions);
        sleep(5);
        processOptions.enableColorPostProcess(false);
        device->setPostProcessOptions(processOptions);
#endif


#if 0
        { // Stream enablement testing
            device->pauseColorStream();
            sleep(2);
            device->pauseDepthStream();
            sleep(2);
            device->pauseIMUStream();
            sleep(4);
            
            device->enableInterleaveMode(true);
            sleep(4);
            
            device->pausePCStream();
            sleep(4);
            
            device->enableInterleaveMode(false);
            device->enablePCStream();
            sleep(4);
            device->enableColorStream();
            sleep(4);
            device->enableDepthStream();
            sleep(4);
            device->enableIMUStream();
            sleep(4);
        }
#endif
#if 0
        { // test IMU data dump
            device->dumpIMUData();
            char* imulogpath;
            imulogpath = device->mIMUDevice->getIMULogPath();
            printf("============log path: %s\n",imulogpath);
            sleep(4);
        }
#endif       

#if 0       
        { // verify snapshut
            if(device->isPlyFilterSupported())    {
                LOG_INFO(LOG_TAG, "Ply filter is supported, enable it...");
                device->enablePlyFilter(true);
            }
            
            device->doSnapshot(0);//Color snaptshot
            sleep(2);
            device->doSnapshot(1);//Depth snapshot
            sleep(2);
            device->doSnapshot(2);//Point cloud snapshot

            
            //device->enablePlyFilter(false);
            //device->doSnapshot();
            
            sleep(3);
        }
#endif
#if 0
        { // verify register read/write features
            libeYs3D::devices::RegisterReadWriteOptions options = device->getRegisterReadWriteOptions();
            options.enablePeriodicRead(true);
            options.enableSaveLog(true);
            device->setRegisterReadWriteOptionsForRead(options);
            sleep(4);
            
            options.enablePeriodicRead(false);
            options.enableSaveLog(false);
            device->setRegisterReadWriteOptionsForWrite(options);
            sleep(4);
        }
#endif
#if 0
        { // verify depth acuracy 
            libeYs3D::devices::DepthAccuracyOptions depthAccuracyOptions = device->getDepthAccuracyOptions();
            depthAccuracyOptions.enable(true);
            depthAccuracyOptions.setRegionRatio(0.8f);
            depthAccuracyOptions.setGroundTruthDistanceMM(300.0f);
            device->setDepthAccuracyOptions(depthAccuracyOptions);
            sleep(4);
        
            depthAccuracyOptions = device->getDepthAccuracyOptions();
            depthAccuracyOptions.enable(false);
            device->setDepthAccuracyOptions(depthAccuracyOptions);
            sleep(4);
        }
#endif
#if 0
        device->enableInterleaveMode(true);
        sleep(4);
        device->enableInterleaveMode(false);
        sleep(4);
        device->enableInterleaveMode(true);
        sleep(4);
        device->enableInterleaveMode(false);
        sleep(4);
#endif

#if 0
        sleep(1200);
        device->enableInterleaveMode(true);
        sleep(1200);
        device->enableInterleaveMode(false);
        sleep(1200);
        device->enableInterleaveMode(true);
        sleep(1200);
        device->enableInterleaveMode(false);
        sleep(1200);
        
        sleep(10000000);
#endif

#if 0
		{
			struct libeYs3D::devices::CameraDeviceProperties::CameraPropertyItem status;
	    	status = device->getCameraDeviceProperty(libeYs3D::devices::CameraDeviceProperties::CAMERA_PROPERTY::EXPOSURE_TIME);
	    	printf("\n\nget exposure time (Support, Valid, Value, Max, Min, Default, Step): (%d, %d, %d, %d, %d, %d, %d)\n", 
				status.bSupport, status.bValid, status.nValue, status.nMax, status.nMin, status.nDefault, status.nStep);
		}
#endif

        sleep(800);

        LOG_INFO(LOG_TAG, "\n\nClosing device stream...\n");
        device->closeStream();
        sleep(2);
    }

	dDevice = nullptr;  // <-- to tell the compiler we don't use camera device anymore.
	eYs3DSystem.reset();
	//device.reset();
}
	LOG_INFO(LOG_TAG, "\n\nReset EYS3DSystem\n\n");

	if (c == 0) {
		c++;
		sleep(8);
		goto start;
	}

	sleep(2);

    return 0;
#endif
}

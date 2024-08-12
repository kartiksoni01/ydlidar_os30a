/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */
#include "EYS3DSystem.h"
#include "devices/CameraDevice.h"
#include "devices/FrameSetPipeline.h"
#include "video/Frame.h"
#include "base/threads/Async.h"
#include "debug.h"

#ifdef _WIN32
    #include "utils.h"
    #include <magic.h>
#else //_WIN32
    #include <unistd.h>
#endif //_WIN32

#define LOG_TAG "FrameSet Pipeline Test"
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
#endif //WIN32

using namespace libeYs3D;

static void frameset_reader(libeYs3D::devices::FrameSetPipeline *pipeline)     {
    libeYs3D::video::FrameSet frameSet;
    char buffer[1024];
    libeYs3D::devices::FrameSetPipeline::RESULT ret;
    //static int64_t lastSerialNumber = 0, c = -1;
    
    while(true) {
        ret = pipeline->waitForFrameSet(&frameSet);
        if(ret < 0)    break;
        if(ret > 0)    continue;
        
    #if 1
        //if(frameSet.colorFrame.serialNumber > 0XFFF0)    {
            LOG_INFO(LOG_TAG, "[# PC #] S/N=%" PRIu32 "", frameSet.pcFrame.serialNumber);
            LOG_INFO(LOG_TAG, "[# COLOR #] S/N=%" PRIu32 "", frameSet.colorFrame.serialNumber);
            LOG_INFO(LOG_TAG, "[# DEPTH #] S/N=%" PRIu32 "", frameSet.depthFrame.serialNumber);
            //frameSet.pcFrame.toStringSimple(buffer, sizeof(buffer));
            //LOG_INFO(LOG_TAG, "[# PC #] %s", buffer);
        //}
        //if(frameSet.colorFrame.serialNumber - lastSerialNumber > 1)
        //	c++;
        //LOG_INFO(LOG_TAG, "Drop frame count = %d (%d->%d)", c, lastSerialNumber, frameSet.colorFrame.serialNumber);
        //lastSerialNumber = frameSet.colorFrame.serialNumber;
    #endif

    #if 0
        LOG_INFO(LOG_TAG, "[# PC #] S/N=%" PRIu32 "", frameSet.pcFrame.serialNumber);
        LOG_INFO(LOG_TAG, "[# COLOR #] S/N=%" PRIu32 "", frameSet.colorFrame.serialNumber);
        LOG_INFO(LOG_TAG, "[# DEPTH #] S/N=%" PRIu32 ", roiDepth=%" PRId32 ", roiZValue=%" PRId32 "",
                 frameSet.depthFrame.serialNumber,
                 frameSet.depthFrame.roiDepth,
                 frameSet.depthFrame.roiZValue);
    #endif
    }
    
    LOG_INFO(LOG_TAG, "Exiting frameset_reader...");
}

int main(int argc, char** argv)    {
#if 1
    LOG_INFO(LOG_TAG, "Starting EYS3DSystem...");

    //std::shared_ptr<EYS3DSystem> eYs3DSystem = EYS3DSystem::getEYS3DSystem();
    std::shared_ptr<EYS3DSystem> eYs3DSystem = std::make_shared<EYS3DSystem>(EYS3DSystem::COLOR_BYTE_ORDER::COLOR_BGR24);
    if(0 == eYs3DSystem->getCameraDeviceCount())    {
        LOG_ERR(LOG_TAG, "NONE camera device found...");
        exit(-1);
    }
    
    std::shared_ptr<libeYs3D::devices::CameraDevice> device = eYs3DSystem->getCameraDevice(0);
    if(!device)    {
        LOG_INFO(LOG_TAG, "Unable to find any camera devices...");
        exit(-1);
    }
	CameraDeviceInfo mCameraDeviceInfo;
	mCameraDeviceInfo = device->getCameraDeviceInfo();
    std::shared_ptr<libeYs3D::devices::FrameSetPipeline> pipeline;
#ifdef WIN32
	std::unique_ptr<ModeConfigOptions> mModeConfigOptions;
	ModeConfig::MODE_CONFIG mode_config;
	std::vector< ModeConfig::MODE_CONFIG > m_modeConfigs;
	int depth_raw_data_type;
	camera_open_config config;
	bool haveColor = true;
	bool haveDepth = true;
	bool useInterleaveMode = false;
#endif //WIN32
    
    int retry = 100;
    while(retry-- > 0)    {
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
	if(mode_config.vecDepthType.size() == 0)
		depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RAW;
	else{
		depth_raw_index = (depth_raw_index < 0 || depth_raw_index >= mode_config.vecDepthType.size()) ? 0 : depth_raw_index;
		switch(mode_config.vecDepthType.at(depth_raw_index)){
			case 8: depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS;			
			        break;
			case 11:
					if(mCameraDeviceInfo.devInfo.wPID == 0x0120 && config.depthHeight == 360)
					{
						depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS;
					}
					else
					{
						depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS;
					}
			        break;
			case 14:
					if(mCameraDeviceInfo.devInfo.wPID == 0x0120 && config.depthHeight == 360)
					{
						depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_14_BITS;
					}
					else
					{
						depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS;
					}
			        break;
		}
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
		color_fps_index = (color_fps_index < 0 || color_fps_index >= mode_config.vecColorFps.size()) ? 0 : color_fps_index;
		config.fps = mode_config.vecColorFps.at(color_fps_index);
	}
	else if(haveDepth){
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

		pipeline = device->initStreamFS((libeYs3D::video::COLOR_RAW_DATA_TYPE)config.colorFormat,
						         config.colorWidth, config.colorHeight, config.fps, 
						         (libeYs3D::video::DEPTH_RAW_DATA_TYPE)config.videoMode,
						         config.depthWidth, config.depthHeight,
						         DEPTH_IMG_COLORFUL_TRANSFER,
						         IMAGE_SN_SYNC,
						         0);
    #if 0
		pipeline = device->initStreamFS((libeYs3D::video::COLOR_RAW_DATA_TYPE)mode_config.eDecodeType_L,
						         mode_config.L_Resolution.Width, mode_config.L_Resolution.Height, mode_config.vecColorFps.at(0), 
						         (libeYs3D::video::DEPTH_RAW_DATA_TYPE)depth_raw_data_type,
						         mode_config.D_Resolution.Width, mode_config.D_Resolution.Height,
						         (DEPTH_TRANSFER_CTRL)DEPTH_IMG_COLORFUL_TRANSFER,
						         IMAGE_SN_SYNC,
						         0);
    #endif
#else //WIN32
    #if 1
        pipeline = device->initStreamFS(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 1280, 720, 60, /* or 24 */
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 1280, 720,
                                 //DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0);
    #endif

    #if 0 // USB2
        pipeline = device->initStreamFS(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_MJPG,
                                 1280, 720, 12, /* or 30 */
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 640, 360,
                                 //DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0);
    #endif

    #if 0 // depth only
        pipeline = device->initStreamFS(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                 0, 0, 60, /* or 24 */
                                 libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                 1280, 720,
                                 //DEPTH_IMG_NON_TRANSFER,
                                 DEPTH_IMG_COLORFUL_TRANSFER,
                                 IMAGE_SN_SYNC,
                                 0);
    #endif

    #if 0 // color only
        pipeline = device->initStreamFS(libeYs3D::video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2,
                                   2560, 720, 60,
                                   libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS,
                                   0, 0,
                                   //DEPTH_IMG_NON_TRANSFER,
                                   DEPTH_IMG_COLORFUL_TRANSFER,
                                   IMAGE_SN_SYNC,
                                   0);
    #endif

#endif //WIN32

        if(pipeline == nullptr)    break;
        
        base::async([&]() { frameset_reader(pipeline.get()); });

        device->enableStream();
        sleep(100000);

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
        { // verify ROI
            device->setDepthROIPixels(40);
            device->setDepthROICenterPoint(1280, 720);
            sleep(4);
        
            device->setDepthROIPixels(8);
            device->setDepthROICenterPoint(-1, -1);
            sleep(4);
            
            device->setDepthROIPixels(8);
            device->setDepthROICenterPoint(-1, 720);
            sleep(4);
            
            device->setDepthROIPixels(8);
            device->setDepthROICenterPoint(1280, -1);
            sleep(4);
            
            device->setDepthROIPixels(8);
            device->setDepthROICenterPoint(640, 320);
            sleep(4);
        }
    #endif

    #if 0
        sleep(12);
        device->enableInterleaveMode(true);
        sleep(12);
        device->enableInterleaveMode(false);
        sleep(1200);
        device->enableInterleaveMode(true);
        sleep(1200);
        device->enableInterleaveMode(false);
        sleep(1200);
    #endif

        LOG_INFO(LOG_TAG, "\n\nClosing device stream...\n");
        device->closeStream();

        sleep(2);
    }
#endif //0    
    return 0;
}

#-------------------------------------------------
#
# Project created by QtCreator 2020-03-24T09:27:12
#
#-------------------------------------------------

QT       += core gui dbus

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DMPreview
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += $$PWD/model/
INCLUDEPATH += $$PWD/model/module/
INCLUDEPATH += $$PWD/view/
INCLUDEPATH += $$PWD/controller/
INCLUDEPATH += $$PWD/manager/
INCLUDEPATH += $$PWD/utility/
INCLUDEPATH += $$PWD/utility/sqlite3/
INCLUDEPATH += $$PWD/../eSPDI/

LIBS += -L$$PWD/../eSPDI -leSPDI_TI_32
LIBS += -lrt -lm -ldl -lpthread -ludev
CONFIG += c++11

QMAKE_LFLAGS += '-Wl,-rpath,\'\$$ORIGIN/../eSPDI\''

QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter
QMAKE_CXXFLAGS_WARN_ON += -Wno-reorder
QMAKE_CXXFLAGS += -ffast-math

FORMS += \
    view/mainwindow.ui \
    view/CVideoDeviceDialog.ui \
    view/CVideoDevicePreviewWidget.ui \
    view/CVideoDeviceRegisterWidget.ui \
    view/CVideoDeviceCameraPropertyWidget.ui \
    view/CVideoDeviceIMUWidget.ui \
    view/CVideoDeviceAudioWidget.ui \
    view/CVideoDeviceDepthAccuracyWidget.ui \
    view/CVideoDeviceDepthFilterWidget.ui

HEADERS += \
    view/mainwindow.h \
    view/CVideoDeviceDialog.h \
    model/CVideoDeviceModel.h \
    controller/CVideoDeviceController.h \
    manager/CEYSDDeviceManager.h \
    model/CCameraPropertyModel.h \
    utility/PlyWriter.h \
    model/PreviewOptions.h \
    model/CIMUModel.h \
    model/CVideoDeviceModelFactory.h \
    view/CVideoDevicePreviewWidget.h \
    view/CEYSDUIView.h \
    manager/CThreadWorkerManage.h \
    manager/CTaskThread.h \
    manager/CTaskInfoManager.h \
    utility/utDisplayMetrics.h \
    view/CPreviewDialog.h \
    model/CImageDataModel.h \
    utility/utImageProcessingUtility.h \
    utility/ColorPaletteGenerator.h \
    view/CMessageDialog.h \
    manager/CMessageManager.h \
    utility/PlyFilter.h \
    view/CVideoDeviceRegisterWidget.h \
    utility/RegisterSettings.h \
    controller/CRegisterReadWriteController.h \
    model/RegisterReadWriteOptions.h \
    controller/CCameraPropertyController.h \
    view/CVideoDeviceCameraPropertyWidget.h \
    utility/ModeConfig.h \
    utility/sqlite3/sqlite3.h \
    utility/sqlite3/sqlite3ext.h \
    model/ModeConfigOptions.h \
    model/module/CVideoDeviceModel_8037.h \
    model/module/CVideoDeviceModel_8040_8054.h \
    model/module/CVideoDeviceModel_8053_8059.h \
    model/module/CVideoDeviceModel_8054.h \
    model/module/CVideoDeviceModel_ColorWithDepth.h \
    model/module/CVideoDeviceModel_Kolor.h \
    model/module/CVideoDeviceModel_8040.h \
    utility/IMUData.h \
    utility/hidapi/hidapi.h \
    view/CVideoDeviceIMUWidget.h \
    controller/CIMUDataController.h \
    model/module/CVideoDeviceModel_8038.h \
    utility/DepthFusionHelper.h \
    model/module/CVideoDeviceModel_8060.h \
    view/CVideoDeviceAudoWidget.h \
    model/module/CVideoDeviceModel_8029.h \
    model/module/CVideoDeviceModel_8036_8052.h \
    model/module/CVideoDeviceModel_8053.h \
    view/CPointCloudViewerWidget.h \
    utility/OpenGLShaderCore.h \
    view/CPointCloudViewerDialog.h \
    utility/FrameGrabber.h \
    utility/FPSCalculator.h \
    utility/ArcBall.h \
    utility/ArcBallMatrix.h \
    utility/OGLBasic.h \
    controller/CDepthAccuracyController.h \
    view/CVideoDeviceDepthAccuracyWidget.h \
    view/CIMUDataViewerWidget.h \
    model/DepthFilterOptions.h \
    controller/CDepthFilterController.h \
    view/CVideoDeviceDepthFilterWidget.h \
    model/module/CVideoDeviceModel_8051.h \
    model/module/CVideoDeviceModel_8062.h \
    model/module/CVideoDeviceModel_8063.h \
    model/module/CVideoDeviceModel_Hypatia.h \
    model/module/CVideoDeviceModel_Hypatia2.h \
    model/module/CVideoDeviceModel_Nora.h \
    model/module/CVideoDeviceModel_8052.h \
    model/module/CVideoDeviceModel_8036.h \
    manager/CIMUDeviceManager.h \
    utility/IMU/IMU_Filter/IMUFilter_AHRS.h \
    utility/IMU/IMU_Filter/IMUFilter_Complementary.h \
    utility/IMU/IMU_Filter/IMUFilter.h \
    utility/IMU/IMU_Filter/StatelessOrientation.h \
    utility/IMU/IMU_Filter/IMUFilter_EYS3D_AHRS.h \
    utility/IMU/IMU_Filter/quaternion.h \
    utility/IMU/IMU_Filter/Quaternion1.h \
    utility/IMU/IMU_Calibration/CIMUCalibration.h \
    manager/CFrameSyncManager.h \
    model/module/CVideoDeviceModel_Grap.h \
    model/module/CVideoDeviceModel_8073.h

SOURCES += \
    view/mainwindow.cpp \
    main.cpp \
    view/CVideoDeviceDialog.cpp \
    model/CVideoDeviceModel.cpp \
    controller/CVideoDeviceController.cpp \
    manager/CEYSDDeviceManager.cpp \
    model/CCameraPropertyModel.cpp \
    utility/PlyWriter.cpp \
    model/PreviewOptions.cpp \
    model/CIMUModel.cpp \
    model/CVideoDeviceModelFactory.cpp \
    view/CVideoDevicePreviewWidget.cpp \
    view/CEYSDUIView.cpp \
    manager/CThreadWorkerManage.cpp \
    manager/CTaskThread.cpp \
    manager/CTaskInfoManager.cpp \
    utility/utDisplayMetrics.cpp \
    view/CPreviewDialog.cpp \
    model/CImageDataModel.cpp \
    utility/utImageProcessingUtility.cpp \
    utility/ColorPaletteGenerator.cpp \
    view/CMessageDialog.cpp \
    manager/CMessageManager.cpp \
    utility/PlyFilter.cpp \
    view/CVideoDeviceRegisterWidget.cpp \
    utility/RegisterSettings.cpp \
    controller/CRegisterReadWriteController.cpp \
    model/RegisterReadWriteOptions.cpp \
    controller/CCameraPropertyController.cpp \
    view/CVideoDeviceCameraPropertyWidget.cpp \
    utility/ModeConfig.cpp \
    utility/sqlite3/sqlite3.c \
    model/ModeConfigOptions.cpp \
    model/module/CVideoDeviceModel_8037.cpp \
    model/module/CVideoDeviceModel_8040_8054.cpp \
    model/module/CVideoDeviceModel_8053_8059.cpp \
    model/module/CVideoDeviceModel_8054.cpp \
    model/module/CVideoDeviceModel_ColorWithDepth.cpp \
    model/module/CVideoDeviceModel_Kolor.cpp \
    model/module/CVideoDeviceModel_8040.cpp \
    utility/IMUData.cpp \
    utility/hidapi/hid_udev.c \
    view/CVideoDeviceIMUWidget.cpp \
    controller/CIMUDataController.cpp \
    model/module/CVideoDeviceModel_8038.cpp \
    utility/DepthFusionHelper.cpp \
    model/module/CVideoDeviceModel_8060.cpp \
    view/CVideoDeviceAudoWidget.cpp \
    model/module/CVideoDeviceModel_8029.cpp \
    model/module/CVideoDeviceModel_8036_8052.cpp \
    model/module/CVideoDeviceModel_8053.cpp \
    view/CPointCloudViewerWidget.cpp \
    view/CPointCloudViewerDialog.cpp \
    utility/FrameGrabber.cpp \
    utility/FPSCalculator.cpp \
    utility/ArcBall.cpp \
    utility/ArcBallMatrix.cpp \
    controller/CDepthAccuracyController.cpp \
    view/CVideoDeviceDepthAccuracyWidget.cpp \
    view/CIMUDataViewerWidget.cpp \
    model/DepthFilterOptions.cpp \
    controller/CDepthFilterController.cpp \
    view/CVideoDeviceDepthFilterWidget.cpp \
    model/module/CVideoDeviceModel_8051.cpp \
    model/module/CVideoDeviceModel_8062.cpp \
    model/module/CVideoDeviceModel_8063.cpp \
    model/module/CVideoDeviceModel_Hypatia.cpp \
    model/module/CVideoDeviceModel_Hypatia2.cpp \
    model/module/CVideoDeviceModel_Nora.cpp \
    model/module/CVideoDeviceModel_8052.cpp \
    model/module/CVideoDeviceModel_8036.cpp \
    manager/CIMUDeviceManager.cpp \
    utility/IMU/IMU_Filter/IMUFilter_AHRS.cpp \
    utility/IMU/IMU_Filter/IMUFilter_Complementary.cpp \
    utility/IMU/IMU_Filter/StatelessOrientation.cpp \
    utility/IMU/IMU_Filter/IMUFilter_EYS3D_AHRS.cpp \
    utility/IMU/IMU_Filter/quaternion.cpp \
    utility/IMU/IMU_Filter/Quaternion1.c \
    utility/IMU/IMU_Calibration/CIMUCalibration.cpp \
    manager/CFrameSyncManager.cpp \
    model/module/CVideoDeviceModel_Grap.cpp \
    model/module/CVideoDeviceModel_8073.cpp

RESOURCES += \
    resource/resource.qrc

DEFINES += UAC_X86_SUPPORTED
DEFINES += OPENCL_SUPPORTED
DEFINES += TI_EVM

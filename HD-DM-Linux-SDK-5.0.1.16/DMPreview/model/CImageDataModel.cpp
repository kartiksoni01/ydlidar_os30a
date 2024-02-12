#include "CImageDataModel.h"
#include "utImageProcessingUtility.h"
#include <string.h>
#include "CVideoDeviceController.h"
#include "CEYSDDeviceManager.h"
#include "CThreadWorkerManage.h"
#include <math.h>

CImageDataModel::CImageDataModel(CVideoDeviceModel::STREAM_TYPE streamType, TYPE modelType, CVideoDeviceController *pVideoDeviceController):
m_nWidth(0),
m_nHeight(0),
m_nDataSize(0),
m_nSerialNumber(0),
m_nSpecificX(0), m_nSpecificY(0),
m_imageType(APCImageType::IMAGE_UNKNOWN),
m_pVideoDeviceController(pVideoDeviceController),
m_pUserData(nullptr),
m_streamType(streamType),
m_type(modelType),
m_pDataTransformTask(nullptr)
{
    switch(streamType){
        case CVideoDeviceModel::STREAM_COLOR: m_sDataName = "Color"; break;
        case CVideoDeviceModel::STREAM_COLOR_SLAVE: m_sDataName = "Slave Color"; break;
        case CVideoDeviceModel::STREAM_KOLOR: m_sDataName = "Kolor"; break;
        case CVideoDeviceModel::STREAM_KOLOR_SLAVE: m_sDataName = "Slave Kolor"; break;
        case CVideoDeviceModel::STREAM_TRACK: m_sDataName = "Track"; break;
        case CVideoDeviceModel::STREAM_DEPTH: m_sDataName = "Depth"; break;
        case CVideoDeviceModel::STREAM_DEPTH_30mm: m_sDataName = "Depth_30mm"; break;
        case CVideoDeviceModel::STREAM_DEPTH_60mm: m_sDataName = "Depth_60mm"; break;
        case CVideoDeviceModel::STREAM_DEPTH_150mm: m_sDataName = "Depth_150mm"; break;
        case CVideoDeviceModel::STREAM_DEPTH_FUSION: m_sDataName = "Depth_Fusion"; break;
        //+[Thermal device]
        case CVideoDeviceModel::STREAM_THERMAL: m_sDataName = "Thermal"; break;
        //-[Thermal device]
        default: break;
    }
}

CImageDataModel::~CImageDataModel()
{
    if (m_pDataTransformTask) CThreadWorkerManage::GetInstance()->RemoveTask(m_pDataTransformTask);
}

bool CImageDataModel::SetImageInfo(APCImageType::Value imageType,
                                   int nWidth, int nHeight)
{
    if (m_imageType == imageType &&
       m_nWidth == nWidth &&
       m_nHeight == nHeight)
        return false;

    m_imageType = imageType;
    m_nWidth = nWidth;
    m_nHeight = nHeight;

    m_rawData.resize(m_nWidth * m_nHeight * GetRawDataBytePerPixel());
    m_rgbData.resize(m_nWidth * m_nHeight * 3);

    m_pDataTransformTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::IMAGE_DATA_RAW_TO_RGB_TRANSFORM, this);
    CThreadWorkerManage::GetInstance()->AddTask(m_pDataTransformTask);

    return true;
}

int CImageDataModel::GetRawDataBytePerPixel()
{
    switch (m_imageType){
        case APCImageType::COLOR_YUY2:
        case APCImageType::COLOR_MJPG:
        case APCImageType::DEPTH_8BITS_0x80:
        case APCImageType::DEPTH_11BITS:
        case APCImageType::DEPTH_14BITS:
            return 2;
        case APCImageType::DEPTH_8BITS:
            return 1;
        case APCImageType::COLOR_RGB24:
            return 3;
        default:
            return 0;
    }
}

QString CImageDataModel::GetImgaeDataInfo()
{
    QString sImageInfo;
    //+[Thermal device]
    if(m_streamType == CVideoDeviceModel::STREAM_THERMAL){
        sImageInfo.sprintf("%s[%d x %d] FPS[%.2f] Temp[%d C]",
                       m_sDataName.toLocal8Bit().data(),
                       m_nWidth, m_nHeight, m_fpsCalculator.GetFPS(), m_nSerialNumber);
     } else {
    //-[Thermal device]
        sImageInfo.sprintf("%s[%d x %d] FPS[%.2f] SN[%d]",
                       m_sDataName.toLocal8Bit().data(),
                       m_nWidth, m_nHeight, m_fpsCalculator.GetFPS(), m_nSerialNumber);
    }
    return sImageInfo;
}

int CImageDataModel::SetRawData(BYTE *pData, int nDataSize, int nSerialNumber)
{
    m_dataMutex.lock();
    memcpy(&m_rawData[0], pData, nDataSize);
    m_dataMutex.unlock();

    m_nDataSize = nDataSize;
    m_nSerialNumber = nSerialNumber;

    m_fpsCalculator.clock();
    return APC_OK;
}

int CImageDataModel::SetUserData(void *pUserData)
{
    m_pUserData = pUserData;
    return APC_OK;
}

///////////////////////////////////////////////////////////////////////
CImageDataModel_Color::CImageDataModel_Color(CVideoDeviceModel::STREAM_TYPE streamType, CVideoDeviceController *pVideoDeviceController):
CImageDataModel(streamType, COLOR, pVideoDeviceController)
{}

CImageDataModel_Color::~CImageDataModel_Color()
{
    if (m_pDataTransformTask) CThreadWorkerManage::GetInstance()->RemoveTask(m_pDataTransformTask);
}

int CImageDataModel_Color::TransformRawToRGB()
{
    QMutexLocker locker(&m_dataMutex);
    int ret;

    if (CVideoDeviceModel::STREAMING == m_pVideoDeviceController->GetVideoDeviceModel()->GetState()){
		//+[Thermal device]
		if(GetImageType() == APCImageType::COLOR_RGB24) {
			if (m_streamType == CVideoDeviceModel::STREAM_THERMAL) {
                                
                                 memcpy( &m_rgbData[0],&m_rawData[0], m_rawData.size());
                                 ret = APC_OK;
        	        } else {
        	             ret = APC_NotSupport;
        	        }
		} else {
		//-[Thermal device]
                    //DMpreview use rgb format to show
                    ret = APC_ColorFormat_to_BGR24(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                           m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                           &m_rgbData[0], &m_rawData[0],
                                           m_rawData.size(),
                                           GetWidth(), GetHeight(),
                                           GetImageType());
		}
    }else {
        ret = APC_NotSupport;
    }

    return ret;
}

///////////////////////////////////////////////////////////////////////
CImageDataModel_Depth::CImageDataModel_Depth(CVideoDeviceModel::STREAM_TYPE streamType,
                                             CVideoDeviceController *pVideoDeviceController,
                                             double dblCamFocus,
                                             double dblBaselineDist):
CImageDataModel(streamType, DEPTH, pVideoDeviceController),
m_nDepth(0),
m_dblCamFocus(dblCamFocus),
m_dblBaselineDist(dblBaselineDist),
m_bEnableDepthAccuracy(false),
m_fDepthAccuracyRegionRatio(0.0f),
m_fDepthAccuracyGroundTruthDistanceMM(0.0f),
m_fDepthAccuracyFillRate(0.0f),
m_fDepthAccuracyDistanceMM(0.0f),
m_fDepthAccuracyZAccuracy(0.0f),
m_fDepthSpatialNoise(0.0f),
m_fDepthAngle(0.0f),
m_fDepthAngleX(0.0f),
m_fDepthAngleY(0.0f),
m_fDepthTemporaNoise(0.0f),
m_listDepth(TemporalNoiseCount),
m_pDepthAccuracyTask(nullptr),
m_pDepthSpatialNoiseTask(nullptr),
m_pDepthTemporaNoiseTask(nullptr),
m_nDepthInvalidBandPixel(0),
m_PostProcessHandle(nullptr)
{
    for (int i = 0 ; i < COLOR_PALETTE_COUNT ; i++){
        m_colorPalette[i].resize(16384);
    }

    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
    UpdateColorPalette(nZNear, nZFar);

    m_lastUpdateDepthTime = QTime::currentTime().addMSecs(-2 * UPDATE_DEPTH_VALUE_MS);

    APC_EnableGPUAcceleration(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                  m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                  true);
}

CImageDataModel_Depth::~CImageDataModel_Depth()
{
    if (m_pDepthAccuracyTask ){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pDepthAccuracyTask);
    }

    if (m_pDepthSpatialNoiseTask){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pDepthSpatialNoiseTask);
    }

    if (m_pDepthTemporaNoiseTask){
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pDepthTemporaNoiseTask);
    }

    if (m_PostProcessHandle) {
        APC_ReleasePostProcess(m_PostProcessHandle);
    }
}

bool CImageDataModel_Depth::SetImageInfo(APCImageType::Value imageType, 
                                         int nWidth, int nHeight){
    
    if (nWidth != m_nWidth || nHeight != m_nHeight){
        if (m_PostProcessHandle){
            APC_ReleasePostProcess(m_PostProcessHandle);
            m_PostProcessHandle = nullptr;
        }

        APC_InitPostProcess(&m_PostProcessHandle, nWidth, nHeight, imageType);
    }

    return CImageDataModel::SetImageInfo(imageType, nWidth, nHeight);
}

int CImageDataModel_Depth::SetRawData(BYTE *pData, int nDataSize, int nSerialNumber)
{
    m_dataMutex.lock();
    DepthFilter(pData);
    // APC_PostProcess(m_PostProcessHandle, pData);
    m_dataMutex.unlock();

    int ret = CImageDataModel::SetRawData(pData, nDataSize, nSerialNumber);
    if (APC_OK != ret) return ret;

    UpdateDepth();
    return APC_OK;
}

void CImageDataModel_Depth::DepthFilter(BYTE *pData)
{
    DepthFilterOptions *pDepthFilterOptions = m_pVideoDeviceController->GetDepthFilterOptions();
    if (!pDepthFilterOptions || !pDepthFilterOptions->IsDepthFilter()) return;

    switch(GetImageType()){
        case APCImageType::DEPTH_8BITS:
            pDepthFilterOptions->SetType(1);
            pDepthFilterOptions->SetBytesPerPixel(1);
            break;
        case APCImageType::DEPTH_11BITS:
            pDepthFilterOptions->SetType(2);
            pDepthFilterOptions->SetBytesPerPixel(2);
            break;
        case APCImageType::DEPTH_14BITS:
            pDepthFilterOptions->SetType(3);
            pDepthFilterOptions->SetBytesPerPixel(2);
            break;
        default: return;
    }

    unsigned char* sub_disparity = pData;
    int new_width = m_nWidth;
    int new_height = m_nHeight;
    bool bIsSubSample = pDepthFilterOptions->IsSubSample();

    if(bIsSubSample)
    {
        new_width = 0;
        new_height = 0;
        sub_disparity = NULL;	// sub_disparity Initialize;
        APC_SubSample( CEYSDDeviceManager::GetInstance()->GetEYSD(),
                           m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                           &sub_disparity,
                           pData,
                           pDepthFilterOptions->GetBytesPerPixel(),
                           m_nWidth, m_nHeight,
                           new_width, new_height,
                           pDepthFilterOptions->GetSubSampleMode(),
                           pDepthFilterOptions->GetSubSampleFactor());
    }

    if (pDepthFilterOptions->IsEdgePreServingFilter())
        APC_EdgePreServingFilter(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                 m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                 sub_disparity,
                                 pDepthFilterOptions->GetType(),
                                 new_width, new_height,
                                 pDepthFilterOptions->GetEdgeLevel(),
                                 pDepthFilterOptions->GetSigma(),
                                 pDepthFilterOptions->GetLumda());

    if (pDepthFilterOptions->IsHoleFill())
        APC_HoleFill(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                         m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                         sub_disparity,
                         pDepthFilterOptions->GetBytesPerPixel(),
                         pDepthFilterOptions->GetKernelSize(),
                         new_width, new_height,
                         pDepthFilterOptions->GetLevel(),
                         pDepthFilterOptions->IsHorizontal());


    if (pDepthFilterOptions->IsTempleFilter())
        APC_TemporalFilter(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                               m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                               sub_disparity,
                               pDepthFilterOptions->GetBytesPerPixel(),
                               new_width, new_height,
                               pDepthFilterOptions->GetAlpha(),
                               pDepthFilterOptions->GetHistory());

    if (bIsSubSample)
        APC_ApplyFilters(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                             m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                             pData,
                             sub_disparity,
                             pDepthFilterOptions->GetBytesPerPixel(),
                             m_nWidth, m_nHeight,
                             new_width, new_height);


    if (pDepthFilterOptions->IsFlyingDepthCancellation())
    {
        if (GetImageType() == APCImageType::DEPTH_8BITS)
            APC_FlyingDepthCancellation_D8(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                               m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                               pData,
                                               m_nWidth, m_nHeight);
        else if (GetImageType() == APCImageType::DEPTH_11BITS)
            APC_FlyingDepthCancellation_D11(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                                pData,
                                                m_nWidth, m_nHeight);
        else if (GetImageType() == APCImageType::DEPTH_14BITS)
        {
            CVideoDeviceModel::ZDTableInfo *pZDTableINfo = m_pVideoDeviceController->GetVideoDeviceModel()->GetZDTableInfo();
            if ( !pZDTableINfo ) return;

            const size_t Depth_Size = m_nWidth * m_nHeight;

            const WORD* zdTable = (WORD*) pZDTableINfo->ZDTable;

            if ( Depth_Size != m_vecZ14ToD11.size()) m_vecZ14ToD11.resize(Depth_Size);

            if ( m_vecTableZ14ToD11.empty() )
            {
                int Desparity = 0;

                m_vecTableZ14ToD11.resize( 16385, 0 );

                for ( int i = 0; i < 2048; i++ )
                {
                    m_vecTableZ14ToD11[ ((zdTable[i] & 0xff) << 8) | ((zdTable[i] & 0xff00) >> 8)] = ((i & 0xff) << 8) | ((i & 0xff00) >> 8);
                }
                for ( int i = 16384; i > 0; i-- )
                {
                    if ( m_vecTableZ14ToD11[ i ] ) Desparity = m_vecTableZ14ToD11[ i ];

                    else m_vecTableZ14ToD11[ i ] = Desparity;
                }
            }
            WORD* pZ14Depth = ( WORD* )pData;

            APC_TableToData(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                m_nWidth, m_nHeight,
                                m_vecTableZ14ToD11.size() * sizeof( WORD ),
                                m_vecTableZ14ToD11.data(),
                                pZ14Depth,
                                m_vecZ14ToD11.data() );
            APC_FlyingDepthCancellation_D11(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                                m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                                ( BYTE* )m_vecZ14ToD11.data(),
                                                m_nWidth, m_nHeight);
            APC_TableToData(CEYSDDeviceManager::GetInstance()->GetEYSD(),
                                m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceSelInfo()[0],
                                m_nWidth, m_nHeight,
                                2048 * sizeof( WORD ),
                                ( unsigned short* )zdTable,
                                m_vecZ14ToD11.data(),
                                pZ14Depth );
        }
    }
}

void CImageDataModel_Depth::UpdateColorPalette(int nZNear, int nZFar)
{
    for (int i = 0 ; i < COLOR_PALETTE_COUNT ; i++){
        memset(&m_colorPalette[i][0], 0, sizeof(RGBQUAD) * 16384);
    }

    if (m_bEnableDepthAccuracy){
        nZNear = 0;
        nZFar = MAX_DEPTH_DISTANCE;
    }

    if (m_dblCamFocus > 0.0 && m_dblBaselineDist > 0.0){
        const int nMinDepth = 130;
        const int nMaxDepth = 2047;

        int nMinZ = 8.0 * m_dblCamFocus * m_dblBaselineDist  / nMaxDepth;
        int nMaxZ = 8.0 * m_dblCamFocus * m_dblBaselineDist  / nMinDepth;

        ColorPaletteGenerator::generatePaletteColor(&m_colorPalette[COLOR_PALETTE_RGB][0],
                                                    nMaxZ + 1, 4,
                                                    nMinZ, nMaxZ,
                                                    true);
        ColorPaletteGenerator::generatePaletteGray(&m_colorPalette[COLOR_PALETTE_GRAY][0],
                                                   nMaxZ + 1, 4,
                                                   nMinZ, nMaxZ,
                                                   false);
    }else{
        ColorPaletteGenerator::generatePaletteColor(&m_colorPalette[COLOR_PALETTE_RGB][0],
                                                    1 << 14, 4,
                                                    nZNear, nZFar,
                                                    true);
        ColorPaletteGenerator::generatePaletteGray(&m_colorPalette[COLOR_PALETTE_GRAY][0],
                                                   1 << 14, 4,
                                                   nZNear, nZFar,
                                                   false);
    }
}

int CImageDataModel_Depth::TransformRawToRGB()
{
    QMutexLocker locker(&m_dataMutex);

    RGBQUAD *pColorPalette = nullptr;

    switch (m_pVideoDeviceController->GetPreviewOptions()->GetDepthDataTransferControl()){
        case DEPTH_IMG_COLORFUL_TRANSFER:
            pColorPalette = &m_colorPalette[COLOR_PALETTE_RGB][0];
            break;
        case DEPTH_IMG_GRAY_TRANSFER:
            pColorPalette = &m_colorPalette[COLOR_PALETTE_GRAY][0];
            break;
        default:
            //if (APCImageType::DEPTH_8BITS == GetImageType()){
                utImageProcessingUtility::convert_yuv_to_rgb_buffer( &m_rawData[0], &m_rgbData[0], GetWidth(), GetHeight());
            //}else{
            //    utImageProcessingUtility::convert_yuv_to_rgb_buffer( &m_rawData[0], &m_rgbData[0], GetWidth() * 2, GetHeight());
            //}
            return APC_OK;
    }

    int nZNear, nZFar;
    if (m_bEnableDepthAccuracy){
        nZNear = 0;
        nZFar = MAX_DEPTH_DISTANCE;
    }else{
        m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
    }

    if  (m_pUserData){
        utImageProcessingUtility::UpdateD11_Fusion_DisplayImage_DIB24(pColorPalette,
                                                                      (WORD *)m_pUserData, (WORD *)&m_rawData[0],
                                                                      &m_rgbData[0],
                                                                      GetWidth(), GetHeight(),
                                                                      m_dblCamFocus, m_dblBaselineDist,
                                                                      nZNear, nZFar);
    }else if (m_dblCamFocus != 0 || m_dblBaselineDist != 0){
        utImageProcessingUtility::UpdateD11_Baseline_DisplayImage_DIB24(pColorPalette,
                                                                        (WORD *)&m_rawData[0], &m_rgbData[0],
                                                                        GetWidth(), GetHeight(),
                                                                        m_dblCamFocus, m_dblBaselineDist,
                                                                        nZNear, nZFar);
    }else{
        CVideoDeviceModel::ZDTableInfo *pZDTableINfo = m_pVideoDeviceController->GetVideoDeviceModel()->GetZDTableInfo();
        switch (GetImageType()){
            case APCImageType::DEPTH_8BITS:
                utImageProcessingUtility::UpdateD8bitsDisplayImage_DIB24(pColorPalette,
                                                                         &m_rawData[0], &m_rgbData[0],
                                                                         GetWidth(), GetHeight(),
                                                                         pZDTableINfo->ZDTable,
                                                                         pZDTableINfo->nTableSize
                                                                         );
                break;
            case APCImageType::DEPTH_11BITS:
                utImageProcessingUtility::UpdateD11DisplayImage_DIB24(pColorPalette,
                                                                      &m_rawData[0], &m_rgbData[0],
                                                                      GetWidth(), GetHeight(),
                                                                      pZDTableINfo->ZDTable);
                break;
            case APCImageType::DEPTH_14BITS:
                utImageProcessingUtility::UpdateZ14DisplayImage_DIB24(pColorPalette,
                                                                      &m_rawData[0], &m_rgbData[0],
                                                                      GetWidth(), GetHeight());
                break;
            default:
                return APC_NotSupport;
        }
    }

    return APC_OK;
}

QString CImageDataModel_Depth::GetImgaeDataInfo()
{
    QString sImageInfo;
    sImageInfo.sprintf("%s[%d x %d] FPS[%.2f] SN[%d] D[%d] Z[%d]",
                       m_sDataName.toLocal8Bit().data(),
                       m_nWidth, m_nHeight, m_fpsCalculator.GetFPS(), m_nSerialNumber,
                       m_nDepth, m_nZValue);
    return sImageInfo;
}

void CImageDataModel_Depth::UpdateDepth()
{
    QTime currentTime = QTime::currentTime();
    if (UPDATE_DEPTH_VALUE_MS > m_lastUpdateDepthTime.msecsTo(currentTime) &&
        m_lastUpdateDepthTime.msecsTo(currentTime) > 0)
        return;

    m_lastUpdateDepthTime = currentTime;
    m_nDepth = GetDepth(m_nSpecificX, m_nSpecificY);
    UpdateZVaule();
}

void CImageDataModel_Depth::UpdateZVaule()
{
    int nDepthROI = m_pVideoDeviceController->GetPreviewOptions()->GetDepthROI();

    if (nDepthROI > 1){

        int iROI_X = m_nSpecificX - nDepthROI / 2;  if ( iROI_X < 0 ) iROI_X = 0;
        int iROI_Y = m_nSpecificY - nDepthROI / 2;  if ( iROI_Y < 0 ) iROI_Y = 0;

        const int iROI_W = ( iROI_X + nDepthROI > GetWidth()  ) ? GetWidth()  - iROI_X : iROI_X + nDepthROI;
        const int iROI_H = ( iROI_Y + nDepthROI > GetHeight() ) ? GetHeight() - iROI_Y : iROI_Y + nDepthROI;

        unsigned long nDepthRoiSum   = 0;
        unsigned long nDepthRoiCount = 0;

        for ( int y2 = iROI_Y; y2 < iROI_H; y2++ )
        {
            for ( int x2 = iROI_X; x2 < iROI_W; x2++ )
            {
                int nZValue = GetZValue( GetDepth(x2, y2) );

                if ( nZValue )
                {
                    nDepthRoiSum += nZValue;
                    nDepthRoiCount++;
                }
            }
        }
        if ( nDepthRoiCount ) m_nZValue = nDepthRoiSum / nDepthRoiCount;
        else m_nZValue = 0;

    }else{
        m_nZValue = GetZValue(m_nDepth);
    }
}

unsigned short CImageDataModel_Depth::GetDepth(int nX, int nY)
{
    if (nX < 0 || nY < 0) return 0;
    if (nX > GetWidth() || nY > GetHeight()) return 0;

    unsigned int nBytePerPixel = GetRawDataBytePerPixel();
    unsigned int nPixelPosition = (nY * GetWidth() + nX) * nBytePerPixel;
    unsigned short nDepth = 0;
    switch (nBytePerPixel){
        case 1:
            nDepth = m_rawData[nPixelPosition];
            break;
        case 2:
            nDepth = *(WORD *)(&m_rawData[nPixelPosition]);
            break;
        default:
            break;
    }

    return nDepth;
}

unsigned short CImageDataModel_Depth::GetZValue(int nDepth)
{
    CVideoDeviceModel::ZDTableInfo *pZDTableInfo = m_pVideoDeviceController->GetVideoDeviceModel()->GetZDTableInfo();
    APCImageType::Value imageType = m_pVideoDeviceController->GetVideoDeviceModel()->GetDepthImageType();
    if (CVideoDeviceModel::STREAMING != m_pVideoDeviceController->GetVideoDeviceModel()->GetState()){
        return 0;
    }

    if (m_dblCamFocus > 0.0 && m_dblBaselineDist > 0.0) {

        if (!nDepth) return 0;

        switch (imageType){
            case APCImageType::DEPTH_8BITS_0x80:
            case APCImageType::DEPTH_8BITS:
                return (WORD)(m_dblCamFocus * m_dblBaselineDist / nDepth);
            case APCImageType::DEPTH_11BITS:
                return (WORD)(8.0 * m_dblCamFocus * m_dblBaselineDist / nDepth);
            case APCImageType::DEPTH_14BITS:
                return nDepth;
            default:
                return 0;
        }
    } else {

        if (APCImageType::DEPTH_14BITS == imageType){
            return nDepth;
        }

        WORD zdIndex = (APCImageType::DEPTH_8BITS == imageType) ?
                        nDepth << 3 :
                        nDepth;

        WORD nDevType = m_pVideoDeviceController->GetVideoDeviceModel()->GetDeviceInformation()[0].deviceInfomation.nDevType;
        if (PUMA != nDevType){
            zdIndex = nDepth;
        }

        if (zdIndex >= pZDTableInfo->nTableSize / 2){
            zdIndex = pZDTableInfo->nTableSize / 2 - 1;
        }

        int nZValue = pZDTableInfo->ZDTable[zdIndex * 2] << 8 | pZDTableInfo->ZDTable[zdIndex * 2 + 1];

        return nZValue;
    }
}

void CImageDataModel_Depth::EnableDepthAccruacy(bool bEnable)
{
    if(m_bEnableDepthAccuracy == bEnable) return;

    m_bEnableDepthAccuracy = bEnable;

    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
    UpdateColorPalette(nZNear, nZFar);

    if(m_bEnableDepthAccuracy){
        m_pDepthAccuracyTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::DEPTH_ACCURACY_CALCULATE, this);
        CThreadWorkerManage::GetInstance()->AddTask(m_pDepthAccuracyTask);

        m_pDepthSpatialNoiseTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::DEPTH_SPATIAL_NOISE_CALCULATE, this);
        CThreadWorkerManage::GetInstance()->AddTask(m_pDepthSpatialNoiseTask);

        for ( auto& i : m_listDepth ) i.resize( m_nWidth * m_nHeight, 0 );
        m_pDepthTemporaNoiseTask = CTaskInfoManager::GetInstance()->RequestTaskInfo(CTaskInfo::DEPTH_TEMPORA_NOISE_CALCULATE, this);
        CThreadWorkerManage::GetInstance()->AddTask(m_pDepthTemporaNoiseTask);        

    }else{
        CThreadWorkerManage::GetInstance()->RemoveTask(m_pDepthAccuracyTask);
        m_pDepthAccuracyTask = nullptr;

        CThreadWorkerManage::GetInstance()->RemoveTask(m_pDepthSpatialNoiseTask);
        m_pDepthSpatialNoiseTask = nullptr;

        CThreadWorkerManage::GetInstance()->RemoveTask(m_pDepthTemporaNoiseTask);
        m_pDepthTemporaNoiseTask = nullptr;
    }
}

QRect CImageDataModel_Depth::GetDepthAccuracyRegion()
{
    int nHorizontalMargin = (int)((( 1.0f - m_fDepthAccuracyRegionRatio ) * (m_nWidth - m_nDepthInvalidBandPixel)) / 2);
    int nVerticalMargin = (int)((( 1.0f - m_fDepthAccuracyRegionRatio ) * m_nHeight) / 2);

    nHorizontalMargin = std::min(nHorizontalMargin, m_nWidth - 1);
    nVerticalMargin = std::min(nVerticalMargin, m_nHeight - 1);

    int nLeft   = nHorizontalMargin + m_nDepthInvalidBandPixel;
    int nTop    = nVerticalMargin;
    int nWidth  = m_nWidth - (2 * nHorizontalMargin) - m_nDepthInvalidBandPixel;
    int nHeight = m_nHeight - (2 * nVerticalMargin);

    nLeft = std::min(nLeft, m_nWidth - 2);
    nTop = std::min(nTop, m_nHeight - 2);
    nWidth = std::max(nWidth, 1);
    nHeight = std::max(nHeight, 1);

    return {nLeft, nTop, nWidth, nHeight};
}

void CImageDataModel_Depth::AdjustDepthInvalidBandPixel()
{
    if (!m_fDepthAccuracyGroundTruthDistanceMM){
        m_nDepthInvalidBandPixel = 0;
        return;
    }

    CVideoDeviceModel *pModel = m_pVideoDeviceController->GetVideoDeviceModel();
    float focalLength, baseline;

    if (m_dblBaselineDist != 0.0 && m_dblCamFocus != 0.0){
        focalLength = m_dblCamFocus;
        baseline = m_dblBaselineDist;
    }else{
        eSPCtrl_RectLogData *pRectLogData = &pModel->GetRectifyLogData(CVideoDeviceModel::STREAM_DEPTH);
        if (0.0 == pRectLogData->ReProjectMat[14]) return;

        float ratio_Mat = (float)GetHeight() / pRectLogData->OutImgHeight;
        focalLength = pRectLogData->ReProjectMat[11] * ratio_Mat;
        baseline = 1.0f / pRectLogData->ReProjectMat[14];
    }

    int nDepthInvalidBandPixel = baseline * focalLength / m_fDepthAccuracyGroundTruthDistanceMM;
    m_nDepthInvalidBandPixel = std::min(nDepthInvalidBandPixel, m_nWidth);
}

void CImageDataModel_Depth::CalculateDepthAccuracyInfo()
{
    int nWidth, nHeight;
    std::vector<WORD> vecDepthZ = GetDepthZOfROI(nWidth, nHeight);

    unsigned long nFillCount = 0;

    for (float depth : vecDepthZ)
    {
        if (!depth) continue;
        ++nFillCount;
    }

    if (!nFillCount) return;

    double a, b, d;
    SortZ(vecDepthZ);
    CalculateFittedPlane(a, b, d, vecDepthZ, nWidth, nHeight);

    int nCenterX = nWidth / 2;
    int nCenerY = nHeight / 2;

    QVector3D vecBefore(-a, -b, 1);
    QVector3D vecAfter(0, 0, 1);

    m_fDepthAccuracyDistanceMM = (a * nCenterX) + (b * nCenerY) + d;;
    m_fDepthAccuracyFillRate = nFillCount / (double) vecDepthZ.size();
    m_fDepthAccuracyZAccuracy = CalculateZAccuracy(vecDepthZ,
                                                   nWidth, nHeight,
                                                   m_fDepthAccuracyGroundTruthDistanceMM,
                                                   vecBefore, vecAfter);
}

void CImageDataModel_Depth::CalculateDepthSpatialNoise()
{
    int nROIWidth, nROIHeight;
    std::vector< WORD > vecDepthZ = GetDepthZOfROI(nROIWidth, nROIHeight);
    double a, b, d;

    SortZ(vecDepthZ);
    CalculateFittedPlane(a, b, d, vecDepthZ, nROIWidth, nROIHeight);

    double DepthZSum  = 0.0;
    int    Count      = 0;
    int    idx        = 0;
    for ( int y = 0; y < nROIHeight; y++ )
    {
        for ( int x = 0; x < nROIWidth; x++ )
        {
            idx = y * nROIWidth + x;

            if ( vecDepthZ[ idx ] )
            {
                Count++;
                DepthZSum += pow( vecDepthZ[ idx ] - ( a * x + b * y + d ), 2 );
            }
        }
    }

    m_fDepthSpatialNoise = Count ? ( sqrt( DepthZSum / Count ) ) : 0.0f;
    if ( m_fDepthAccuracyGroundTruthDistanceMM > 0.0f){
        m_fDepthSpatialNoise = m_fDepthSpatialNoise / m_fDepthAccuracyGroundTruthDistanceMM;
    }else{
        m_fDepthSpatialNoise = 0.0f;
    }

    m_fDepthAngle = acos( 1.0f / sqrt( a * a + b * b + 1 ) ) * 180.0f / M_PI;
    m_fDepthAngleX = acos( 1.0f / sqrt( a * a + 1 ) ) * 180.0f / M_PI;
    if (a < 0) m_fDepthAngleX *= -1.0f;
    m_fDepthAngleY = acos( 1.0f / sqrt( b * b + 1 ) ) * 180.0f / M_PI;
    if (b > 0) m_fDepthAngleY *= -1.0f;
}

void CImageDataModel_Depth::CalculateDepthTemporaNoise()
{
    const int nDepthSize = m_nWidth * m_nHeight;

    int nDepthZSum = 0;
    int nCount = 0;
    short nAvgDepth = 0;

    std::vector<float> vecSTD(nDepthSize, 0);

    int std_cnt = 0;
    int idx = 0;

    const QRect region = GetDepthAccuracyRegion();
    const int nLeft   = region.left();
    const int nTop    = region.top();
    const int nRight  = region.right();
    const int nBottom = region.bottom();

    std::vector< short >& vecDepthZ = *m_listDepth.rbegin();

    std_cnt = 0;

    for ( int y = nTop; y < nBottom; y++ )
    {
        for ( int x = nLeft; x < nRight; x++ )
        {
            vecDepthZ[ y * m_nWidth + x ] = GetZValue( GetDepth(x, y) );
        }
    }
    for ( int y = nTop; y < nBottom; y++ )
    {
        for ( int x = nLeft; x < nRight; x++ )
        {
            idx = y * m_nWidth + x;

            nDepthZSum = 0;
            nCount     = 0;

            for ( auto& vecDepth : m_listDepth )
            {
                if ( vecDepth[ idx ] ) nCount++;

                nDepthZSum += vecDepth[ idx ];
            }
            if ( nCount )
            {
                nAvgDepth = nDepthZSum / nCount;

                nDepthZSum = 0;

                for ( auto& vecDepth : m_listDepth )
                {
                    if ( vecDepth[ idx ] )
                    {
                        nDepthZSum += pow( vecDepth[ idx ] - nAvgDepth, 2 );
                    }
                }
                vecSTD[ std_cnt++ ] = sqrt( nDepthZSum / ( float )nCount );
            }
        }
    }
    if ( std_cnt )
    {
        std::sort( vecSTD.begin(), vecSTD.begin() + std_cnt );
        m_fDepthTemporaNoise = vecSTD[ std_cnt / 2 ];
        if ( m_fDepthAccuracyGroundTruthDistanceMM > 0.0f){
            m_fDepthTemporaNoise = m_fDepthTemporaNoise / m_fDepthAccuracyGroundTruthDistanceMM;
        }else{
            m_fDepthTemporaNoise = 0.0f;
        }
    }
    m_listDepth.splice( m_listDepth.end(), m_listDepth, m_listDepth.begin() );

}

std::vector< WORD > CImageDataModel_Depth::GetDepthZOfROI(int &nWidth, int &nHeight)
{
    const QRect region = GetDepthAccuracyRegion();
    const int nLeft   = region.left();
    const int nTop    = region.top();
    const int nRight  = region.right();
    const int nBottom = region.bottom();

    nWidth = region.width();
    nHeight = region.height();

    std::vector< WORD > vecDepthZ( nWidth * nHeight, 0);
    for ( int y = nTop; y <= nBottom; y++ )
    {
        for ( int x = nLeft; x <= nRight; x++ )
        {
            vecDepthZ[ (y - nTop) * nWidth + (x - nLeft) ] = GetZValue( GetDepth(x, y) );
        }
    }

    return vecDepthZ;
}

void CImageDataModel_Depth::CalculateFittedPlane(double &a, double&b, double &d,
                                                 std::vector< WORD > &vecDepthZ, int nWidth, int nHeight)
{
    double MatrixXX = 0.0;
    double MatrixYY = 0.0;
    double MatrixXY = 0.0;
    double MatrixX  = 0.0;
    double MatrixY  = 0.0;
    double MatrixN  = 0.0;
    double MatrixXZ = 0.0;
    double MatrixYZ = 0.0;
    double MatrixZ  = 0.0;
    double MatrixBase = 0.0;
    int    idx        = 0;

    for ( int y = 0; y < nHeight; y++ )
    {
        for ( int x = 0; x < nWidth; x++ )
        {
            idx = y * nWidth + x;
            if ( vecDepthZ[ idx ] )
            {
                MatrixXX += (x * x);
                MatrixYY += (y * y);
                MatrixXY += (x * y);
                MatrixX += x;
                MatrixY += y;
                MatrixN++;
                MatrixXZ += (x * vecDepthZ[ idx ]);
                MatrixYZ += (y * vecDepthZ[ idx ]);
                MatrixZ += vecDepthZ[ idx ];
            }
        }
    }

    MatrixBase = MatrixXX * MatrixYY * MatrixN + 2 * MatrixXY * MatrixX * MatrixY - MatrixX  * MatrixX  * MatrixYY
                                                                                  - MatrixY  * MatrixY  * MatrixXX
                                                                                  - MatrixXY * MatrixXY * MatrixN;
    a = ( MatrixXZ * MatrixYY * MatrixN + MatrixZ  * MatrixXY * MatrixY + MatrixYZ * MatrixX  * MatrixY -
          MatrixZ  * MatrixYY * MatrixX - MatrixXZ * MatrixY  * MatrixY - MatrixYZ * MatrixXY * MatrixN ) / MatrixBase;
    b = ( MatrixYZ * MatrixXX * MatrixN + MatrixXZ * MatrixX  * MatrixY + MatrixZ  * MatrixXY * MatrixX -
          MatrixYZ * MatrixX  * MatrixX - MatrixZ  * MatrixXX * MatrixY - MatrixXZ * MatrixXY * MatrixN ) / MatrixBase;
    d = ( MatrixZ  * MatrixXX * MatrixYY + MatrixYZ * MatrixXY * MatrixX + MatrixXZ * MatrixXY * MatrixY -
          MatrixXZ * MatrixYY * MatrixX  - MatrixYZ * MatrixXX * MatrixY - MatrixZ  * MatrixXY * MatrixXY ) / MatrixBase;
}


double CImageDataModel_Depth::CalculateZAccuracy(std::vector< WORD > &vecDepthZ,
                                                 int nWidth, int nHeight,
                                                 double grandtrue,
                                                 QVector3D vecBefore, QVector3D vecAfter)
{
    if (!grandtrue) return 0.0;

    int ROI_PointNum = nWidth * nHeight;
    int center_x = nWidth / 2;
    int center_y = nHeight / 2;

    QVector3D u = QVector3D::crossProduct(vecBefore, vecAfter);
    double dotresult = QVector3D::dotProduct(vecBefore, vecAfter);
    double n_before = vecBefore.length();
    double n_after = vecAfter.length();

    double rotationangle = acos(dotresult / (n_before * n_after));

    double norm = u.length();

    double rotatinMatrix[3][3] = { 0 };

    u = u / norm;

    rotatinMatrix[0][0] = cos(rotationangle) + u.x() * u.x() * (1 - cos(rotationangle));
    rotatinMatrix[0][1] = u[0] * u.y() * (1 - cos(rotationangle) - u.z() * sin(rotationangle));
    rotatinMatrix[0][2] = u.y() * sin(rotationangle) + u.x() * u.z() * (1 - cos(rotationangle));

    rotatinMatrix[1][0] = u.z() * sin(rotationangle) + u.x() * u.y() * (1 - cos(rotationangle));
    rotatinMatrix[1][1] = cos(rotationangle) + u.y() * u.y() * (1 - cos(rotationangle));
    rotatinMatrix[1][2] = -u[0] * sin(rotationangle) + u.y() * u.z() * (1 - cos(rotationangle));

    rotatinMatrix[2][0] = -u.y() * sin(rotationangle) + u.x() * u.z() * (1 - cos(rotationangle));
    rotatinMatrix[2][1] = u[0] * sin(rotationangle) + u.y() * u.z() * (1 - cos(rotationangle));
    rotatinMatrix[2][2] = cos(rotationangle) + u.z() * u.z() * (1 - cos(rotationangle));

    int index = center_y * nWidth + center_x;
    double trans_z = rotatinMatrix[2][0] * center_x + rotatinMatrix[2][1] * center_y + rotatinMatrix[2][2] * vecDepthZ[index] - m_fDepthAccuracyDistanceMM;

    std::vector<float> vecZError(nWidth * nHeight, 0.0f);

    for (int y = 0; y < nHeight; y++)
    {
        for (int x = 0; x < nWidth; x++)
        {
            int index = y * nWidth + x;
            double z = vecDepthZ[index];
            if (z)
            {
#if 1 // rotation flag
                vecZError[index] = rotatinMatrix[2][0] * x + rotatinMatrix[2][1] * y + rotatinMatrix[2][2] * z - grandtrue - trans_z;
#else
                vecZError[index] = z - grandtrue;
#endif
            }
        }
    }

    std::sort(vecZError.begin(), vecZError.end());

    double mid_erro = vecZError[ROI_PointNum / 2];

    return mid_erro / grandtrue;
}

void CImageDataModel_Depth::SortZ(std::vector< WORD > &vecDepthZ, double dblDeleteBoundaryRatio)
{
    std::vector< WORD > vecDepthZ_bak = vecDepthZ;
    std::sort(vecDepthZ_bak.begin(), vecDepthZ_bak.end());

    auto iterFirstNotZero = std::find_if(vecDepthZ_bak.begin(), vecDepthZ_bak.end(), [](float val){ return val != 0;});

    if (iterFirstNotZero == vecDepthZ_bak.end()) return;

    int nBoundCount = std::distance(iterFirstNotZero, vecDepthZ_bak.end()) * dblDeleteBoundaryRatio;

    if (!nBoundCount) return;

    WORD nUpperBoundVal = *(vecDepthZ_bak.end() - nBoundCount);
    WORD nLowerBoundVal = *(iterFirstNotZero + nBoundCount);

    for (WORD &depth : vecDepthZ)
    {
        if (!depth) continue;

        if (depth > nUpperBoundVal || depth < nLowerBoundVal)
        {
            depth = 0;
        }
    }
}

///////////////////////////////////////////////////////////////////////

CImageDataModel *CImageDataModelFactory::CreateCImageDataModel(CVideoDeviceModel::STREAM_TYPE type, CVideoDeviceController *pVideoDeviceController)
{
    switch (type){
        case CVideoDeviceModel::STREAM_COLOR:
        case CVideoDeviceModel::STREAM_COLOR_SLAVE:
        case CVideoDeviceModel::STREAM_KOLOR:
        case CVideoDeviceModel::STREAM_KOLOR_SLAVE:
        case CVideoDeviceModel::STREAM_TRACK:
        //+[Thermal device]
        case CVideoDeviceModel::STREAM_THERMAL:
        //-[Thermal device]
            return new CImageDataModel_Color(type, pVideoDeviceController);
        case CVideoDeviceModel::STREAM_DEPTH:
            return new CImageDataModel_Depth(type, pVideoDeviceController);
        case CVideoDeviceModel::STREAM_DEPTH_30mm:
        case CVideoDeviceModel::STREAM_DEPTH_FUSION:
            return new CImageDataModel_Depth_MultiBaseline(type,
                                                           pVideoDeviceController,
                                                           pVideoDeviceController->GetVideoDeviceModel()->GetCameraFocus(),
                                                           30.0);
        case CVideoDeviceModel::STREAM_DEPTH_60mm:
            return new CImageDataModel_Depth_MultiBaseline(type,
                                                           pVideoDeviceController,
                                                           pVideoDeviceController->GetVideoDeviceModel()->GetCameraFocus(),
                                                           60.0);
        case CVideoDeviceModel::STREAM_DEPTH_150mm:
            return new CImageDataModel_Depth_MultiBaseline(type,
                                                           pVideoDeviceController,
                                                           pVideoDeviceController->GetVideoDeviceModel()->GetCameraFocus(),
                                                           150.0);
        default:
            return nullptr;
    }
}

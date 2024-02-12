#include "PlyWriter.h"
#include <sstream>
#include <fstream>
#include <math.h>
#include <string.h>

PlyWriter::PlyWriter() {
}

int PlyWriter::writePly(std::vector<CloudPoint>& cloud, std::string filename) {
    std::ofstream fs;
    const int MAX_PROPERTY_STIRING_SIZE = 1024;
    int vertexSize = cloud.size();

    // Open file
    fs.open(filename.c_str());
    if (!fs)
    {
        return (-1);
    }

    fs << generateHeader(false, vertexSize); //
    /*Output point*/

    char buf[MAX_PROPERTY_STIRING_SIZE];
    std::stringstream oss;
    for (int i = 0; i < vertexSize; i++) {
        sprintf(buf, "%d %d %d %.8f %.8f %.8f \n", cloud[i].r, cloud[i].g, cloud[i].b, cloud[i].x, -cloud[i].y, -cloud[i].z);
        oss << buf;

        if (i != 0 && i % 50000 == 0)
        {
            fs << oss.str();
            fs.flush();
            oss.str("");
            oss.clear();
        }
    }
    fs << oss.str();
    fs.flush();
    fs.close();
    return 0;
}

//int PlyWriter::EYSDFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, float zNear, float zFar, bool removeINF = true, bool useDepthResolution = true, float scale_ratio = 1.0f) {
//	return EYSDFrameTo3D(depthWidth, depthHeight, dArray, colorWidth, colorHeight, colorArray, rectLogData, depthType, output, true, zNear, zFar, removeINF, useDepthResolution, scale_ratio);
//}

//int PlyWriter::EYSDFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool removeINF = true, bool useDepthResolution = true, float scale_ratio = 1.0f) {
//	return EYSDFrameTo3D(depthWidth, depthHeight, dArray, colorWidth, colorHeight, colorArray, rectLogData, depthType, output, false,0.0f,0.0f, removeINF, useDepthResolution, scale_ratio);
//}
int PlyWriter::EYSDFrameTo3D_8029(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear,float zFar,bool removeINF, bool useDepthResolution, float scale_ratio){

    float ratio_Mat = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    if (useDepthResolution)
    {
        outputWidth = depthWidth;
        outputHeight = depthHeight;
    }
    else
    {
        outputWidth = colorWidth;
        outputHeight = colorHeight;
    }

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat = (float)outputHeight / rectLogData->OutImgHeight;

    output.clear();
    std::vector<CloudPoint> tempPointCloud;
    std::vector<unsigned char> colorArrayResized(outputWidth * outputHeight * 3);
    std::vector<unsigned char> dArrayResized(outputWidth * outputHeight * 2);

    if (depthHeight != outputHeight) {
        if ( depthType == APCImageType::DEPTH_8BITS ) MonoBilinearFineScaler( &dArray[0], &dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
        else MonoBilinearFineScaler_short( (unsigned short*)&dArray[0], (unsigned short*)&dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        dArrayResized.assign(dArray.begin(), dArray.end());
    }

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    int count = 0;

    float centerX = -1.0f*rectLogData->ReProjectMat[3] * ratio_Mat;
    float centerY = -1.0f*rectLogData->ReProjectMat[7] * ratio_Mat;
    float focalLength = rectLogData->ReProjectMat[11] * ratio_Mat;
    float baseline = 1.0f / rectLogData->ReProjectMat[14];
    float diff = rectLogData->ReProjectMat[15] * ratio_Mat;

    /*Generate table for disparity to W ( W = disparity / baseline)*/
    float disparityToW[2048];
    int tableSize = 1 << 8;

    for (int i = 0; i < tableSize; i++)
        disparityToW[i] = (i * ratio_Mat) / baseline + diff;

    unsigned char *colorBuf = &colorArrayResized[0];
    unsigned char *dArrayBuf = &dArrayResized[0];

    float W = 0.0f;
    float X = 0.0f;
    float Y = 0.0f;
    float Z = 0.0f;

    for (int j = 0; j < outputHeight; j++)
    {
        for (int i = 0; i < outputWidth; i++)
        {
            W = disparityToW[ *dArrayBuf++ ] * 0.5f;

            if ( W > 0 )
            {
                Z = focalLength / W;
                X = (i - centerX) / W;
                Y = (j - centerY) / W;

                if ( Z < zFar && Z > zNear )
                {
                    CloudPoint point;
                    point.r = *(colorBuf++);
                    point.g = *(colorBuf++);
                    point.b = *(colorBuf++);
                    point.x = X;
                    point.y = Y;
                    point.z = Z;

                    tempPointCloud.push_back(point);
                    count++;
                    continue;
                }
            }
            colorBuf += 3; //RGB
        }
    }
    output.assign(tempPointCloud.begin(), tempPointCloud.begin()+count);

    return 0;
}

int PlyWriter::EYSDFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear,float zFar,bool removeINF = true, bool useDepthResolution = true, float scale_ratio = 1.0f){

    float ratio_Mat = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    if (useDepthResolution)
    {
        outputWidth = depthWidth;
        outputHeight = depthHeight;
    }
    else
    {
        outputWidth = colorWidth;
        outputHeight = colorHeight;
    }

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat = (float)outputHeight / rectLogData->OutImgHeight;

    output.clear();
    std::vector<CloudPoint> tempPointCloud;
    std::vector<unsigned char> colorArrayResized(outputWidth * outputHeight * 3);
    std::vector<unsigned char> dArrayResized(outputWidth * outputHeight * 2);

    if (depthHeight != outputHeight) {
        if ( depthType == APCImageType::DEPTH_8BITS ) MonoBilinearFineScaler( &dArray[0], &dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
        else MonoBilinearFineScaler_short( (unsigned short*)&dArray[0], (unsigned short*)&dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        dArrayResized.assign(dArray.begin(), dArray.end());
    }

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    int count = 0;

    float centerX = -1.0f*rectLogData->ReProjectMat[3] * ratio_Mat;
    float centerY = -1.0f*rectLogData->ReProjectMat[7] * ratio_Mat;
    float focalLength = rectLogData->ReProjectMat[11] * ratio_Mat;
    float baseline = 1.0f / rectLogData->ReProjectMat[14];
    float diff = rectLogData->ReProjectMat[15] * ratio_Mat;


    /*Generate table for disparity to W ( W = disparity / baseline)*/
    float disparityToW[2048];
    int tableSize;
    switch (depthType)
    {
        case APCImageType::DEPTH_8BITS_0x80:
        case APCImageType::DEPTH_8BITS:
            tableSize = 1 << 8;
            for (int i = 0; i < tableSize; i++)
                disparityToW[i] = (i * ratio_Mat) / baseline + diff;
            break;
        case APCImageType::DEPTH_11BITS:
            tableSize = 1 << 11;
            for (int i = 0; i < tableSize; i++)
                disparityToW[i] = (i * ratio_Mat / 8.0f) / baseline + diff;
            break;
        case APCImageType::DEPTH_14BITS:
            break;
    }

    unsigned char *colorBuf = &colorArrayResized[0];
    unsigned char *dArrayBuf = &dArrayResized[0];

    for (int j = 0; j < outputHeight; j++) {
        for (int i = 0; i < outputWidth; i++) {
            int disparity = 0;
            float W = 0.0f;
            float X = 0.0f;
            float Y = 0.0f;
            float Z = 0.0f;

            switch (depthType)
            {
            case APCImageType::DEPTH_8BITS_0x80:
            case APCImageType::DEPTH_11BITS:
                disparity = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                W = disparityToW[disparity];
                if (W > 0)
                {
                    Z = focalLength / W;
                }
                break;
            case APCImageType::DEPTH_8BITS:
                disparity = *(dArrayBuf++);
                W = disparityToW[disparity];
                if (W > 0)
                {
                    Z = focalLength / W;
                }
                break;
            case APCImageType::DEPTH_14BITS:
                Z = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                if (Z > 0)
                {
                    W = focalLength / Z;
                }
                break;
            }

            if (!(removeINF && W == 0.0f)) {
                X = (i - centerX) / W;
                Y = (j - centerY) / W;

                if (!(clipping && (Z > zFar || Z < zNear))) {
                    CloudPoint point;
                    point.r = *(colorBuf++);
                    point.g = *(colorBuf++);
                    point.b = *(colorBuf++);
                    point.x = X;
                    point.y = Y;
                    point.z = Z;

                    tempPointCloud.push_back(point);
                    count++;
                    continue;
                }
            }
            colorBuf += 3; //RGB
        }
    }

    output.assign(tempPointCloud.begin(), tempPointCloud.begin()+count);

    return 0;
}

int PlyWriter::EYSDFrameTo3D_PlyFilterFloat(int depthWidth, int depthHeight, std::vector<float>& dFloatArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF = true, bool useDepthResolution = true, float scale_ratio = 1.0f) {

    float ratio_Mat = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    if (useDepthResolution)
    {
        outputWidth = depthWidth;
        outputHeight = depthHeight;
    }
    else
    {
        outputWidth = colorWidth;
        outputHeight = colorHeight;
    }

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat = (float)outputHeight / rectLogData->OutImgHeight;

    output.clear();
    std::vector<unsigned char> colorArrayResized(outputWidth * outputHeight * 3);
    std::vector<float> zArrayResized(outputWidth * outputHeight * 1);


    if (depthHeight != outputHeight) {
        MonoBilinearFineScaler_float( &dFloatArray[0], &zArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        zArrayResized.assign(dFloatArray.begin(), dFloatArray.end());
    }

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    float centerX = -1.0f*rectLogData->ReProjectMat[3] * ratio_Mat;
    float centerY = -1.0f*rectLogData->ReProjectMat[7] * ratio_Mat;
    float focalLength = rectLogData->ReProjectMat[11] * ratio_Mat;
    float baseline = 1.0f / rectLogData->ReProjectMat[14];
    float diff = rectLogData->ReProjectMat[15] * ratio_Mat;

    output.clear();
    for (int j = 0; j < outputHeight; j++) {
        for (int i = 0; i < outputWidth; i++) {
            float disparity = zArrayResized[(j*outputWidth + i)];

            /*disparity transfers to Distance.*/
            float Z = (float)8.0f*disparity; /*5000000 =fbc , its range "5000000~50000000, it effects guass.*/
            float X = (i*1.0f - centerX) * Z / focalLength;
            float Y = (j*1.0f - centerY) * Z / focalLength;

            if (clipping) {
                if (Z > zFar || Z < zNear) {
                    continue;
                }
            }

            CloudPoint point;
            point.r = colorArrayResized[(j*outputWidth + i) * 3 + 0];
            point.g = colorArrayResized[(j*outputWidth + i) * 3 + 1];
            point.b = colorArrayResized[(j*outputWidth + i) * 3 + 2];
            point.x = X;
            point.y = Y;
            point.z = Z;

            output.push_back(point);
        }
    }

    return 0;
}


int PlyWriter::EYSDFrameTo3DMultiSensor(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData* rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool isDownSampleK, float scale_ratio, int degreeOfRectifyLogK){

    float ratio_Mat_L = 1.0f;
    float ratio_Mat_K = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    if (isDownSampleK)
    {
        outputWidth = depthWidth;
        outputHeight = depthHeight;
    }
    else
    {
        outputWidth = colorWidth;
        outputHeight = colorHeight;
    }

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat_L = (float)outputHeight / rectLogDataL->OutImgHeight;
    ratio_Mat_K = (float)outputHeight / rectLogDataK->OutImgHeight;

    output.clear();
    std::vector<CloudPoint> tempPointCloud;
    std::vector<unsigned char> colorArrayResized(outputWidth * outputHeight * 3);
    std::vector<unsigned char> dArrayResized(outputWidth * outputHeight * 2);

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    if (depthHeight != outputHeight) {
        if ( depthType == APCImageType::DEPTH_8BITS ) MonoBilinearFineScaler( &dArray[0], &dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
        else MonoBilinearFineScaler_short( (unsigned short*)&dArray[0], (unsigned short*)&dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        dArrayResized.assign(dArray.begin(), dArray.end());
    }

    int count = 0;

    float centerX_L = -1.0f*rectLogDataL->ReProjectMat[3] * ratio_Mat_L;
    float centerY_L = -1.0f*rectLogDataL->ReProjectMat[7] * ratio_Mat_L;
    float focalLength_L = rectLogDataL->ReProjectMat[11] * ratio_Mat_L;
    float baseline_L = 1.0f / rectLogDataL->ReProjectMat[14];
    float diff_L = rectLogDataL->ReProjectMat[15] * ratio_Mat_L;


    //float centerX_K = -1.0f*rectLogDataK->ReProjectMat[3] * ratio_Mat_K;
    //float centerY_K = -1.0f*rectLogDataK->ReProjectMat[7] * ratio_Mat_K;
    float focalLength_K = rectLogDataK->ReProjectMat[11] * ratio_Mat_K;
    float baseline_K = 1.0f / rectLogDataK->ReProjectMat[14];
    float diff_K = rectLogDataK->ReProjectMat[15] * ratio_Mat_K;

    /*Generate table for disparity to W ( W = disparity / baseline)*/
    float disparityToW[2048];
    int tableSize;

    switch (depthType)
    {
    case APCImageType::DEPTH_8BITS_0x80:
    case APCImageType::DEPTH_8BITS:
        tableSize = 1 << 8;
        for (int i = 0; i < tableSize; i++)
            disparityToW[i] = (i * ratio_Mat_L) / baseline_L + diff_L;
        break;
    case APCImageType::DEPTH_11BITS:
        tableSize = 1 << 11;
        for (int i = 0; i < tableSize; i++)
            disparityToW[i] = ((i * ratio_Mat_L) / 8.0f) / baseline_L + diff_L;
        break;
    case APCImageType::DEPTH_14BITS:
        break;
    }

    unsigned char *dArrayBuf = &dArrayResized[0];

    for (int j = 0; j < outputHeight; j++) {
        for (int i = 0; i < outputWidth; i++) {

            int disparity = 0;

            float W = 0.0f;
            float X = 0.0f;
            float Y = 0.0f;
            float Z = 0.0f;

            switch (depthType)
            {
            case APCImageType::DEPTH_8BITS_0x80:
            case APCImageType::DEPTH_11BITS:
                disparity = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                W = disparityToW[disparity];
                if (W > 0)
                {
                    Z = focalLength_L / W;
                }
                break;
            case APCImageType::DEPTH_8BITS:
                disparity = *(dArrayBuf++);
                W = disparityToW[disparity];
                if (W > 0)
                {
                    Z = focalLength_L / W;
                }
                break;
            case APCImageType::DEPTH_14BITS:
                Z = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                if (Z > 0)
                {
                    W = focalLength_L / Z;
                }
                break;
            }

            if (!(removeINF && W == 0.0f)) {
                X = (i - centerX_L) / W;
                Y = (j - centerY_L) / W;
                int shift = ((focalLength_K / Z) - diff_K) * baseline_K;
                if (!(clipping && (Z > zFar || Z < zNear))) {
                    CloudPoint point;

                    if (degreeOfRectifyLogK == 0)
                    {
                        if ((i - shift) < 0 || (i - shift) > (outputWidth - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[(j * outputWidth + (i - shift)) * 3 + 0];
                            point.g = colorArrayResized[(j * outputWidth + (i - shift)) * 3 + 1];
                            point.b = colorArrayResized[(j * outputWidth + (i - shift)) * 3 + 2];
                        }
                    }
                    else if (degreeOfRectifyLogK == 90)
                    {
                        if ((j - shift) < 0 || (j - shift) > (outputHeight - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 0];
                            point.g = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 1];
                            point.b = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 2];
                        }
                    }
                    else if (degreeOfRectifyLogK == 180)
                    {

                        if ((i + shift) < 0 || (i + shift) > (outputWidth - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[(j * outputWidth + (i + shift)) * 3 + 0];
                            point.g = colorArrayResized[(j * outputWidth + (i + shift)) * 3 + 1];
                            point.b = colorArrayResized[(j * outputWidth + (i + shift)) * 3 + 2];
                        }
                    }
                    else if (degreeOfRectifyLogK == 270)
                    {

                        if ((j + shift) < 0 || (j + shift) > (outputHeight - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[((j + shift) * outputWidth + i) * 3 + 0];
                            point.g = colorArrayResized[((j + shift) * outputWidth + i) * 3 + 1];
                            point.b = colorArrayResized[((j + shift) * outputWidth + i) * 3 + 2];
                        }
                    }

                    point.x = X;
                    point.y = Y;
                    point.z = Z;

                    tempPointCloud.push_back(point);
                    count++;
                    continue;
                }
            }
        }
    }
    output.assign(tempPointCloud.begin(), tempPointCloud.begin() + count);
    return 0;
}

int PlyWriter::EYSDFrameTo3DMultiSensorPlyFilterFloat(int depthWidth, int depthHeight, std::vector<float>& dFloatArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool isDownSampleK, float scale_ratio, int degreeOfRectifyLogK) {

    float ratio_Mat_L = 1.0f;
    float ratio_Mat_K = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    if (isDownSampleK)
    {
        outputWidth = depthWidth;
        outputHeight = depthHeight;
    }
    else
    {
        outputWidth = colorWidth;
        outputHeight = colorHeight;
    }

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat_L = (float)outputHeight / rectLogDataL->OutImgHeight;
    ratio_Mat_K = (float)outputHeight / rectLogDataK->OutImgHeight;

    output.clear();
    std::vector<CloudPoint> tempPointCloud;
    std::vector<unsigned char> colorArrayResized(depthWidth*depthHeight * 3);
    std::vector<float> zArrayResized(outputWidth * outputHeight * 1);

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    if (depthHeight != outputHeight) {
        MonoBilinearFineScaler_float( &dFloatArray[0], &zArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        zArrayResized.assign(dFloatArray.begin(), dFloatArray.end());
    }

    int count = 0;

    float centerX_L = -1.0f*rectLogDataL->ReProjectMat[3] * ratio_Mat_L;
    float centerY_L = -1.0f*rectLogDataL->ReProjectMat[7] * ratio_Mat_L;
    float focalLength_L = rectLogDataL->ReProjectMat[11] * ratio_Mat_L;
    float baseline_L = 1.0f / rectLogDataL->ReProjectMat[14];
    float diff_L = rectLogDataL->ReProjectMat[15] * ratio_Mat_L;


    //float centerX_K = -1.0f*rectLogDataK->ReProjectMat[3] * ratio_Mat_K;
    //float centerY_K = -1.0f*rectLogDataK->ReProjectMat[7] * ratio_Mat_K;
    float focalLength_K = rectLogDataK->ReProjectMat[11] * ratio_Mat_K;
    float baseline_K = 1.0f / rectLogDataK->ReProjectMat[14];
    float diff_K = rectLogDataK->ReProjectMat[15] * ratio_Mat_K;

    /*Generate table for disparity to W ( W = disparity / baseline)*/

    for (int j = 0; j < outputHeight; j++) {
        for (int i = 0; i < outputWidth; i++) {
            float Z = 8.0f*zArrayResized[(j*outputWidth + i)];
            float W = focalLength_L / Z;
            if (!(removeINF && W == 0.0f)) {
                float X = (i - centerX_L) / W;
                float Y = (j - centerY_L) / W;
                int shift = ((focalLength_K / Z) - diff_K)*baseline_K;
                if (!(clipping && (Z > zFar || Z < zNear))) {
                    CloudPoint point;
                    if (degreeOfRectifyLogK == 0)
                    {

                        if ((i - shift) < 0 || (i - shift) > (outputWidth - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[(j * outputWidth + (i - shift)) * 3 + 0];
                            point.g = colorArrayResized[(j * outputWidth + (i - shift)) * 3 + 1];
                            point.b = colorArrayResized[(j * outputWidth + (i - shift)) * 3 + 2];
                        }
                    }
                    else if (degreeOfRectifyLogK == 90)
                    {
                        if ((j - shift) < 0 || (j - shift) > (outputHeight - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 0];
                            point.g = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 1];
                            point.b = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 2];
                        }
                    }
                    else if (degreeOfRectifyLogK == 180)
                    {

                        if ((i + shift) < 0 || (i + shift) > (outputWidth - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[(j * outputWidth + (i + shift)) * 3 + 0];
                            point.g = colorArrayResized[(j * outputWidth + (i + shift)) * 3 + 1];
                            point.b = colorArrayResized[(j * outputWidth + (i + shift)) * 3 + 2];
                        }
                    }
                    else if (degreeOfRectifyLogK == 270)
                    {

                        if ((j + shift) < 0 || (j + shift) > (outputHeight - 1)) {
                            point.r = 0;
                            point.g = 0;
                            point.b = 0;
                        }
                        else
                        {
                            point.r = colorArrayResized[((j + shift) * outputWidth + i) * 3 + 0];
                            point.g = colorArrayResized[((j + shift) * outputWidth + i) * 3 + 1];
                            point.b = colorArrayResized[((j + shift) * outputWidth + i) * 3 + 2];
                        }
                    }
                    point.x = X;
                    point.y = Y;
                    point.z = Z;

                    tempPointCloud.push_back(point);
                    count++;
                    continue;
                }
            }
        }
    }
    output.assign(tempPointCloud.begin(), tempPointCloud.begin() + count);
    return 0;
}
int PlyWriter::EYSDFrameTo3DCylinder(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, float scale_ratio)
{

    float ratio_Mat = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    outputWidth = depthWidth;
    outputHeight = depthHeight;

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat = (float)outputHeight / rectLogData->OutImgHeight;
    //if (outputWidth > outputHeight) {
    //	ratio_Mat = (float)outputHeight / 1080; //rectLogData->OutImgHeight is error, and current EX8040S only support mode 4, so hard code rectLogData->OutImgHeight as 1080
    //}
    //else
    //{
    //	ratio_Mat = (float)outputHeight / 1920; //rectLogData->OutImgHeight is error, and current EX8040S only support mode 4, so hard code rectLogData->OutImgHeight as 1920
    //}

    output.clear();
    std::vector<CloudPoint> tempPointCloud;
    std::vector<unsigned char> colorArrayResized(outputWidth * outputHeight * 3);
    std::vector<unsigned char> dArrayResized(outputWidth * outputHeight * 2);

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    if (depthHeight != outputHeight) {
        if ( depthType == APCImageType::DEPTH_8BITS ) MonoBilinearFineScaler( &dArray[0], &dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
        else MonoBilinearFineScaler_short( (unsigned short*)&dArray[0], (unsigned short*)&dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        dArrayResized.assign(dArray.begin(), dArray.end());
    }

    int count = 0;

    float centerX = -1.0f*rectLogData->ReProjectMat[3] * ratio_Mat;
    float centerY = -1.0f*rectLogData->ReProjectMat[7] * ratio_Mat;
    float focalLength = rectLogData->ReProjectMat[11] * ratio_Mat;
    float baseline = 1.0f / rectLogData->ReProjectMat[14];
    float diff = rectLogData->ReProjectMat[15] * ratio_Mat;
    const float PI = 3.14159265359f;
    /*Generate table for disparity to W ( W = disparity / baseline)*/
    float disparityToW[2048];
    int tableSize;
    switch (depthType)
    {
    case APCImageType::DEPTH_8BITS_0x80:
    case APCImageType::DEPTH_8BITS:
        tableSize = 1 << 8;
        for (int i = 0; i < tableSize; i++)
            disparityToW[i] = (i * ratio_Mat) / baseline + diff;
        break;
    case APCImageType::DEPTH_11BITS:
        tableSize = 1 << 11;
        for (int i = 0; i < tableSize; i++)
            disparityToW[i] = (i * ratio_Mat / 8.0f) / baseline + diff;
        break;
    case APCImageType::DEPTH_14BITS:
        break;
    }

    unsigned char *colorBuf = &colorArrayResized[0];
    unsigned char *dArrayBuf = &dArrayResized[0];

    for (int j = 0; j < outputHeight; j++) {
        for (int i = 0; i < outputWidth; i++) {
            int disparity = 0;

            float W = 0.0f;
            float X = 0.0f;
            float Y = 0.0f;
            float Z = 0.0f;
            float angle = 0.0f;

            switch (depthType)
            {
            case APCImageType::DEPTH_8BITS_0x80:
            case APCImageType::DEPTH_11BITS:
                disparity = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                W = disparityToW[disparity];
                break;
            case APCImageType::DEPTH_8BITS:
                disparity = *(dArrayBuf++);
                W = disparityToW[disparity];
                break;
            case APCImageType::DEPTH_14BITS:
                int DtoZ = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                if (DtoZ > 0)
                {
                    W = focalLength / DtoZ;
                }
                break;
            }

            if (!(removeINF && W == 0.0f)) {
                if (outputWidth > outputHeight)
                {
                    angle = (i - centerX) * (PI / outputWidth);
                    X = sin(angle) * (focalLength / W);
                    Y = (j - centerY) / W;
                    Z = cos(angle) * (focalLength / W);
                }
                else
                {
                    angle = (j - centerY) * (PI / outputHeight);
                    X = (i - centerX) / W;
                    Y = sin(angle) * (focalLength / W);
                    Z = cos(angle) * (focalLength / W);
                }

                if (!(clipping && (Z > zFar || Z < zNear))) {
                    CloudPoint point;
                    point.r = *(colorBuf++);
                    point.g = *(colorBuf++);
                    point.b = *(colorBuf++);
                    point.x = X;
                    point.y = Y;
                    point.z = Z;

                    tempPointCloud.push_back(point);
                    count++;
                    continue;
                }
            }
            colorBuf += 3; //RGB
        }
    }
    output.assign(tempPointCloud.begin(), tempPointCloud.begin() + count);

    return 0;
}

int PlyWriter::EYSDFrameTo3DCylinder(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, float scale_ratio)
{
    float ratio_Mat = 1.0f;
    float ratio_Mat_K = 1.0f;

    int outputWidth = 0;
    int outputHeight = 0;

    outputWidth = depthWidth;
    outputHeight = depthHeight;

    outputWidth *= scale_ratio;
    outputHeight *= scale_ratio;

    ratio_Mat = (float)outputHeight / rectLogDataL->OutImgHeight;
    ratio_Mat_K = (float)outputHeight / rectLogDataK->OutImgHeight;

    output.clear();
    std::vector<CloudPoint> tempPointCloud;
    std::vector<unsigned char> colorArrayResized(outputWidth * outputHeight * 3);
    std::vector<unsigned char> dArrayResized(outputWidth * outputHeight * 2);

    if (colorHeight != outputHeight) {
        resampleImage(colorWidth, colorHeight, &colorArray[0], outputWidth, outputHeight, &colorArrayResized[0], 3);
    }
    else
    {
        colorArrayResized.assign(colorArray.begin(), colorArray.end());
    }

    if (depthHeight != outputHeight) {
        if ( depthType == APCImageType::DEPTH_8BITS ) MonoBilinearFineScaler( &dArray[0], &dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
        else MonoBilinearFineScaler_short( (unsigned short*)&dArray[0], (unsigned short*)&dArrayResized[0], depthWidth, depthHeight, outputWidth, outputHeight, 1 );
    }
    else
    {
        dArrayResized.assign(dArray.begin(), dArray.end());
    }

    int count = 0;

    float centerX = -1.0f*rectLogDataL->ReProjectMat[3] * ratio_Mat;
    float centerY = -1.0f*rectLogDataL->ReProjectMat[7] * ratio_Mat;
    float focalLength = rectLogDataL->ReProjectMat[11] * ratio_Mat;
    float baseline = 1.0f / rectLogDataL->ReProjectMat[14];
    float diff = rectLogDataL->ReProjectMat[15] * ratio_Mat;

    float focalLength_K = rectLogDataK->ReProjectMat[11] * ratio_Mat_K;
    float baseline_K = 1.0f / rectLogDataK->ReProjectMat[14];
    float diff_K = rectLogDataK->ReProjectMat[15] * ratio_Mat_K;

    const float PI = 3.14159265359f;
    /*Generate table for disparity to W ( W = disparity / baseline)*/
    float disparityToW[2048];
    int tableSize;
    switch (depthType)
    {
    case APCImageType::DEPTH_8BITS_0x80:
    case APCImageType::DEPTH_8BITS:
        tableSize = 1 << 8;
        for (int i = 0; i < tableSize; i++)
            disparityToW[i] = (i * ratio_Mat) / baseline + diff;
        break;
    case APCImageType::DEPTH_11BITS:
        tableSize = 1 << 11;
        for (int i = 0; i < tableSize; i++)
            disparityToW[i] = (i * ratio_Mat / 8.0f) / baseline + diff;
        break;
    case APCImageType::DEPTH_14BITS:
        break;
    }

    unsigned char *colorBuf = &colorArrayResized[0];
    unsigned char *dArrayBuf = &dArrayResized[0];

    for (int j = 0; j < outputHeight; j++) {
        for (int i = 0; i < outputWidth; i++) {
            int disparity = 0;

            float W = 0.0f;
            float X = 0.0f;
            float Y = 0.0f;
            float Z = 0.0f;
            float angle = 0.0f;

            switch (depthType)
            {
            case APCImageType::DEPTH_8BITS_0x80:
            case APCImageType::DEPTH_11BITS:
                disparity = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                W = disparityToW[disparity];
                break;
            case APCImageType::DEPTH_8BITS:
                disparity = *(dArrayBuf++);
                W = disparityToW[disparity];
                break;
            case APCImageType::DEPTH_14BITS:
                int DtoZ = *(WORD*)dArrayBuf;
                dArrayBuf = dArrayBuf + sizeof(WORD);
                if (DtoZ > 0)
                {
                    W = focalLength / DtoZ;
                }
                break;
            }

            if (!(removeINF && W == 0.0f)) {

                if (outputWidth > outputHeight)
                {
                    angle = (i - centerX) * (PI / outputWidth);
                    X = sin(angle) * (focalLength / W);
                    Y = (j - centerY) / W;
                    Z = cos(angle) * (focalLength / W);
                }
                else
                {
                    angle = (j - centerY) * (PI / outputHeight);
                    X = (i - centerX) / W;
                    Y = sin(angle) * (focalLength / W);
                    Z = cos(angle) * (focalLength / W);
                }
                int shift = ((focalLength_K / Z) - diff_K) * baseline_K;
                if (!(clipping && (Z > zFar || Z < zNear)) && abs(X) < outputWidth && abs(Y) < outputHeight) {
                    CloudPoint point;
                    if ((j - shift) < 0 || j - shift > (outputHeight - 1)) {
                        point.r = 0;
                        point.g = 0;
                        point.b = 0;
                    }
                    else
                    {
                        point.r = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 0];
                        point.g = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 1];
                        point.b = colorArrayResized[((j - shift) * outputWidth + i) * 3 + 2];
                    }
                    //point.r = *(colorBuf++);
                    //point.g = *(colorBuf++);
                    //point.b = *(colorBuf++);
                    point.x = X;
                    point.y = Y;
                    point.z = Z;

                    tempPointCloud.push_back(point);
                    count++;
                    continue;
                }
            }
            colorBuf += 3; //RGB
        }
    }
    output.assign(tempPointCloud.begin(), tempPointCloud.begin() + count);

    return 0;
}

std::string PlyWriter::generateHeader(bool binary,int size)
{
    std::ostringstream oss;
    bool endian = true;

    // Begin header
    oss << "ply";
    if (!binary)
        oss << "\nformat ascii 1.0";
    else
    {
        if (endian)
            oss << "\nformat binary_big_endian 1.0";
        else
            oss << "\nformat binary_little_endian 1.0";
    }

    oss << "\ncomment no face";

    oss << "\nelement vertex " << size;

    oss <<	"\nproperty uchar red"
            "\nproperty uchar green"
            "\nproperty uchar blue";

    oss <<	"\nproperty float x"
            "\nproperty float y"
            "\nproperty float z";
    // End header
    oss << "\nend_header\n";
    return (oss.str());
}

void PlyWriter::resampleImage(int srcWidth, int srcHeight, unsigned char* srcBuf, int dstWidth, int dstHeight, unsigned char* dstBuf, int bytePerPixel) {

    float ratio_W = (float)srcWidth / dstWidth;
    float ratio_H = (float)srcHeight / dstHeight;

    for (int j = 0; j < dstHeight; j++) {
        for (int i = 0; i < dstWidth; i++) {
            int index = ((int)(j * ratio_H) * srcWidth + (int)(i * ratio_W)) * bytePerPixel;
            memcpy(dstBuf, &srcBuf[index], bytePerPixel);
            dstBuf += bytePerPixel;
        }
    }
}

//void PlyWriter::resampleImage_float(int srcWidth, int srcHeight, float* srcBuf, int dstWidth, int dstHeight, float* dstBuf) {
//
//	float ratio_W = (float)srcWidth / dstWidth;
//	float ratio_H = (float)srcHeight / dstHeight;
//
//	for (int j = 0; j < dstHeight; j++) {
//		for (int i = 0; i < dstWidth; i++) {
//			int index = ((int)(j * ratio_H) * srcWidth + (int)(i * ratio_W));
//			memcpy(dstBuf, &srcBuf[index], sizeof(float));
//			dstBuf ++;
//		}
//	}
//}

void PlyWriter::MonoBilinearFineScaler(unsigned char* pIn, unsigned char* pOut, int width, int height, int outwidth, int outheight, int zero_invalid)
{
    float x_fac = (float)width/outwidth;
    float y_fac = (float)height/outheight;
    float outx = 0.0f;
    float outy = 0.0f;
    float rx,ry;
    unsigned char p00,p01,p10,p11;
    int x0,x1,y0,y1;

    for (int y=0; y<outheight; y++)
    {
        y0 = (int) outy;
        y1 = y0 < height - 1 ? y0+1 : y0;
        ry = outy - y0;

        outx = 0.0f;
        for (int x=0; x<outwidth; x++)
        {
            x0 = (int) outx;
            rx = outx - x0;
            x1 = x0 < width - 1 ? x0+1 : x0;
            p00 = pIn[y0*width + x0];
            p01 = pIn[y0*width + x1];
            p10 = pIn[y1*width + x0];
            p11 = pIn[y1*width + x1];

            if (zero_invalid & ( (p00 == 0) | (p01 == 0) | (p10 == 0) | (p11 == 0)))
            {
                *(pOut++) = 0;
            }
            else
            {
                float p0 = rx*p01 + (1-rx)*p00;
                float p1 = rx*p11 + (1-rx)*p10;
                *(pOut++) = (unsigned char)(ry*p1 + (1-ry)*p0);
            }
            outx += x_fac;
        }
        outy += y_fac;
    }
}

void PlyWriter::MonoBilinearFineScaler(unsigned char *pIn, unsigned char *pOut, int width, int height, int outwidth, int outheight, int zero_invalid, int bytePerPixel)
{
    float x_fac = (float)width / outwidth;
    float y_fac = (float)height / outheight;
    float outx = 0.0f;
    float outy = 0.0f;
    float rx, ry;
    unsigned char p00, p01, p10, p11;
    int x0, x1, y0, y1;

    for (int y = 0; y < outheight; y++)
    {
        y0 = (int)outy;
        y1 = y0 < height - 1 ? y0 + 1 : y0;
        ry = outy - y0;

        outx = 0.0f;
        for (int x = 0; x < outwidth; x++)
        {
            x0 = (int)outx;
            rx = outx - x0;
            x1 = x0 < width - 1 ? x0 + 1 : x0;
            for (int byte = 0 ; byte < bytePerPixel ; ++byte)
            {
                p00 = pIn[(y0 * width + x0) * bytePerPixel + byte];
                p01 = pIn[(y0 * width + x1) * bytePerPixel + byte];
                p10 = pIn[(y1 * width + x0) * bytePerPixel + byte];
                p11 = pIn[(y1 * width + x1) * bytePerPixel + byte];

                if (zero_invalid & ((p00 == 0) | (p01 == 0) | (p10 == 0) | (p11 == 0)))
                {
                    *(pOut++) = 0;
                }
                else
                {
                    unsigned char p0 = rx * p01 + (1 - rx) * p00;
                    unsigned char p1 = rx * p11 + (1 - rx) * p10;
                    *(pOut++) = (ry * p1 + (1 - ry) * p0);
                }
            }
            outx += x_fac;
        }
        outy += y_fac;
    }
}

void PlyWriter::MonoBilinearFineScaler_short(unsigned short* pIn, unsigned short* pOut, int width, int height, int outwidth, int outheight, int zero_invalid)
{
    float x_fac = (float)width/outwidth;
    float y_fac = (float)height/outheight;
    float outx = 0.0f;
    float outy = 0.0f;
    float rx,ry;
    unsigned short p00,p01,p10,p11;
    int x0,x1,y0,y1;

    for (int y=0; y<outheight; y++)
    {
        y0 = (int) outy;
        y1 = y0 < height - 1 ? y0+1 : y0;
        ry = outy - y0;

        outx = 0.0f;
        for (int x=0; x<outwidth; x++)
        {
            x0 = (int) outx;
            rx = outx - x0;
            x1 = x0 < width - 1 ? x0+1 : x0;
            p00 = pIn[y0*width + x0];
            p01 = pIn[y0*width + x1];
            p10 = pIn[y1*width + x0];
            p11 = pIn[y1*width + x1];

            if (zero_invalid & ( (p00 == 0) | (p01 == 0) | (p10 == 0) | (p11 == 0)))
            {
                *(pOut++) = 0;
            }
            else
            {
                float p0 = rx*p01 + (1-rx)*p00;
                float p1 = rx*p11 + (1-rx)*p10;
                *(pOut++) = (unsigned short)(ry*p1 + (1-ry)*p0);
            }
            outx += x_fac;
        }
        outy += y_fac;
    }
}

void PlyWriter::MonoBilinearFineScaler_float(float* pIn, float* pOut, int width, int height, int outwidth, int outheight, int zero_invalid)
{
    float x_fac = (float)width/outwidth;
    float y_fac = (float)height/outheight;
    float outx = 0.0f;
    float outy = 0.0f;
    float rx,ry;
    float p00,p01,p10,p11;
    int x0,x1,y0,y1;

    for (int y=0; y<outheight; y++)
    {
        y0 = (int) outy;
        y1 = y0 < height - 1 ? y0+1 : y0;
        ry = outy - y0;

        outx = 0.0f;
        for (int x=0; x<outwidth; x++)
        {
            x0 = (int) outx;
            rx = outx - x0;
            x1 = x0 < width - 1 ? x0+1 : x0;
            p00 = pIn[y0*width + x0];
            p01 = pIn[y0*width + x1];
            p10 = pIn[y1*width + x0];
            p11 = pIn[y1*width + x1];

            if (zero_invalid & ( (p00 == 0) | (p01 == 0) | (p10 == 0) | (p11 == 0)))
            {
                *(pOut++) = 0;
            }
            else
            {
                float p0 = rx*p01 + (1-rx)*p00;
                float p1 = rx*p11 + (1-rx)*p10;
                *(pOut++) = (float)(ry*p1 + (1-ry)*p0);
            }
            outx += x_fac;
        }
        outy += y_fac;
    }
}

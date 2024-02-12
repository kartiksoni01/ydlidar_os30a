#include "ColorPaletteGenerator.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "eSPDI.h"
#include <stdio.h>
 #include <unistd.h>

double ColorPaletteGenerator::maxHueAngle = 270.0f;
int ColorPaletteGenerator::defaultAlpha = 255;

void ColorPaletteGenerator::generatePalette(unsigned char *palette, int paletteSize, bool reverseRedToBlue) {

    generatePalette(palette, paletteSize, 0, paletteSize);
}

void ColorPaletteGenerator::generatePalette(unsigned char *palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseRedToBlue) {
    generatePalette(palette, paletteSize, controlPoint1, 0.1f, controlPoint2,0.9f);
}


void ColorPaletteGenerator::generatePalette(unsigned char *palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseRedToBlue) {

    double R, G, B;
    double step = 0;

    if (controlPoint1 > controlPoint2)
        controlPoint1 = controlPoint2;


    step = (controlPoint1Value - 0.0f) / (float)(controlPoint1 - 0);



    int count = 0;
    for (int i = 0; i < controlPoint1; i++) {
        getRGB(step * i, R, G, B, reverseRedToBlue);
        palette[count*4 + 0] = (unsigned char)R;
        palette[count*4 + 1] = (unsigned char)G;
        palette[count*4 + 2] = (unsigned char)B;
        palette[count*4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (controlPoint2Value - controlPoint1Value) / (double)(controlPoint2 - controlPoint1);

    for (int i = 0; i< controlPoint2 - controlPoint1; i++) {
        getRGB(step * i + controlPoint1Value, R, G, B, reverseRedToBlue);
        palette[count * 4 + 0] = (unsigned char)R;
        palette[count * 4 + 1] = (unsigned char)G;
        palette[count * 4 + 2] = (unsigned char)B;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (1.0f - controlPoint2Value) / (double)(paletteSize - controlPoint2);
    for (int i = 0; i < paletteSize - controlPoint2; i++) {
        getRGB(step * i + controlPoint2Value, R, G, B, reverseRedToBlue);
        palette[count * 4 + 0] = (unsigned char)R;
        palette[count * 4 + 1] = (unsigned char)G;
        palette[count * 4 + 2] = (unsigned char)B;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    palette[0] = 0;
    palette[1] = 0;
    palette[2] = 0;
    palette[3] = (unsigned char)defaultAlpha;
}

void ColorPaletteGenerator::generatePaletteColor(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseRedToBlue) {
    double ROI2 = 1.0f;
    double ROI1 = 0.0f;

    //The value ranges from 0.0f ~ 1.0f as hue angle
    double ROI2Value = 1.0f;
    double ROI1Value = 0.0f;

    unsigned char* buf = (unsigned char*)malloc(4 * size);

    //Set ROI by mode setting.The bigger the disparity the nearer the distance
    switch (mode) {
    case 1: //near
        ROI2 = 0.8f;
        ROI1 = 0.5f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 2: //midle
        ROI2 = 0.7f;
        ROI1 = 0.3f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 3: //far
        ROI2 = 0.6f;
        ROI1 = 0.2f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 4: //custom
        ROI2 = 1.0f*customROI2 / size;
        ROI1 = 1.0f*customROI1 / size;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    default: //normal
        ROI2 = 1.0f;
        ROI1 = 0.0f;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    }

    generatePalette(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseRedToBlue);

    for (int i = 0; i < size; i++) {
        palette[i].rgbRed		= buf[i * 4 + 0];
        palette[i].rgbGreen		= buf[i * 4 + 1];
        palette[i].rgbBlue		= buf[i * 4 + 2];
        palette[i].rgbReserved	= buf[i * 4 + 3];
    }

    usleep(100 * 1000);
    free(buf);
}


void ColorPaletteGenerator::generatePaletteGray(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseGraylevel){
    float ROI1 = 0.0f;
    float ROI2 = 1.0f;


    //The value ranges from 0.0f ~ 1.0f as hue angle
    float ROI1Value = 0.0f;
    float ROI2Value = 1.0f;


    unsigned char* buf = (unsigned char*)malloc(sizeof(unsigned char) * 4 * size);
    //unsigned char buf[(size) * 4];
    //Set ROI by mode setting.The bigger the disparity the nearer the distance
    switch (mode) {
    case 1: //near
        ROI2 = 0.8f;
        ROI1 = 0.5f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 2: //midle
        ROI2 = 0.7f;
        ROI1 = 0.3f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 3: //far
        ROI2 = 0.6f;
        ROI1 = 0.2f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 4: //custom
        ROI2 = 1.0f*customROI2 / size;
        ROI1 = 1.0f*customROI1 / size;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    default: //normal
        ROI2 = 1.0f;
        ROI1 = 0.0f;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    }

    generatePaletteGray(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseGraylevel);
    for (int i = 0; i < size; i++) {
        palette[i].rgbBlue = buf[i * 4 + 2];
        palette[i].rgbGreen = buf[i * 4 + 1];
        palette[i].rgbRed = buf[i * 4 + 0];
        palette[i].rgbReserved = buf[i * 4 + 3];
    }

    free(buf);
}

void ColorPaletteGenerator::generatePaletteGray(unsigned char * palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseGraylevel) {
    generatePalette(palette, paletteSize, controlPoint1, 0.1f, controlPoint2, 0.9f);
}

void ColorPaletteGenerator::generatePaletteGray(unsigned char * palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseGraylevel) {
    float step;
    step = (controlPoint1Value - 0.0f) / (float)(controlPoint1 - 0);

    int count = 0;
    for (int i = 0; i < controlPoint1; i++) {
        int grayValue = 255 * (reverseGraylevel ? 1.0f - (step * i) : (step * i));
        palette[count * 4 + 0] = (unsigned char)0;
        palette[count * 4 + 1] = (unsigned char)0;
        palette[count * 4 + 2] = (unsigned char)0;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (controlPoint2Value - controlPoint1Value) / (float)(controlPoint2 - controlPoint1);
    for (int i = 0; i< controlPoint2 - controlPoint1; i++) {
        int grayValue = 255 * (reverseGraylevel ? 1.0f - ((step * i + controlPoint1Value)) : ((step * i + controlPoint1Value)));
        palette[count * 4 + 0] = (unsigned char)grayValue;
        palette[count * 4 + 1] = (unsigned char)grayValue;
        palette[count * 4 + 2] = (unsigned char)grayValue;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (1.0f - controlPoint2Value) / (float)(paletteSize - controlPoint2);
    for (int i = 0; i < paletteSize - controlPoint2; i++) {
        int grayValue = 255 * (reverseGraylevel ? 1.0f - ((step * i + controlPoint2Value)) : ((step * i + controlPoint2Value)));
        palette[count * 4 + 0] = (unsigned char)0;
        palette[count * 4 + 1] = (unsigned char)0;
        palette[count * 4 + 2] = (unsigned char)0;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    palette[0] = 0;
    palette[1] = 0;
    palette[2] = 0;
    palette[3] = (unsigned char)defaultAlpha;
}

void ColorPaletteGenerator::getRGB(double value, double &R, double &G, double &B, bool reverseRedToBlue) {

    if (reverseRedToBlue) value = 1.0f - value;
    HSV_to_RGB(value * maxHueAngle, 1.0f,1.0f,R,G,B);
}

void ColorPaletteGenerator::getRGB_14bits(double value, double &R, double &G, double &B, bool reverseRedToBlue) {

    if (reverseRedToBlue) value = 1.0f - value;
    HSV_to_RGB(value * maxHueAngle, 1.0f,1.0f,R,G,B);
}


void ColorPaletteGenerator::DmColorMode14(RGBQUAD *palette,float zFar,float zNear, bool bReverColor)
{
    const int size = 1 << 14; // 14 bits
    bool reverseRedtoBlue = bReverColor;
    int mode = 4; //custom
    generatePaletteColor(palette, size,mode, zNear, zFar, reverseRedtoBlue);
}

void ColorPaletteGenerator::DmGrayMode14(RGBQUAD *palette,  float zFar,float zNear, bool bReverColor)
{
    const int size = 1 << 14; // 14bit 16384
    int mode = 4;//custom
    bool reverseGraylevel = bReverColor;
    generatePaletteGray(palette, size, mode, zNear, zFar, reverseGraylevel);
}

void ColorPaletteGenerator::HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B)
{
    float nMax, nMin;
    float fDet;
    //
    while (H<0.0) H += 360.0;
    while (H >= 360.0) H -= 360.0;
    H /= 60.0;
    if (V<0.0) V = 0.0;
    if (V>1.0) V = 1.0;
    V *= 255.0;
    if (S<0.0) S = 0.0;
    if (S>1.0) S = 1.0;
    //
    if (V == 0.0) {
        R = G = B = 0;
    }
    else {
        fDet = S * V;
        nMax = (V);
        nMin = (V - fDet);

        if (H <= 0.005) { //R>=G>=B, H=(G-B)/fDet
            B = 0;
            G = 0;
            R = 0;
        }
        else if (H <= 1.0) { //R>=G>=B, H=(G-B)/fDet
            R = nMax;
            B = nMin;
            G = (H*fDet + B);
        }
        else if (H <= 2.0) { //G>=R>=B, H=2+(B-R)/fDet
            G = nMax;
            B = nMin;
            R = ((2.0 - H)*fDet + B);
        }
        else if (H <= 3.0) { //G>=B>=R, H=2+(B-R)/fDet
            G = nMax;
            R = nMin;
            B = ((H - 2.0)*fDet + R);
        }
        else if (H <= 4.0) { //B>=G>=R, H=4+(R-G)/fDet
            B = nMax;
            R = nMin;
            G = ((4.0 - H)*fDet + R);
        }
        else if (H <= 4.49) { //B>=R>=G, H=4+(R-G)/fDet
            B = nMax;
            G = nMin;
            R = ((H - 4.0)*fDet + G);
        }
        else if (H <= 5.0) { //B>=R>=G, H=4+(R-G)/fDet
            B = 0;
            G = 0;
            R = 0;
        }
        else { // if (H<6.0) //R>=B>=G, H=(G-B)/fDet+6
            R = nMax;
            G = nMin;
            B = ((6.0 - H)*fDet + G);
        }
    }
}

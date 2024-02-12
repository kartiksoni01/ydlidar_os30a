#include "utImageProcessingUtility.h"
#include <QDebug>
#include <math.h>

int utImageProcessingUtility::convert_yuv_to_rgb_pixel(int y, int u, int v)
{
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;

    r = y + (1.370705 * (v-128));
    g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
    b = y + (1.732446 * (u-128));

    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;

    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;

    pixel[0] = r * 220 / 256;
    pixel[1] = g * 220 / 256;
    pixel[2] = b * 220 / 256;

    return pixel32;
}

int utImageProcessingUtility::convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height)
{
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    int y0, u, y1, v;
    for (in = 0; in < width * height * 2; in += 4) {
        pixel_16 =
        yuv[in + 3] << 24 |
        yuv[in + 2] << 16 |
        yuv[in + 1] <<  8 |
        yuv[in + 0];

        y0 = (pixel_16 & 0x000000ff);
        u  = (pixel_16 & 0x0000ff00) >>  8;
        y1 = (pixel_16 & 0x00ff0000) >> 16;
        v  = (pixel_16 & 0xff000000) >> 24;

        pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);

        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];

        pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);

        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];
    }

    return 0;
}

void utImageProcessingUtility::UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, unsigned char *pDepthZ14, unsigned char *pDepthDIB24, int width, int height)
{
    int x,y,nBPS;
    unsigned short *pWSL,*pWS;
    unsigned char *pDL,*pD;
    RGBQUAD *pClr;
    //
    if ((width <= 0) || (height <= 0)) return;
    //
    nBPS = width * 3;
    pWSL = (unsigned short *)pDepthZ14;
    pDL = pDepthDIB24;
    for (y=0; y<height; y++) {
        pWS = pWSL;
        pD = pDL;
        for (x=0; x<width; x++) {
            //++ avoid afterimage on outdoor
            if ( pWS[x] >= COLOR_PALETTE_MAX_COUNT) {
                pWS[x] = COLOR_PALETTE_MAX_COUNT-1;
            }
            //-- avoid afterimage on outdoor
            pClr = &(pColorPaletteZ14[pWS[x]]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;
            pD += 3;
        }
        pWSL += width;
        pDL += nBPS;
    }
}

void utImageProcessingUtility::UpdateD11DisplayImage_DIB24(RGBQUAD* pColorPalette, unsigned char *pDepth, unsigned char *pRGB, int width, int height, BYTE *pZDTable)
{
    if (width <=0 || height <= 0 ) return;

    int nBPS = ((width * 3 + 3 ) / 4 ) * 4;
    BYTE* pDL    = pRGB;
    BYTE* pD     = NULL;
    const RGBQUAD* pClr = NULL;
    unsigned short z = 0;
    unsigned depthValueLimit11Bit = ((1 << 12) - 1);
    for (int y = 0; y < height; y++) {
        pD = pDL;
        for (int x = 0; x < width; x++) {
            int pixelIndex = y * width + x;
            unsigned short depth = pDepth[pixelIndex * sizeof(unsigned short) + 1] << 8 |  pDepth[pixelIndex * sizeof(unsigned short)];
            //++ avoid afterimage on outdoor
            if (depth > depthValueLimit11Bit) {
                depth = depthValueLimit11Bit;
            }
            //-- avoid afterimage on outdoor
            unsigned short zdIndex = depth * sizeof(unsigned short);
            z = (((unsigned short)pZDTable[zdIndex]) << 8) + pZDTable[zdIndex + 1];
            //++ avoid afterimage on outdoor
            if ( z >= COLOR_PALETTE_MAX_COUNT) {
                z = COLOR_PALETTE_MAX_COUNT-1;
            }
            //-- avoid afterimage on outdoor
            pClr = &(pColorPalette[z]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;
            pD += 3;
        }
        pDL += nBPS;
    }
}

void utImageProcessingUtility::UpdateD8bitsDisplayImage_DIB24(RGBQUAD *pColorPalette, unsigned char *pDepth, unsigned char *pRGB, int width, int height, BYTE *pZDTable, int nZDTableSize)
{
    int nBPS = width * 3;;
    BYTE* pDL    = pRGB;
    BYTE* pD     = NULL;
    const RGBQUAD* pClr = NULL;
    unsigned short z = 0;
    unsigned short zdIndex;

    if ((width <= 0) || (height <= 0)) return;

    for (int y = 0; y < height; y++) {
        pD = pDL;
        for (int x = 0; x < width; x++) {
            int pixelIndex = y * width + x;
            unsigned short depth = (unsigned short)pDepth[pixelIndex];
            if (nZDTableSize == APC_ZD_TABLE_FILE_SIZE_11_BITS)
                zdIndex = (depth << 3) * sizeof(unsigned short);
            else
                zdIndex = depth * sizeof(unsigned short);
            z = (((unsigned short)pZDTable[zdIndex]) << 8) + pZDTable[zdIndex + 1];
            //++ avoid afterimage on outdoor
            if ( z >= COLOR_PALETTE_MAX_COUNT) {
                z = COLOR_PALETTE_MAX_COUNT-1;
            }
            //-- avoid afterimage on outdoor

            pClr = &(pColorPalette[z]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;

            pD += 3;
        }
        pDL += nBPS;
    }
}

void utImageProcessingUtility::UpdateD11_Baseline_DisplayImage_DIB24(RGBQUAD* pColorPalette, WORD *pDepth,
                                                                     unsigned char *pRGB, int width, int height,
                                                                     double dblCamFocus, double dblBaselineDist,
                                                                     int nZNear, int nZFar)
{
    if ( width <= 0 || height <= 0 ) return;

    int nBPS = ( ( width * 3 + 3 ) / 4 ) * 4;
    BYTE* pDL    = pRGB;
    BYTE* pD     = 0;
    const RGBQUAD* pClr = 0;
    static const RGBQUAD empty = { 0, 0, 0, 0 };
    WORD wDepth = 0;

    for ( int y = 0; y < height; y++ )
    {
        pD = pDL;
        for ( int x = 0; x < width; x++ )
        {
            wDepth = ( pDepth[ x ] ? ( WORD )( 8.0 * dblCamFocus * dblBaselineDist / pDepth[ x ] ) : 0 );

            if ( wDepth > nZFar || wDepth < nZNear )
                pClr = &empty;
            else {
                //++ avoid afterimage on outdoor
                if ( wDepth >= COLOR_PALETTE_MAX_COUNT) {
                    wDepth = COLOR_PALETTE_MAX_COUNT-1;
                }
                //-- avoid afterimage on outdoor
                pClr = &(pColorPalette[wDepth]);
            }

            pD[0] = pClr->rgbRed; //R
            pD[1] = pClr->rgbGreen; //G
            pD[2] = pClr->rgbBlue; //B
            pD += 3;
        }
        pDepth += width;
        pDL += nBPS;
    }
}

void utImageProcessingUtility::UpdateD11_Fusion_DisplayImage_DIB24(RGBQUAD* pColorPalette, WORD *pDepthFs, WORD *pDepth,
                                                                   unsigned char *pRGB, int width, int height,
                                                                   double dblCamFocus, double dblBaselineDist,
                                                                   int nZNear, int nZFar)
{
    if ( width <= 0 || height <= 0 ) return;

    int nBPS = ( ( width * 3 + 3 ) / 4 ) * 4;
    BYTE* pDL    = pRGB;
    BYTE* pD     = 0;
    const RGBQUAD* pClr = 0;
    static const RGBQUAD empty = { 0, 0, 0, 0 };
    WORD wDepth = 0;

    for ( int y = 0; y < height; y++ )
    {
        pD = pDL;
        for ( int x = 0; x < width; x++ )
        {
            wDepth = ( pDepth[ x ] ? ( WORD )( 8.0 * dblCamFocus * dblBaselineDist / pDepth[ x ] ) : 0 );

            if ( wDepth < nZNear || wDepth > nZFar )
                pClr = &empty;
            else {
                wDepth = ( pDepthFs[ x ] ? ( WORD )( 8.0 * dblCamFocus * dblBaselineDist / pDepthFs[ x ] ) : 0 );
                //++ avoid afterimage on outdoor
                if ( wDepth >= COLOR_PALETTE_MAX_COUNT) {
                    wDepth = COLOR_PALETTE_MAX_COUNT-1;
                }
                //-- avoid afterimage on outdoor
                pClr = &(pColorPalette[wDepth]);
            }

            pD[0] = pClr->rgbRed; //R
            pD[1] = pClr->rgbGreen; //G
            pD[2] = pClr->rgbBlue; //B
            pD += 3;
        }
        pDepth += width;
        pDepthFs += width;
        pDL += nBPS;
    }
}

void utImageProcessingUtility::Rotate2Dpoint(float cx, float cy, float cxNew, float cyNew, float angle, float &x, float &y)
{

    const float PI = 3.14159265;
    const float val = PI / 180.0f;
    float s = sin(angle*val);
    float c = cos(angle*val);

    // translate point back to origin:
    x -= cx;
    y -= cy;

    // rotate point
    float xnew = x * c - y * s;
    float ynew = x * s + y * c;

    // translate point back:
    x = xnew + cxNew;
    y = ynew + cyNew;
}


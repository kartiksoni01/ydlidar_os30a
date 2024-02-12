#ifndef V4L2_H
#define V4L2_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <unistd.h>
#include <QDebug>
#include <QQueue>
#include <QTime>
#include <qcoreapplication.h>
#include "mapping.h"
#include "QMutex"

class v4l2
{
public:
   v4l2();
   virtual ~v4l2();
   unsigned char *rgb24 = nullptr;
   short *Y16data = nullptr;
   short *YUVdata = nullptr;
   unsigned char *ParamLinedata = nullptr;
   unsigned char *srcData;

    int Open_Camera(char* videonum);
   void Get_Camera_Capability(int fd);
   void setsize(int video_w, int viedo_h);
   void Set_Video_Format(int camera,int video_w,int video_h);
   v4l2_format Get_Current_Format(int fd);
   QList<QString> Get_Camera_Supportformat(int fd);
   void Set_Frames(int fd,int frameRates);
    int Get_Frames(int fd);

   void StartVideoPrePare(int fd);
   void StartVideo(int fd);
    int GetFrame(int fd);
   void StopVideo(int fd);
    int convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);
   void convert_y16_to_rgb_buffer(short* y16,int video_w,int video_h,unsigned char *rgb);
   void CloseVideoStream(int fd);
   void CloseCamera(int fd);
   void ReleaseVideoStream();
   void delay(unsigned int msec);
   void frame2queue(uchar* tempdata);

   void distributeData(uchar* Data,int video_w);
   void InitDataMemory(int video_w,int video_h);
   enum ImageType {YUV, Y16, YUV_Y16, Y16_Param,YUV_Param,YUV_Y16_Param};
   ImageType imgType;
   Mapping mapping;

   int FD;
   FILE *fp;
   QQueue<QByteArray> queue_frame;
   int colorIndex;
   int VIDEO_WIDTH = 360;
   int VIDEO_HEIGHT = 240;
};

#endif // V4L2_H







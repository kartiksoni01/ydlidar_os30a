#include "v4l2.h"
#define BUFFER_COUNT 4

typedef struct VideoBuffer
{
    void   *start;
    size_t  length;
} VideoBuffer;

struct v4l2_requestbuffers reqbuf;
VideoBuffer*  framebuf;
VideoBuffer*  framebuf2;
struct v4l2_buffer buf;
short* tempy16 = nullptr;

v4l2::v4l2()
{
    colorIndex = 0;
    FD = 0;
    fp = nullptr;
}

v4l2::~v4l2(){}

int  v4l2::Open_Camera(char* devVideo)
{
    int fd=open(devVideo,O_RDWR);
    FD = fd;
    if(fd<0)
        qDebug()<<"opoened error\n";
    return fd;
}

void v4l2::CloseCamera(int fd)
{
    close(fd);
    ReleaseVideoStream();
}

void  v4l2::Get_Camera_Capability(int camera)
{
    struct v4l2_capability cap;
    if(-1 == ioctl(camera, VIDIOC_QUERYCAP, &cap)){
        qDebug()<<"VIDIOC_QUERYCAP failed!!";
    }
    qDebug()<<"Capability Informations:";
    qDebug()<<"driver:   "<<cap.driver;
    qDebug()<<"card:     "<<cap.card;
    qDebug()<<"bus_info: "<<cap.bus_info;
    qDebug()<<"version:  "<<cap.version;
    qDebug()<<"capabilities:"<<cap.capabilities<<"\n";
}

v4l2_format  v4l2::Get_Current_Format(int camera)
{
    struct v4l2_format fmt;
    memset(&fmt,0,sizeof(fmt));
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(camera, VIDIOC_G_FMT, &fmt);
    #if 0
    qDebug()<<"Stream Format Informations:";
    qDebug()<<"type:"<<fmt.type;
    qDebug()<<"width:"<<fmt.fmt.pix.width;
    qDebug()<<"height:"<<fmt.fmt.pix.height;
    #endif
    char fmtstr[8];
    memset(fmtstr, 0, 8);
    memcpy(fmtstr, &fmt.fmt.pix.pixelformat, 4);
    #if 0
    qDebug()<<"pixelformat:"<<fmtstr;
    qDebug()<<"field:"<<fmt.fmt.pix.field;
    qDebug()<<"bytesperline:"<<fmt.fmt.pix.bytesperline;
    qDebug()<<"sizeimage:"<<fmt.fmt.pix.sizeimage;
    qDebug()<<"colorspace:"<<fmt.fmt.pix.colorspace;
    qDebug()<<"priv:"<<fmt.fmt.pix.priv;
    qDebug()<<"raw_date:"<<fmt.fmt.raw_data;
    #endif
    return fmt;
}

void v4l2::Set_Video_Format(int camera,int video_w,int video_h)
{
    struct v4l2_format fmt;
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(camera, VIDIOC_G_FMT, &fmt);
   fmt.fmt.pix.width = video_w;
   fmt.fmt.pix.height = video_h;
   fmt.fmt.pix.field = 1;
   fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
   fmt.fmt.pix.bytesperline = video_w*2;
   fmt.fmt.pix.sizeimage =  video_w * video_h * 2;
   ioctl(camera, VIDIOC_S_FMT, &fmt);
   ioctl(camera, VIDIOC_G_FMT, &fmt);

   srcData = (unsigned char*)malloc(video_w*video_h*2*sizeof(char));
   #if 0
   qDebug()<<"width:"<<fmt.fmt.pix.width;
   qDebug()<<"height:"<<fmt.fmt.pix.height;
   qDebug()<<"bytesperline:"<<fmt.fmt.pix.bytesperline;
   qDebug()<<"sizeimage:"<<fmt.fmt.pix.sizeimage;
   #endif
}

void  framePerSecond2(int camera, struct v4l2_frmsizeenum* frmsize)
{
    struct v4l2_frmivalenum frmval;
    frmval.index = 0;
    frmval.pixel_format = frmsize->pixel_format;
    frmval.width = frmsize->discrete.width;
    frmval.height = frmsize->discrete.height;
    qDebug()<<"Frame rate:  ";
    while(!ioctl(camera, VIDIOC_ENUM_FRAMEINTERVALS, &frmval))
    {
        frmval.index++;
        unsigned int frmrate;
        frmrate = (frmval.discrete.denominator / frmval.discrete.numerator);
        qDebug()<<frmrate;
    }
}

QList<QString> supportResolution(int camera, struct v4l2_fmtdesc* format)
{
    struct v4l2_frmsizeenum frmsize;
    frmsize.index = 0;
    frmsize.pixel_format = format->pixelformat;
    qDebug()<<"Support fram size:";
    QList<QString> list;
    while(!ioctl(camera, VIDIOC_ENUM_FRAMESIZES, &frmsize))
    {
        frmsize.index++;
        //only support : 360*240 with Y16+parm
        if(360 == frmsize.discrete.width && 240 == frmsize.discrete.height){
            qDebug()<<"width:"<<frmsize.discrete.width<<"height:"<<frmsize.discrete.height;
            framePerSecond2(camera, &frmsize);//get framerate
            list<<QString::number(frmsize.discrete.width)+"*"+QString::number(frmsize.discrete.height);
        }
    }
    return list;
}

QList<QString> v4l2::Get_Camera_Supportformat(int camera )
{
    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index=0;
    fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    QList<QString> list;
    qDebug()<<"Supportformat:";
    while(ioctl(camera,VIDIOC_ENUM_FMT,&fmtdesc)!=-1)
    {
       qDebug()<<fmtdesc.index+1<<" "<<fmtdesc.description;
       fmtdesc.index++;
       list = supportResolution(camera,&fmtdesc);
    }
    return list;
}

void  v4l2::Set_Frames(int camera,int frameRates)
{
    struct v4l2_streamparm Stream_Parm;
    memset(&Stream_Parm, 0, sizeof(struct v4l2_streamparm));
    Stream_Parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    Stream_Parm.parm.capture.timeperframe.denominator = frameRates;
    Stream_Parm.parm.capture.timeperframe.numerator = 1;
    ioctl(camera, VIDIOC_S_PARM, &Stream_Parm);
}

int  v4l2::Get_Frames(int fd)
{
    struct v4l2_streamparm Stream_Parm;
    memset(&Stream_Parm, 0, sizeof(struct v4l2_streamparm));
    Stream_Parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_G_PARM, &Stream_Parm);
    int fm=Stream_Parm.parm.capture.timeperframe.denominator;
    int fz =Stream_Parm.parm.capture.timeperframe.numerator;
    return fm/fz;
}

int convert_yuv_to_rgb_pixel(int y, int u, int v)
{
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;
    r = y + (1.370705 * (v-128));
    g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
    b = y + (1.732446 * (u-128));
    if(r > 255) r = 255;
    if(g > 255) g = 255;
    if(b > 255) b = 255;
    if(r < 0) r = 0;
    if(g < 0) g = 0;
    if(b < 0) b = 0;
    pixel[0] = r ;
    pixel[1] = g ;
    pixel[2] = b ;
    return pixel32;
}

int  v4l2::convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height)
{
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    int y0, u, y1, v;

    for(in = 0; in < width * height * 2; in += 4)
    {
        pixel_16 = yuv[in + 3] << 24 |
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

void v4l2::StartVideo(int fd)
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret = ioctl(fd, VIDIOC_STREAMON, &type);//Start Video Stream
    if (ret < 0) {qDebug()<<"VIDIOC_STREAMON failed:"<<ret;}
}

void v4l2::delay(unsigned int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void  v4l2::StartVideoPrePare(int fd)
{
    memset(&(reqbuf), 0, sizeof(reqbuf));

    reqbuf.count = BUFFER_COUNT;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    int ret = ioctl(fd , VIDIOC_REQBUFS, &reqbuf);
    if(ret < 0)
    {
        qDebug()<<"VIDIOC_REQBUFS failed:"<<ret;
    }

    framebuf =(struct VideoBuffer*) calloc( reqbuf.count, sizeof(VideoBuffer));
    if(framebuf == NULL)
        perror("framebuf is NULL");
    else
        assert (framebuf != NULL);
     
    framebuf2 =(struct VideoBuffer*) calloc( reqbuf.count, sizeof(VideoBuffer));
    if(framebuf2 == NULL)
        perror("framebuf2 is NULL");
    else
        assert (framebuf2 != NULL);
    
    for (int i = 0; i < reqbuf.count; i++)
    {
        memset (&buf, 0, sizeof (buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ret = ioctl(fd , VIDIOC_QUERYBUF, &buf);
        if(ret < 0)
        {
            qDebug()<<"VIDIOC_QUERYBUF "<<i<<"failed:"<<ret;
        }

        //mmap buffer
        framebuf[i].length = buf.length;
        framebuf[i].start = (char *) mmap(0, buf.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (framebuf[i].start == MAP_FAILED)
        {
            qDebug()<<"mmap"<<i<<"failed:"<<strerror(errno);
        }


        framebuf2[i].length = buf.length;
        framebuf2[i].start = (char *) mmap(0, buf.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (framebuf2[i].start == MAP_FAILED)
        {
            qDebug()<<"mmap"<<i<<"failed:"<<strerror(errno);
        }
    }

    for(unsigned int ii = 0; ii < reqbuf.count; ii++)
    {
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = ii;
        if (ioctl(fd,VIDIOC_QBUF,&buf)==-1)
        {
            perror("VIDIOC_QBUF failed");
        }
    }
}

void v4l2::CloseVideoStream(int fd)
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd,VIDIOC_STREAMOFF,&type) == -1) {
        perror("VIDIOC_STREAMOFF failed");
    }
}

void v4l2::ReleaseVideoStream()
{
    int i;
    if(framebuf){
        for (i = 0; i < (int)reqbuf.count; i++)
        {
                if(-1 == munmap (framebuf[i].start, framebuf[i].length)){
                    printf("munmap framebuf: %s\n", strerror(errno));
                }
        }
        if(framebuf){
            free(framebuf);
            framebuf = NULL;
        }
     }

    if(framebuf2){
          for (i = 0; i < (int)reqbuf.count; i++)
          {
                  if(munmap (framebuf2[i].start, framebuf2[i].length)){
                      printf("munmap framebuf2: %s\n", strerror(errno));
                  }
          }
          if(framebuf2){
              free(framebuf2);
              framebuf2 = NULL;
          }
       }
    
    free(rgb24);
    free(Y16data);
    free(YUVdata);
    free(ParamLinedata);
    queue_frame.clear();
    rgb24 = nullptr;
    Y16data = nullptr;
    YUVdata = nullptr;
    ParamLinedata = nullptr;
}

void v4l2::StopVideo(int fd)
{
    CloseVideoStream(fd);
    ReleaseVideoStream();
}



int ts;
fd_set fds;
struct timeval tv;

int i = 2;
int v4l2::GetFrame(int fd)
{
    int ret = 0;
    memset(&(buf), 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    FD_ZERO (&fds);
    FD_SET (fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    ts = select (fd + 1, &fds, NULL, NULL, &tv);
    if(ts == 0)
    {
        //Timeout
        qDebug()<<"*";
        qDebug()<<"**";
        qDebug()<<"***";
        qDebug()<<"****";
        qDebug()<<"*****";
    }
    
    ret = ioctl(fd, VIDIOC_DQBUF, &buf) ;
    if(ret < 0)
    //if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1)
    {
       perror("GetFrame VIDIOC_DQBUF Failed...");
       printf("VIDIOC_DQBUF(%s) :  EYSD_GET_IMAGE_FAIL, errno:%d\n", strerror(errno), errno);
       return ret;
    }
    else
    {
        if(i%2 == 0)
        {
            frame2queue((uchar*)framebuf[buf.index].start);
            ++i;
        }
        else
        {
            frame2queue((uchar*)framebuf2[buf.index].start);
            i = 2;
        }
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
        {
            perror("GetFrame VIDIOC_QBUF Failed");
            return -1;
        }
        return 0;
    }
}

void  v4l2::setsize(int video_w, int viedo_h)
{
    VIDEO_WIDTH = video_w;
    VIDEO_HEIGHT = viedo_h;
}

void v4l2::convert_y16_to_rgb_buffer(short* y16,int video_w,int video_h,unsigned char* rgb24)
{
    mapping.Data16ToRGB24(y16, rgb24, video_w * video_h, 2/*colorIndex*/);
}

void v4l2::frame2queue(uchar* data)
{
    QByteArray array;
    for (int i = 0;i < buf.length;i++)
    {
        array.append(data[i]);
    }
    queue_frame.enqueue(array);
}

void v4l2::distributeData(uchar* Data,int video_w)
{
    int length = VIDEO_WIDTH*VIDEO_HEIGHT*2;
    for (int i = 0;i < buf.length/2 ; i++)
    {
        srcData[2 * i] = Data[2 * i + 1];
        srcData[2 * i + 1] = Data[2 * i];
    }

    switch (imgType)
    {
        case YUV:
            memcpy(YUVdata,srcData,length);
            break;
        case Y16:
            memcpy(Y16data,srcData,length);
            break;
        case YUV_Y16:
            memcpy(Y16data,srcData,length);
            memcpy(YUVdata,srcData+length,length);
            break;
        case Y16_Param:
            memcpy(Y16data,srcData,length);
            memcpy(ParamLinedata,srcData+length,video_w*2);
            break;
        case YUV_Param:
            memcpy(YUVdata,srcData,length);
            memcpy(ParamLinedata,srcData+length,video_w*2);
            break;
        case YUV_Y16_Param:
            memcpy(Y16data,srcData,length);
            memcpy(YUVdata,srcData+length,length);
            memcpy(ParamLinedata,srcData+length*2,video_w*2);
            break;
        default:
            break;
    }
}

void v4l2::InitDataMemory(int video_w,int video_h)
{
    ParamLinedata =(unsigned char*)malloc(video_w*2*sizeof(char));
    rgb24 = (unsigned char*)malloc(video_w*video_h*3*sizeof(char));
    YUVdata =(short*)malloc(video_w*video_h*sizeof(short));
    Y16data = (short*)malloc(video_w*video_h*sizeof(short));
}

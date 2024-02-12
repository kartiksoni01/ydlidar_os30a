#include "FrameGrabber.h"
#include "eSPDI_def.h"
#include <string.h>

FrameGrabber::FrameGrabber(
	size_t queueSize,
	FrameGrabberCallbackFn callbackFn, void* callbackParam) :
	m_callbackFn(callbackFn), 
	m_callbackParam(callbackParam)	
{
    m_threadStart = true;
    m_checkFrameReadyThread = new std::thread(FrameGrabber::CheckFrameReadyThreadFn, this);
}

FrameGrabber::~FrameGrabber()
{
    Close();
}

void FrameGrabber::UpdateFrameData(int index,int serialNumber, unsigned char*buf, size_t size)
{
    std::lock_guard<std::mutex> lock( FRAME_POOL_INDEX_COLOR == index ? m_mutex_color : m_mutex_depth );

    FramePool& fp = ( FRAME_POOL_INDEX_COLOR == index ? m_color : m_depth );

    if ( fp.data.size() < size ) {
        fp.data.resize( size );
    }

    memcpy( fp.data.data(), buf, size );

    fp.sn = serialNumber;
}

void FrameGrabber::SetFrameFormat(int index, int width, int height, int bytesPerPixel)
{
    FramePool& fp = ( FRAME_POOL_INDEX_COLOR == index ? m_color : m_depth );

	fp.m_width = width;
	fp.m_height = height;
}

void FrameGrabber::Close()
{
    if(!m_threadStart) return;

    m_threadStart = false;

    m_checkFrameReadyThread->join();
}

void FrameGrabber::CheckFrameReadyThreadFn( void* pvoid )
{
    FrameGrabber* pThis = ( FrameGrabber* )pvoid;

    BOOL synchronized = true;
    int last_sn = -1;
    FramePool color;
    FramePool depth;

    while (pThis->m_threadStart)
    {
        {
            std::lock_guard<std::mutex> lock(pThis->m_mutex_depth);

            if ( pThis->m_depth.sn == last_sn /*|| pThis->m_color.data.empty()*/ )
            {
                synchronized = false;
            }
            else
            {
                synchronized = true;

                std::lock_guard<std::mutex> lock2(pThis->m_mutex_color);

                last_sn = pThis->m_depth.sn;
                color = pThis->m_color;
                depth = pThis->m_depth;
            }
        }
        if ( synchronized )
        {
            pThis->m_callbackFn( depth.data, depth.m_width, depth.m_height,
                                    color.data, color.m_width, color.m_height,
                                    last_sn, pThis->m_callbackParam);
        }
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return;
}

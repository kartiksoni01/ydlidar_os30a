#pragma once
#include <vector>
#include <thread>
#include <mutex>

typedef void (*FrameGrabberCallbackFn)(	std::vector<unsigned char>& bufDepth, int widthDepth, int heightDepth,
										std::vector<unsigned char>& bufColor, int widthColor, int heightColor,
										int serialNumber, void* pParam);

class FrameGrabber
{
	public:
  //      const static int FRAME_POOL_MAX_SIZE = 2;
		const static int FRAME_POOL_INDEX_COLOR = 0;
		const static int FRAME_POOL_INDEX_DEPTH = 1;

		FrameGrabber(size_t queueSize,FrameGrabberCallbackFn callbackFn, void* callbackParam);
	    virtual ~FrameGrabber();
	    
	   // void UpdateColorData(int serialNumber, unsigned char* colorBuf, size_t colorSize);// rgb
	   // void UpdateDepthData(int serialNumber, unsigned char* depthBuf, size_t depthSize);// id is 0-base
		void UpdateFrameData(int index,int serialNumber, unsigned char* buf, size_t size);
		void SetFrameFormat(int index, int width, int height, int bytesPerPixel);
		//void SetDisableSerialSyn(bool value);
        void Close();
	private:
	    //class Frame
	    //{
	    //public:
	    //    Frame(int serialNumber, unsigned char* buf, size_t size);
	
	    //    int sn;
	    //    std::vector<unsigned char> data;
	    //};
	
	    class FramePool
	    {
	    public:
			FramePool() : m_width(0), m_height(0), sn(-1) {}
			//FramePool(size_t poolSize);
	        //void UpdateData(int serialNumber, unsigned char* dataBuf, size_t dataSize);
	
			int m_width;
			int m_height;
            int sn;
	        std::vector<unsigned char> data;
			//int m_bytesPerPixel;
			//size_t m_dataSize;
	  //      std::queue<Frame> m_frames;
	  //      size_t maxSize;
	    };

		FrameGrabberCallbackFn m_callbackFn;
	    void* m_callbackParam;
	    volatile bool m_threadStart;
        std::thread *m_checkFrameReadyThread;
	    std::mutex m_mutex_color;//[FRAME_POOL_MAX_SIZE];
        std::mutex m_mutex_depth;
	    //std::vector<FramePool> m_framePools;
        FramePool m_color;
        FramePool m_depth;
        int m_last_sn;
		//bool disableSerialSyn = false; 

        static void CheckFrameReadyThreadFn( void* pvoid );
	    //bool FindAndDropOutOfDateItem(int targetSn, FramePool& dataPool) const;
	    //void DropItem(int index,int targetSn);
		//bool GetFrameData(int index, int targetSn, std::vector<unsigned char>& outBuf);
};

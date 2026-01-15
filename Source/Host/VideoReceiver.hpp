#ifndef HOST_VIDEO_RECEIVER_HPP_
#define HOST_VIDEO_RECEIVER_HPP_

#include <atomic>
#include <thread>

#include "FrameBuffer.hpp"

// Asynchronous video receiver using FFMpeg

class VideoReceiver
{
private:
    AVFormatContext* FormatContext;
    AVCodecContext* CodecContext;
    AVStream* VideoStream;
    int VideoStreamIndex;

    FrameBuffer* Buffer;

    AVPacket* Packet;
    AVFrame* Frame;

    std::atomic<bool> bNetLoop;
    std::thread NetThread;

    // Init FFMpeg data objects
    int Init(const char* Url);

    void DecodeLoop();
    
public:
    /**
     * @brief Creates asynchronous FFMpeg video receiver.
     * @param Url Url for network connection to video server.
     * @param BufferPtr Pointer to frame buffer object to put frame objects in.
	 */
    VideoReceiver(const char* Url, FrameBuffer* BufferPtr);
    
    int GetVideoWidth();
    
    int GetVideoHeight();

    /**
     * @brief Spawns new thread to continously receive and decode frames from network.
	 */
    void StartReceiveLoop();

    ~VideoReceiver();
};

#endif // HOST_VIDEO_RECEIVER_HPP_

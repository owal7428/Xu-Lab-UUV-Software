#ifndef HOST_VIDEO_RECEIVER_HPP_
#define HOST_VIDEO_RECEIVER_HPP_

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

// This explicitly assumes YUV420P pixel format

struct VideoFrameData
{
    int FrameWidth;
    int FrameHeight;
    uint8_t* YData;
    int YLinesize;
    uint8_t* UData;
    int ULinesize;
    uint8_t* VData;
    int VLinesize;
};

class VideoReceiver
{
private:
    AVFormatContext* FormatContext;
    AVCodecContext* CodecContext;
    AVStream* VideoStream;
    int VideoStreamIndex;

    AVPacket* Packet;
    AVFrame* Frame;

public:
    VideoReceiver()
    {
        FormatContext = nullptr;
        CodecContext = nullptr;
        VideoStream = nullptr;
        VideoStreamIndex = 0;

        Packet = nullptr;
        Frame = nullptr;
    }

    /**
	 * @brief Initializes FFMpeg video receiver.
	 * @param Url Url for network connection to video server.
	 */
    int Init(const char* Url);

    int GetVideoWidth();

    int GetVideoHeight();

    /**
	 * @brief Reads next single packet over network connection.
	 */
    int ReadPacket();

    /**
	 * @brief Clears packet to default values.
	 */
    void ClearPacket();

    /**
	 * @brief Determines if the current packet contains video.
     * @returns True if packet contains a video frame, false otherwise.
	 */
    bool IsPacketVideo();

    /**
	 * @brief Sends current packet to the decoder to be decoded.
	 */
    void EnqueueDecode();

    /**
	 * @brief Gets a raw video frame from the decoder.
     * @returns Error code (0 if successful).
	 */
    int GetFrameFromDecoder();

    VideoFrameData GetVideoFrameData();

    ~VideoReceiver();
};

#endif // HOST_VIDEO_RECEIVER_HPP_

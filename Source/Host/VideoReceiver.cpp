#include "VideoReceiver.hpp"

#include <cstdio>

int VideoReceiver::Init(const char *Url)
{
    avformat_network_init();

    this->FormatContext = nullptr;

    // Autodetects global stream format
    if (avformat_open_input(&this->FormatContext, Url, nullptr, nullptr) < 0) 
    {
        fprintf(stderr, "Failed to open stream\n");
        return -1;
    }

    // Reads packets to infer stream-specific format info
    if (avformat_find_stream_info(this->FormatContext, nullptr) < 0) 
    {
        fprintf(stderr, "No stream info\n");
        return -1;
    }

    this->VideoStreamIndex = -1;

    // Search streams in format context for the specific video stream
    for (int i = 0; i < this->FormatContext->nb_streams; i++) 
    {
        AVStream* TempStream = this->FormatContext->streams[i];

        if (TempStream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) 
        {
            this->VideoStream = TempStream;
            this->VideoStreamIndex = i;
            break;
        }
    }

    if (this->VideoStreamIndex < 0)
    {
        fprintf(stderr, "No video stream\n");
        return -1;
    }

    // Determine codec and parameters

    const AVCodec* Codec = avcodec_find_decoder(this->VideoStream->codecpar->codec_id);
    
    this->CodecContext = avcodec_alloc_context3(Codec);
    
    avcodec_parameters_to_context(this->CodecContext, this->VideoStream->codecpar);
    avcodec_open2(this->CodecContext, Codec, nullptr);

    this->Packet = av_packet_alloc();
    this->Frame = av_frame_alloc();

    return 0;
}

int VideoReceiver::GetVideoWidth() 
    {
        if (this->CodecContext != nullptr)
            return this->CodecContext->width;
        else
            return 0;
    }

int VideoReceiver::GetVideoHeight() 
{
    if (this->CodecContext != nullptr)
        return this->CodecContext->height;
    else
        return 0;
}

int VideoReceiver::ReadPacket()
{
    return av_read_frame(this->FormatContext, this->Packet);
}

void VideoReceiver::ClearPacket()
{
    av_packet_unref(this->Packet);
}

bool VideoReceiver::IsPacketVideo()
{
    if (this->Packet != nullptr)
        return this->Packet->stream_index == this->VideoStreamIndex;
    else
        return false;
}

void VideoReceiver::EnqueueDecode()
{
    avcodec_send_packet(this->CodecContext, this->Packet);
}

int VideoReceiver::GetFrameFromDecoder()
{
    return avcodec_receive_frame(this->CodecContext, this->Frame);
}

VideoFrameData VideoReceiver::GetVideoFrameData()
{
    VideoFrameData Data {};

    Data.FrameWidth = this->Frame->width;
    Data.FrameHeight = this->Frame->height;
    Data.YData = this->Frame->data[0];
    Data.YLinesize = this->Frame->linesize[0];
    Data.UData = this->Frame->data[1];
    Data.ULinesize = this->Frame->linesize[1];
    Data.VData = this->Frame->data[2];
    Data.VLinesize = this->Frame->linesize[2];

    return Data;
}

VideoReceiver::~VideoReceiver()
{
    av_frame_free(&this->Frame);
    av_packet_free(&this->Packet);
    avcodec_free_context(&CodecContext);
    avformat_close_input(&FormatContext);
}

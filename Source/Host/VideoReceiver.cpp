#include "VideoReceiver.hpp"

#include <cstdio>

VideoReceiver::VideoReceiver(const char *Url, FrameBuffer *BufferPtr)
{
    this->FormatContext = nullptr;
    this->CodecContext = nullptr;
    this->VideoStream = nullptr;
    this->VideoStreamIndex = 0;
    
    this->Buffer = BufferPtr;

    this->Packet = av_packet_alloc();
    this->Frame = av_frame_alloc();

    // Init FFMpeg stuff, ignore errors for now
    this->Init(Url);
}

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

void VideoReceiver::StartReceiveLoop()
{
    this->FormatContext->interrupt_callback.opaque = this;

    // Set callback in case of stall to exit thread
    this->FormatContext->interrupt_callback.callback = [](void* opaque) -> int
    {
        auto* Self = static_cast<VideoReceiver*>(opaque);
        return !(Self->bNetLoop); // Exit (return 1) if loop should end
    };
    
    this->bNetLoop = true;

    // Start thread
    this->NetThread = std::thread([this] { this->DecodeLoop(); });
}

void VideoReceiver::DecodeLoop()
{
    while(this->bNetLoop)
    {
        // Get packet from the network
        if (av_read_frame(this->FormatContext, this->Packet) < 0)
            continue;
        
        // Check packet contains video info
        if (this->Packet->stream_index != this->VideoStreamIndex)
        {
            av_packet_unref(this->Packet);
            continue;
        }

        // Enqueue packet for decoding
        if (avcodec_send_packet(this->CodecContext, this->Packet) != 0)
        {
            av_packet_unref(this->Packet);
            continue;
        }
        
        // Packet data is no longer needed and can be reset for next receive
        av_packet_unref(this->Packet);

        while (avcodec_receive_frame(this->CodecContext, this->Frame) == 0)
        {
            Buffer->Push(this->Frame);
            av_frame_unref(this->Frame);
        }
    }
}

VideoReceiver::~VideoReceiver()
{
    this->bNetLoop = false;

    // Wait for network thread to finish
    if (this->NetThread.joinable())
        this->NetThread.join();

    av_packet_free(&this->Packet);
    av_frame_free(&this->Frame);
    avcodec_free_context(&this->CodecContext);
    avformat_close_input(&this->FormatContext);
}

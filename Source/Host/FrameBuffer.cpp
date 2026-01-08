#include "FrameBuffer.hpp"

FrameBuffer::FrameBuffer(size_t Size)
{
    // Buffer cannot be smaller than 2
    this->BufferSize = (Size >= 2) ? Size : 2;

    this->Buffer = std::vector<AVFrame*>(this->BufferSize);

    for (size_t i = 0; i < this->BufferSize; i++)
        this->Buffer[i] = av_frame_alloc();

    this->ReadIndex = 0;
    this->WriteIndex = 0;
}

// Network thread

int FrameBuffer::Push(AVFrame* Frame)
{
    if (!Frame) 
        return -1;

    size_t TempRead = this->ReadIndex.load(std::memory_order_acquire);
    size_t TempWrite = this->WriteIndex.load(std::memory_order_relaxed);
    
    size_t NextWrite = (TempWrite + 1) % this->BufferSize;

    // Check if next write index is read index. If so, increment read index to next oldest.
    if (NextWrite == TempRead)
        this->ReadIndex.store((TempRead + 1) % this->BufferSize, std::memory_order_release);

    av_frame_unref(this->Buffer[TempWrite]); // Previous data must be cleared so ref count can decrement
    int Status = av_frame_ref(this->Buffer[TempWrite], Frame);

    this->WriteIndex.store(NextWrite, std::memory_order_release);

    return Status;
}

// Main thread

int FrameBuffer::PopFrame(AVFrame *&RenderFrame)
{
    size_t TempWrite = this->WriteIndex.load(std::memory_order_acquire);
    size_t TempRead = this->ReadIndex.load(std::memory_order_relaxed);

    // Check if we've caught up to the most recent data
    if (TempRead == TempWrite)
        return -1;
    
    int Status = av_frame_ref(RenderFrame, this->Buffer[TempRead]);

    this->ReadIndex.store((TempRead + 1) % this->BufferSize, std::memory_order_release);

    return Status;
}

size_t FrameBuffer::GetOccupancy()
{
    size_t TempWrite = this->WriteIndex.load(std::memory_order_acquire);
    size_t TempRead = this->ReadIndex.load(std::memory_order_relaxed);

    return (TempWrite - TempRead + this->BufferSize) % this->BufferSize;
}

FrameBuffer::~FrameBuffer()
{
    for (AVFrame* Frame : Buffer)
    {
        if (Frame)
            av_frame_free(&Frame);
    }
}

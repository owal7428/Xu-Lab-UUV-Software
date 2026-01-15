#ifndef HOST_FRAME_BUFFER_HPP_
#define HOST_FRAME_BUFFER_HPP_

#include <vector>
#include <atomic>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

// This is a thread-safe SPSC ring buffer

class FrameBuffer
{
private:
    size_t BufferSize;
    std::vector<AVFrame*> Buffer;
    std::atomic<size_t> ReadIndex;
    std::atomic<size_t> WriteIndex;

public:
    /**
	 * @brief Initializes decoded frame buffer.
	 * @param Size Number of slots in the buffer (must be 2 or greater).
	 */
    FrameBuffer(size_t Size);

    /**
	 * @brief Pushes a frame into the buffer.
	 * @param Frame Frame pointer to be pushed into the buffer.
     * @returns Error status
     * @note Buffer will continuously override old frames.
	 */
    int Push(AVFrame* Frame);

    /**
	 * @brief Pops a frame from the buffer.
	 * @param RenderFrame Frame pointer reference to pop the frame into.
     * @returns Error status
	 */
    int PopFrame(AVFrame*& RenderFrame);

    /**
	 * @brief Gets the number of active frames in the buffer.
     * @returns The number of active frames in the buffer as a size_t.
	 */
    size_t GetOccupancy();

    ~FrameBuffer();
};

#endif // HOST_FRAME_BUFFER_HPP_

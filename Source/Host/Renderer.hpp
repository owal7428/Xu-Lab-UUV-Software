#ifndef HOST_RENDERER_HPP_
#define HOST_RENDERER_HPP_

#include <memory>

#include <glad/gl.h>

#include "FrameBuffer.hpp"
#include "Shader.hpp"

class Renderer
{
private:
    GLuint VAO;
    GLuint VBO;
    GLuint EBO;

    // Textures to hold frame Y, U, and V data
    GLuint TextureY;
    GLuint TextureU;
    GLuint TextureV;

    std::unique_ptr<Shader> ShaderProgram;

    FrameBuffer* Buffer;
    AVFrame* Frame;

    size_t BufferingCutoff;
    bool bIsBuffering;

    int CheckBufferingStatus();

    void UpdateFullscreenQuadTexture();

    void Draw();

public:
    /**
     * @brief Creates OpenGL renderer.
     * @param Width Horizontal resolution of the video stream to be rendered.
     * @param Height Vertical resolution of the video stream to be rendered.
     * @param Cutoff Number of frames to buffer before rendering.
     * @param BufferPtr Pointer to frame buffer object from which frames are received.
     * @param ShaderName Name of shader program the renderer will use.
	 */
    Renderer(int Width, int Height, size_t Cutoff, FrameBuffer* BufferPtr, const char* ShaderName);

    /**
     * @brief Updates OpenGL viewport size.
     * @param Width Width of the window to which the renderer is rendering.
     * @param Height Height of the window to which the renderer is rendering.
     * @note Viewport size is separate from video stream resolution.
	 */
    void UpdateViewport(int Width, int Height);

    /**
     * @brief Renders video frame to the window.
     * @param CurrentTime Current time in seconds.
     * @param NextRenderTime Reference to object containing next time in seconds a frame should be rendered.
     * @note The NextRenderTime object is overwritten in this function.
	 */
    int Render(double CurrentTime, double &NextRenderTime);

    ~Renderer();
};

#endif // HOST_RENDERER_HPP_

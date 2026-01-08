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

    bool bIsBuffering;

    int CheckBufferingStatus();

    void UpdateFullscreenQuadTexture();

    void Draw();

public:
    Renderer(int Width, int Height, FrameBuffer* BufferPtr, const char* ShaderName);

    void UpdateViewport(int Width, int Height);

    int Render();

    ~Renderer();
};

#endif // HOST_RENDERER_HPP_

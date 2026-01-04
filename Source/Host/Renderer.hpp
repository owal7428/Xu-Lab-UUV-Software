#ifndef HOST_RENDERER_HPP_
#define HOST_RENDERER_HPP_

#include <memory>

#include <glad/gl.h>
#include <SDL3/SDL.h>

#include "Shader.hpp"
#include "VideoReceiver.hpp"

// Render assumes all video frames are in YUV420P pixel format

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

    VideoReceiver* Receiver;
    std::unique_ptr<Shader> ShaderProgram;

public:
    Renderer(int Width, int Height, VideoReceiver* ReceiverPtr, const char* ShaderName);

    int Render(SDL_Window* Window);
};

#endif // HOST_RENDERER_HPP_

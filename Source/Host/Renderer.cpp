#include "Renderer.hpp"

Renderer::Renderer(int Width, int Height, FrameBuffer *BufferPtr, const char *ShaderName)
{
    this->Buffer = BufferPtr;

    this->Frame = av_frame_alloc();

    this->ShaderProgram = std::make_unique<Shader>(ShaderName);
    this->ShaderProgram->Bind();

    // Set YUV to textures 0, 1, and 2 respectively

    glUniform1i(glGetUniformLocation(this->ShaderProgram->GetProgram(),"texY"), 0);
    glUniform1i(glGetUniformLocation(this->ShaderProgram->GetProgram(),"texU"), 1);
    glUniform1i(glGetUniformLocation(this->ShaderProgram->GetProgram(),"texV"), 2);

    // Fullscreen quad (flipped vertically)
    float Vertices[] = 
    {
        -1, -1,  0, 1,
         1, -1,  1, 1,
         1,  1,  1, 0,
        -1,  1,  0, 0
    };

    unsigned Indexes[] = {0,1,2, 2,3,0};

    // Setup full-screen quad buffers

    glGenVertexArrays(1, &this->VAO);
    glGenBuffers(1, &this->VBO);
    glGenBuffers(1, &this->EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indexes), Indexes, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(1);

    // Setup YUV textures

    glGenTextures(1, &this->TextureY);
    glGenTextures(1, &this->TextureU);
    glGenTextures(1, &this->TextureV);

    auto InitTexture = [](GLuint Texture, int Width, int Height) 
    {
        glBindTexture(GL_TEXTURE_2D, Texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, Width, Height, 0,GL_RED, GL_UNSIGNED_BYTE, nullptr);
    };

    InitTexture(this->TextureY, Width, Height);
    InitTexture(this->TextureU, Width/2, Height/2);
    InitTexture(this->TextureV, Width/2, Height/2);

    this->bIsBuffering = true;
}

void Renderer::UpdateViewport(int Width, int Height)
{
    glViewport(0, 0, Width, Height);
}

int Renderer::Render()
{
    // NOTE: Render assumes all video frames are in YUV420P pixel format

    int BufferingStatus = this->CheckBufferingStatus();

    if (BufferingStatus < 0)
        return BufferingStatus;

    if (this->Buffer->PopFrame(this->Frame) == 0)
    {
        this->UpdateFullscreenQuadTexture();
        this->Draw();
    }

    av_frame_unref(this->Frame);
    
    return 0;
}

int Renderer::CheckBufferingStatus()
{
    size_t BufferOccupancy = this->Buffer->GetOccupancy();

    if (this->bIsBuffering)
    {
        // Check if we're done buffering
        if (BufferOccupancy < 14)
            return -1;

        this->bIsBuffering = false;
    }

    // Check for underflow
    if (BufferOccupancy == 0)
    {
        this->bIsBuffering = true;
        return -2;
    }

    return 0;
}

void Renderer::UpdateFullscreenQuadTexture()
{
    // Ensure 1-byte alignment
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    // Bind YUV textures
    
    glPixelStorei(GL_UNPACK_ROW_LENGTH, this->Frame->linesize[0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, this->TextureY);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,0, this->Frame->width, this->Frame->height, GL_RED, GL_UNSIGNED_BYTE, this->Frame->data[0]);
    
    glPixelStorei(GL_UNPACK_ROW_LENGTH, this->Frame->linesize[1]);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, this->TextureU);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,0, this->Frame->width / 2, this->Frame->height / 2, GL_RED, GL_UNSIGNED_BYTE, this->Frame->data[1]);
    
    glPixelStorei(GL_UNPACK_ROW_LENGTH, this->Frame->linesize[2]);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, this->TextureV);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,0, this->Frame->width / 2, this->Frame->height / 2, GL_RED, GL_UNSIGNED_BYTE, this->Frame->data[2]);
    
    // Reset row length
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

void Renderer::Draw()
{
    // Render fullscreen quad to the screen

    glClear(GL_COLOR_BUFFER_BIT);

    glBindVertexArray(this->VAO);
    
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

Renderer::~Renderer()
{
    av_frame_free(&this->Frame);
}

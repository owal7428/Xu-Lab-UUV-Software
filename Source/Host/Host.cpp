#include <cstdio>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "Shader.hpp"
#include "VideoReceiver.hpp"

void Cleanup(SDL_Window* Window)
{
    SDL_DestroyWindow(Window);
    SDL_Quit();
}

int main(int argc, char* argv[]) 
{
    const char* URL = "udp://127.0.0.1:1234?fifo_size=5000000&overrun_nonfatal=1";

    VideoReceiver Receiver = VideoReceiver();
    
    if (Receiver.Init(URL) != 0)
        return -1;

    int Width = Receiver.GetVideoWidth();
    int Height = Receiver.GetVideoHeight();

    // Init SDL
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
     {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "SDL3 Failed to Initialize.", nullptr);
        return -1;
    }
    
    SDL_Window* Window = SDL_CreateWindow("Host - Video Playback", Width, Height, SDL_WINDOW_OPENGL);

    if (!Window) 
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Window failed to initialize.", nullptr);
        Cleanup(Window);
        return -1;
    }

    SDL_GLContext GLContext = SDL_GL_CreateContext(Window);

    if (gladLoadGL((GLADloadfunc) SDL_GL_GetProcAddress) == 0) 
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "OpenGL failed to load.", nullptr);
        Cleanup(Window);
        return -1;
    }

    // Assuming executable is running from build directory
    Shader YUVToRGBShader = Shader("../Shaders/YUVToRGB");
    YUVToRGBShader.Bind();

    // Fullscreen quad (flipped vertically)
    float verts[] = {
        -1, -1,  0, 1,
         1, -1,  1, 1,
         1,  1,  1, 0,
        -1,  1,  0, 0
    };
    unsigned idx[] = {0,1,2, 2,3,0};

    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(1);

    // YUV textures
    GLuint texY, texU, texV;
    glGenTextures(1, &texY);
    glGenTextures(1, &texU);
    glGenTextures(1, &texV);

    auto init_tex = [](GLuint t, int w, int h) 
    {
        glBindTexture(GL_TEXTURE_2D, t);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, w, h, 0,GL_RED, GL_UNSIGNED_BYTE, nullptr);
    };

    init_tex(texY, Width, Height);
    init_tex(texU, Width/2, Height/2);
    init_tex(texV, Width/2, Height/2);

    glUniform1i(glGetUniformLocation(YUVToRGBShader.GetProgram(),"texY"), 0);
    glUniform1i(glGetUniformLocation(YUVToRGBShader.GetProgram(),"texU"), 1);
    glUniform1i(glGetUniformLocation(YUVToRGBShader.GetProgram(),"texV"), 2);

    printf("Press keys or controller buttons. ESC or window close to quit.\n\n");

    SDL_Gamepad* Gamepad = nullptr;

    bool IsRunning = true;

    // Begin event loop
    while (IsRunning) 
    {
        // Get input

        SDL_Event Event;
        while (SDL_PollEvent(&Event)) 
        {
            switch (Event.type) 
            {
                case SDL_EVENT_QUIT:
                    IsRunning = false;
                    break;

                case SDL_EVENT_KEY_DOWN:
                    printf("Key down: %s\n", SDL_GetKeyName(Event.key.key));
                    if (Event.key.key == SDLK_ESCAPE)
                        IsRunning = false;
                    break;

                case SDL_EVENT_KEY_UP:
                    printf("Key up: %s\n", SDL_GetKeyName(Event.key.key));
                    break;

                case SDL_EVENT_GAMEPAD_ADDED:
                    if (Gamepad == nullptr) 
                    {
                        printf("Gamepad connected: id=%d\n", Event.gdevice.which);
                        Gamepad = SDL_OpenGamepad(Event.gdevice.which);
                        
                        if (!Gamepad) 
                            fprintf(stderr, "Failed to open gamepad ID %u: %s", (unsigned int) Event.gdevice.which, SDL_GetError());
                    }
                    break;

                case SDL_EVENT_GAMEPAD_REMOVED:
                    if (Gamepad && (SDL_GetGamepadID(Gamepad) == Event.gdevice.which)) 
                    {
                        printf("Gamepad disconnected: id=%d\n", Event.gdevice.which);
                        SDL_CloseGamepad(Gamepad);
                        Gamepad = nullptr;
                    }
                    break;

                case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
                    printf("Gamepad button down: %d\n", Event.gbutton.button);
                    break;

                case SDL_EVENT_GAMEPAD_BUTTON_UP:
                    printf("Gamepad button up:   %d\n", Event.gbutton.button);
                    break;

                case SDL_EVENT_GAMEPAD_AXIS_MOTION:
                    switch(Event.gaxis.axis)
                    {
                        case SDL_GAMEPAD_AXIS_LEFTX:
                            printf("Gamepad left stick horizontal: %f\n", static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                        
                        case SDL_GAMEPAD_AXIS_LEFTY:
                            printf("Gamepad left stick vertical: %f\n", static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                        
                        case SDL_GAMEPAD_AXIS_RIGHTX:
                            printf("Gamepad right stick horizontal: %f\n", static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                        
                        case SDL_GAMEPAD_AXIS_RIGHTY:
                            printf("Gamepad right stick vertical: %f\n", static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                        
                        case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                            printf("Gamepad left trigger: %f\n", static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                        
                        case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                            printf("Gamepad right trigger: %f\n", static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                        
                        default:
                            printf("Gamepad axis %d: %f\n", Event.gaxis.axis, static_cast<float>(Event.gaxis.value) / 32767.0f);
                            break;
                    }
                    break;

                default:
                    break;
            }
        }

        // Render video

        // Read frame off the network if one exists
        if (Receiver.ReadPacket() < 0) continue;

        if (!Receiver.IsPacketVideo())
        {
            Receiver.ClearPacket();
            continue;
        }

        Receiver.EnqueueDecode();

        while (Receiver.GetFrameFromDecoder() == 0) 
        {
            VideoFrameData Data = Receiver.GetVideoFrameData();

            // Ensure 1-byte alignment
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            glPixelStorei(GL_UNPACK_ROW_LENGTH, Data.YLinesize);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, texY);
            glTexSubImage2D(GL_TEXTURE_2D,0,0,0, Data.FrameWidth, Data.FrameHeight, GL_RED, GL_UNSIGNED_BYTE, Data.YData);

            glPixelStorei(GL_UNPACK_ROW_LENGTH, Data.ULinesize);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, texU);
            glTexSubImage2D(GL_TEXTURE_2D,0,0,0, Data.FrameWidth / 2, Data.FrameHeight / 2, GL_RED, GL_UNSIGNED_BYTE, Data.UData);

            glPixelStorei(GL_UNPACK_ROW_LENGTH, Data.VLinesize);
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, texV);
            glTexSubImage2D(GL_TEXTURE_2D,0,0,0, Data.FrameWidth / 2, Data.FrameHeight / 2, GL_RED, GL_UNSIGNED_BYTE, Data.VData);

            glClear(GL_COLOR_BUFFER_BIT);
            glBindVertexArray(VAO);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
            SDL_GL_SwapWindow(Window);
            SDL_Delay(16);
        }
    }

    Cleanup(Window);
    printf("Program exit.\n");
    return 0;
}

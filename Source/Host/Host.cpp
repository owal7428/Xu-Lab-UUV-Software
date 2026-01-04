#include <cstdio>

#include <glad/gl.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "Renderer.hpp"
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

    // Initialize SDL

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
     {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "SDL3 Failed to Initialize.", nullptr);
        return -1;
    }

    // Set OpenGL to version 3.3
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    // Setup SDL window
    
    SDL_Window* Window = SDL_CreateWindow("Host - Video Playback", Width, Height, SDL_WINDOW_OPENGL);

    if (!Window) 
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Window failed to initialize.", nullptr);
        Cleanup(Window);
        return -1;
    }

    SDL_SetWindowResizable(Window, true);

    // Setup GL context with GLAD

    SDL_GLContext GLContext = SDL_GL_CreateContext(Window);

    if (gladLoadGL((GLADloadfunc) SDL_GL_GetProcAddress) == 0) 
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "OpenGL failed to load.", nullptr);
        Cleanup(Window);
        return -1;
    }

    Renderer FrameRenderer = Renderer(Width, Height, &Receiver, "../Shaders/YUVToRGB");

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
                
                case SDL_EVENT_WINDOW_RESIZED:
                    Width = Event.window.data1;
                    Height = Event.window.data2;
                    FrameRenderer.UpdateViewport(Width, Height);
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
        FrameRenderer.Render(Window);
    }

    Cleanup(Window);
    printf("Program exit.\n");
    return 0;
}

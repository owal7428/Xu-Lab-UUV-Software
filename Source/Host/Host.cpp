#include <cstdio>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

void Cleanup(SDL_Window* Window)
{
    SDL_DestroyWindow(Window);
    SDL_Quit();
}

int main(int argc, char* argv[]) 
{
    // Init SDL
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
     {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "SDL3 Failed to Initialize.", nullptr);
        return 1;
    }
    
    // Open 720p blank window
    SDL_Window* Window = SDL_CreateWindow("My Window", 1280, 720, 0);

    if (!Window) 
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Window failed to initialize.", nullptr);
        Cleanup(Window);
        return 1;
    }

    SDL_Gamepad* Gamepad = nullptr;

    printf("SDL3 initialized.\n");
    printf("Press keys or controller buttons. ESC or window close to quit.\n\n");

    bool IsRunning = true;

    // Begin event loop
    while (IsRunning) 
    {
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
                    printf("Gamepad axis %d value %d\n", Event.gaxis.axis, Event.gaxis.value);
                    break;

                default:
                    break;
            }
        }

        SDL_Delay(10);
    }

    Cleanup(Window);
    printf("Program exit.\n");
    return 0;
}

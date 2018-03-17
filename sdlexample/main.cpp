#include "SDL.h"

const int windowSizeX = 500;
const int windowSizeY = 500;

void drawGrid(SDL_Renderer** renderer)
{
  SDL_SetRenderDrawColor(*renderer, 230, 230, 230, SDL_ALPHA_OPAQUE);

  int gridStep = windowSizeX / 20;
  while(gridStep < windowSizeX)
  {
    SDL_RenderDrawLine(*renderer, gridStep, 0, gridStep, windowSizeY);
    gridStep += windowSizeX / 20;
  }
  gridStep = windowSizeY / 20;
  while(gridStep < windowSizeY)
  {
    SDL_RenderDrawLine(*renderer, 0, gridStep, windowSizeX, gridStep);
    gridStep += windowSizeY / 20;
  }

  SDL_RenderPresent(*renderer);
}

int main(int argc, char* argv[])
{
    if (SDL_Init(SDL_INIT_VIDEO) == 0) {
        SDL_Window* window = NULL;
        SDL_Renderer* renderer = NULL;

        if (SDL_CreateWindowAndRenderer(windowSizeX, windowSizeY, 0, &window, &renderer) == 0) {
            SDL_bool done = SDL_FALSE;

            while (!done) {
                SDL_Event event;

                SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
                SDL_RenderClear(renderer);

                drawGrid(&renderer);

                while (SDL_PollEvent(&event)) {
                    if (event.type == SDL_QUIT) {
                        done = SDL_TRUE;
                    }
                }
            }
        }

        if (renderer) {
            SDL_DestroyRenderer(renderer);
        }
        if (window) {
            SDL_DestroyWindow(window);
        }
    }
    SDL_Quit();
    return 0;
}

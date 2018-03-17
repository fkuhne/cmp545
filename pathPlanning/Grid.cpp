
#include "Grid.h"
#include <iostream>
#include <vector>
#include <string>
#include "Geometric2DCSpace.h"
#include "SBLPlanner.h"
#include <SDL2/SDL_ttf.h>

Grid::Grid(int _sizeX, int _sizeY):
  sizeX(_sizeX), sizeY(_sizeY)
{

  SDL_Init(SDL_INIT_EVERYTHING);

  window = SDL_CreateWindow("Path Planning",
    SDL_WINDOWPOS_UNDEFINED,
    SDL_WINDOWPOS_UNDEFINED,
    sizeX, sizeY,
    SDL_WINDOW_RESIZABLE);

  if(window == NULL)
  {
    cout << "Cannot create window: " << SDL_GetError() << endl;
    return;
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  SDL_SetRenderDrawColor(renderer, bgColor.r, bgColor.g, bgColor.b, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);

  TTF_Init();

  cspace.domain.bmin.set(0, 0);
  cspace.domain.bmax.set(sizeX, sizeY);

  AABB2D temp;
	temp.bmin.set(cellSize * 10, 0);
	temp.bmax.set(cellSize * 11, cellSize * 10);
	cspace.Add(temp);

  temp.bmin.set(cellSize * 5, cellSize * 18);
	temp.bmax.set(cellSize * 20, cellSize * 19);
	cspace.Add(temp);

  temp.bmin.set(cellSize * 20, cellSize * 18);
	temp.bmax.set(cellSize * 21, sizeY);
	cspace.Add(temp);

  temp.bmin.set(cellSize * 33, cellSize * 7);
  temp.bmax.set(cellSize * 34, cellSize * 18);
  cspace.Add(temp);

  temp.bmin.set(cellSize * 34, cellSize * 7);
  temp.bmax.set(cellSize * 42, cellSize * 8);
  cspace.Add(temp);

  temp.bmin.set(cellSize * 34, cellSize * 17);
  temp.bmax.set(cellSize * 43, cellSize * 18);
  cspace.Add(temp);

  temp.bmin.set(cellSize * 43, 0);
  temp.bmax.set(cellSize * 44, sizeY);
  cspace.Add(temp);

  drawBackground();

  running = true;
}

Grid::~Grid()
{
  if(renderer) SDL_DestroyRenderer(renderer);
  if(window) SDL_DestroyWindow(window);
  TTF_Quit();
  SDL_Quit();
}

bool Grid::isRunning()
{
  return running;
}

bool isPointEmpty(const SDL_Point& point)
{
  if((point.x == 0) && (point.y == 0))
    return true;
  return false;
}

void Grid::paintCell(const SDL_Point& cell, const SDL_Color& color)
{
  if(isPointEmpty(cell))
    return;

  int cellX = floor(cell.x / cellSize) * cellSize;
  int cellY = floor(cell.y / cellSize) * cellSize;

  SDL_Rect rect = {cellX + 3, cellY + 3, cellSize - 6, cellSize - 6};
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, SDL_ALPHA_OPAQUE);
  SDL_RenderFillRect(renderer, &rect);
}

void Grid::drawMapLimits()
{
  for(int x = 0; x < sizeX; x += cellSize)
  {
    SDL_Point p{x, 0};
    paintCell(p, obstacleColor);
    p = {x, sizeY - cellSize};
    paintCell(p, obstacleColor);
  }
  for(int y = 1; y < sizeY - cellSize; y += cellSize)
  {
    SDL_Point p{0, y};
    paintCell(p, obstacleColor);
    p = {sizeX - cellSize, y};
    paintCell(p, obstacleColor);
  }
}

void Grid::drawObstacles()
{
  SDL_SetRenderDrawColor(renderer, obstacleColor.r, obstacleColor.g, obstacleColor.b, SDL_ALPHA_OPAQUE);
  for(int i = 0; i < cspace.aabbs.size(); i++)
  {
    SDL_Rect rect{static_cast<int>(cspace.aabbs[i].bmin.x),
      static_cast<int>(cspace.aabbs[i].bmin.y),
      static_cast<int>(cspace.aabbs[i].bmax.x - cspace.aabbs[i].bmin.x),
      static_cast<int>(cspace.aabbs[i].bmax.y - cspace.aabbs[i].bmin.y)};
    SDL_RenderFillRect(renderer, &rect);
  }
}

void Grid::drawText(string& text, SDL_Point& location)
{
  TTF_Font* Sans = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeMono.ttf", CELL_SIZE / 2 + 2); //this opens a font style and sets a size
  SDL_Surface* surfaceMessage = TTF_RenderText_Solid(Sans, text.c_str(), obstacleColor); // as TTF_RenderText_Solid could only be used on SDL_Surface then you have to create the surface first
  SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage); //now you can convert it into a texture

  int w,h;
  SDL_QueryTexture(Message, NULL, NULL, &w, &h);
  SDL_Rect Message_rect{location.x - w / 2, location.y - h / 2, w, h}; //create a rect
  SDL_RenderCopy(renderer, Message, NULL, &Message_rect); //you put the renderer's name first, the Message, the crop size(you can ignore this if you don't want to dabble with cropping), and the rect which is the size and coordinate of your texture
  //SDL_RenderPresent(renderer);

  TTF_CloseFont(Sans);
  SDL_FreeSurface(surfaceMessage);
  SDL_DestroyTexture(Message);
}

void Grid::drawBackground()
{
  SDL_SetRenderDrawColor(renderer, bgColor.r, bgColor.g, bgColor.b, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  SDL_SetRenderDrawColor(renderer, bgColor.r - 8, bgColor.g - 8, bgColor.b - 8, SDL_ALPHA_OPAQUE);

  int gridStep = cellSize;
  while(gridStep < sizeX)
  {
    SDL_RenderDrawLine(renderer, gridStep, 0, gridStep, sizeY);
    gridStep += cellSize;
  }
  gridStep = cellSize;
  while(gridStep < sizeY)
  {
    SDL_RenderDrawLine(renderer, 0, gridStep, sizeX, gridStep);
    gridStep += cellSize;
  }

  //drawMapLimits();
  drawObstacles();
}

void Grid::drawStartEndPoints()
{
  if(!isPointEmpty(startPoint))
  {
    paintCell(startPoint, redColor);
    //string s("Start");
    //drawText(s, startPoint);
  }

  if(!isPointEmpty(endPoint))
  {
    paintCell(endPoint, blueColor);
    //string s("Goal");
    //drawText(s, endPoint);
  }
}

bool Grid::hasEvent()
{
  if(SDL_PollEvent(NULL))
    return true;
  return false;
}

void Grid::handleEvent()
{
  SDL_Event event;
  SDL_PollEvent(&event);
  if(event.type == SDL_QUIT)
  {
    running = false;
    return;
  }
  else if(event.type == SDL_WINDOWEVENT)
  {
    switch(event.window.event)
    {
      case SDL_WINDOWEVENT_RESIZED:
      case SDL_WINDOWEVENT_SIZE_CHANGED:
      case SDL_WINDOWEVENT_MAXIMIZED:
      case SDL_WINDOWEVENT_RESTORED:
        sizeX = static_cast<int>(event.window.data1);
        sizeY = static_cast<int>(event.window.data2);
        drawBackground();
        drawStartEndPoints();
        break;
      default: break;
    }
  }
  /*else if(event.type == SDL_MOUSEMOTION)
  {
    int mouseX = event.motion.x;
    int mouseY = event.motion.y;
    //drawBackground();
    SDL_Color color{0, 180, 0};
    paintCell(mouseX, mouseY, color);
  }*/
  else if(event.type == SDL_MOUSEBUTTONDOWN)
  {
    if(event.button.button == SDL_BUTTON_LEFT)
    {
      drawBackground();
      if(isPointEmpty(startPoint))
      {
        /* The point will be centralized at the cell. */
        startPoint.x = floor(event.button.x / cellSize) * cellSize + cellSize / 2;
        startPoint.y = floor(event.button.y / cellSize) * cellSize + cellSize / 2;
        drawStartEndPoints();
      }
      else if(!isPointEmpty(startPoint) && isPointEmpty(endPoint))
      {
        /* The point will be centralized at the cell. */
        endPoint.x = floor(event.button.x / cellSize) * cellSize + cellSize / 2;
        endPoint.y = floor(event.button.y / cellSize) * cellSize + cellSize / 2;
        drawStartEndPoints();
        readyToComputePath = true;
      }
      else if(!isPointEmpty(startPoint) && !isPointEmpty(endPoint))
      {
        paintCell(startPoint, bgColor);
        paintCell(endPoint, bgColor);

        startPoint.x = event.button.x;
        startPoint.y = event.button.y;
        endPoint.x = endPoint.y = 0;

        drawStartEndPoints();
      }
    }
  }
  else if(event.type == SDL_KEYDOWN)
  {
    switch(event.key.keysym.sym)
    {
      case 'c':
        drawBackground();
        paintCell(startPoint, bgColor);
        paintCell(endPoint, bgColor);
        startPoint.x = startPoint.y = 0;
        endPoint.x = endPoint.y = 0;
        break;
      case 'q':
        running = false;
        break;
      case 'p':
      {
        if(isPointEmpty(startPoint) || isPointEmpty(endPoint))
        {
          cout << "Start and Goal undefined. Define them first by clicking on the map." << endl;
          return;
        }

        vector<SDL_Point> path;
        //path.reserve(3000+1);
        bool ret = SBLPlanner(&cspace, startPoint, endPoint, path, renderer);

        drawBackground();
        paintCell(startPoint, redColor);
        paintCell(endPoint, blueColor);

        if(ret == true)
        {
          //plot path
          SDL_Point prev;
          for(vector<SDL_Point>::iterator it = path.begin(); it != path.end(); it++)
          {
            if(it->x == 0) break;

            /* Centralize the point in the cell. */
            it->x = floor(it->x / cellSize) * cellSize + cellSize / 2;
            it->y = floor(it->y / cellSize) * cellSize + cellSize / 2;

            //cout << "Point" << ": (" << it->x << "," << it->y << ")" << endl;

            if(it - path.begin() == 0) prev = startPoint;

            //string s = to_string(it - path.begin());
            //SDL_SetRenderDrawColor(renderer, lineColor.r, lineColor.g, lineColor.b, SDL_ALPHA_OPAQUE);
            //SDL_RenderDrawLine(renderer, prev.x, prev.y, it->x, it->y);
            //drawText(s, *it);
            //SDL_Delay(2);

            //SDL_RenderPresent(renderer);
            //SDL_PollEvent(NULL);
            prev = *it;
          }
          string s("DONE!");
          SDL_Point p{CELL_SIZE * 2, CELL_SIZE};
          drawText(s, p);
        }
        else
        {
          string s("INFEASIBLE!");
          SDL_Point p{CELL_SIZE * 2, CELL_SIZE};
          drawText(s, p);
        }
        break;
      }
      case SDLK_UP:
      {
        SDL_RenderClear(renderer);
        cellSize = (cellSize += 5) > 50 ? 50 : cellSize;
        drawBackground();
        drawStartEndPoints();
        break;
      }
      case SDLK_DOWN:
      {
        SDL_RenderClear(renderer);
        cellSize = (cellSize -= 5) < 5 ? 5 : cellSize;
        drawBackground();
        drawStartEndPoints();
        break;
      }
      default:
        break;
    }
  }

  SDL_RenderPresent(renderer);
}

void drawRectangle(const int x, const int y) // x and y in pixels.
{

}

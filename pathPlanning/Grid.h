#ifndef __GRID_H__
#define __GRID_H__


#include <SDL.h>
#include "myGlobals.h"
#include "Geometric2DCSpace.h"
//#include "MotionPlanning/AnyMotionPlanner.h"
//#include "misc/Miscellany.h"
//#include "MotionPlanning/CSpace.h"
//#include "misc/Vector.h"
//#include <misc/Random.h>

using namespace MathGeometric;
using namespace std;

class Grid
{
private:
  SDL_Window *window = NULL;
  SDL_Renderer *renderer = NULL;

  int sizeX, sizeY;
  int cellSize = CELL_SIZE;

  SDL_Point startPoint = {0, 0};
  SDL_Point endPoint = {0, 0};

  bool running = false;
  bool readyToComputePath = false;

  SDL_Color redColor{255, 0, 0};
  SDL_Color greenColor{0, 255, 0};
  SDL_Color blueColor{0, 0, 255};
  SDL_Color bgColor{210, 210, 210};
  SDL_Color obstacleColor{0, 0, 0};
  SDL_Color lineColor{255, 102, 0};

  Geometric2DCSpace cspace;

public:
  Grid(int sizeX, int sizeY);
  ~Grid();

  bool isRunning();
  void paintCell(const SDL_Point& cell, const SDL_Color& color);
  void drawBackground();
  void drawObstacles();
  void drawText(std::string& text, SDL_Point& location);
  void drawMapLimits();
  void drawStartEndPoints();
  bool hasEvent();
  void handleEvent();
  void drawRectangle(const int x, const int y); // x and y in pixels.
};

#endif

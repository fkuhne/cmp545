#ifndef __SBL_PLANNER_H__
#define __SBL_PLANNER_H__

#include "Geometric2DCSpace.h"
#include "MotionPlanning/AnyMotionPlanner.h"
#include <SDL.h>

bool SBLPlanner(Geometric2DCSpace* cspace, const SDL_Point &start, const SDL_Point &goal, std::vector<SDL_Point> &path, SDL_Renderer *renderer);

#endif

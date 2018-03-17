

#include "SBLPlanner.h"
#include "misc/Miscellany.h"
#include "Timer.h"
#include "myGlobals.h"

using namespace std;

const int maxPlanIters = 6000;
const double maxTime = 5000;

bool SBLPlanner(Geometric2DCSpace* cspace, const SDL_Point &start, const SDL_Point &goal, vector<SDL_Point> &path, SDL_Renderer *renderer)
{
  cout << "Planning for start(" << start.x << ","
       << start.y << ") and (" << goal.x << ","
       << goal.y << ")" << endl;

  Config startConfig(2);
  startConfig[0] = start.x;
  startConfig[1] = start.y;

  Config goalConfig(2);
  goalConfig[0] = goal.x;
  goalConfig[1] = goal.y;

  if(!cspace->IsFeasible(startConfig))
  {
    cout << "Start configuration is infeasible!" << endl;
    return false;
  }
  if(!cspace->IsFeasible(goalConfig))
  {
    cout << "Goal configuration is infeasible!" << endl;
    return false;
  }

  MotionPlannerFactory factory;
  factory.type = MotionPlannerFactory::SBL;
  factory.useGrid = true;
  factory.connectionThreshold = CELL_SIZE * 3;
  factory.gridResolution  = CELL_SIZE;
  factory.perturbationRadius = CELL_SIZE;
  //factory.bidirectional = false;
  MotionPlannerInterface* planner = factory.Create(cspace);
  //SBLPlanner *planner = new SBLPlanner(cspace);
  int mstart = planner->AddMilestone(startConfig);
  int mgoal = planner->AddMilestone(goalConfig);
  //planner->Init(startConfig, goalConfig);

  bool solved = false;
  Timer timer;
  cout << "Iteration ";

  //while(!planner->IsDone())
  //  planner->Extend();

  //path.clear();
  SDL_Point prev;

  for(int iter = 1; iter <= maxPlanIters; iter++)
  {
    cout << iter << " " << flush;
    if(planner->IsConnected(mstart, mgoal))
    {
      cout << " solved!" << endl;
      solved = true;

      if(renderer)
      {
        SDL_SetRenderDrawColor(renderer, 255, 0, 127, SDL_ALPHA_OPAQUE);
        SDL_RenderDrawLine(renderer, start.x, start.y, goal.x, goal.y);
        SDL_RenderPresent(renderer);
      }
      break;
    }

    RoadmapPlanner roadmap(cspace);
    planner->GetRoadmap(roadmap);
    int pathSize = roadmap.roadmap.nodes.size();
    cout << pathSize << " nodes." << endl;
    Config c(2);
    c = roadmap.roadmap.nodes[iter];
    SDL_Point p{static_cast<int>(floor(c[0])), static_cast<int>(floor(c[1]))};
    path.push_back(p);

    prev = path[iter - 1];
    if(renderer)
    {
      SDL_SetRenderDrawColor(renderer, 255, 0, 127, SDL_ALPHA_OPAQUE);
      SDL_RenderDrawLine(renderer, prev.x, prev.y, p.x, p.y);
      SDL_RenderPresent(renderer);
    }

    cout << "(" << p.x << "," << p.y << ")" << endl;

  //  if(solved) break;

    //if(planner->PlanMore() < 0) break;
    planner->PlanMore();

    if(timer.ElapsedTime() > maxTime)
    {
      cout << endl << "Timed out at %g sec." << timer.ElapsedTime() << endl;
      return false;
    }
  }

  if(solved == false)
  {
    cout << endl << "Infeasible path!" << endl;
    return false;
  }

  MilestonePath milestones;

  RoadmapPlanner roadmap(cspace);
  planner->GetRoadmap(roadmap);
  //planner->GetPath(mgoal, mstart, milestones);
  int pathSize = roadmap.roadmap.nodes.size();
  //int pathSize = milestones.NumMilestones();
  cout << pathSize << " nodes." << endl;

  path.clear();
  path.reserve(pathSize);
  for(int i = 0; i < pathSize; i++)
  {
    //if(i==1) continue;
    Config c(2);
    c = roadmap.roadmap.nodes[i];
    //c = milestones.GetMilestone(i);
    SDL_Point p{static_cast<int>(floor(c[0])), static_cast<int>(floor(c[1]))};
    path.push_back(p);
  }

  return true;
}

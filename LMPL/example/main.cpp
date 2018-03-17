/*
This file is part of LMPL.

    LMPL is free software: you can redistribute it and/or modify
    it under the terms of the Lesser GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LMPL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Lesser
    GNU General Public License for more details.

    You should have received a copy of the Lesser GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Geometric2DCSpace.h"
//#include "PlannerTest.h"
#include <misc/Random.h>

#include "Timer.h"
#include "MotionPlanning/AnyMotionPlanner.h"
#include "misc/Miscellany.h"

//using namespace ConstantHelper;

using namespace std;

#define EUCLIDEAN_SPACE 1

void SetupBoxObstacle(Geometric2DCSpace& cspace,double width,double height)
{
	cspace.euclideanSpace = EUCLIDEAN_SPACE;
	cspace.domain.bmin.set(0,0);
	cspace.domain.bmax.set(1,1);
	AABB2D temp;
	temp.bmin.set(0.5-0.5*width,0.5-0.5*height);
	temp.bmax.set(0.5+0.5*width,0.5+0.5*height);
	cspace.Add(temp);
}

void SetupTrianglePassage(Geometric2DCSpace& cspace,double passageWidth,double baseWidth)
{
	cspace.euclideanSpace = EUCLIDEAN_SPACE;
	cspace.domain.bmin.set(0,0);
	cspace.domain.bmax.set(1,1);

	Triangle2D t;
	t.a.set(0.5+baseWidth*0.5,0.0);
	t.b.set(0.5,0.5-passageWidth*0.5);
	t.c.set(0.5-baseWidth*0.5,0.0);
	cspace.Add(t);
	t.a.set(0.5-baseWidth*0.5,1);
	t.b.set(0.5,0.5+passageWidth*0.5);
	t.c.set(0.5+baseWidth*0.5,1);
	cspace.Add(t);
}

void SetupCircularField(Geometric2DCSpace& cspace,int numCircles,double passageWidth)
{
	cspace.euclideanSpace = EUCLIDEAN_SPACE;
	cspace.domain.bmin.set(0,0);
	cspace.domain.bmax.set(1,1);

	Circle2D c;
	double width = (1.0-2.0*passageWidth)/numCircles;
	double start = width*0.5+passageWidth;
	c.radius = passageWidth*width;
	for(int i=0;i<numCircles;i++) {
		if(i%2==0) {
			for(int j=0;j+1<numCircles;j++) {
				c.center.set(width*(double(j)+0.5)+start,width*double(i)+start);
				cspace.Add(c);
			}
		}
		else {
			for(int j=0;j<numCircles;j++) {
				c.center.set(width*double(j)+start,width*double(i)+start);
				cspace.Add(c);
			}
		}
	}
}

void SetupKink(Geometric2DCSpace& cspace,double passageWidth,double width,double kinkLength)
{
	AABB2D temp;
	cspace.euclideanSpace = EUCLIDEAN_SPACE;
	cspace.domain.bmin.set(0,0);
	cspace.domain.bmax.set(1,1);

	//bottom left of kink
	temp.bmin.set(0.5-0.5*width,0);
	temp.bmax.set(0.5-0.5*passageWidth,0.5-0.5*passageWidth+kinkLength*0.5);
	cspace.Add(temp);
	//bottom right of kink
	temp.bmin.set(0.5-0.5*passageWidth,0);
	temp.bmax.set(0.5+0.5*width,0.5-0.5*passageWidth-kinkLength*0.5);
	cspace.Add(temp);
	//top left of kink
	temp.bmin.set(0.5-0.5*width,0.5+0.5*passageWidth+kinkLength*0.5);
	temp.bmax.set(0.5+0.5*passageWidth,1);
	cspace.Add(temp);
	//top right of kink
	temp.bmin.set(0.5+0.5*passageWidth,0.5+0.5*passageWidth-kinkLength*0.5);
	temp.bmax.set(0.5+0.5*width,1);
	cspace.Add(temp);
}


void SetupWindy(Geometric2DCSpace& cspace,int numWinds,double passageWidth,double kinkWidth)
{
	AABB2D temp;
	//TODO: what might the 'border' be? Parameter or constant?
	double border = 0.1;

	cspace.euclideanSpace = EUCLIDEAN_SPACE;
	cspace.domain.bmin.set(0,0);
	cspace.domain.bmax.set(1,1);

	double width = 1.0/double(numWinds+1);
	for(int i=0;i<numWinds;i++) {
		double center = double(i+1)/double(numWinds+1);
		if(i%2 == 1) {
			temp.bmin.set(center-width*kinkWidth,0);
			temp.bmax.set(center+width*kinkWidth,1.0-border-passageWidth);
		}
		else {
			temp.bmin.set(center-width*kinkWidth,border+passageWidth);
			temp.bmax.set(center+width*kinkWidth,1.0);
		}
		cspace.Add(temp);
	}
}

void SetupPassage(Geometric2DCSpace& cspace,double passageWidth,double width)
{
	AABB2D temp;
	cspace.euclideanSpace = EUCLIDEAN_SPACE;
	cspace.domain.bmin.set(0,0);
	cspace.domain.bmax.set(1,1);

	temp.bmin.set(0.5-0.5*width,0);
	temp.bmax.set(0.5+0.5*width,0.5-passageWidth);
	cspace.Add(temp);
	temp.bmin.set(0.5-0.5*width,0.5);
	temp.bmax.set(0.5+0.5*width,1);
	cspace.Add(temp);
}

void PlannerTest(MotionPlannerFactory& factory,CSpace* cspace,
					  const Config& start,const Config& goal)
{
  if(!cspace->IsFeasible(start)) {
    printf("Warning: start configuration is infeasible\n");
  }
  if(!cspace->IsFeasible(goal)) {
    printf("Warning: goal configuration is infeasible\n");
  }
	double meanTime, minTime, _maxTime;
	bool solved(false);

	Timer timer;

	MotionPlannerInterface* planner = factory.Create(cspace);
	int mstart=planner->AddMilestone(start);
	int mgoal=planner->AddMilestone(goal);

	int maxPlanIters = 10000;
	double maxTime = ConstantHelper::Inf;
	int iter=0;
	for(iter=0;iter<maxPlanIters;iter++)
	{
		if(planner->IsConnected(mstart,mgoal)) {
	  	printf("Solved on iteration %d.\n",iter);
			solved=true;
			break;
		}
		planner->PlanMore();
		if(timer.ElapsedTime() > maxTime) {
	  	printf("Timed out at %gs.\n",timer.ElapsedTime());
			break;
		}
	}
	//RoadmapPlanner roadmap(cspace);
	//planner->GetRoadmap(roadmap);
	//cout<<roadmap.roadmap.nodes.size()<<" nodes"<<endl;
	if(!solved)
	{
		if(planner->IsConnected(mstart,mgoal))
		{
			printf("Solved.\n");
			solved=true;
		}
		else
	  	printf("Timed out on iteration %d.\n",iter);
	}

	delete planner;

	printf ("Planning time: %g\n", timer.ElapsedTime());
}

int main(int argc,char** argv)
{
  RandHelper::srand(time(NULL));

	Geometric2DCSpace cspace;
//	SetupPassage(cspace,0.01,0.3);
	SetupBoxObstacle(cspace, 0.79, 0.79);

	MotionPlannerFactory factory;
	//factory.type = MotionPlannerFactory::PRM;
	factory.type = MotionPlannerFactory::SBL;

	Config start(2),goal(2);
	start[0] =0.1;
	start[1] =0.1; //0.2;
	goal[0] = 0.9;
	goal[1] = 0.9; //0.2;
	int numIters=10000;

	//PrintPlannerTest(factory,&cspace,start,goal,10,numIters);

	PlannerTest(factory,&cspace,start,goal);

	/*int maxPlanIters = numIters;
	double maxTime = Inf;

	MotionPlannerInterface* planner = factory.Create(&cspace);
	int mstart = planner->AddMilestone(start);
	int mgoal = planner->AddMilestone(goal);

	bool solved = false;
	Timer timer;

	int iter = 0;
	for(iter = 0; iter < maxPlanIters; iter++)
	{
		if(planner->IsConnected(mstart, mgoal))
		{
			printf("Solved on iteration %d.\n", iter);
			solved = true;
			break;
		}

		planner->PlanMore();

		if(timer.ElapsedTime() > maxTime)
		{
			printf("Timed out at %gs.\n", timer.ElapsedTime());
			break;
		}
	}

	if(!solved)
	{
		if(planner->IsConnected(mstart, mgoal))
		{
			printf("Solved.\n");
			solved = true;
		}
		else
			printf("Timed out on iteration %d.\n", iter);
	}

	printf ("Planning time: %g\n", timer.ElapsedTime());*/

	return 0;
}

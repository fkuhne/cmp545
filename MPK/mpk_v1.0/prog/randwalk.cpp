#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <vector>

#include "mpk_rand.h"
#include "mpk_defs.h"
#include "mpk_inventor.h"

#include "mpkIncludeFile.H"
#include "mpkRobot.H"
#include "mpkRobotCollection.H"
#include "mpkObstacle.H"
#include "mpkObstacleCollection.H"
#include "mpkCollPairSet.H"
#include "mpkBVControl.H"
#include "mpkGUI.H"
#include "mpkTraceVis.H"

#ifdef ADAPT_COLLCHECKER
#include "mpkAdaptSegmentChecker.H"
#else
#include "mpkSimpleSegmentChecker.H"
#endif

//#define TRACE_VIS       // show path of trace point

double shrink_fact = 0.5; // sampling window shrink factor

bool cycle_through_params=false;
int curr_dof = 0;

int curr_step = 0;
int num_samples=0;
int max_steps=-1;
int curr_anim_steps = 10;
int anim_steps = 2;

int num_segs = 0;
int num_segs_total = 0;

double delta = 0.01;
double epsilon = 0.012;

mpkRobotCollection* robots;
mpkObstacleCollection* obstacles;
mpkCollPairSet* collcheck_pairs;

mpkConfigChecker* config_checker[2];
mpkConfig config[2];
int start=0, goal=1-start;

vector<mpkConfig> all_confs;
mpkTraceVis* tracevis=0;

void myIdleCB(void *data, SoSensor *sensor)
{
  // Interpolate new position on segment between current start and
  // goal configurations (that define the current segment)
  float fact = float(curr_step)/curr_anim_steps;

  for ( int i=0, c=0; i<robots->num_robots(); i++ ) {

    mpkBaseRobot* robi = (*robots)[i];

    for ( int d=0; d < robi->num_params; d++ ) {
      robi->params[d] = (1-fact)*config[start][c] + fact*config[goal][c];
      c++;
    }

    robi->compute_forward_kinematics();
    robi->update_iv_transf();
  }

  mpkBVControl::update_iv_transf();

  // If we have reached goal configuration on current segment, then
  // goal becomes start and a new goal position has to be sampled
  if ( ++curr_step == curr_anim_steps ) {

    num_segs++;
    cout << num_segs << " free segs, "
	 << num_segs_total << " segs total" << endl;
    
    if ( max_steps > 0 && num_segs >= max_steps )
      exit(0);
    
    start = goal;
    goal  = 1-start;
    
    // sample random free goal point so that segment (start,goal) is free
    bool seg_coll;
    double sample_window = 1;
    bool point_coll;
    int num_coll_points=0;
    do {
      do {
	
	int idx1=0, idx2=config[start].size()-1;
	if ( cycle_through_params ) {
	  for ( int i=idx1; i<=idx2; i++ )
	    config[goal][i] = config[start][i];
	  idx1=idx2=curr_dof;
	}
	for ( int i=idx1; i<=idx2; i++ ) {
	  double le = config[start][i] - sample_window;
	  if ( le < 0 ) le = 0;
	  double ri = config[start][i] + sample_window;
	  if ( ri > 1 ) ri = 1;
	  config[goal][i] = le + (ri-le)*mpk_drand();
	}
	
	sample_window *= shrink_fact;
	
	point_coll = (config_checker[goal]->clearance(&(config[goal])) <= 0);
	if ( point_coll ) num_coll_points++;
	
      } while (point_coll);
      
      num_segs_total++;
#ifdef ADAPT_COLLCHECKER
      mpkAdaptSegmentChecker seg_check(config_checker[start],
				       config_checker[goal]);
#else
      mpkSimpleSegmentChecker seg_check(config_checker[0],
					&(config[start]),
					&(config[goal]),
					epsilon);
#endif
      while (seg_check.iteration_step())
	;
      seg_coll = seg_check.collision();

      cout << "sampling window: " << sample_window << " ";
#ifndef SAVE_MEMORY
      cout << "BV:   " << seg_check.num_bv_tests << " " 
	   << "Tri:  " << seg_check.num_tri_tests << endl;
      if ( seg_coll ) cout << "  colliding at " << seg_check.coll_t;
      else cout << "  collision-free.";
#endif
      cout << endl;

    } while (seg_coll);
    cout << endl;
    if (++curr_dof >= config[start].size() ) curr_dof=0;
    
    double ddim = robots->num_params();
    curr_anim_steps =
      int(anim_steps * config[start].dist(config[goal]) / sqrt(ddim));
    if ( curr_anim_steps == 0 )
      curr_anim_steps = 2;
    curr_step = 0;

#ifdef TRACE_VIS
    all_confs.push_back(config[goal]);
    if ( tracevis ) {
      mpkGUI::scene->removeChild(tracevis->ivroot());
      delete tracevis;
    }
    tracevis = new mpkTraceVis(robots, all_confs, 0.01);
    tracevis->set_show(true);
    mpkGUI::scene->addChild(tracevis->ivroot());
#endif  
  
  }
  sensor->schedule();
}



void myKeyPressCB(void *userData, SoEventCallback *eventCB)
{
  const SoEvent *event = eventCB->getEvent();

  if ( SO_KEY_PRESS_EVENT(event,S) ) {
    SoIdleSensor *myIdle = new SoIdleSensor(myIdleCB,0);
    myIdle->schedule();
  }
}


int main(int argc, char **argv)
{
  mpk_initrand();

  if ( argc < 2 ) {
    cerr << "Usage: randwalk <scenefile> [options]" << endl;
    cerr << "Options:" << endl;
    cerr << "  a <anim_steps>           animation speed.\n";
    
    cerr << "  d <delta>                minimum workspace clearance.\n";
#ifndef ADAPT_COLLCHECKER
    cerr << "  e <epsilon>              configuration space resolution.\n";
#endif
    cerr << "  n <num_samples>          first check <num_samples> random samples and\n";
    cerr << "                           report how many of them are colliding.\n";
    cerr << "  m <max_steps>            terminate after <max_steps> steps.\n";
    cerr << "  f <shrink-fact>          sampling window shrink factor.\n";
    cerr << "  y                        cycle through parameters while sampling only\n";
    cerr << "                           a single DOF at each step.\n";
    exit(1);
  }
  
  for (int i=2; i<argc; i++ )
    switch(argv[i][0] ) {
    case 'a': assert(++i<argc); anim_steps = atoi(argv[i]); break;
    case 'd': assert(++i<argc); delta = atof(argv[i]); break;
#ifndef ADAPT_COLLCHECKER
    case 'e': assert(++i<argc); epsilon = atof(argv[i]); break;
#endif
    case 'n': assert(++i<argc); num_samples = atoi(argv[i]); break;
    case 'm': assert(++i<argc); max_steps = atoi(argv[i]); break;
    case 'f': assert(++i<argc); shrink_fact = atof(argv[i]); break;
    case 'y': cycle_through_params = true; break;
    }

  mpkGUI::init(argv, myKeyPressCB, 0);
  mpkGUI::read_scene(argv[1]);
  robots = mpkGUI::get_robots();
  obstacles = mpkGUI::get_obstacles();
  collcheck_pairs = new mpkCollPairSet(robots,obstacles,delta);
  //collcheck_pairs->print();

  mpkBVControl::init(robots,obstacles);
  mpkBVControl::set_shown_bv(mpkBVControl::OBB);
 
  config[0] = mpkConfig(robots->num_params());
  config[1] = mpkConfig(robots->num_params());
  config_checker[0] =
    new mpkConfigChecker(&collcheck_pairs->test_pairs, robots);
  config_checker[1] =
    new mpkConfigChecker(&collcheck_pairs->test_pairs, robots);

  int numcoll = 0;
  if ( num_samples ) {

    for (int k=0; k<num_samples; k++ ) {
      for ( int i=0; i<config[start].size(); i++ ) {
	config[start][i] = mpk_drand();
      }
      if ( config_checker[start]->clearance(&(config[start])) <= 0 )
	numcoll++;
    }
    cout << numcoll << " colliding samples of " << num_samples << " total." << endl;
  }

  for ( int i=0; i<config[start].size(); i++ )
    config[start][i] = 0.5;
  numcoll = 0;
  while ( config_checker[start]->clearance(&(config[start])) <= 0) {
    numcoll++;
    for ( int i=0; i<config[start].size(); i++ ) {
      config[start][i] = mpk_drand();
    }
  } 
  cout << numcoll
       << " colliding configurations tested to find initial configuration."
       << endl;

  robots->set_config(config[start]);
  robots->compute_forward_kinematics();
  robots->update_iv_transf();    

  config[goal] = config[start];
  config_checker[goal]->clearance(&(config[goal]));

  all_confs.clear();
  all_confs.push_back(config[start]);

  double ddim = robots->num_params();
  curr_anim_steps =
    int(anim_steps * config[start].dist(config[goal]) / sqrt(ddim));
  if ( curr_anim_steps == 0 )
    curr_anim_steps = 2;

  mpkGUI::start("Random walk");
}



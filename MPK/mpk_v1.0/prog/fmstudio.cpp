#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <vector>
#include <string>

using namespace std;

#include "mpk_rand.h"
#include "mpk_defs.h"

#include "mpkObstacleCollection.H"
#include "mpkRobotCollection.H"

#include "mpkCollPairSet.H"
#include "mpkConfigChecker.H"

#include "sblPlanner.H"

#include "mpkPathSmoother.H"

#include "mpkBVControl.H"
#include "mpkMouseControl.H"
#include "mpkTraceVis.H"
#include "mpkGUI.H"

#include "GetTime.h"

#if ADAPT_COLLCHECKER
#define PLANNER "A-SBL"
#else
#define PLANNER "SBL"
#endif

mpkConfigChecker* config_check;

mpkRobotCollection* robots;
mpkObstacleCollection* obstacles;
mpkCollPairSet* collcheck_pairs;

double delta = 0.01;
double epsilon = 0.012;
double rho = 0.15;
int max_iter = 30000;
int max_animsteps = 100;
double step_size = 0.1;
mpkTraceVis* path_trace = 0;
float path_trace_col[] = {1,1,0};

int curr_animseg = 0;
int curr_animstep = 0;
double curr_animt = 0;
bool run_anim = false;
SoIdleSensor* myIdle;
string video_fname="-";

bool shift_pressed = false;
bool ctrl_pressed = false;

int bv_level=0;

enum MODE { cartesian, parameter, target };
MODE control_mode = parameter;

struct config {
  config() {};
  config(const mpkConfig& q, bool is_key=true)
  {this->q = q; this->is_key = is_key;};
  bool is_key;
  mpkConfig q;
};

string conf_fname;
vector<config> database;
int database_idx = -1;
vector<config> plan;
int plan_idx = -1;
double planner_time = -999;

int smoothe_steps = 50;

mpkConfig conf_buf;

void print_status()
{
  mpkConfig q(robots->num_params());
  robots->get_config(q);
#if ADAPT_COLLCHECKER | DO_TOLERANCE_TEST
  bool is_free = (config_check->clearance(&q) > 0);
#else
  bool is_free = !config_check->collision(&q);
#endif
  
  cout << "------------------------------------------------" << endl;
  cout << "                 FMSTUDIO v1.0                  " << endl;
  cout << "------------------------------------------------" << endl;
  cout << "(ESC) switch between camera and keyboard control" << endl;
  cout << "(F1)  toggle cartesian/parameter/target mode (curr: "
       << ((control_mode==cartesian)? "cartesian":
	   (control_mode==parameter)? "parameter": "target")
       << ")" << endl;
  cout << "(F2)  plan (" << PLANNER << ")" << endl;
  cout << "(F3)  smoothe" << endl;
  cout << "(F4)  plan (" << PLANNER << ") + smoothe" << endl;
  cout << "(F5)  start/stop plan animation" << endl;
  cout << "(F6)  reset animation" << endl;
  cout << "(F11) freeze/unfreeze current link" << endl;
  cout << "(F12) freeze/unfreeze current robot" << endl;
  cout << endl;

  cout << "(Space/Num5) stepsize : " << step_size << endl;
#if ADAPT_COLLCHECKER | DO_TOLERANCE_TEST
  cout << "(D)elta               : " << delta << endl;
#endif
#ifndef ADAPT_COLLCHECKER
  cout << "(E)psilon             : " << epsilon << endl;
#endif
  cout << "(R)ho                 : " << rho << " (sampling window size)" << endl;
  cout << "(M)ax. planner iter.  : " << max_iter << endl;
  cout << "Sm(O)othe steps       : " << smoothe_steps << endl;
  cout << "(A)nimsteps (max.)    : " << max_animsteps << endl;
  cout << endl;

  cout << "Sample (F)ree random config" << endl;
  cout << "(B)ounding boxes (OBB/RSS/off)" << endl;
  cout << "(T)raced end-effector path on/off" << endl;
#ifndef WIN32
  cout << "(I)mage snapshot..." << endl;
  cout << "(V)ideo file name: " << video_fname.c_str() << endl;
#endif
  cout << "(Q)uit" << endl;
  cout << endl;

  char buf[100];
  if (plan_idx+1 > plan.size()) sprintf(buf,"end");
  else sprintf(buf,"%d", plan_idx+1);
  cout << "Plan config     : " << buf << "/" << plan.size();
  if ( plan_idx >= 0 && plan[plan_idx].is_key ) cout << " *";
  if ( planner_time >= 0 ) {
    cout << " - planned in " << planner_time << " secs.";
  }
  else {
    if ( planner_time == -999 ) cout << " - not planned yet";
    else cout << " - planning failed";
  }
  cout << endl;
  if (database_idx+1 > database.size()) sprintf(buf,"end");
  else sprintf(buf,"%d", database_idx+1);
  cout << "Database config : " << buf << "/" << database.size()
       << " (database file: " << conf_fname.c_str() << ")" << endl;

  cout << "(Ins)ert conf. into plan at current position" << endl;
  cout << "(Del)ete conf. from plan at current position" << endl;
  cout << "(Enter)  append conf. at end of plan" << endl;
  cout << "(Home)   step to next conf. in plan" << endl;
  cout << "(End)    step to previous conf. in plan" << endl;
  cout << "(+Shift) applies commands to database" << endl;
  cout << "(Ctrl+Shift+Del) clears entire plan" << endl;
  cout << endl;

  cout << "(K)ey/unkey current configuration in plan" << endl;
  cout << "(G)oto database config #..." << endl;
  cout << "(L)oad task/plan..." << endl;
  cout << "(S)ave task/plan..." << endl;
  cout << endl;

  cout << "Picked joint & link: ";
  if ( mpkMouseControl::picked_rob )
    cout << mpkMouseControl::picked_rob->get_name()
	 << ":"
	 << mpkMouseControl::picked_joint->name;
  else
    cout << "none";
  if ( mpkMouseControl::picked_rob ) {
    int idx = robots->first_param_idx(mpkMouseControl::picked_rob_idx) +
      mpkMouseControl::picked_joint->param_idx;
    if (mpkMouseControl::picked_rob->param_opts[mpkMouseControl::picked_joint->param_idx].is_passive)
      cout << " (passive)";
    else if (mpkMouseControl::picked_rob->
	     param_opts[mpkMouseControl::picked_joint->param_idx].is_frozen)
      cout << " (frozen)";
  }
  cout << endl;

#ifdef ROBOT_BASE_MOVABLE
  cout << "Picked robot: ";
  if ( mpkMouseControl::picked_rob )
    cout << mpkMouseControl::picked_rob->get_name() << " ("
	 << mpkMouseControl::picked_rob->base_T.T[0] << ","
	 << mpkMouseControl::picked_rob->base_T.T[1] << ","
	 << mpkMouseControl::picked_rob->base_T.T[2]
	 << "), (Ctrl+cursor keys move robot) ";
  else
    cout << "none";
#endif
  cout << endl;
  
  if ( mpkMouseControl::picked_rob ) {
    bool rob_frozen = true;
    for ( int i=0; i<mpkMouseControl::picked_rob->num_params; i++ )
      if (!mpkMouseControl::picked_rob->param_opts[i].is_frozen) {
	rob_frozen = false;
	break;
      }
    if ( rob_frozen ) cout << "(Entire mpkRobot frozen)" << endl;
  }

  cout << "Colliding pair: ";
  if ( !is_free ) {
    cout << "("
	 << collcheck_pairs->test_pairs[config_check->collpair_idx()].o1.name 
	 << ","
	 << collcheck_pairs->test_pairs[config_check->collpair_idx()].o2.name
	 << ")" << endl;
  }
  else
    cout << "(none)" << endl;

  cout << "Collision set size: " << collcheck_pairs->test_pairs.size()
       << " (" << collcheck_pairs->num_obst_pruned + collcheck_pairs->num_rob_pruned << " pairs pruned)" << endl;
}

void load_confs(const char* fname, vector<config>& confs)
{
  confs.clear();
  ifstream f(fname);
  if (!f) return;
  mpkConfig q(robots->num_params());
  while (f) {
    char c;
    f >> c;
    int i;
    for ( i=0; f && i<q.size(); i++ )
      f >> q[i];
    if ( i < q.size() ) break;
    confs.push_back(config(q,c=='*'));
  }
}

void save_confs(const char* fname, vector<config>& confs)
{
  ofstream f(fname);
  if (!f) return;
  for ( int n=0; n<confs.size(); n++ ) {
    f << ((confs[n].is_key)? "*" : " ");
    f << "\t";
    for ( int i=0; f && i < confs[n].q.size(); i++ )
      f << confs[n].q[i] << " ";
    f << endl;
  }
}

void insert(vector<config>& confs, int& idx, const mpkConfig& q)
{
  if ( idx < 0 || idx >= confs.size() ) {
    confs.push_back(config(q));
    idx = confs.size()-1;
  }
  else {
    confs.resize(confs.size()+1);
    for ( int i=confs.size()-1; i>idx; i-- )
      confs[i] = confs[i-1];
    confs[idx] = config(q);
  }
}

void erase(vector<config>& confs, int& idx)
{
  if ( idx < 0 || idx >= confs.size() ) {
    idx = confs.size()-1;
  }
  else {
    conf_buf = confs[idx].q;
    for ( int i=idx; i<confs.size()-1; i++ )
      confs[i] = confs[i+1];
    confs.resize(confs.size()-1);
    if ( idx >= confs.size() ) idx = confs.size()-1;
  }
}

void call_smoother(vector<config>& plan)
{
  if ( plan.size() <= 2 ) return;

  vector<config> smoothed_plan(0);
  int first = 0;
  while (first < plan.size()-1) {

    vector<mpkConfig> sub_path(1);
    sub_path[0] = plan[first].q;
    int last;
    for ( last = first+1; last < plan.size() && !plan[last].is_key; last++ )
      sub_path.push_back(plan[last].q);
    if ( last < plan.size() )
      sub_path.push_back(plan[last].q);

#if ADAPT_COLLCHECKER
    mpkPathSmoother smoother(sub_path, &collcheck_pairs->test_pairs,robots,0);
#else
    mpkPathSmoother smoother(sub_path,&collcheck_pairs->test_pairs,robots,0,
			     epsilon);
#endif
    smoother.smoothe(smoothe_steps);
    smoother.get_path(sub_path);

    smoothed_plan.push_back(config(sub_path[0],true));
    for ( int i=1; i<sub_path.size()-1; i++ )
      smoothed_plan.push_back(config(sub_path[i], false));
    
    first = last;

  }

  smoothed_plan.push_back(config(plan[plan.size()-1].q,true));
  plan = smoothed_plan;
}

void call_planner(vector<config>& plan)
{
  if ( plan.size() < 2 ) return;
  double time_start = GetTime();
  mpkConfig qtmp(robots->num_params());
  robots->get_config(qtmp);
  vector<config> tmp_plan;
  int start=0;
  while ( start < plan.size()-1 ) {

    int goal;
    for ( goal = start+1; goal < plan.size() && !plan[goal].is_key; goal++ )
      ;
    if ( goal >= plan.size() ) goal = plan.size()-1;

#if ADAPT_COLLCHECKER
    sblPlanner planner(robots, &collcheck_pairs->test_pairs);
#else
    sblPlanner planner(robots, &collcheck_pairs->test_pairs, epsilon);
#endif
    list<mpkConfig> clist;
    bool ok = planner.Query(plan[start].q, plan[goal].q, clist, rho, max_iter);

    if ( !ok ) {
      cerr << "Failure" << endl;
      robots->set_config(qtmp);
      planner_time = -1;
      return;
    }
    list<mpkConfig>::iterator it;
    for ( it = clist.begin(); it != clist.end(); ++it )
      tmp_plan.push_back(config(*it, it==clist.begin()));
    tmp_plan.pop_back();

    start = goal;
  }
  tmp_plan.push_back(plan[plan.size()-1]);

  plan = tmp_plan;
  robots->set_config(qtmp);
  cout << endl;
  double delta_t = GetTime() - time_start;
  cout << delta_t << " s" << endl;
  planner_time = delta_t;
}


void myIdleCB(void *data, SoSensor *sensor)
{
  if ( !run_anim || plan.size() < 2 ) return;

  if ( curr_animseg >= plan.size()-1 ) {
    curr_animseg = 0;
    curr_animt = 0;
    curr_animstep = 0;
  }

  double dist2go = 1.0 / max_animsteps;
  mpkConfig qtmp(robots->num_params());
  while ( 1 ) {
    curr_animstep++;
    const mpkConfig& start = plan[curr_animseg].q;
    const mpkConfig& goal = plan[curr_animseg+1].q;
    qtmp.lin_interpol(curr_animt, start, goal);
    double goaldist = qtmp.Linf_dist(goal, robots->param_weights);
    if ( goaldist >= dist2go ) {
      curr_animt += dist2go / start.Linf_dist(goal, robots->param_weights);
      qtmp.lin_interpol(curr_animt, start, goal);
      break;
    }
    else {
      if ( ++curr_animseg >= plan.size()-1 ) {
	curr_animseg=0;
	curr_animstep=0;
      }
      curr_animt = 0;
      dist2go -= goaldist;
    }
  }

  robots->set_config(qtmp); 
  robots->compute_forward_kinematics();
  robots->update_iv_transf();
  mpkBVControl::update_iv_transf();

  //print_status();

  if ( video_fname.find(".") != string::npos ) {
    char fname[1000];
    sprintf(fname, "anim_%5.5d_%s",
	    curr_animstep, video_fname.c_str());
    cout << "Creating " << fname << endl;
    mpkGUI::save_snapshot(fname);
  }

  sensor->schedule();
}


mpkTraceVis* update_pathvis(mpkTraceVis* pathvis, const vector<config>* path, float* rgb=0)
{
  vector<mpkConfig> confs;
  confs.clear();
  if ( path )
    for ( int i=0; i<path->size(); i++ )
      confs.push_back((*path)[i].q);
  bool is_shown = false;
  if ( pathvis ) {
    is_shown = pathvis->is_shown();
    mpkGUI::scene->removeChild(pathvis->ivroot());
    delete pathvis;
  }
  pathvis = new mpkTraceVis(robots, confs, epsilon, rgb);
  mpkGUI::scene->addChild(pathvis->ivroot());
  pathvis->set_show(is_shown);
  return pathvis;
}


void myKeyPressCB(void *userData, SoEventCallback *eventCB)
{
  bool config_changed = false;
  bool cross_moved = false;

  const SoEvent *event = eventCB->getEvent();

  if ( SO_KEY_PRESS_EVENT(event,RIGHT_SHIFT) ||
       SO_KEY_PRESS_EVENT(event,LEFT_SHIFT))
    shift_pressed = true;
  else if ( SO_KEY_RELEASE_EVENT(event,RIGHT_SHIFT) ||
	    SO_KEY_RELEASE_EVENT(event,LEFT_SHIFT))
    shift_pressed = false;
  else if ( SO_KEY_PRESS_EVENT(event,RIGHT_CONTROL) ||
       SO_KEY_PRESS_EVENT(event,LEFT_CONTROL))
    ctrl_pressed = true;
  else if ( SO_KEY_RELEASE_EVENT(event,RIGHT_CONTROL) ||
	    SO_KEY_RELEASE_EVENT(event,LEFT_CONTROL))
    ctrl_pressed = false;

  else if (SO_KEY_PRESS_EVENT(event,F1)) {
    control_mode = (control_mode==parameter) ? cartesian :
      ((control_mode==cartesian)? target : parameter);
    mpkMouseControl::set_show_cross_cursor(control_mode==cartesian);
    if ( control_mode == target )
      mpkMouseControl::move_to_target_mode = true;
    else
      mpkMouseControl::move_to_target_mode = false;
    print_status();
  }
  else if (SO_KEY_PRESS_EVENT(event,F2) && plan.size() >= 2 ) {
    call_planner(plan);
    path_trace = update_pathvis(path_trace, &plan,
				      path_trace_col);
    print_status();
  }
  else if (SO_KEY_PRESS_EVENT(event,F3)) {
    call_smoother(plan);
    path_trace = update_pathvis(path_trace, &plan,
				      path_trace_col);
    print_status();
  }
  else if (SO_KEY_PRESS_EVENT(event,F4)) {
    // plan + smoothe
    call_planner(plan);
    call_smoother(plan);
    path_trace = update_pathvis(path_trace, &plan,
				      path_trace_col);
    print_status();
  }

  // toggle animation
  else if (SO_KEY_PRESS_EVENT(event,F5)) {
    run_anim = !run_anim;
    myIdle->schedule();
  }

  // reset animation
  else if (SO_KEY_PRESS_EVENT(event,F6)) {
    run_anim = false;
    curr_animseg = 0;
    curr_animstep = 0;
    if ( plan.size() ) {
      robots->set_config(plan[0].q);
      config_changed=true;
    }
  }

  else if (SO_KEY_PRESS_EVENT(event,F11)) {
    if ( mpkMouseControl::picked_joint ) {
      int d = mpkMouseControl::picked_joint->param_idx;
      for ( int i=0; i<mpkMouseControl::picked_joint->Tq->num_params(); i++ ) {
	mpkMouseControl::picked_rob->param_opts[d+i].is_frozen =
	  !mpkMouseControl::picked_rob->param_opts[d+i].is_frozen;
      }
    }
    print_status();
  }

  else if (SO_KEY_PRESS_EVENT(event,F12)) {
    if ( mpkMouseControl::picked_rob ) {
      for ( int i=0; i<mpkMouseControl::picked_rob->num_params; i++ ) {
	mpkMouseControl::picked_rob->param_opts[i].is_frozen =
	  !mpkMouseControl::picked_rob->param_opts[i].is_frozen;
      }
    }
    print_status();
  }

  else if (SO_KEY_PRESS_EVENT(event,SPACE) ||
	   SO_KEY_PRESS_EVENT(event,PAD_5)) {
    step_size /= 10;
    if ( step_size < 0.000001 ) step_size = 0.1;
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,Q) ) {
    save_confs(conf_fname.c_str(), database);
    exit(0);
  }

  else if ( SO_KEY_PRESS_EVENT(event,A) ) {
    cout << "Enter new value for max. animsteps: " << flush;
    cin >> max_animsteps;
    print_status();
  }

#if ADAPT_COLLCHECKER | DO_TOLERANCE_TEST
  else if ( SO_KEY_PRESS_EVENT(event,D) ) {
    delete collcheck_pairs;
    delete config_check;
    cout << "Enter new value for delta: " << flush;
    cin >> delta;
    collcheck_pairs = new mpkCollPairSet(robots,obstacles,delta);
    config_check = new mpkConfigChecker(&collcheck_pairs->test_pairs, robots);
    print_status();
  }
#endif

#ifndef ADAPT_COLLCHECKER
  else if ( SO_KEY_PRESS_EVENT(event,E) ) {
    cout << "Enter new value for epsilon: " << flush;
    cin >> epsilon;
    print_status();
  }
#endif

  else if ( SO_KEY_PRESS_EVENT(event,R) ) {
    cout << "Enter new value for rho: " << flush;
    cin >> rho;
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,M) ) {
    cout << "Enter new value for max. iter: " << flush;
    cin >> max_iter;
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,O) ) {
    cout << "Enter new value for smoothe steps: " << flush;
    cin >> smoothe_steps;
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,T) ) {
    if ( path_trace->is_shown() )
      path_trace->set_show(false);
    else
      path_trace->set_show(true);
  }

  else if ( SO_KEY_PRESS_EVENT(event,L) ) {
    cout << "Load task/path - please enter file name: " << flush;
    string fname;
    cin >> fname;
    ifstream f(fname.c_str());
    if ( !f ) cout << "Error reading file." << endl;
    else {
      load_confs(fname.c_str(), plan);
      if ( plan.size() ) {
	plan_idx = 0;
	robots->set_config(plan[0].q);
	path_trace = update_pathvis(path_trace, &plan,
					  path_trace_col);
	config_changed = true;
      }
      else plan_idx = -1;
    }
    print_status();
  }
  else if ( SO_KEY_PRESS_EVENT(event,S) ) {
    cout << "Save task/path - please enter file name: " << flush;
    string fname;
    cin >> fname;
    ofstream f(fname.c_str());
    if ( !f ) cout << "Error creating file." << endl;
    else { save_confs(fname.c_str(), plan); }
    print_status();
  }

#ifndef WIN32
  else if ( SO_KEY_PRESS_EVENT(event,I) ) {
    cout << "Save screen to image - please enter file name: " << flush;
    string fname;
    cin >> fname;
    cout << "Saving... " << flush;
    mpkGUI::save_snapshot(fname.c_str());
    cout << "done." << endl;
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,V) ) {
    cout << "Please enter video file name: " << flush;
    cin >> video_fname;
    print_status();
  }
#endif

  else if ( SO_KEY_PRESS_EVENT(event,F) ) {
    mpkConfig old_conf, rand_conf(robots->num_params());
    robots->get_config(old_conf);
    int i;
    const int max_tries = 1000;
    for ( i=0; i<max_tries; i++ ) {
      for ( int i=0; i<rand_conf.size(); i++ )
	rand_conf[i] = mpk_drand();
      robots->set_config(rand_conf);
      robots->compute_forward_kinematics();
#if ADAPT_COLLCHECKER | DO_TOLERANCE_TEST
      bool is_free = config_check->clearance(&rand_conf) > 0;
#else
      bool is_free = !config_check->collision(&rand_conf);
#endif
      if ( is_free ) {
	config_changed = true;
	break;
      }
    }
    if ( i == max_tries ) {
      robots->set_config(old_conf);
      robots->compute_forward_kinematics();
    }
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,G) ) {
    cout << "Please enter config number: " << flush;
    int num;
    cin >> num;
    if ( num >= 1 && num <= database.size() ) {
      database_idx = num-1;
      robots->set_config(database[num-1].q);
      config_changed = true;
    }
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,B) )
    mpkBVControl::set_shown_bv((mpkBVControl::get_shown_bv() == mpkBVControl::none)?
			    mpkBVControl::OBB : 
			    (mpkBVControl::get_shown_bv() == mpkBVControl::OBB)?
			    mpkBVControl::RSS :
			    mpkBVControl::none);

  else if ( SO_KEY_PRESS_EVENT(event,ENTER) ||
	    SO_KEY_PRESS_EVENT(event,PAD_ENTER) ) {
    mpkConfig qtmp(robots->num_params());
    robots->get_config(qtmp);
    if ( shift_pressed ) {
      database_idx = database.size();
      insert(database,database_idx,qtmp);
    }
    else {
      plan_idx = plan.size();
      insert(plan,plan_idx,qtmp);
    }
    print_status();
  }

  else if (SO_KEY_PRESS_EVENT(event,K)) {
    if (shift_pressed) {
      for ( int i=1; i<plan.size()-1; i++ ) {
	plan[i].is_key = false;
      }
    } else if ( plan_idx >= 0 )
      plan[plan_idx].is_key = !plan[plan_idx].is_key;
    print_status();
  }

  else if ( SO_KEY_PRESS_EVENT(event,INSERT) ||
	    SO_KEY_PRESS_EVENT(event,PAD_0) ) {
    mpkConfig qtmp(robots->num_params());
    robots->get_config(qtmp);
    if ( shift_pressed )
      insert(database,database_idx,qtmp);
    else
      insert(plan,plan_idx,qtmp);
    print_status();
  }

#ifdef WIN32
  else if ( SO_KEY_PRESS_EVENT(event,KEY_DELETE) ||
#else
  else if ( SO_KEY_PRESS_EVENT(event,DELETE) ||
#endif
	    SO_KEY_PRESS_EVENT(event,PAD_PERIOD) ) {
    if ( shift_pressed ) {
      if ( ctrl_pressed ) {
	plan.resize(0);
	plan_idx = -1;
      }
      else {
	erase(database,database_idx);
	if ( database.size() ) robots->set_config(database[database_idx].q);
      }
    }
    else {
      erase(plan,plan_idx);
      if ( plan.size() ) robots->set_config(plan[plan_idx].q);
    }
    config_changed = true;
  }

  else if ( SO_KEY_PRESS_EVENT(event,LEFT_ARROW) ||
	    SO_KEY_PRESS_EVENT(event,PAD_4)) {
#ifdef ROBOT_BASE_MOVABLE
    if ( ctrl_pressed ) {
      if ( mpkMouseControl::picked_rob ) {
	mpkMouseControl::picked_rob->
	  translate_base(-mpkMouseControl::scene_size*step_size, 0, 0);
	config_changed = true;
      }
    }
    else
#endif
      {
	if ( control_mode == cartesian ) {
	  mpkMouseControl::
	    move_cross_cursor_1d(0,-2*mpkMouseControl::scene_size*step_size);
	  cross_moved = true;
	}
	else if ( mpkMouseControl::picked_rob &&
		  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
		  -step_size >= 0.0 ) {
	  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
	    -= step_size;
	  config_changed = true;
	}
      }
  }

  else if ( SO_KEY_PRESS_EVENT(event,RIGHT_ARROW) ||
	    SO_KEY_PRESS_EVENT(event,PAD_6)) {
#ifdef ROBOT_BASE_MOVABLE
    if ( ctrl_pressed ) {
      if ( mpkMouseControl::picked_rob ) {
	mpkMouseControl::picked_rob->
	  translate_base(mpkMouseControl::scene_size*step_size, 0, 0);
	config_changed = true;
      }
    }
    else
#endif
      {
	if ( control_mode == cartesian ) {
	  mpkMouseControl::
	    move_cross_cursor_1d(0,2*mpkMouseControl::scene_size*step_size);
	  cross_moved = true;
	}
	else if (mpkMouseControl::picked_rob &&
		 mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
		 +step_size <= 1.0 ) {
	  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
	    += step_size;
	  config_changed = true;
	}
      }
  }
  
  else if ( SO_KEY_PRESS_EVENT(event,DOWN_ARROW) ||
	    SO_KEY_PRESS_EVENT(event,PAD_2)) {
#ifdef ROBOT_BASE_MOVABLE
    if ( ctrl_pressed ) {
      if ( mpkMouseControl::picked_rob ) {
	mpkMouseControl::picked_rob->
	  translate_base(0,0,-mpkMouseControl::scene_size*step_size);
	config_changed = true;
      }
    }
    else
#endif
      {
	if ( control_mode == cartesian ) {
	  mpkMouseControl::
	    move_cross_cursor_1d(2,-2*mpkMouseControl::scene_size*step_size);
	  cross_moved = true;
	}
	else if ( mpkMouseControl::picked_rob &&
		  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
		  -step_size >= 0.0 ) {
	  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
	    -= step_size;
	  config_changed = true;
	}
      }
  }

  else if ( SO_KEY_PRESS_EVENT(event,UP_ARROW) ||
	    SO_KEY_PRESS_EVENT(event,PAD_8)) {
#ifdef ROBOT_BASE_MOVABLE
    if ( ctrl_pressed ) {
      if ( mpkMouseControl::picked_rob ) {
	mpkMouseControl::picked_rob->
	  translate_base(0,0,mpkMouseControl::scene_size*step_size);
	config_changed = true;
      }
    }
    else
#endif
      {
	if ( control_mode == cartesian ) {
	  mpkMouseControl::
	    move_cross_cursor_1d(2,2*mpkMouseControl::scene_size*step_size);
	  cross_moved = true;
	}
	else if ( mpkMouseControl::picked_rob &&
		  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
		  +step_size <= 1.0 ) {
	  mpkMouseControl::picked_rob->params[mpkMouseControl::picked_dof]
	    += step_size;
	  config_changed = true;
	}
      }
  }

  else if ( SO_KEY_PRESS_EVENT(event,PAGE_DOWN) ||
	    SO_KEY_PRESS_EVENT(event,PAD_3) ) {
#ifdef ROBOT_BASE_MOVABLE
    if ( ctrl_pressed ) {
      if ( mpkMouseControl::picked_rob ) {
	mpkMouseControl::picked_rob->
	  translate_base(0,-mpkMouseControl::scene_size*step_size, 0);
	config_changed = true;
      }
    }
    else
#endif
      if ( control_mode == cartesian ) {
	mpkMouseControl::
	  move_cross_cursor_1d(1,-2*mpkMouseControl::scene_size*step_size);
	cross_moved = true;
      }
  }
  else if ( SO_KEY_PRESS_EVENT(event,PAGE_UP) ||
	    SO_KEY_PRESS_EVENT(event,PAD_9) ) {
#ifdef ROBOT_BASE_MOVABLE
    if ( ctrl_pressed ) {
      if ( mpkMouseControl::picked_rob ) {
	mpkMouseControl::picked_rob->
	  translate_base(0,mpkMouseControl::scene_size*step_size, 0);
	config_changed = true;
      }
    }
    else
#endif
      if ( control_mode == cartesian ) {
	mpkMouseControl::
	  move_cross_cursor_1d(1,2*mpkMouseControl::scene_size*step_size);
	cross_moved = true;
      }
  }

  else if ( SO_KEY_PRESS_EVENT(event,HOME) ||
	    SO_KEY_PRESS_EVENT(event,PAD_7) ) {
    if ( shift_pressed ) {
      if (database_idx > 0 ) {
	database_idx--;
	robots->set_config(database[database_idx].q);
	config_changed = true;
      }
    }
    else if ( plan_idx > 0 ) {
      plan_idx--;
      robots->set_config(plan[plan_idx].q);
      config_changed = true;
    }
  }

  else if ( SO_KEY_PRESS_EVENT(event,END) ||
	    SO_KEY_PRESS_EVENT(event,PAD_1) ) {
    if ( shift_pressed ) {
      if ( database_idx < database.size() ) {
	database_idx++;
	if ( database_idx < database.size() ) {
	  robots->set_config(database[database_idx].q);
	  config_changed = true;
	}
      }
    }
    else if ( plan_idx < plan.size() ) {
      plan_idx++;
      if ( plan_idx < plan.size() ) {
	robots->set_config(plan[plan_idx].q);
	config_changed = true;
      }
    }
    print_status();
  }

  if ( cross_moved && mpkMouseControl::link_attached_to_cursor ||
       config_changed ) {
    robots->compute_forward_kinematics();
    robots->update_iv_transf();
    mpkBVControl::update_iv_transf();
    print_status();
  }
}

int main(int argc, char **argv)
{
  int i;
  mpk_initrand();
  if ( argc < 2 ) {
    cerr << "Please call with scene file name as parameter." << endl;
#ifdef WIN32
    getchar();
#endif
    exit(1);
  }

  mpkGUI::init(argv, myKeyPressCB, mpkMouseControl::myMouseButtonCB );
  mpkGUI::read_scene(argv[1]);
  robots = mpkGUI::get_robots();
  obstacles = mpkGUI::get_obstacles();
  collcheck_pairs = new mpkCollPairSet(robots,obstacles,delta);
  collcheck_pairs->print();
  config_check = new mpkConfigChecker(&collcheck_pairs->test_pairs, robots);

  mpkMouseControl::init(mpkGUI::scene, robots, obstacles, print_status);
  mpkGUI::scene->addChild(mpkMouseControl::ivroot());

  path_trace = update_pathvis(path_trace, 0, path_trace_col);
  
  conf_fname = argv[1];

  vector<int> idx;

  // parse command line / load configuration database
  for ( i=2; i<argc; i++ ) {
    if ( strcmp(argv[i],"-db")==0 ) {
      if ( ++i < argc ) {
	conf_fname = argv[i];
	load_confs(conf_fname.c_str(), database);
      }
    }
    else if ( strcmp(argv[i],"-b")==0 ) {
      if ( ++i < argc ) bv_level = atoi(argv[i]);
    }
    else
      idx.push_back(atoi(argv[i]));
  }
  if ( !database.size() ) {
    int pos = conf_fname.find(".iv");
    if ( pos != string::npos )
      conf_fname.replace(pos, 3, ".conf");
    load_confs(conf_fname.c_str(), database);
  }
  if ( database.size() ) {
    database_idx = 0;
    robots->set_config(database[0].q);
  }

  mpkBVControl::init(robots, obstacles, bv_level);
  mpkGUI::scene->addChild(mpkBVControl::ivroot());

  // setup planning task (from command line parameters)
  for ( i=0; i<idx.size(); i++ ) {
    if ( idx[i] >= 1 && idx[i] <= database.size() )
      plan.push_back(database[idx[i]-1]);
    else {
      cerr << "Error: configuration " << idx[i] << " not in database." << endl;
      cerr << "Please start program with correct configuration indices" << endl;
      cerr << "or without any configuration index." << endl;
#ifdef WIN32
      getchar();
#endif
      exit(1);
    }
  }
  if ( plan.size() ) {
    plan_idx = 0;
    robots->set_config(plan[0].q);
  }

  robots->compute_forward_kinematics();
  robots->update_iv_transf();
  mpkBVControl::update_iv_transf();

  print_status();

  myIdle = new SoIdleSensor(myIdleCB,0);

  mpkGUI::start("FMSTUDIO");
}



#include <stdlib.h>

#include "mpk_rand.h"
#include "mpkGUI.H"
#include "mpkRobotCollection.H"
#include "mpkObstacleCollection.H"
#include "mpkCollPairSet.H"
#include "mpkAdaptSegmentChecker.H"
#include "mpkSimpleSegmentChecker.H"

// C-space resolution for simple segment collision checker
const double epsilon = 0.012;

// Workspace resolution for adaptive segment collision checker
const double delta = 0.01;

// Collection of all robots to be read from scene file
mpkRobotCollection* robots;

// Collection of all static obstacles to be read from scene file
mpkObstacleCollection* obstacles;

// Pairs of rigid bodies that will be checked for collision
mpkCollPairSet* collcheck_pairs;

// Collision checkers for single configurations
mpkConfigChecker* config_checker1;
mpkConfigChecker* config_checker2;


// Open Inventor keyboard callback
// Exits when 'Q' is pressed; when SPACE is pressed, the robots are
// moved to a random collision-free configuration that is visible from
// the current configuration (i.e., the line segment in c-space
// between the two configurations is free)
void myKeyPressCB(void *userData, SoEventCallback *eventCB)
{
  const SoEvent *event = eventCB->getEvent();

  if (SO_KEY_PRESS_EVENT(event,Q))
    exit(0);

  // if the user presses SPACE: sample a new free configuration that
  // can be connected to the current configuration in c-space by a
  // free straight line segment
  else if (SO_KEY_PRESS_EVENT(event,SPACE)) {

    cout << "========================" << endl;

    mpkConfig qcurr, qnew(robots->num_params());

    // copy current configuration (the state of the robots will be
    // modified in the following)
    robots->get_config(qcurr);

    // outer loop: break when collision-free segment is found
    while (1) {

      // inner loop: break when collision-free sample (candidate
      // endpoint of segment) is found
      while (1) {

	// sample random configuration in [0,1]^d
	for ( int i=0; i<qnew.size(); i++ )
	  qnew[i] = mpk_drand();

	// note that config_checker2 was initialized in the main program
	if ( config_checker2->clearance(&qnew) > 0 ) {
	  cout << "Free configuration" << endl;
	  break;  // exit from the inner while-loop
	}
	
	cout << "Colliding configuration" << endl;
      }

      // now we know that qnew is free, so test if the segment
      // [qcurr,qnew] is free, too
#ifdef ADAPT_COLLCHECKER
      mpkAdaptSegmentChecker seg_checker(config_checker1, config_checker2);
#else
      mpkSimpleSegmentChecker seg_checker(config_checker1, &qcurr, &qnew, epsilon);
#endif

      // the segment checker is lazy, i.e. it samples only a subset of
      // points on the segment per iteration step and returns true if
      // there is work left.  Once it returns false, the segment
      // status has been determined.  Note that the simple segment
      // checker may erroneously declare a segment collision-free if
      // epsilon is too big (in contrast, the adaptive checker is
      // guaranted to find all collisions).
      while ( seg_checker.iteration_step() )
	;

      // now we query the segment status and we can stop generating
      // candidate segments if the current segment is free
      if ( !seg_checker.collision() ) {
	cout << "Free segment" << endl;
	break; // this exits the outer while-loop
      }

      cout << "Colliding segment" << endl;
    }

#ifdef ADAPT_COLLCHECKER
    // copy the config checker as we move to the new configuration
    *config_checker1 = *config_checker2;
#endif

    // update the configuration of the robots
    robots->set_config(qnew);

    // explicitly update the transform matrices of the joints (for the
    // collision models only)
    robots->compute_forward_kinematics();

    // explicitly update the joint transform matrices in the scene
    // graph (for visualization)
    robots->update_iv_transf();
  }
}

int main(int argc, char **argv)
{
  if ( argc < 2 ) {
    cerr << "Please call with scene file name as parameter." << endl;
    exit(1);
  }

  // setup the mpkGUI
  mpkGUI::init(argv, myKeyPressCB);
  
  // read the scene file (that contains the robot and obstacle
  // descriptions)
  mpkGUI::read_scene(argv[1]);

  // get robots and obstacles from the scene
  robots = mpkGUI::get_robots();
  obstacles = mpkGUI::get_obstacles();

  // define which pairs of rigid bodies will be tested for collision
  // (the class mpkCollPairSet constructs all default pairs and stores
  // them in its member test_pairs (a list of pairs); instead, if only
  // specific pairs need to be tested, a vector<mpkCollCandPair> that
  // contains these pairs could be constructed at this point by some
  // customized code)
  collcheck_pairs = new mpkCollPairSet(robots,obstacles,delta);
  collcheck_pairs->print();

  // construct collision checkers for single configurations based on
  // the collision candidate pairs defined in
  // collcheck_pairs->test_pairs
  config_checker1 = new mpkConfigChecker(&collcheck_pairs->test_pairs, robots);
  config_checker2 = new mpkConfigChecker(&collcheck_pairs->test_pairs, robots);

  // sample a random free starting configuration
  mpkConfig q(robots->num_params());
  while (1) {

    // sample a random configuration in [0,1]^d
    for ( int i=0; i<q.size(); i++ )
      q[i] = mpk_drand(); // random number in [0,1]

    // test if the sampled configuration is free
    if ( config_checker1->clearance(&q) > 0 )
      break; // this breaks the while-loop
  }

  // update the configuration of the robots
  robots->set_config(q);
  
  // explicitly update the transform matrices of the joints (for the
  // collision models only)
  robots->compute_forward_kinematics();
  
  // explicitly update the joint transform matrices in the scene
  // graph (for visualization)
  robots->update_iv_transf();

  // start the main loop of the mpkGUI
  mpkGUI::start("Hello mpkRobot");
}



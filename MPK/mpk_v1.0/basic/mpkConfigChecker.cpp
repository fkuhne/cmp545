#include "mpkConfigChecker.H"

#define CHECK_SPHERES_FIRST

int mpkConfigChecker::num_tri_tests;
int mpkConfigChecker::num_bv_tests;
int mpkConfigChecker::num_obj_tests;

mpkConfigChecker::
mpkConfigChecker(const vector<mpkCollPair>* test_pairs,
		  mpkRobotCollection* robots)
{
  this->robots = robots;
  this->test_pairs = test_pairs;
  this->q = 0;
  this->model_id = 0;
}

bool
mpkConfigChecker::
collision(const mpkConfig* q, mpk_idx model_id)
{
  assert(model_id==0 || model_id==1);

  num_tri_tests = 0;
  num_bv_tests = 0;
  num_obj_tests = 0;
  this->model_id = model_id;
  this->q = 0;
  this->min_sep = -1; // disable: collision test does not determine min_sep

  robots->set_config(*q);
  robots->compute_forward_kinematics();

  for ( int i=0; i < test_pairs->size(); i++ ) {

    num_obj_tests++;

    const mpkCollPair& curr_pair = (*test_pairs)[i]; 

#ifdef CHECK_SPHERES_FIRST
    // perform simple sphere overlap test first for top-level bounding spheres
    if ( curr_pair.o2.rob ) {
      num_bv_tests++;
      double c1[3], c2[3];
      mpkBaseRobot::Joint& j1 =
	curr_pair.o1.rob->joints[curr_pair.o1.joint_idx];
      mpkBaseRobot::Joint& j2 =
	curr_pair.o2.rob->joints[curr_pair.o2.joint_idx];
      curr_pair.o1.Tr->apply(c1, j1.sphere_c);
      curr_pair.o2.Tr->apply(c2, j2.sphere_c);
      if (mpkCollDistAlgo::SphereDist(c1, c2,
				      j1.sphere_r, j2.sphere_r) > 0)
	continue; // spheres do not overlap --> continue with next pair
    }
#endif

    bool coll =
      mpkCollDistAlgo::Collision(curr_pair.o1.Tr->R, curr_pair.o1.Tr->T,
				 curr_pair.o1.pqp[model_id], 
				 curr_pair.o2.Tr->R, curr_pair.o2.Tr->T,
				 curr_pair.o2.pqp[model_id]);

    num_tri_tests += mpkCollDistAlgo::coll_res.num_tri_tests;
    num_bv_tests  += mpkCollDistAlgo::coll_res.num_bv_tests;

    if ( coll ) {
      coll_idx = i;
      return true;
    }

  }
  return false;
}

double
mpkConfigChecker::
clearance(const mpkConfig* q, mpk_idx model_id, bool store_dist)
{
  assert(model_id==0 || model_id==1);

  num_tri_tests = 0;
  num_bv_tests = 0;
  num_obj_tests = 0;
  this->model_id = model_id;
  this->q = q;
  this->min_sep = DBL_MAX;
  if ( store_dist && lbdist.size() != test_pairs->size() )
    lbdist.resize(test_pairs->size());

  robots->set_config(*q);
  robots->compute_forward_kinematics();

  for ( int i=0; i < test_pairs->size(); i++ ) {

    num_obj_tests++;

    const mpkCollPair& curr_pair = (*test_pairs)[i]; 

    double dist = 0;

#ifdef CHECK_SPHERES_FIRST
    // perform simple sphere distance computation first but only if we
    // do not store distances (greedy lower bounds from spheres tend
    // to be very bad) and only for pairs of two robot links (spheres
    // of obstacles and robot links are most likely overlapping)
    if ( !store_dist && curr_pair.o2.rob ) {
      double c1[3], c2[3];
      mpkBaseRobot::Joint& j1 =
	curr_pair.o1.rob->joints[curr_pair.o1.joint_idx];
      mpkBaseRobot::Joint& j2 =
	curr_pair.o2.rob->joints[curr_pair.o2.joint_idx];
      curr_pair.o1.Tr->apply(c1, j1.sphere_c);
      curr_pair.o2.Tr->apply(c2, j2.sphere_c);
      dist =
	mpkCollDistAlgo::SphereDist(c1, c2, j1.sphere_r, j2.sphere_r)
	- curr_pair.delta;
    }
#endif

    if ( dist <= 0 ) {
      dist =
	mpkCollDistAlgo::GreedyDistance(curr_pair.o1.Tr->R, curr_pair.o1.Tr->T,
					curr_pair.o1.pqp[model_id], 
					curr_pair.o2.Tr->R, curr_pair.o2.Tr->T,
					curr_pair.o2.pqp[model_id],
					curr_pair.delta);
      num_tri_tests += mpkCollDistAlgo::dist_res.num_tri_tests;
      num_bv_tests  += mpkCollDistAlgo::dist_res.num_bv_tests;
    }
    
    if ( dist <= 0 ) {
      coll_idx = i;
      return 0;
    }
    if ( dist < min_sep )
      min_sep = dist;
    if ( store_dist )
      lbdist[i] = dist;

  }

  return min_sep;
}



#ifndef MPK_COLL_PAIR_SET_H
#define MPK_COLL_PAIR_SET_H

#include <vector>

#include "mpkCollPair.H"
#include "mpkCollDistAlgo.H"
#include "mpkRobotCollection.H"
#include "mpkObstacleCollection.H"

/** @memo Represents a collection of candidate rigid body pairs (robot
    link, obstacle) and (robot link, robot link) that can be passed to
    a collision checker class.  In particular, the public member {\bf
    test_pairs} can be passed to
    \URL[SimplePointChecker]{SimplePointChecker.html},
    \URL[SimplePointChecker]{AdaptPointChecker.html} or
    \URL[SimplePointChecker]{mpkSimpleSegmentChecker.html}.  mpkRobot links
    that do not move, because they have no parameter dependencies, are
    not paired with any (static) obstacles.

    @see SimplePointChecker
    @see AdaptPointChecker
    @see mpkAdaptSegmentChecker
*/
class mpkCollPairSet {
  
public:

  /** @param 1 {\bf robots} must point to a properly
      initialized robot collection.

      @param 2 {\bf obstacles} similar to robots.

      @param 3 {\bf delta} is the minimum workspace clearance (the
      same for all pairs of objects).

      @see mpkCollPair
  */
  mpkCollPairSet(mpkRobotCollection* robots, mpkObstacleCollection* obstacles,
		 double delta=0);

  /// @doc Outputs the candidate rigid object pairs to f (cout by default).
  void print(ostream* f=&cout);

  /// @memo The vector of collision candidate pairs constructed by this class.
  vector<mpkCollPair> test_pairs;

  // number of pairs that were pruned because they cannot possibly collide
  int num_obst_pruned;
  int num_rob_pruned;

private:
  
  bool prune_pair(mpkBaseRobot* rob1, int joint_idx1,
		  mpkBaseRobot* rob2, int joint_idx2,
		  double delta);
  bool prune_pair(mpkBaseRobot* rob1, int joint_idx1,
		  PQP_Model* obst, mpkTransform& Tr,
		  double delta);

};


inline
bool
mpkCollPairSet::
prune_pair(mpkBaseRobot* rob1, int joint_idx1,
	   mpkBaseRobot* rob2, int joint_idx2,
	   double delta)
{
  // if the base spheres were not computed we cannot prune this
  // pair from collision checking
  if ( !rob1->joints[joint_idx1].bounds_available || 
       !rob2->joints[joint_idx2].bounds_available )
    return false;

  double c1[3], c2[3];
  double r1 = rob1->joints[joint_idx1].base_sphere_r;
  double r2 = rob2->joints[joint_idx2].base_sphere_r;
  rob1->base_T.apply(c1, rob1->joints[joint_idx1].base_sphere_c);
  rob2->base_T.apply(c2, rob2->joints[joint_idx2].base_sphere_c);
  bool inters = mpkCollDistAlgo::SphereDist(c1,c2,r1,r2) <= delta;
  if ( !inters ) num_rob_pruned++;
  return !inters;
}

inline
bool
mpkCollPairSet::
prune_pair(mpkBaseRobot* rob1, int joint_idx1,
	   PQP_Model* obst, mpkTransform& Tr,
	   double delta)
{
  // if the base spheres were not computed we cannot prune this
  // pair from collision checking
  if ( !rob1->joints[joint_idx1].bounds_available )
    return false;

  double c1[3];
  double r1 = rob1->joints[joint_idx1].base_sphere_r;
  rob1->base_T.apply(c1, rob1->joints[joint_idx1].base_sphere_c);

  bool inters=(mpkCollDistAlgo::GreedyDistance(c1,r1,Tr.R,Tr.T,obst,delta)<=0);
  if ( !inters ) num_obst_pruned++;
  return !inters;
}

inline 
mpkCollPairSet::
mpkCollPairSet(mpkRobotCollection* robots, mpkObstacleCollection* obstacles,
	       double delta)
{
  test_pairs.clear();
  num_rob_pruned=0;
  num_obst_pruned=0;
  
  // 1) add pairs for all robot-obstacle collisions
  //    and collisions with other robots
  for ( int i=0; i<robots->num_robots(); i++ ) {
    mpkBaseRobot* robi = robots->rob[i];

    for ( int j=robi->num_joints-1; j >= 0; j-- )

      // if there is a pqp_model and the check_coll flag is set, then:
      if (robi->joints[j].pqp_model[0] &&
	  robi->joints[j].check_coll) {

	// test link j of robot i against:
	// A) obstacles
	if ( robi->joints[j].can_move )
	  for ( int k=0; k<obstacles->num_obstacles(); k++ ) {
	    if ( !prune_pair(robi, j,
			     obstacles->pqp[k], obstacles->Tr[k], delta) )
	      test_pairs.push_back(mpkCollPair(robi,
					       robots->first_param_idx(i), j,
					       obstacles->pqp[k],
					       &obstacles->Tr[k],
					       obstacles->names[k],
					       delta));
	    else {
	      cerr << robi->get_name() << ":"
		   << robi->joints[j].name << " - "
		   << obstacles->names[k]
		   << " pruned" << endl;
	    }
	  }
	
	// B) designated links of other robots
	for ( int k=0; k<i; k++ ) {
	  mpkBaseRobot* robk = robots->rob[k];
	  for ( int l=robk->num_joints-1; l >= 0; l-- )
	    if ( robk->joints[l].pqp_model[0] &&
		 robk->joints[l].check_coll &&
		 (robi->joints[j].can_move ||
		  robk->joints[l].can_move) )
	      if (!prune_pair(robi, j, robk, l, delta))
		test_pairs.push_back(mpkCollPair(robi,
						 robots->first_param_idx(i),
						 j,
						 robk,
						 robots->first_param_idx(k),
						 l,
						 delta));
	      else {
		cerr << robi->get_name() << ":"
		     << robi->joints[j].name << " - "
		     << robk->get_name() << ":"
		     << robk->joints[l].name
		     << " pruned" << endl;
	      }
	}
      }
  }
  
  // 2) add pairs for self-collisions (if required)
  for ( int i=0; i<robots->num_robots(); i++ ) {
    mpkBaseRobot* robi = robots->rob[i];

    // if not self collision info is given, ignore self-collisions
    if ( robi->selfcoll_info.size() == 0 ) continue;

    for ( int p=0; p < robi->selfcoll_info.size(); p++ ) {
      int j = robi->selfcoll_info[p].joint1;
      int k = robi->selfcoll_info[p].joint2;
      if ( robi->joints[j].pqp_model[0] &&
	   robi->joints[k].pqp_model[0] ) {
	test_pairs.push_back(mpkCollPair(robi,
					 robots->first_param_idx(i),
					 j,
					 robi,
					 robots->first_param_idx(i),
					 k,
					 delta));
      }
    }
  }
}


inline
void
mpkCollPairSet::print(ostream* f)
{
  for ( int i=0; i<test_pairs.size(); i++ ) {
    (*f) << "  " << test_pairs[i].o1.name
	 << " - " << test_pairs[i].o2.name << endl;
  }
  (*f) << "mpkCollPairSet consists of the above " << test_pairs.size()
       << " pairs (after pruning of "
       << num_obst_pruned << " (robot-link,obstacle) pairs and "
       << num_rob_pruned  << " (robot-link,robot-link) pairs)" << endl;
}

#endif

#ifndef MPK_CONFIG_CHECKER_H
#define MPK_CONFIG_CHECKER_H

#include <queue>
#include "mpkConfig.H"
#include "mpkCollPair.H"
#include "mpkRobotCollection.H"
#include "mpkCollDistAlgo.H"

///
class mpkConfigChecker {

public:

  ///
  mpkConfigChecker(const vector<mpkCollPair>* test_pairs,
		   mpkRobotCollection* robots);
  
  /// tells if the configuration q is colliding
  bool collision(const mpkConfig* q, mpk_idx model_id=0);

  /** returns lower bound on clearance (0 if colliding or separation <
      delta for some pair) */
  double clearance(const mpkConfig* q, mpk_idx model_id=0,
		   bool store_dist=true);

  /** the index (in test_pairs) of the colliding (or too
      close) pair (undef if no collision). */
  mpk_idx collpair_idx() {return coll_idx;};

  static int num_tri_tests;
  static int num_bv_tests;
  static int num_obj_tests;

private:
  
  // general information
  mpkRobotCollection* robots;
  const vector<mpkCollPair>* test_pairs;

  // status corresponding to most recent operation
  mpk_idx model_id;
  const mpkConfig* q;
  vector<double> lbdist; // lower distance bounds (after clearance() only)
  double min_sep;  // minimum of lbdist (after clearance() only)
  mpk_idx coll_idx; // only valid if collision / too close

  //
  friend class mpkSimpleSegmentChecker;
  friend class mpkDiscrSegmentChecker;
  friend class mpkAdaptSegmentChecker;

};

#endif

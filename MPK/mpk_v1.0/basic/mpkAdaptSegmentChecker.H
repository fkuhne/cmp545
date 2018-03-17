#ifndef MPK_ADAPT_SEGMENT_CHECKER_H
#define MPK_ADAPT_SEGMENT_CHECKER_H

#include <queue>
#include "mpkConfigChecker.H"
#ifdef TRANSFORM_CACHE
#include "mpkTransformCache.H"
#endif

/**@memo Adaptive collision checker for straight line segments in
   c-space.
   
   @doc This checker is guaranteed not to miss collisions.  It relates
   workspace distance to bounds on the motions of objects and uses a
   collision-free predicate to adapt the local resolution at which the
   segment is checked.

   The checker is 'lazy' in the sense that it does not finish the
   segment checking in one step.  Instead, it has a function
   iteration_step() that performs the next scheduled step each time it
   is called.  This is useful for so-called 'lazy' collision checking
   PRM planners such as SBL.  If lazy collision checking is not
   desired, simply call iteration_step() in a loop until it returns
   false and then query the result using collision().  Note that in
   general, the return value of collision() is not meaningful until
   iteration_step() has returned false.
   
   Using the transform cache is generally recommended.  Disable it
   only if you encounter storage problems.  In a lazy collision
   checking planner, many adaptive segment checkers might be active at
   the same time and thus the memory consumption by the internal
   priority queues might be very high.  If this creates problems, the
   size of the internal priority queues can be limited by calling
   iteration_step() multiple times until queue_size() returns a value
   below a desired limit.
 */
class mpkAdaptSegmentChecker {

public:

  /**@memo The recommended constructor.  It takes pointers to two
     configuration checkers that certify that refer to the endpoints
     of the segment and certify that these are free (be sure to call
     clearance() on both configuration checkers before passing them to
     this contructor).  The segment checker accesses the
     configurations by following the pointers to the configuration
     checkers.  Thus the configuration checkers cannot be used for
     testing different configurations while the segment checker is
     active.  Instead, create a new configuration checker for each
     configuration.  Transform caching should be disabled only if
     memory consumption is too high otherwise.
   */
  mpkAdaptSegmentChecker(const mpkConfigChecker* ccheck0,
			 const mpkConfigChecker* ccheck1,
			 bool use_transf_cache=true);

  /**@memo This constructor accepts two configurations directly.
     However, it is not as efficient as the other constructor and
     therefore not recommended if speed is an issue.  Transform
     caching should be disabled only if memory consumption is too high
     otherwise.
   */
  mpkAdaptSegmentChecker(const mpkConfigChecker* ccheck,
			 const mpkConfig* q0, const mpkConfig* q1,
			 bool use_transf_cache=true);
  
  ~mpkAdaptSegmentChecker() {clear_mem();};

  /**@memo Returns the size of the internal priority queue.*/
  int queue_size() {return seg_queue.size();};

  /**@memo Returns the priority of the first subsegment test in the
      internal priority queue.*/
  double prio() {return (seg_queue.size())? seg_queue.top()->prio():-1;};

  /**@memo Performs one iteration step of (lazy) segment checking and
      returns false if the status of the segment has been determined
      (no more steps necessary) or true if more iteration steps are
      necessary.  This function may increase or decrease the size of
      the internal priority queue.
   */
  bool iteration_step();

  /**@memo Returns true as long as it is not clear whether the segment
      is free (more calls of iteration_step() necessary) and false if
      it the segment has been found to be free */
  bool collision() { assert(done); return coll_status; };

#ifndef SAVE_MEMORY

  ///@memo number of BV pairs tested so far.
  int num_bv_tests;

  ///@memo number of triangle pairs tested so far.
  int num_tri_tests;

  ///@memo number of distance computations for object pairs so far.
  int num_pairs_dist_comp;

  /**@memo if a collision was found, this value parameterizes (in
     [0,1]) the collision point on the segment.*/
  double coll_t;

#endif


private:

  void clear_mem();

  // initialization (used by both constructors)
  void init(const mpkRobotCollection* robots,
	    const vector<double>* lbdist0,
	    const vector<double>* lbdist1);

  // these pointers refer to the locations passed in the constructors
  const vector<mpkCollPair>* test_pairs;
  const mpkConfig* q0;
  const mpkConfig* q1;
  
  // which model (0 or 1) is checked.
  mpk_idx model_id;

  bool coll_status; // collision status (true: we assume collision, false: free)
  bool done; // true if no more calls to iteration_step() are necessary

  // The checker maintains a priority queue of pair-specific
  // subsegments for which the collision-free predicate has failed.
  // Further processing (splitting) is thus required and will either
  // reveal a collision or the collision-free predicate will
  // eventually hold for all subsegments.
  struct testpair_subseg {

    mpk_idx pair_idx;     // index into test_pairs
    unsigned t0, t1;      // parameter values of endpoints of path
		          // section (discretization to facilitate
		          // caching)
    double lbdist0;       // lower bound on workspace distance (at t0)
    double lbdist1;       // lower bound on workspace distance (at t1)
    double pathlen_bound; // upper bound on pathlength (sum of both if
			  // both bodies moving)
    // prio(): priority value that determines position of testpair
    // subsegment in priority queue.  Returns <= 0 if and only if the
    // segment is covered and higher positive values if segment is
    // more likely to contain a collision
    double prio() const {
      return pathlen_bound - lbdist0 - lbdist1;
    }

  }; // end struct testpair_subseg

  friend struct testpair_subseg;

  // for ordering the priority queue
  struct cmp_testpair_subseg_prio {
    int operator()( const testpair_subseg* x, const testpair_subseg* y )
    {
      return x->prio() < y->prio();
    }
  };

  // The priority queue of pair-specific subsegments
  priority_queue<testpair_subseg*, vector<testpair_subseg*>,
  		 cmp_testpair_subseg_prio> seg_queue;

  // Cache to store and re-use transforms computed during the segment
  // checking.  It helps avoiding redundant computations, especially
  // for long kinematic chains and multi-robot systems.
#ifdef TRANSFORM_CACHE
  mpkTransformCache* transf_cache;
#endif

};

#endif

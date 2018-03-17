#ifndef MPK_COLL_PAIR_H
#define MPK_COLL_PAIR_H

#include "mpkConfig.H"
#include "mpkBaseRobot.H"

#ifdef TRANSFORM_CACHE
#include "mpkTransformCache.H"
#endif

/**@memo Represents a candidate pair of rigid objects to be tested for
   collisions or for which a minimum workspace clearance is desired.
   A list or vector of all pairs that could potentially collide
   defines the necessary collision checks for the entire scenario (see
   mpkCollPairSet).

   @doc To distinguish robot links in a scenario with multiple
   identical robots, the object identifiers (names) created by this
   class concatenate the robot name and the link name to get a unique
   name for each link.

   @see mpkCollPairSet
 */
class mpkCollPair {

public:

  /**@doc Creates a collision test pair (robot link, obstacle).  The
     robot and obstacle information can be taken from mpkRobotCollection
     and mpkObstacleCollection, respectively.
     @param rob1 pointer to robot
     @param first_param_idx1 position of first parameter of rob in the
     c-space (see {@link first_param_idx first_param_idx} in
     {@link mpkRobotCollection mpkRobotCollection}).
     Equals 0 if only one robot.
     @param joint_idx1 index of the joint with which the candidate link
     is associated
     @param obst_pqp PQP model of the obstacle
     @param obst_transf mpkTransform of the obstacle
     @param obst_name Name of the obstacle
     @param delta Minimum workspace clearance
     @see mpkRobotCollection
     @see mpkObstacleCollection
   */
  mpkCollPair(mpkBaseRobot* rob1, int first_param_idx1, int joint_idx1,
	      PQP_Model* obst_pqp, mpkTransform* obst_transf,
	      const char obst_name[], double delta = 1e-3);
  
  /**@doc Creates a collision test pair (robot link, robot link) from
     one or two different robots.  For notes on parameters, see above
     constructor for pair (robot link, obstacle).
  */
  mpkCollPair(mpkBaseRobot* rob1, int first_param_idx1, int joint_idx1,
	      mpkBaseRobot* rob2, int first_param_idx2, int joint_idx2,
	      double delta = 1e-3);
  
  mpkCollPair(const mpkCollPair& x);
  
  ~mpkCollPair() { delete [] o1.name; delete [] o2.name;};
  
  
  /**@doc Computes the transforms for both objects (or for the first
     object only if the second object is a fixed obstacle).  Note that
     this modifies the robot transforms in the robot(s) that were
     passed in the constructor.
  */
  void compute_transf(const mpkConfig& q) const;
#ifdef TRANSFORM_CACHE
  void compute_transf(const mpkConfig& q, unsigned t, mpkTransformCache* cache) const;
#endif
  
  /**@memo Structure storing information about a single collision
     candidate object (rigid body).
   */
  struct coll_obj {
    mpkBaseRobot* rob;   // ==0 if obstacle
    int first_param_idx; // first parameter of this robot in c-vector
			 // (only valid if robot link)
    int joint_idx;       // index of joint (only valid if rob != 0)
    PQP_Model* pqp[2];   // pointer to the coll model(s)
    mpkTransform* Tr;    // pointer to the transform
    char* name;          // name (including obstacle/robot name)
    bool operator==(const coll_obj& x) const {
      return pqp[0] == x.pqp[0] && pqp[1] == x.pqp[1] && Tr == x.Tr;
    };
  };
  
  ///@memo Information about the first candidate object of the pair.
  coll_obj o1;
  ///@memo Information about the other candidate object of the pair.
  coll_obj o2;

  ///@memo Minimum workspace clearance (delta) for this pair.
  double delta;

  // index of first common static reference frame
  // equals -1 (world coordinate system) if link+static obstacle pair
  // or two links on different robots
  // otherwise equals index of common parent joint on same robot
  int root_idx;

};

#endif

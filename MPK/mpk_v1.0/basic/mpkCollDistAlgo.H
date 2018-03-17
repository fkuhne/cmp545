#ifndef MPK_COLL_DIST_ALGO_H
#define MPK_COLL_DIST_ALGO_H

#include "mpk_defs.h"
#include "PQP.h"

/**@memo Collision and distance computation algorithms.
 */
class mpkCollDistAlgo {

public:

  //@memo Returns the distance between two spheres.
  static
  double SphereDist(double c1[3], double c2[3],
		    double r1=0, double r2=0);

  /**@memo Determines whether two models are colliding.*/
  static
  bool Collision(double R1[3][3], double T1[3], PQP_Model *o1,
		 double R2[3][3], double T2[3], PQP_Model *o2);

  /**@memo Computes a simple but fast lower bound on the distance
     between two models.*/
  static
  double GreedyDistance(double R1[3][3], double T1[3], PQP_Model *o1,
			double R2[3][3], double T2[3], PQP_Model *o2,
			double delta);

  /**@memo Computes a simple but fast lower bound on the distance
     between a sphere and a model.*/
  static
  double GreedyDistance(double center[3], double radius,
			double R2[3][3], double T2[3], PQP_Model *o2,
			double delta);

  /**@memo Returns the (approximate) distance between two models. Note
    that the original version of PQP returns an upper bound on the
    distance when a bounded error is allowed.  However, in our
    adaptive segment checker, we need a lower bound.  In principle,
    PQP's approximate distance algorithm can be modified to return a
    lower bound, too.  Still, approximate distance computation has
    been found to be slower than greedy distance computation.
  */
  static
  double Distance(double R1[3][3], double T1[3], PQP_Model *o1,
		  double R2[3][3], double T2[3], PQP_Model *o2,
		  double rel_err=0.0, double abs_err=0.0);

  static PQP_CollideResult   coll_res;
  static PQP_DistanceResult  dist_res;
  static PQP_ToleranceResult tol_res;

private:

  // recursions of GreedyDistance
  static
  double
  GreedyDistanceRecurse(double R[3][3], double T[3], // b2 relative to b1
			PQP_Model *o1, int b1,
			PQP_Model *o2, int b2,
			double delta);

  static
  double
  GreedyDistanceRecurse(double R[3][3], double T[3], // b2 relative to sphere
			BV* sphere,
			PQP_Model *o2, int b2,
			double delta);

};

#endif

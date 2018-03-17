#ifndef MPK_PATH_SMOOTHER_H
#define MPK_PATH_SMOOTHER_H

#include <list>
#include "mpkConfig.H"
#include "mpkCollPair.H"

#ifdef ADAPT_COLLCHECKER
#include "mpkAdaptSegmentChecker.H"
#else
#include "mpkSimpleSegmentChecker.H"
#endif

/**@memo Simple path smoother that iteratively picks a random pair of
   points on the path and then tries to shortcut the section between
   the two points by a straight line segment.  If compiled with {\bf
   ADAPT_COLLCHECKER} defined, it uses
   {@link mpkAdaptSegmentChecker mpkAdaptSegmentChecker} and otherwise it
   uses {@link mpkSimpleSegmentChecker mpkSimpleSegmentChecker}.
*/
class mpkPathSmoother {

public:

  /**@doc Takes a path (to be shortened) and an array of pairs to test
  for collisions.  The parameter {\bf min_shortcut_len} is the minimum
  length (relative to the length of {\bv path}) of a shortcut to be
  accepted (can be used to avoid too fine approximations).  It further
  requires a pointer {\bf robots} to the corresponding robot
  collection.  The c-space resolution {\bf epsilon} is only used if
  compiled for the simple segment checker. */
  mpkPathSmoother(const vector<mpkConfig>& path,
		  const vector<mpkCollPair>* test_pairs,
		  mpkRobotCollection* robots,
		  double min_shortcut_len=1e-3, double epsilon=0);

  ~mpkPathSmoother();

  ///@doc Perform nsteps of smoothing
  void smoothe(int nsteps=10);

  ///@doc Returns the number of segments in current internal copy of the path
  int num_segs() {return num_points-1;};

  ///@doc Returns length of current internal copy of the path
  double length() {return len;};

  ///@doc To get a copy of the internal path
  void get_path(vector<mpkConfig>& smoothed_path);
  ///@doc To get a copy of the internal path
  void get_path(list<mpkConfig>& smoothed_path);

private:

  // Internally, the path is copied to a simple linked list that is
  // used during smoothing.  This is the node structure for this list.
  struct node {
    mpkConfig q;
    node* next;
  };

  // returns the first (left) node of the segment with index idx.
  node* seg_node(int seg_idx);

  // returns the point on the path that corresponds to t (the
  // parameter of the path in [0,1]).  t==0 is the starting point and
  // t==1 is the endpoint of the path.
  node* lin_interpol(double t, mpkConfig& q);

  // replaces the path subsection between the two nodes by the
  // straight line segment between q1 and q2.
  void replace_section(node* seg_node1, node* seg_node2,
		       const mpkConfig& q1, const mpkConfig& q2);

  int num_points; // number of points in path
  double len; // length of path (in c-space)
  double min_shortcut_len;  // minimum rel. length of shortcut segment
  node* first; // points to first node in internal list for path
  mpkConfig p1, p2; // two candidate points

  const vector<mpkCollPair>* test_pairs; // pointer to collision test pairs
  mpkRobotCollection* robots;

  // epsilon used with mpkSimpleSegmentChecker only
  double epsilon;

};

#endif


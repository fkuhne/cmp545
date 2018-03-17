#ifndef MPK_TRACE_VIS_H
#define MPK_TRACE_VIS_H

#include "mpk_inventor.h"
#include "mpkRobotCollection.H"

/**@memo Class for representing and visualizing the curve traced by a
   tracePoint on a robot link.*/
class mpkTraceVis {

public:

  ///
  mpkTraceVis(mpkRobotCollection* robots, const vector<mpkConfig>& path,
	  double eps, const float* rgb=0);
  ///
  ~mpkTraceVis() {
#ifndef WIN32 // don't do this if Coin is used:
    root->unref();
    // Coin (Win port of inventor) obviously does not care about the
    // reference to the root that we explicitly established in the
    // mpkTraceVis constructor (root->ref()) and deletes the node root as
    // soon as it is detached from the scene graph.  As a consequence,
    // at this point, root is alrady an invalid pointer if compiling
    // with Coin (Win).
#endif
  };
 
  ///
  SoSeparator* ivroot() {return root;};
 
  ///
  bool is_shown()
  {return ivswitch->whichChild.getValue() != SO_SWITCH_NONE;};

  ///
  void set_show(bool on)
  {ivswitch->whichChild = (on)? SO_SWITCH_ALL : SO_SWITCH_NONE;};

private:

  ///
  struct point3 {///
double x,y,z;};

  ///
  SoSeparator* root;
  ///
  SoSwitch* ivswitch;

  ///
  vector<point3> vts;

  ///
  mpkRobotCollection* robots;
  ///
  double eps;

  ///
  void add_vertex(int rob_idx, int joint_idx, const mpkConfig& q);
  ///
  void add_sphere(int rob_idx, int joint_idx, const mpkConfig& q, bool seg_endp=false);
  ///
  void approx_seg(int rob_idx, int joint_idx,
		  const mpkConfig& q0, const mpkConfig& q1);

};
#endif


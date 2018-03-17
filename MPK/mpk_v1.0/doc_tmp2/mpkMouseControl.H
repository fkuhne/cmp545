#ifndef MPK_MOUSE_CONTROL_H
#define MPK_MOUSE_CONTROL_H

#include <vector>
#include "mpk_inventor.h"
#include "mpkRobotCollection.H"
#include "mpkObstacleCollection.H"

/**@memo Functions for realizing mouse interaction (picking links,
   controlling the configuration with a cartesian interface, etc.)*/

class mpkMouseControl {

public:

  ///
  static void init(SoGroup* scene_root,
		   mpkRobotCollection* robots, mpkObstacleCollection* obstacles,
		   void (*status_callback)()=0);
  
  ///
  static void myMouseButtonCB(void *userData, SoEventCallback *eventCB);

  ///
  static SoSeparator* ivroot() {return root;};

  ///
  static bool get_show_cross_cursor();
  ///
  static void set_show_cross_cursor(bool on);
  ///
  static void move_cross_cursor_1d(int ax, double incr);
  ///
  static void get_cross_cursor_pos(double& x, double& y, double& z);
  ///
  static void set_cross_cursor_pos(double x, double y, double z);

  ///
  static void update_rubber_line();
  ///
  static void move_robot_point_to(const double pgoal[3]);

  ///
  static PQP_Model* mouseptr;

  ///
  static double scene_size;
  ///
  static bool link_attached_to_cursor;
  ///
  static bool move_to_target_mode;
  ///
  static mpkBaseRobot* picked_rob;
  ///
  static int picked_rob_idx;
  ///
  static int picked_dof;
  ///
  static mpkBaseRobot::Joint* picked_joint;
  ///
  static int picked_joint_idx;

private:

  ///
  static mpkRobotCollection* robots;
  ///
  static mpkObstacleCollection* obstacles;

  ///
  static SoSeparator* root;

  ///
  static SoSwitch* cursor_switch;
  ///
  static SoTranslation* cursor_pos;

  ///
  static SoSwitch* rline_switch;
  ///
  static SoVertexProperty* rline_vts;

  ///
  static double point_on_robot[3];
  ///
  static double target_point[3];

  ///
  static vector<double> backup_params;
  ///
  static vector<bool> picked_rob_param_frozen;

  ///
  static void (*status_callback)();

  ///
  static double robot_target_dist(double p[]);
  ///
  static void deriv_robot_target_dist(double p[], double dp[]);

};


#endif


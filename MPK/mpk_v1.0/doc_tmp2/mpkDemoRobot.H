#ifndef MPK_DEMOROBOT_H
#define MPK_DEMOROBOT_H

#include "mpkBaseRobot.H"

///
class mpkDemoRobot : public mpkBaseRobot {

public:

  mpkDemoRobot(const char* name, const mpkTransform& base_transf,
	     double scalef=1.0)
    : mpkBaseRobot(name, num_joints, jointdefs, base_transf, scalef)
  {
    mpkBaseRobot::tracepoint_joint_idx = num_joints-1;
  };

  ~mpkDemoRobot();

private:

  static int num_joints;
  static mpkBaseRobot::JointDef jointdefs[];

  static void sample_func_init(mpkTransform& tr, const double* param);
  static void sample_func_transf(mpkTransform& tr, const double* param);

};

#endif

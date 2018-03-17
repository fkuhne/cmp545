#ifndef MPK_FREEFLYINGOBJECT_H
#define MPK_FREEFLYINGOBKECT_H

#include "mpkBaseRobot.H"

///
class mpkFreeFlyingObject : public mpkBaseRobot {

public:

  mpkFreeFlyingObject(const char* name, const mpkTransform& base_transf,
		double scalef=1.0)
    : mpkBaseRobot(name, num_joints, jointdefs, base_transf, scalef) {

    param_opts[3].is_cyclic = true;
    param_opts[4].is_cyclic = true;
    param_opts[5].is_cyclic = true;
    tracepoint_joint_idx = 6;
  };
  ~mpkFreeFlyingObject();

  static mpkBaseRobot::JointDef jointdefs[];

private:

  static int num_joints;

  static double bounds[6][2];

};

#endif

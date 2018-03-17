#include "mpkDemoRobot.H"

int mpkDemoRobot::num_joints = 2;

mpkBaseRobot::JointDef mpkDemoRobot::jointdefs[] =
{
  {
    "Link0",
    -1,  // father
    new mpkTransform(),
    new mpkTransl1(1,0,0,-1,1),
    0, // param idx
    "DemoRobot/l0.iv",
    "DemoRobot/l0.iv",
    true
  },
  {
    "Link1",
    0, // father
    new mpkTransform(),
    new mpkFuncTransf(1,
		      mpkDemoRobot::sample_func_transf,
		      mpkDemoRobot::sample_func_init),
    1, // first param idx
    "DemoRobot/l1.iv",
    "DemoRobot/l1.iv",
    false
  }
};


void mpkDemoRobot::sample_func_init(mpkTransform& tr, const double* param)
{
  tr.set_identity();
}

void mpkDemoRobot::sample_func_transf(mpkTransform& tr, const double* param)
{
  tr.T[1] = cos(4*M_PI*param[0]);
  tr.T[2] = 2*param[0];
}

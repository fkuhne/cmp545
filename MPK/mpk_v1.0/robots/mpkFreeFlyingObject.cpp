#include "mpkFreeFlyingObject.H"

int mpkFreeFlyingObject::num_joints = 7;

double mpkFreeFlyingObject::bounds[6][2] = {
  {-10, 10},
  {-10, 10},
  {-10, 10},
  {-3.142, 3.142},
  {-3.142, 3.142},
  {-3.142, 3.142}
};

mpkBaseRobot::JointDef mpkFreeFlyingObject::jointdefs[] =
{
  {
    "x-transl",
    -1,
    new mpkTransform(),
    new mpkTransl1(1,0,0, bounds[0][0], bounds[0][1]),
    0,
    0,
    0,
    false
  },
  {
    "y-transl",
    0,
    new mpkTransform(),
    new mpkTransl1(0,1,0, bounds[1][0], bounds[1][1]),
    1,
    0,
    0,
    false
  },
  {
    "z-transl",
    1,
    new mpkTransform(),
    new mpkTransl1(0,0,1, bounds[2][0], bounds[2][1]),
    2,
    0,
    0,
    false
  },
  {
    "x-rot",
    2,
    new mpkTransform(),
    new mpkRot1(1,0,0, bounds[3][0], bounds[3][1]),
    3,
    0,
    0,
    false
  },
  {
    "y-rot",
    3,
    new mpkTransform(),
    new mpkRot1(0,1,0, bounds[4][0], bounds[4][1]),
    4,
    0,
    0,
    false
  },
  {
    "z-rot",
    4,
    new mpkTransform(),
    new mpkRot1(0,0,1, bounds[5][0], bounds[5][1]),
    5,
    "FreeFlyingObject/object.iv",
    "FreeFlyingObject/object.iv",
    true
  },
  {
    "TracePoint",
    5,
    new mpkTransform(),    
    new mpkTransform(),    
    -1,
    0,
    0,
    false
  }
};

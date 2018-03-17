#include "mpkRobot.H"
#include "mpk_robot_type.h"

SO_NODE_SOURCE(mpkRobot);

void
mpkRobot::initClass()
{
  SO_NODE_INIT_CLASS(mpkRobot, SoSeparator, "Separator");
}

mpkRobot::mpkRobot()
{
  SO_NODE_CONSTRUCTOR(mpkRobot);
  
  SO_NODE_ADD_FIELD(robotType,       	(GENERIC));
  SO_NODE_ADD_FIELD(translation,	(0, 0, 0));
  SO_NODE_ADD_FIELD(rotation,		(1, 0, 0, 0));
  SO_NODE_ADD_FIELD(scaleFactor,	(1));
  SO_NODE_ADD_FIELD(fileName,	        (""));
  SO_NODE_ADD_FIELD(fileName2,	        (""));
  
  SO_NODE_DEFINE_ENUM_VALUE(mpk_robot_type, GENERIC);
  SO_NODE_DEFINE_ENUM_VALUE(mpk_robot_type, FREEFLYINGOBJECT);
  SO_NODE_DEFINE_ENUM_VALUE(mpk_robot_type, DEMOROBOT);
  
  SO_NODE_SET_SF_ENUM_TYPE(robotType, mpk_robot_type);
}

void mpkRobot::init()
{
  const SbVec3f& Tv = translation.getValue();
  const SbVec4f& Rv = rotation.getValue();
  mpkTransform base_transf = mpkConstTransl(Tv[0], Tv[1], Tv[2]);
  if ( Rv[0] || Rv[1] || Rv[2] )
    base_transf.append(mpkConstRot(Rv[0], Rv[1], Rv[2], Rv[3]));
  switch ( robotType.getValue() ) {
  case GENERIC:
    robot = new mpkBaseRobot(getName().getString(),fileName.getValue().getString(),
			     base_transf,scaleFactor.getValue());
    break;
  case FREEFLYINGOBJECT:
    //mpkFreeFlyingObject::jointdefs[5].iv_fname0
      //= fileName.getValue().getString();
    //mpkFreeFlyingObject::jointdefs[5].iv_fname1
      //= fileName2.getValue().getString();
    robot = new mpkFreeFlyingObject(getName().getString(),base_transf,
				    scaleFactor.getValue());
    break;
  case DEMOROBOT:
    robot = new mpkDemoRobot(getName().getString(),
			     base_transf,
			     scaleFactor.getValue());
    break;
  }

  addChild(robot->iv_graph);

}



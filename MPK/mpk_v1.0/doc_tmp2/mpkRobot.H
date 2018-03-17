#ifndef MPK_ROBOT_H
#define MPK_ROBOT_H

#include "mpk_inventor.h"

#include "mpkBaseRobot.H"


/**@memo Extended Inventor node type (derived from SoSeparator) to
   define robot nodes in the scene file.  

   @doc mpkRobots are found in the scene graph using an Inventor search
   action that looks for nodes of type mpkRobot (see
   {@link mpkRobotCollection mpkRobotCollection}).  Each robot can be given
   an individual name.  The following shows how to defined a robot in
   a scene file:\\ \\

   {\tt DEF <name> mpkRobot {\\
     robotType <type>\\
     translation <x> <y> <z>\\
     rotation <ax_x> <ax_y> <ax_z> <angle>\\
     scaleFactor <fact>\\
   }}\\ \\

   where <type> is one of the constants defined in file
   \URL[mpk_robot_type.h]{../../basic/mpk_robot_type.h}
 */
class mpkRobot : public SoSeparator {
  
  SO_NODE_HEADER(mpkRobot);
  
 public:
    
  ///@memo see \URL[mpk_robot_type.h]{../../basic/mpk_robot_type.h}
  SoSFEnum    robotType;
  ///
  SoSFVec3f   translation;
  ///
  SoSFVec4f   rotation;
  ///
  SoSFFloat   scaleFactor;
  ///
  SoSFString  fileName; // model name for FreeFlyingObject/RobotFile
  ///
  SoSFString  fileName2; // alternative model name for FreeFlyingObject
  
  static void initClass();

  ///
  mpkRobot();

  /**@memo Reads the link models and builds the kinematic structure of
     the robot according to the value in {\bf robotType}.  After
     calling {\bf init()}, the pointer {\bf robot} points to the
     instantiated robot.)
  */
  void init();

  ///@memo Pointer to robot instantiated by {\bf init()}.
  mpkBaseRobot* robot;

private:

  virtual ~mpkRobot() {delete robot;};
    
};

#endif

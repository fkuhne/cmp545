#ifndef MPK_OBSTACLE_H
#define MPK_OBSTACLE_H

#include "mpk_inventor.h"

/**@memo Extended Inventor node type (derived from SoSeparator) to
   define obstacle nodes in the scene file.

   @doc mpkObstacles are found in the scene graph using an Inventor
   search action that looks for nodes of type mpkObstacle (see
   {@link mpkObstacleCollection mpkObstacleCollection}).  For each obstacle,
   an individual name can be given.  The following example shows how
   obstacles are defined in a scene file:\\ \\

   {\tt DEF testObst mpkObstacle {\\
     DEF __triangulate__ File { name "obj1.iv" }\\
     mpkTransform { translation 1 0 0 }\\
     DEF __triangulate__ File { name "obj2.iv" }\\
   }}\\ \\

   This defines an obstacle named "testObst" whose triangles are read
   from the files "obj1.iv" and "obj2.iv".  Since mpkObstacle is derived
   from SoSeparator, its internal structure in the file (its children
   in the scene graph) is quite flexible.  Children of an mpkObstacle
   node whose triangles should be added to the collision model have to
   be named with a {\tt __triangulate__} tag.

   @see mpkObstacleCollection
 */
class mpkObstacle : public SoSeparator {
  
  SO_NODE_HEADER(mpkObstacle);
  
public:

  static void		initClass();
  
  ///
  mpkObstacle();
  
private:
  virtual ~mpkObstacle() {};
  
};

#endif

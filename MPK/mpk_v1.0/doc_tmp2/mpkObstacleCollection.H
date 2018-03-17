#ifndef MPK_OBSTACLE_COLLECTION_H
#define MPK_OBSTACLE_COLLECTION_H

#include <vector>
#include "PQP.h"
#include "mpkTransform.H"

using namespace std;

/**@memo Extracts all obstacles (nodes of type {@link mpkObstacle mpkObstacle})
   from an Inventor scene graph and builds PQP collision models for
   them.
   
   @see mpkObstacle
 */
class mpkObstacleCollection {

public:

  /**@memo {\bf scenegraph} is the inventor scene graph which will be
     parsed for mpkObstacle nodes.
  */
  mpkObstacleCollection(SoGroup* scenegraph);

  ~mpkObstacleCollection();

  ///@memo Returns the number of obstacles
  int num_obstacles() {return pqp.size();};

  ///@memo PQP models of all obstacles.
  vector<PQP_Model*> pqp;

  ///@memo Corresponding transforms of all obstacles.
  vector<mpkTransform> Tr;

  /**@memo Corresponding names of all obstacles (may be prepended by a
     file name if the obstacle was read from an included file).
  */
  vector<const char*> names;


private:

  // information for triangle callback
  struct tri_info {
    tri_info(PQP_Model* model, const mpkTransform& transf,
	     const SbVec3f& scaleFactor, int cnt=0) {
      pqp_model = model; Tr = transf; scalef = scaleFactor; tri_cnt=cnt;
    };
    PQP_Model* pqp_model;
    mpkTransform Tr;
    SbVec3f scalef;
    int tri_cnt;
  };

  // the triangulation callback (to get the triangles from the
  // inventor nodes into the PQP models).  data will be passed as a
  // pointer to struct tri_info (see above).
  static void triang_CB(void *data, SoCallbackAction *,
			const SoPrimitiveVertex *vertex1,
			const SoPrimitiveVertex *vertex2,
			const SoPrimitiveVertex *vertex3);
};

#endif


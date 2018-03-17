#include "mpkObstacleCollection.H"
#include "mpkObstacle.H"

mpkObstacleCollection::mpkObstacleCollection(SoGroup* scenegraph)
{
  // setup search action for mpkObstacles
  SoSearchAction searchobst;
  searchobst.setType(mpkObstacle::getClassTypeId());
  searchobst.setInterest(SoSearchAction::ALL);
  searchobst.setSearchingAll(TRUE);
  searchobst.apply(scenegraph);
  SoPathList obstpathlist = searchobst.getPaths();

  // process mpkObstacles
  for ( int k=0; k < obstpathlist.getLength(); k++ ) {

    SoNode *obstnode = obstpathlist[k]->getNodeFromTail(0);

    SbVec3f translation, obstScale, collScale;
    SbRotation rotation, scaleOrientation;

    // get transform of obstacle
    SbViewportRegion dummy;
    SoGetMatrixAction get_matrix(dummy);
    get_matrix.apply(obstpathlist[k]);
    SbMatrix matrix = get_matrix.getMatrix();
    matrix.getTransform(translation, rotation, obstScale, scaleOrientation);    
    double tx = translation.getValue()[0];
    double ty = translation.getValue()[1];
    double tz = translation.getValue()[2];

    mpkTransform obst_transf(rotation, SbVec3f(tx,ty,tz));
    Tr.push_back(obst_transf);

    // store name of obstacle
    names.push_back(obstnode->getName().getString());

    // find nodes named "__triangulate__" in mpkObstacle (obstnode) and triangulate them
    SoSearchAction searchcoll;
    searchcoll.setName("__triangulate__");
    searchcoll.setInterest(SoSearchAction::ALL);
    searchcoll.setSearchingAll(TRUE);
    searchcoll.apply(obstnode);
    SoPathList collpathlist = searchcoll.getPaths();

    if ( collpathlist.getLength() == 0 ) {
      cerr << "Error: mpkObstacle " << obstnode->getName().getString()
	   << " has no nodes declared for triangulation by" << endl
	   << "Hint: use \"DEF __triangulate__ <node-type>\"" << endl;
#ifdef WIN32
      getchar();
#endif
      exit(1);
    }
    else {

      PQP_Model* pqp_model = new PQP_Model;
      pqp_model->BeginModel();
      
      int obst_num_tri=0;
      for ( int i=0; i < collpathlist.getLength(); i++ ) {
	
	get_matrix.apply(collpathlist[i]);
	matrix = get_matrix.getMatrix();
	matrix.getTransform(translation, rotation, collScale, scaleOrientation);    
	double tx = translation.getValue()[0]*obstScale[0];
	double ty = translation.getValue()[1]*obstScale[1];
	double tz = translation.getValue()[2]*obstScale[2]; 
	mpkTransform coll_transf(rotation, SbVec3f(tx,ty,tz));
	
	SoCallbackAction triAction;
	
	collScale[0] *= obstScale[0];
	collScale[1] *= obstScale[1];
	collScale[2] *= obstScale[2];
	
	tri_info info(pqp_model,coll_transf,collScale);
	triAction.addTriangleCallback(SoNode::getClassTypeId(),
				      triang_CB, (void*)&info);
	triAction.apply(collpathlist[i]);
	obst_num_tri += info.tri_cnt;
      }
      pqp_model->EndModel();
      
      pqp.push_back(pqp_model);
      cerr << "  mpkObstacle with " << obst_num_tri << " triangles." << endl;
    }
  }
}


mpkObstacleCollection::~mpkObstacleCollection()
{
  for ( int i=0; i<pqp.size(); i++ )
    delete pqp[i];
}

void mpkObstacleCollection::triang_CB(void *data, SoCallbackAction *,
			    const SoPrimitiveVertex *vertex1,
			    const SoPrimitiveVertex *vertex2,
			    const SoPrimitiveVertex *vertex3)
{
  tri_info* info = (tri_info*)data;
  if ( !info->pqp_model ) return;

  SbVec3f points[] =
    { vertex1->getPoint(), vertex2->getPoint(), vertex3->getPoint() };
  double verts[3][3];

  for (int i = 0; i < 3; i++) {
    double vert[3];
    for (int j = 0; j < 3; j++) {
      vert[j] = points[i][j] * info->scalef[j];
      //cout << points[i][j] << " ";
    }
    info->Tr.apply(verts[i], vert);
    //cout << " * ";
  }
  info->pqp_model->AddTri(verts[0], verts[1], verts[2], info->tri_cnt++);

  //cout << endl;
}

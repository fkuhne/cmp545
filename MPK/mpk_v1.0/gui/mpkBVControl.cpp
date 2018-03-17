#include "mpk_rand.h"
#include "mpkBVControl.H"

// By default, bounding volumes are rendered as wireframe boxes.  Use
// the TRANSPARENT_BV switch to show transparent bounding volumes
// instead.

//#define TRANSPARENT_BV


mpkRobotCollection* mpkBVControl::robots;
vector<SoTransform*> mpkBVControl::bv_transf;
mpkBVControl::bv_type mpkBVControl::curr_type;
SoSwitch* mpkBVControl::root = 0;
SoSeparator* mpkBVControl::obb_sep;
SoSeparator* mpkBVControl::rss_sep;

  
void
mpkBVControl::
init(mpkRobotCollection* robots, mpkObstacleCollection* obstacles, int bv_level)
{
  mpkBVControl::robots = robots;

  if ( root ) root->unref();
  root = new SoSwitch;
  root->ref();

  obb_sep = new SoSeparator;
  root->addChild(obb_sep);
  SoMaterial* obb_mat = new SoMaterial;
  obb_sep->addChild(obb_mat);
  obb_mat->transparency = 0.5;

  rss_sep = new SoSeparator;
  root->addChild(rss_sep);
  SoMaterial* rss_mat = new SoMaterial;
  rss_sep->addChild(rss_mat);
  rss_mat->transparency = 0.5;

  for ( int i=0; i<obstacles->pqp.size(); i++ ) {

    SoSeparator* sep1 = new SoSeparator;
    obb_sep->addChild(sep1);
    SoTransform* tr = new SoTransform;
    sep1->addChild(tr);
    tr->setMatrix(obstacles->Tr[i]);
    add_bv_level(sep1, OBB, obstacles->pqp[i], bv_level);

    SoSeparator* sep2 = new SoSeparator;
    rss_sep->addChild(sep2);
    sep2->addChild(tr);
    add_bv_level(sep2, RSS, obstacles->pqp[i], bv_level);

  }

  bv_transf.clear();
  {for ( int i=0; i<robots->num_robots(); i++ ) {
    for ( int j=0; j < (*robots)[i]->num_joints; j++ ) {
      if ((*robots)[i]->joints[j].pqp_model[0]) {

	SoSeparator* sep1 = new SoSeparator;
	obb_sep->addChild(sep1);
	SoTransform* tr = new SoTransform;
	sep1->addChild(tr);
	tr->setMatrix((*robots)[i]->joints[j].Taccu);
	bv_transf.push_back(tr);
	add_bv_level(sep1, OBB, (*robots)[i]->joints[j].pqp_model[0], bv_level);

	SoSeparator* sep2 = new SoSeparator;
	rss_sep->addChild(sep2);
	sep2->addChild(tr);
	add_bv_level(sep2, RSS, (*robots)[i]->joints[j].pqp_model[0], bv_level);

      }
      else
	bv_transf.push_back(0);
    }
  }}

  set_shown_bv(none);
}


void
mpkBVControl::
add_bv_level(SoSeparator* sep,
	     bv_type bv_type, PQP_Model* pqp, int level2add,
	     mpkTransform parent_tr, int rec_level, int bv_idx)
{
  if ( rec_level >= level2add ) {
    add_bv(sep, bv_type, pqp->b[bv_idx], parent_tr);
    return;
  }

  if ( pqp->b[bv_idx].first_child >= 0 ) {

    mpkTransform t;
    for ( int i=0; i<3; i++ )
      for ( int j=0; j<3; j++ )
	t.R[i][j] = pqp->b[bv_idx].R[i][j];
    if ( bv_type ==  OBB ) {
      for ( int i=0; i<3; i++ )
	t.T[i] = pqp->b[bv_idx].To[i];
    }
    else { // RSS
      for ( int i=0; i<3; i++ )
	t.T[i] = pqp->b[bv_idx].Tr[i];
    }
    parent_tr.append(t);
    add_bv_level(sep, bv_type, pqp, level2add,
		 parent_tr, rec_level+1, pqp->b[bv_idx].first_child);
    add_bv_level(sep, bv_type, pqp, level2add,
		 parent_tr, rec_level+1, pqp->b[bv_idx].first_child+1);
  }
}


void
mpkBVControl::
set_shown_bv(bv_type new_bv_type)
{
  curr_type = new_bv_type;
  switch (new_bv_type) {
  case OBB: root->whichChild = 0; break;
  case RSS: root->whichChild = 1; break;
  default:  root->whichChild = SO_SWITCH_NONE; break;
  }
}

void 
mpkBVControl::
update_iv_transf()
{
  int c=0;
  for ( int i=0; i<robots->num_robots(); i++ ) {
    for ( int j=0; j < (*robots)[i]->num_joints; j++ ) {
      if (bv_transf[c])
	bv_transf[c]->setMatrix((*robots)[i]->joints[j].Taccu);
      c++;
    }
  }
}

void
mpkBVControl::
add_bv(SoSeparator* sep, bv_type bv_type, const BV& bv, mpkTransform parent_tr)
{
  SoSeparator* bv_sep = new SoSeparator;
  sep->addChild(bv_sep);
  SoTransform* tr1 = new SoTransform;
  bv_sep->addChild(tr1);
  mpkTransform bv_tr;
  for ( int n=0; n<3; n++ ) {
    for ( int m=0; m<3; m++ )
      bv_tr.R[n][m] = bv.R[n][m];
  }
  if ( bv_type == OBB ) {
    bv_tr.T[0] = bv.To[0];
    bv_tr.T[1] = bv.To[1];
    bv_tr.T[2] = bv.To[2];
  }
  else { // RSS
    bv_tr.T[0] = bv.Tr[0];
    bv_tr.T[1] = bv.Tr[1];
    bv_tr.T[2] = bv.Tr[2];
    SoTranslation* tr2 = new SoTranslation;
    bv_sep->addChild(tr2);
    tr2->translation.setValue(bv.l[0]/2,bv.l[1]/2,0);
  }
  mpkTransform t;
  t.set_product(parent_tr,bv_tr);
  tr1->setMatrix(t);

  // add a cube node
  double w,h,d;
  if ( bv_type == OBB )
    w = 2*bv.d[0], h = 2*bv.d[1], d = 2*bv.d[2];
  else
    w = bv.l[0]+2*bv.r, h = bv.l[1]+2*bv.r, d = 2*bv.r;

#ifdef TRANSPARENT_BV
  SoCube* cub = new SoCube;
  bv_sep->addChild(cub);  
  cub->width.setValue(w);
  cub->height.setValue(h);
  cub->depth.setValue(d);
#else // wireframe BV
  double min[] = {-w/2,-h/2,-d/2};
  double max[] = {w/2,h/2,d/2};
  mpkBVControl::add_wire_cube(bv_sep,min,max,0,1,0);
#endif
}


void
mpkBVControl::
add_wire_cube(SoSeparator* root, double min[3], double max[3],
	      double r, double g, double b)
{
  SoSeparator* sep = new SoSeparator;
  root->addChild(sep);

  SoDrawStyle *linestyle = new SoDrawStyle;
  sep->addChild(linestyle);
  linestyle->lineWidth.setValue(2);

  SoBaseColor* col = new SoBaseColor;
  sep->addChild(col);
  float ccol[][3] = {{r,g,b}};
  col->rgb.setValues(0,3,ccol);

  SoLineSet* lines = new SoLineSet;
  sep->addChild(lines);
  lines->numVertices.set1Value(0, 5);
  lines->numVertices.set1Value(1, 5);
  lines->numVertices.set1Value(2, 2);
  lines->numVertices.set1Value(3, 2);
  lines->numVertices.set1Value(4, 2);
  lines->numVertices.set1Value(5, 2);

  SoVertexProperty* vts = new SoVertexProperty;
  lines->vertexProperty = vts;
  // back square
  vts->vertex.set1Value(0, min[0], min[1], min[2]);
  vts->vertex.set1Value(1, max[0], min[1], min[2]);
  vts->vertex.set1Value(2, max[0], max[1], min[2]);
  vts->vertex.set1Value(3, min[0], max[1], min[2]);
  vts->vertex.set1Value(4, min[0], min[1], min[2]);

  // frot square
  vts->vertex.set1Value(5, min[0], min[1], max[2]);
  vts->vertex.set1Value(6, max[0], min[1], max[2]);
  vts->vertex.set1Value(7, max[0], max[1], max[2]);
  vts->vertex.set1Value(8, min[0], max[1], max[2]);
  vts->vertex.set1Value(9, min[0], min[1], max[2]);

  // four joining lines
  vts->vertex.set1Value(10, min[0], min[1], min[2]);
  vts->vertex.set1Value(11, min[0], min[1], max[2]);

  vts->vertex.set1Value(12, min[0], max[1], min[2]);
  vts->vertex.set1Value(13, min[0], max[1], max[2]);

  vts->vertex.set1Value(14, max[0], min[1], min[2]);
  vts->vertex.set1Value(15, max[0], min[1], max[2]);

  vts->vertex.set1Value(16, max[0], max[1], min[2]);
  vts->vertex.set1Value(17, max[0], max[1], max[2]);
}


// samples a point uniformly at random from RSS bv and stores the
// result in p
void 
mpkBVControl::
sample_on_RSS(double p[3], const BV& bv)
{
  double sphere_surf = 4*M_PI * bv.r*bv.r;
  double cyl0_surf = bv.l[0] * 2*M_PI * bv.r;
  double cyl1_surf = bv.l[1] * 2*M_PI * bv.r;
  double box_surf = 2 * bv.l[0] * bv.l[1];

  double sum0_surf = sphere_surf + cyl0_surf;
  double sum1_surf = sum0_surf + cyl1_surf;
  double rss_surf = sum1_surf + box_surf;

  double rval = mpk_drand() * rss_surf;

  double q[3];
  if ( rval < sphere_surf ) { // sample on sphere
    double a = 2*M_PI*mpk_drand();
    double b = acos(1-2*mpk_drand());
    double sb = sin(b);
    q[0] = bv.r * sb*cos(a);
    q[1] = bv.r * sb*sin(a);
    q[2] = bv.r * cos(b);
    // split sphere into four:
    if ( q[0] >= 0 ) q[0] += bv.l[0];
    if ( q[1] >= 0 ) q[1] += bv.l[1];
  }
  else if ( rval < sum0_surf ) { // sample on cylinder 0
    double a = 2*M_PI*mpk_drand();
    q[0] = bv.l[0] * mpk_drand();
    q[1] = bv.r * cos(a);
    q[2] = bv.r * sin(a);
    if ( q[1] >= 0 ) q[1] += bv.l[1];
  }
  else if ( rval < sum1_surf ) { // sample on cylinder 1
    double a = 2*M_PI*mpk_drand();
    q[1] = bv.l[1] * mpk_drand();
    q[0] = bv.r * cos(a);
    q[2] = bv.r * sin(a);
    if ( q[0] >= 0 ) q[0] += bv.l[0];
  }
  else { // sample on box
    q[0] = bv.l[0]*mpk_drand();
    q[1] = bv.l[1]*mpk_drand();
    if ( mpk_drand() < 0.5 ) q[2] = bv.r;
    else q[2] = -bv.r;
  }

  // transform from local frame of RSS into frame of PQP_Model
  p[0] = bv.R[0][0] * q[0] + bv.R[0][1] * q[1] + bv.R[0][2] * q[2] + bv.Tr[0];
  p[1] = bv.R[1][0] * q[0] + bv.R[1][1] * q[1] + bv.R[1][2] * q[2] + bv.Tr[1];
  p[2] = bv.R[2][0] * q[0] + bv.R[2][1] * q[1] + bv.R[2][2] * q[2] + bv.Tr[2];
}


#include "mpkMouseControl.H"

#define SHOW_AXES_LABELS // default: no axes labels for cross cursor

#ifdef MPK_OPTIMIZE_IMPLEMENTED
extern void
mpk_optimize(double p[], int n, double ftol, int *iter, double *fret,
	     double (*func)(double []), void (*dfunc)(double [], double []));
#endif

PQP_Model* mpkMouseControl::mouseptr = 0;

mpkRobotCollection* mpkMouseControl::robots;
mpkObstacleCollection* mpkMouseControl::obstacles;

double mpkMouseControl::scene_size;

SoSeparator* mpkMouseControl::root = 0;

SoSwitch* mpkMouseControl::cursor_switch;
SoTranslation* mpkMouseControl::cursor_pos;

SoSwitch* mpkMouseControl::rline_switch;
SoVertexProperty* mpkMouseControl::rline_vts;

double mpkMouseControl::point_on_robot[3];
double mpkMouseControl::target_point[3];

mpkBaseRobot* mpkMouseControl::picked_rob;
int mpkMouseControl::picked_rob_idx;
mpkBaseRobot::Joint* mpkMouseControl::picked_joint;
int mpkMouseControl::picked_joint_idx;
int mpkMouseControl::picked_dof;
bool mpkMouseControl::link_attached_to_cursor = false;
bool mpkMouseControl::move_to_target_mode = false;

vector<double> mpkMouseControl::backup_params;
vector<bool> mpkMouseControl::picked_rob_param_frozen;

void (*mpkMouseControl::status_callback)();

inline double sqrdist3(const double a[], const double b[])
{
  double dx = a[0]-b[0];
  double dy = a[1]-b[1];
  double dz = a[2]-b[2];
  return dx*dx + dy*dy + dz*dz;
}

void
mpkMouseControl::
init(SoGroup* scene_root,
     mpkRobotCollection* robots, mpkObstacleCollection* obstacles,
     void (*status_callback)())
{
  mpkMouseControl::robots = robots;
  mpkMouseControl::obstacles = obstacles;
  picked_rob = 0;
  picked_dof = -1;
  mpkMouseControl::status_callback = status_callback;

  backup_params.resize(robots->num_params());

  // get bounding box of entire scene (for scaling the cross)
  SbViewportRegion dummy;
  SoGetBoundingBoxAction bbox(dummy);
  bbox.apply(scene_root);
  double dx =
    bbox.getBoundingBox().getMax()[0] -
    bbox.getBoundingBox().getMin()[0];
  double dy =
    bbox.getBoundingBox().getMax()[1] -
    bbox.getBoundingBox().getMin()[1];
  double dz =
    bbox.getBoundingBox().getMax()[2] -
    bbox.getBoundingBox().getMin()[2];
  double max = (dx > dy) ? dx: dy;
  max = (dz > max)? dz: max;
  scene_size = max;

  // setup a separator to group mouse cross cursor and rubber line
  if ( root ) root->unref();
  root = new SoSeparator;
  root->ref();

  // setup the cross cursor
  SoSeparator* sep1 = new SoSeparator;
  root->addChild(sep1);
  cursor_switch = new SoSwitch;
  sep1->addChild(cursor_switch);
  cursor_switch->whichChild = SO_SWITCH_NONE;

  SoBaseColor* cursor_col = new SoBaseColor;
  cursor_switch->addChild(cursor_col);
  float ccol[][3] = {{1,0,0}};
  cursor_col->rgb.setValues(0,3,ccol);

  cursor_pos = new SoTranslation;
  cursor_switch->addChild(cursor_pos);

  SoLineSet *cursor_axes = new SoLineSet;
  cursor_switch->addChild(cursor_axes);
  cursor_axes->numVertices.set1Value(0, 2);
  cursor_axes->numVertices.set1Value(1, 2);
  cursor_axes->numVertices.set1Value(2, 2);

  SoVertexProperty* cursor_vts = new SoVertexProperty;
  cursor_axes->vertexProperty = cursor_vts;
  double ax_len = 0.1*scene_size;
  cursor_vts->vertex.set1Value(0, -ax_len, 0, 0);
  cursor_vts->vertex.set1Value(1,  ax_len, 0, 0);
  cursor_vts->vertex.set1Value(2, 0, -ax_len, 0);
  cursor_vts->vertex.set1Value(3, 0,  ax_len, 0);
  cursor_vts->vertex.set1Value(4, 0, 0, -ax_len);
  cursor_vts->vertex.set1Value(5, 0, 0,  ax_len);

  // add axes labels
#ifdef SHOW_AXES_LABELS
  SoTranslation* text1pos = new SoTranslation;
  SoTranslation* text2pos = new SoTranslation;
  SoTranslation* text3pos = new SoTranslation;
  SoTranslation* text4pos = new SoTranslation;
  SoTranslation* text5pos = new SoTranslation;
  SoTranslation* text6pos = new SoTranslation;
  SoText2* text1 = new SoText2;
  SoText2* text2 = new SoText2;
  SoText2* text3 = new SoText2;
  SoText2* text4 = new SoText2;
  SoText2* text5 = new SoText2;
  SoText2* text6 = new SoText2;
  cursor_switch->addChild(text1pos);
  cursor_switch->addChild(text1);
  cursor_switch->addChild(text2pos);
  cursor_switch->addChild(text2);
  cursor_switch->addChild(text3pos);
  cursor_switch->addChild(text3);
  cursor_switch->addChild(text4pos);
  cursor_switch->addChild(text4);
  cursor_switch->addChild(text5pos);
  cursor_switch->addChild(text5);
  cursor_switch->addChild(text6pos);
  cursor_switch->addChild(text6);
  text1->string = "Left";
  text2->string = "Right";
  text3->string = "PgDn";
  text4->string = "PgUp";
  text5->string = "Down";
  text6->string = "Up";
  text1pos->translation.setValue(-ax_len,0,0);
  text2pos->translation.setValue(2*ax_len,0,0);
  text3pos->translation.setValue(-ax_len,-ax_len,0);
  text4pos->translation.setValue(0,2*ax_len,0);
  text5pos->translation.setValue(0,-ax_len,-ax_len);
  text6pos->translation.setValue(0,0,2*ax_len);
#endif

  // add a (hidden) rubber band line
  SoSeparator* sep2 = new SoSeparator;
  root->addChild(sep2);
  rline_switch = new SoSwitch;
  sep2->addChild(rline_switch);
  rline_switch->whichChild = SO_SWITCH_NONE;

  SoBaseColor* rline_col = new SoBaseColor;
  rline_switch->addChild(rline_col);
  float col[][3] = {{0,1,0}};
  rline_col->rgb.setValues(0,3,col);

  SoLineSet *rline = new SoLineSet;
  rline_switch->addChild(rline);
  rline->numVertices.set1Value(0, 2);

  rline_vts = new SoVertexProperty;
  rline->vertexProperty = rline_vts;
  rline_vts->vertex.set1Value(0, 0, 0, 0);
  rline_vts->vertex.set1Value(1, 1, 1, 1);

  // setup pqp model for mouse ptr (with one tiny triangle)
  // to use pqp for distance queries
  if ( !mouseptr ) {
    double p[][3] = {{0,0,0}, {1e-8,0,0}, {0,1e-8,0}};
    mouseptr = new PQP_Model();
    mouseptr->BeginModel();
    mouseptr->AddTri(p[0],p[1],p[2],0);
    mouseptr->EndModel();
  }
}


bool
mpkMouseControl::
get_show_cross_cursor()
{
  return cursor_switch->whichChild.getValue() != SO_SWITCH_NONE;
}

void 
mpkMouseControl::
move_cross_cursor_1d(int ax, double incr)
{
  if ( !get_show_cross_cursor() ) return;
  double newp[] = {cursor_pos->translation.getValue()[0],
		     cursor_pos->translation.getValue()[1],
		     cursor_pos->translation.getValue()[2]};
  newp[ax] += incr;
  cursor_pos->translation.setValue(newp[0], newp[1], newp[2]);
  if ( link_attached_to_cursor )
    move_robot_point_to(newp);
}

void 
mpkMouseControl::
set_cross_cursor_pos(double x, double y, double z)
{
  cursor_pos->translation.setValue(x, y, z);
  double newp[] = {x,y,z};
  if ( link_attached_to_cursor )
    move_robot_point_to(newp);
}

void
mpkMouseControl::
get_cross_cursor_pos(double& x, double& y, double& z)
{
  x = cursor_pos->translation.getValue()[0];
  y = cursor_pos->translation.getValue()[1];
  z = cursor_pos->translation.getValue()[2];
}

void
mpkMouseControl::
update_rubber_line()
{
  double cursor_point[]=
    {cursor_pos->translation.getValue()[0],
     cursor_pos->translation.getValue()[1],
     cursor_pos->translation.getValue()[2]};
  double rob_point_global[3];
  picked_joint->Taccu.apply(rob_point_global, point_on_robot);
  rline_vts->vertex.set1Value(0,
			      cursor_point[0],
			      cursor_point[1],
			      cursor_point[2]);
  rline_vts->vertex.set1Value(1,
			      rob_point_global[0],
			      rob_point_global[1],
			      rob_point_global[2]);
  if ( link_attached_to_cursor )
    rline_switch->whichChild = SO_SWITCH_ALL;
}

double
mpkMouseControl::
robot_target_dist(double p[])
{
  for ( int i=0; i<picked_rob->num_params; i++ )
    picked_rob->params[i] = p[i+1];
  picked_rob->compute_forward_kinematics();
 
  double rob_point_global[3];
  picked_joint->Taccu.apply(rob_point_global, point_on_robot);
 
  return sqrt(sqrdist3(target_point, rob_point_global));
}

void
mpkMouseControl::
deriv_robot_target_dist(double p[], double dp[])
{
  const double eps=1e-5;
  for ( int i=0; i < picked_rob->num_params; i++ ) {
    backup_params[i] = picked_rob->params[i];
    picked_rob->params[i] = p[i+1];
  }
  picked_rob->compute_forward_kinematics();
  double rob_point_global[3];
  picked_joint->Taccu.apply(rob_point_global, point_on_robot);
  double dist_at_p = sqrt(sqrdist3(target_point, rob_point_global));

  {for ( int i=0; i < picked_rob->num_params; i++ ) {
    if ( picked_rob_param_frozen[i] )
      dp[i+1] = 0;
    else {
      picked_rob->params[i] += eps;
      picked_rob->compute_forward_kinematics();
      picked_joint->Taccu.apply(rob_point_global, point_on_robot);
      dp[i+1] = sqrt(sqrdist3(target_point, rob_point_global)) - dist_at_p;
      picked_rob->params[i] -= eps;
    }
  }}
  {for ( int i=0; i < picked_rob->num_params; i++ ) {
    picked_rob->params[i] = backup_params[i];
  }}
}


// moves the point_on_robot towards the cross
void
mpkMouseControl::
move_robot_point_to(const double pgoal[3])
{

#ifndef MPK_OPTIMIZE_IMPLEMENTED

  cerr << "Warning: cartesian interface requires implementing mpk_optimize function (see documentation)" << endl;
  cerr << "Please press any key to continue" << endl;
  getchar();

#else

  target_point[0] = pgoal[0];
  target_point[1] = pgoal[1];
  target_point[2] = pgoal[2];

  double* p = new double[1+picked_rob->num_params];

  picked_rob_param_frozen.resize(picked_rob->num_params);
  for ( int i=0; i<picked_rob->num_params; i++ ) {
    p[i+1] = picked_rob->params[i];
    picked_rob_param_frozen[i] =
      picked_rob->param_opts[i].is_frozen ||
      picked_rob->param_opts[i].is_passive;
  }

  // constrained minimization (in [0,1]^d) of distance of picked point
  // on robot to origin of cross cursor axes
  double prev_fret = DBL_MAX;
  double fret;
  int iter;
  mpk_optimize(p, picked_rob->num_params, 1e-5, &iter, &fret, 
	       robot_target_dist, deriv_robot_target_dist);
  
  // do further iterations while keeping those DOFs which
  // reached the interval [0,1] boundary frozen.
  // iterate until no improvement can be made
  while ( fret < prev_fret ) {
    for ( int i=0; i < picked_rob->num_params; i++ ) {
      if ( p[i+1] < 0 ) { p[i+1] = 0; picked_rob_param_frozen[i] = true; }
      else if ( p[i+1] > 1 ) { p[i+1] = 1; picked_rob_param_frozen[i] = true; }
      picked_rob->params[i] = p[i+1];
    }
    prev_fret = fret;
    mpk_optimize(p, picked_rob->num_params, 1e-5, &iter, &fret, 
		 robot_target_dist, deriv_robot_target_dist);
  }

  delete [] p;
 
  picked_rob->compute_forward_kinematics();
  picked_rob->update_iv_transf();
  update_rubber_line();

#endif

}


void 
mpkMouseControl::
myMouseButtonCB(void *userData, SoEventCallback *eventCB)
{
  const SoEvent *event = eventCB->getEvent();
  
  if ( SO_MOUSE_PRESS_EVENT(event,BUTTON1) && eventCB->getPickedPoint() ) {
    const SbVec3f& p = eventCB->getPickedPoint()->getPoint();
    //cerr << p[0] << " " << p[1] << " " << p[2] << endl;
    double mouse_rot[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    double mouse_pos[3] = {p[0],p[1],p[2]};

    PQP_ToleranceResult res;
    picked_joint_idx=-1;
    picked_rob_idx=-1;
    for ( int i=0; i<robots->num_robots(); i++ ) {
      mpkBaseRobot* robi = robots->rob[i];
      for ( int j=0; j<robi->num_joints; j++ ) {
	mpkBaseRobot::Joint& jointj = robi->joints[j];
	if ( !jointj.pqp_model[0] ) continue;
	PQP_Tolerance(&res,
		      mouse_rot, mouse_pos, mouseptr,
		      jointj.Taccu.R, jointj.Taccu.T, jointj.pqp_model[0],
		      scene_size*1e-3);
	if ( res.CloserThanTolerance() ) {
	  picked_rob_idx = i;
	  picked_joint_idx = j;
	  if ( get_show_cross_cursor() ) link_attached_to_cursor = true;
	  break;
	}
      }
      if ( picked_rob_idx >= 0 ) break;
    }
    if ( picked_rob_idx >= 0 ) { // clicked on robot
      picked_rob = robots->rob[picked_rob_idx];
      picked_joint = &picked_rob->joints[picked_joint_idx];
      picked_dof = picked_joint->param_idx;
      picked_joint->Taccu.apply_inv(point_on_robot, mouse_pos);
      if ( get_show_cross_cursor() ) {
	set_cross_cursor_pos(mouse_pos[0], mouse_pos[1], mouse_pos[2]);
	update_rubber_line();
      }
      if (status_callback) status_callback();
    }
    else { // clicked on obstacle: deactivate robot
      if ( picked_rob && move_to_target_mode ) {
	picked_joint = &picked_rob->joints[picked_rob->tracepoint_joint_idx];
	move_robot_point_to(mouse_pos);
	point_on_robot[0] = point_on_robot[1] = point_on_robot[2] = 0;
      }
      else {
	picked_rob = 0;
	picked_joint = 0;
	picked_dof = -1;
	link_attached_to_cursor = false;
	rline_switch->whichChild = SO_SWITCH_NONE;
      }
      if (status_callback) status_callback();
    }
  }
  else if ( SO_MOUSE_PRESS_EVENT(event,BUTTON2) && picked_rob ) {
    if ( ++picked_joint_idx >= picked_rob->num_joints )
      picked_joint_idx = 0;
    picked_joint = &picked_rob->joints[picked_joint_idx];
    picked_dof = picked_joint->param_idx;
    if (status_callback) status_callback();
  }

}

void
mpkMouseControl::
set_show_cross_cursor(bool on)
{
  if ( on ) {
    cursor_switch->whichChild = SO_SWITCH_ALL;
  }
  else {
    cursor_switch->whichChild = SO_SWITCH_NONE;
  }
  rline_switch->whichChild = SO_SWITCH_NONE;    
  link_attached_to_cursor = false;
}


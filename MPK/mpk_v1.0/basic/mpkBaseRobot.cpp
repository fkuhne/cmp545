#include <fstream>
#include "mpkBaseRobot.H"

// By default, we center bounding spheres of all models at the OBB
// centers.  However, this does not yield the smallest possible
// spheres.  Optimal spheres can be used instead, but this requires
// external code from http://vision.ucsd.edu/~dwhite/ball.html

// use this switch if you have the optimal ball code installed:
//#define OPTIMAL_BOUNDING_SPHERES
#ifdef OPTIMAL_BOUNDING_SPHERES
#include "ball.h"
#endif

#define DEBUG_SPHERE_VISUALIZATION
#ifdef DEBUG_SPHERE_VISUALIZATION
#include "../gui/mpkGUI.H"
#endif

#include <Inventor/nodes/SoSphere.h> 

using namespace std;

inline
void
copy_point(double out[3], const double in[3])
{
  out[0]=in[0], out[1]=in[1], out[2]=in[2];
}

// sets q equal to a + t*v
inline
void
get_point_on_line(double q[3], const double a[3], double t, const double v[3])
{
  q[0] = a[0] + t*v[0];
  q[1] = a[1] + t*v[1];
  q[2] = a[2] + t*v[2];
}

// returns lambda and sets q such that q = a + lambda*v is the projection
// of p onto a + t*v
inline
double
project_onto_line(double q[3],
		  const double p[3], const double a[3], const double v[3])
{
  double ap[] = {p[0]-a[0],p[1]-a[1],p[2]-a[2]};
  double lambda = (ap[0]*v[0] + ap[1]*v[1] + ap[2]*v[2]) /
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  get_point_on_line(q,a,lambda,v);
  return lambda;
}

// returns distance of p from q
inline
double
point_dist(const double p[3], const double q[3])
{
  double pq[] = {p[0]-q[0],p[1]-q[1],p[2]-q[2]};
  return sqrt(pq[0]*pq[0] + pq[1]*pq[1] + pq[2]*pq[2]);
}

// returns a copy of s
inline char* copy_string(const char* s)
{
  if ( !s ) return 0;
  char* s2 = new char[strlen(s)+1];
  strcpy(s2,s);
  return s2;
}

// find a string in a vector of strings (not really efficient but good
// - and simple - enough for our purposes)
inline int find_name(const string& s, const vector<string>& names)
{
  int j;
  for (j=0; j<names.size(); j++)
    if ( names[j]==s )
      break;
  return (j==names.size())? -1 : j;
}


mpkBaseRobot::~mpkBaseRobot()
{
  delete [] params;
  delete [] param_opts;
  for ( int i=0; i<num_joints; i++ ) {
    //..delete joints[i].name;
    delete joints[i].T0;
    delete joints[i].Tq;
    delete joints[i].iv_fname0;
    delete joints[i].iv_fname1;
    delete joints[i].pqp_model[0];
    delete joints[i].pqp_model[1];
  }
  delete [] joints;
  iv_graph->unref();
}

// the function that returns upper bound on motions in workspace.  It
// works for all generic kinematic trees with any combinations of
// revolute and prismatic joints.  If your robot contains other types
// of joints and more general kinematics, you need to derive a
// 'hardcoded' robot from mpkBaseRobot and overload this function.
// See mpkIrb2400 for an example of how to 'hardwire' a robot.
double
mpkBaseRobot::
pathlen_upbound(const mpkConfig& q0, const mpkConfig& q1,
		int first_param_idx, int joint_idx, int root_idx)
{
  if ( !joints[joint_idx].bounds_available ) {
    // no bound factors computed: we can't compute generic bound for
    // workspace pathlength
    cerr << "Error: mpkBaseRobot::pathlen_upbound(): no generic pathlength bound factors for\n"
	 << "robot link " << get_name() << ":" << joints[joint_idx].name << "\n"
	 << "Please either exclude this link from collision checking"
	 << "or overload pathlen_upbound for this robot type with "
	 << "specialized code (generic factors are only computed "
	 << "for trees with revolute and prismatic DOFs)." << endl;
    mpk_exit(1);
  }

  double upb = 0;

  /*cout << endl;
  cout << "pathlen_upbound() for "
       << get_name() << ":" << joints[joint_idx].name << " w.r.t. ";
  if ( root_idx < 0 )
    cout << "world coordinate system" << endl;
  else
    cout << get_name() << ":" << joints[root_idx].name << endl;
  */

  for ( int curr=joint_idx, i=0;
	curr > root_idx;
	curr = joints[curr].parent_idx, i++) {
    //cout << "  " << joints[curr].name;
    if ( joints[curr].Tq->num_params() ) {
      int idx = joints[curr].param_idx;
      upb +=
	// from compute_workspace_bounds()
	joints[joint_idx].bound_fact[i] *
	
	// absolute difference of (properly scaled) parameter values
	fabs(joints[curr].Tq->max_param()-joints[curr].Tq->min_param()) *
	fabs(q0[first_param_idx + idx]-q1[first_param_idx + idx]);
      /*cout << " param=" << idx
	   << " R=" << joints[joint_idx].bound_fact[i]
	   << " F="
	   << fabs(joints[curr].Tq->max_param()-joints[curr].Tq->min_param())
	   << " q0="
	   << q0[first_param_idx + idx]
	   << " q1="
	   << q1[first_param_idx + idx];
      */   
    }
    //cout << " upb=" << upb << endl;
  }

  return upb;
}


SoSeparator* mpkBaseRobot::read_link_model(const char link_fname[])
{
  SoInput linkInput;
  linkInput.addDirectoryFirst("ivmodels");
  linkInput.addEnvDirectoriesFirst("IVMODELS");
  linkInput.addDirectoryLast("ivmodels.private");
  linkInput.openFile(link_fname);
  SoSeparator* iv_model = SoDB::readAll(&linkInput);
  if ( !iv_model )
    mpk_exit(1);
  return iv_model;
}


// constructor with parser that reads the robot from a .rob file
mpkBaseRobot::mpkBaseRobot(const char* name, const char* rob_file_name,
			   const mpkTransform& base_transf, double scalef)
{
  int i;

  RobFileScanner pass1(rob_file_name, "robots", getenv("MPK_ROBOTS"));
  string s;
  vector<string> joint_names;
  while (pass1.get_token(s,0,false)) {
    if ( s=="joint" ) {
      pass1.get_token(s);
      joint_names.push_back(s);
    }
    pass1.get_token(s,"{");
    int cnt=1;
    while (cnt) {
      pass1.get_token(s); 
      if ( s=="{" ) cnt++;
      else if ( s=="}" ) cnt--;
    }
  }

  RobFileScanner pass2(rob_file_name, "robots", getenv("MPK_ROBOTS"));
  
  JointDef* jointdefs = new JointDef[joint_names.size()];
  vector<param_info> pinfo;
  selfcoll_info.clear();
  tracepoint_joint_idx = -1;

  i=-1;

  while (pass2.get_token(s,0,false)) {
    
    if ( s=="joint" ) {
      
      i++;
      jointdefs[i].parent_idx = -1;
      jointdefs[i].T0 = new mpkTransform;
      jointdefs[i].Tq = new mpkTransform;
      jointdefs[i].param_idx = -1;
      jointdefs[i].iv_fname0 = 0;
      jointdefs[i].iv_fname1 = 0;
      jointdefs[i].check_coll = true;
    
      pass2.get_token(s,joint_names[i].c_str());
      jointdefs[i].name = copy_string(s.c_str());
      pass2.get_token(s,"{");
      
      while (s != "}") {

	pass2.get_token(s);
	
	if ( s=="parent" ) {
	  pass2.get_token(s);
	  int j = find_name(s,joint_names);
	  if ( j >= 0 )	jointdefs[i].parent_idx = j;
	  else {
	    cerr << "Error in robot file " << rob_file_name << ": parent "
		 << s << " of " << joint_names[i] << " undefined." << endl;
	    mpk_exit(1);
	  }
	}
	else if ( s=="ConstTransl" ) {
	  pass2.get_token(s); double tx = atof(s.c_str());
	  pass2.get_token(s); double ty = atof(s.c_str());
	  pass2.get_token(s); double tz = atof(s.c_str());
	  jointdefs[i].T0->append(mpkConstTransl(tx,ty,tz));
	}
	else if ( s=="ConstRot" ) {
	  pass2.get_token(s); double ax = atof(s.c_str());
	  pass2.get_token(s); double ay = atof(s.c_str());
	  pass2.get_token(s); double az = atof(s.c_str());
	  pass2.get_token(s); double ang = atof(s.c_str());
	  jointdefs[i].T0->append(mpkConstRot(ax,ay,az,ang));
	}
	else if ( s=="Transl1" ) {
	  pass2.get_token(s); double tx = atof(s.c_str());
	  pass2.get_token(s); double ty = atof(s.c_str());
	  pass2.get_token(s); double tz = atof(s.c_str());
	  pass2.get_token(s); double min = atof(s.c_str());
	  pass2.get_token(s); double max = atof(s.c_str());
	  delete jointdefs[i].Tq;
	  jointdefs[i].Tq = new mpkTransl1(tx,ty,tz,min,max);
	}
	else if ( s=="Rot1" ) {
	  pass2.get_token(s); double ax = atof(s.c_str());
	  pass2.get_token(s); double ay = atof(s.c_str());
	  pass2.get_token(s); double az = atof(s.c_str());
	  pass2.get_token(s); double min = atof(s.c_str());
	  pass2.get_token(s); double max = atof(s.c_str());
	  delete jointdefs[i].Tq;
	  jointdefs[i].Tq = new mpkRot1(ax,ay,az,min,max);
	}
	else if ( s=="ConstTransl_Rot1" ) {
	  pass2.get_token(s); double tx = atof(s.c_str());
	  pass2.get_token(s); double ty = atof(s.c_str());
	  pass2.get_token(s); double tz = atof(s.c_str());
	  pass2.get_token(s); double ax = atof(s.c_str());
	  pass2.get_token(s); double ay = atof(s.c_str());
	  pass2.get_token(s); double az = atof(s.c_str());
	  pass2.get_token(s); double min = atof(s.c_str());
	  pass2.get_token(s); double max = atof(s.c_str());
	  delete jointdefs[i].Tq;
	  jointdefs[i].Tq = new mpkConstTransl_Rot1(tx,ty,tz,ax,ay,az,min,max);
	}
	else if ( s=="param" ) {
	  pass2.get_token(s);
	  jointdefs[i].param_idx = atoi(s.c_str());
	}
	else if ( s=="model0" ) {
	  pass2.get_string_token(s);
	  jointdefs[i].iv_fname0 = copy_string(s.c_str());
	}
	else if ( s=="model1" ) {
	  pass2.get_string_token(s);
	  jointdefs[i].iv_fname1 = copy_string(s.c_str());
	}
	else if ( s=="coll" ) {
	  pass2.get_token(s);
	  if ( s=="FALSE" ) jointdefs[i].check_coll=false;
	}
	else if ( s=="tracePoint" ) {
	  tracepoint_joint_idx = i;
	}
	else if (s!="}") {
	  cerr << "Error: unexpected token '" << s << "' in file " << rob_file_name << endl;
	  mpk_exit(1);
	}
      }

    }
    else if ( s=="selfcoll" ) {

      pass2.get_token(s,"{");

      while (1) {
	pass2.get_token(s);
	if ( s=="}" ) break;
	string s2;
	pass2.get_token(s2);	
	int j1 = find_name(s,joint_names);
	int j2 = find_name(s2,joint_names);
	if (j1<0 || j2<0) {
	  cerr << "Error in robot file " << rob_file_name
	       << ": undefined joint in self-collision set." << endl;
	  mpk_exit(1);
	}
	selfcoll_info.push_back(selfcoll_pair(j1,j2));
      }

    }
    else if ( s=="param" ) {

      pass2.get_token(s,"{");

      while (1) {

	pass2.get_token(s);
	if ( s=="}" ) break;

	int param_idx = atoi(s.c_str());
	if ( param_idx >= pinfo.size() ) pinfo.resize(param_idx+1);

	pass2.get_token(s,"{");

	while (1) {
	  pass2.get_token(s);
	  if ( s=="}" ) break;
	  if ( s=="cyclic" ) pinfo[param_idx].is_cyclic = true;
	  else if ( s=="passive" ) pinfo[param_idx].is_passive = true;
	  else if ( s=="frozen" ) pinfo[param_idx].is_frozen = true;
	  else if ( s=="weight" ) {
	    pass2.get_token(s);
	    pinfo[param_idx].weight = atof(s.c_str());
	  }
	  else if ( s=="default" ) {
	    pass2.get_token(s);
	    pinfo[param_idx].default_val = atof(s.c_str());
	  }
	}
      }
    }

  } 

  init(name,joint_names.size(),jointdefs,base_transf,scalef);
  
  for ( i=0; i<num_params; i++ ) {
    if ( i >= pinfo.size() ) break;
    param_opts[i] = pinfo[i];
    params[i] = pinfo[i].default_val;
  }
  
  // delete temporary jointdefs (which have been copied into joints by init())
  for ( i=0; i<joint_names.size(); i++ ) {
    //..delete jointdefs[i].name;
    delete jointdefs[i].T0;
    delete jointdefs[i].Tq;
    delete jointdefs[i].iv_fname0;
    delete jointdefs[i].iv_fname1;
  }
  delete [] jointdefs;
}



// constructs 'hardwired' robot from array of joint definitions
// pointed to by jointdefs
mpkBaseRobot::mpkBaseRobot(const char* name,
			   int num_joints, JointDef* jointdefs,
			   const mpkTransform& base_transf, double scalef)
{
  tracepoint_joint_idx = -1;
  selfcoll_info.clear();  // no self-collision checking by default
  init(name,num_joints,jointdefs,base_transf,scalef);
}


// initialization of robot kinematics (to be called by constructors only)
void mpkBaseRobot::init(const char* name, int num_joints, JointDef* jointdefs,
			const mpkTransform& base_transf, double scalef)
{
  char buf[100];
  int robot_tri_cnt = 0;

  // setup root of iv scene graph for robot
  iv_graph = new SoSeparator();
  iv_graph->ref();
  iv_graph->setName(name);

  scale_factor = scalef;
#ifdef ROBOT_BASE_MOVABLE
  base_T = base_transf;
  iv_base_T = new SoTransform;
  iv_base_T->setMatrix(base_transf);
  iv_graph->addChild(iv_base_T);
#endif

  // allocate array for joints
  this->num_joints = num_joints;
  joints = new Joint[num_joints];
  this->num_params = 0;

  // process the joint definitions in jointdefs
  for ( int i=0; i<num_joints; i++ ) {

    // determine how many parameters will be required
     if ( jointdefs[i].param_idx >= 0 ) {
      int max_params = jointdefs[i].param_idx + jointdefs[i].Tq->num_params();
      if ( max_params > this->num_params )
	this->num_params = max_params;
    }

    // copy joint definitions
    int parent_idx = jointdefs[i].parent_idx;
    joints[i].parent_idx = parent_idx;
    joints[i].can_move = false;
    int it = i;
    while ( 1 ) {
      // check robot definition for cycles...
      for ( int k=0; k<joints[i].kchain_idx.size(); k++ )
	if (joints[i].kchain_idx[k] == it) {
	  cerr << "Error: robot definition of " << name
	       << " contains cycle at " << jointdefs[i].name << endl;
	  mpk_exit(1);
	}
      // ... and out of bounds indices
      if ( it < -1 || it >= num_joints ) {
	cerr << "Error: robot definition of " << name
	     << " contains invalid parent index at "
	     << jointdefs[i].name << endl;
	mpk_exit(1);	
      }
      // append to parent idx path
      if ( it < 0 ) break;
      if ( jointdefs[it].Tq->num_params() ) joints[i].can_move = true;
      joints[i].kchain_idx.push_back(it);
      it = jointdefs[it].parent_idx;
    }

    mpkTransform scaled_T0(*jointdefs[i].T0);
    scaled_T0.rescale(scalef);
    joints[i].T0 = new mpkTransform(*jointdefs[i].T0);
#ifndef ROBOT_BASE_MOVABLE
    if ( parent_idx < 0 ) {
      joints[i].T0->set_product(base_transf, scaled_T0);
      joints[i].Taccu = *joints[i].T0;
    }
    else
#endif
    *joints[i].T0 = scaled_T0;

    joints[i].Tq = new mpkTransform(*jointdefs[i].Tq);
    joints[i].Tq->rescale(scalef);
    joints[i].param_idx = jointdefs[i].param_idx;
    joints[i].name = copy_string(jointdefs[i].name);
    joints[i].iv_fname0 = copy_string(jointdefs[i].iv_fname0);
    joints[i].iv_fname1 = copy_string(jointdefs[i].iv_fname1);
    joints[i].check_coll = jointdefs[i].check_coll;

    // add nodes for joint to iv scene graph of robot
    joints[i].iv_sep = new SoSeparator;
    if ( parent_idx < 0 ) {
      iv_graph->addChild(joints[i].iv_sep);
    }
    else {
      if ( parent_idx >= i ) {
	cerr << "Error in robot definition: "
	     << "parent index must be smaller than joint index." << endl;
	mpk_exit(1);
      }
      joints[parent_idx].iv_sep->addChild(joints[i].iv_sep);
    }
    sprintf(buf,"JointSeparator_%d",i);
    joints[i].iv_sep->setName(buf);

    SoTransform* tmpT = new SoTransform;
    joints[i].iv_sep->addChild(tmpT);
    tmpT->setMatrix(*joints[i].T0);
    sprintf(buf,"T0[%d]",i);
    tmpT->setName(buf);

    joints[i].iv_Tq = new SoTransform;
    joints[i].iv_sep->addChild(joints[i].iv_Tq);
    joints[i].iv_Tq->setMatrix(*joints[i].Tq);
    sprintf(buf,"Tq[%d]",i);
    joints[i].iv_Tq->setName(buf);

    if ( jointdefs[i].iv_fname0 == 0 ||
	 strlen(jointdefs[i].iv_fname0) == 0 )
      joints[i].pqp_model[0] = joints[i].pqp_model[1] = 0;
    // process iv model of link
    else {
      SoSeparator* scaled_link = new SoSeparator;
      joints[i].iv_sep->addChild(scaled_link);
      sprintf(buf,"ScaledLinkSeparator_%d",i);
      scaled_link->setName(buf);
      if ( scalef != 1.0 ) {
	SoScale* scale_transf = new SoScale;
	scaled_link->addChild(scale_transf);
	scale_transf->scaleFactor.setValue(scalef,scalef,scalef);
      }

      sprintf(buf,"LinkModel_%d",i);
      SoSeparator* pqp_model0 = read_link_model(jointdefs[i].iv_fname0);
      scaled_link->addChild(pqp_model0);
      pqp_model0->setName(buf);

      // triangulate and create PQP model...
      SoCallbackAction triAction;      
      joints[i].pqp_model[0] = new PQP_Model;
      tri_info info(joints[i].pqp_model[0],scalef);
      
      joints[i].pqp_model[0]->BeginModel();
      triAction.addTriangleCallback(SoShape::getClassTypeId(),
				    triang_CB, (void*)&info);
      triAction.apply(pqp_model0);
      joints[i].pqp_model[0]->EndModel();
      
      //cerr << "  link model with "
      //	<< info.tri_cnt << " triangles." << endl;
      robot_tri_cnt += info.tri_cnt;

      // load second collision model of the link (if there)
      if ( jointdefs[i].iv_fname1 &&
	   strlen(jointdefs[i].iv_fname1) > 0 ) {

	SoSeparator* pqp_model1 = read_link_model(jointdefs[i].iv_fname1);
	pqp_model1->ref();
	
	// triangulate and create PQP model...
	SoCallbackAction triAction;      
	joints[i].pqp_model[1] = new PQP_Model;
	tri_info info(joints[i].pqp_model[1],scalef);
	
	joints[i].pqp_model[1]->BeginModel();
	triAction.addTriangleCallback(SoShape::getClassTypeId(),
				      triang_CB, (void*)&info);
	triAction.apply(pqp_model1);
	joints[i].pqp_model[1]->EndModel();
	
	pqp_model1->unref(); // inventor nodes not required anymore
	
      }

      // compute cylinder and sphere (for link model 0 only)
      const double Orig[] = {0,0,0};
      const double* ax = joints[i].Tq->get_axis();
      PQP_Model* pqp = joints[i].pqp_model[0];
      double min_t=DBL_MAX, max_t=-DBL_MAX;
      double q1[3], q2[3], q3[3];

      joints[i].cyl_r = 0;
      joints[i].sphere_r = 0;

#ifdef OPTIMAL_BOUNDING_SPHERES
      // find smallest enclosing sphere for the vertices of the model.
      // requires external code (by Dave White) which can be obtained from
      // http://vision.ucsd.edu/~dwhite/ball.html

      // first, copy the vertices into temporary buffer
      float** verts = new (float*)[3*pqp->num_tris];
      for ( int j=0; j<pqp->num_tris; j++ ) {
	verts[3*j] = new float[3];
	verts[3*j+1] = new float[3];
	verts[3*j+2] = new float[3];
	for ( int d=0; d<3; d++ ) {
	  verts[3*j][d]   = pqp->tris[j].p1[d];
	  verts[3*j+1][d] = pqp->tris[j].p2[d];
	  verts[3*j+2][d] = pqp->tris[j].p3[d];
	}
      }
 
      float opt_center[3], opt_radius;
      const unsigned bufsz = 1000000;
      char* buf = new char[bufsz];
      EnclosingBall(3, 3*pqp->num_tris, verts, 0, opt_center, opt_radius,
		    0, 0, 0, 0, buf, bufsz);
      copy_point(joints[i].sphere_c, opt_center);
      joints[i].sphere_r = opt_radius;
      for ( int j=0; j<3*pqp->num_tris; j++ )
	delete [] verts[j];
      delete [] verts;
      delete [] buf;
#else
      // simpler approach: use center of obb as center of sphere
      copy_point(joints[i].sphere_c,pqp->b[0].To);
      // and determine the radius as max. distance of vertices from
      // this center (in for-loop below)
#endif

      double d1, d2, d3, t1, t2, t3;
      for ( int j=0; j<pqp->num_tris; j++ ) {

#ifndef OPTIMAL_BOUNDING_SPHERES
	// the simple approach: adjust sphere radius
	d1 = point_dist(pqp->tris[j].p1, joints[i].sphere_c);
	d2 = point_dist(pqp->tris[j].p2, joints[i].sphere_c);
	d3 = point_dist(pqp->tris[j].p3, joints[i].sphere_c);
	if ( d1 > joints[i].sphere_r ) joints[i].sphere_r = d1;
	if ( d2 > joints[i].sphere_r ) joints[i].sphere_r = d2;
	if ( d3 > joints[i].sphere_r ) joints[i].sphere_r = d3;
#endif

	// cylinder
	t1 = project_onto_line(q1, pqp->tris[j].p1, Orig, ax);
	t2 = project_onto_line(q2, pqp->tris[j].p2, Orig, ax);
	t3 = project_onto_line(q3, pqp->tris[j].p3, Orig, ax);
	if ( t1 < min_t ) min_t = t1;
	if ( t2 < min_t ) min_t = t2;
	if ( t3 < min_t ) min_t = t3;
	if ( t1 > max_t ) max_t = t1;
	if ( t2 > max_t ) max_t = t2;
	if ( t3 > max_t ) max_t = t3;
	d1 = point_dist(pqp->tris[j].p1, q1);
	d2 = point_dist(pqp->tris[j].p2, q2);
	d3 = point_dist(pqp->tris[j].p3, q3);
	if ( d1 > joints[i].cyl_r ) joints[i].cyl_r = d1;
	if ( d2 > joints[i].cyl_r ) joints[i].cyl_r = d2;
	if ( d3 > joints[i].cyl_r ) joints[i].cyl_r = d3;

      }
      get_point_on_line(joints[i].cyl_c1, Orig, min_t, ax);
      get_point_on_line(joints[i].cyl_c2, Orig, max_t, ax);      

    }

  }

  // setup the parameter vector and the attributes
  params = new double[num_params];
  param_opts = new param_info[num_params];
  for ( int d=0; d<num_params; d++ )
    params[d] = param_opts[d].default_val;

  {for ( int i=0; i<num_joints; i++ )
    if ( joints[i].param_idx >= 0 )
      joints[i].Tq->bind(params + joints[i].param_idx);
  }

  compute_forward_kinematics();
  update_iv_transf();

  cerr << "  robot has "
  << robot_tri_cnt << " triangles." << endl;

  compute_workspace_bounds();
}

#ifdef DEBUG_SPHERE_VISUALIZATION
// to show the spheres computed by compute_workspace_bounds()
// (for debugging purposes)
void
show_sphere(mpkBaseRobot::Joint* joints,
	    int which, int at_which, double center[3], double r)
{
  int which_=-1, at_which_=-1;
  char* p = getenv("MPK_SPHERE");
  if ( p ) which_=atoi(p);
  p = getenv("MPK_JOINT");
  if ( p ) at_which_=atoi(p);
  if ( which_==which && at_which_==at_which ) {
    SoSeparator* sep = new SoSeparator;
    if ( at_which==-99 ) at_which = which;
    if ( at_which < 0 )
      mpkGUI::scene->addChild(sep);
    else
      joints[at_which].iv_sep->addChild(sep);
    SoTransform* tr = new SoTransform;
    tr->translation.setValue(center[0],center[1],center[2]);
    sep->addChild(tr);
    SoMaterial* mat = new SoMaterial;
    sep->addChild(mat);
    mat->transparency = 0.5;
    SoSphere* s = new SoSphere;
    s->radius = r;
    sep->addChild(s);
  }
}
#endif

// computes overestimates for bound_fact of links around all possible
// axes plus for each link a sphere (in the world frame) that is
// guaranteed to contain the link in any configuration (base_c, base_r
// in Joint)
void
mpkBaseRobot::
compute_workspace_bounds()
{
  const double Orig[] = {0,0,0};

  // for each joint: compute bound factors for all transforms
  for ( int i=0; i<num_joints; i++ ) {

#ifdef DEBUG_SPHERE_VISUALIZATION
    show_sphere(joints,i,-99,joints[i].sphere_c,joints[i].sphere_r);
#endif

    joints[i].bound_fact.clear();
    joints[i].bounds_available=true;

    double sphere_center[3];
    double sphere_radius;

    if ( joints[i].Tq->get_type() == mpkTransform::constant ) {
      copy_point(sphere_center, joints[i].sphere_c);
      sphere_radius = joints[i].sphere_r;
      if ( joints[i].pqp_model[0] )
	joints[i].bound_fact.push_back(0);
    }
    else if ( joints[i].Tq->get_type() == mpkTransform::transl1 ) {
      // sweep sphere along translation within bounds [0,1]
      double param_val;
      double c0[3], c1[3];
      mpkTransform T(*joints[i].Tq);
      T.bind(&param_val);
      param_val=0; T.update(); T.apply(c0, joints[i].sphere_c);
      param_val=1; T.update(); T.apply(c1, joints[i].sphere_c);
      sphere_center[0] = 0.5 * (c0[0] + c1[0]);
      sphere_center[1] = 0.5 * (c0[1] + c1[1]);
      sphere_center[2] = 0.5 * (c0[2] + c1[2]);
      sphere_radius = joints[i].sphere_r + 0.5 * point_dist(c0,c1);
      
      // for prismatic joint, factor equals axis length:
      if ( joints[i].pqp_model[0] )
	joints[i].bound_fact.push_back(point_dist(Orig,joints[i].Tq->get_axis()));
    }
    else if ( joints[i].Tq->get_type() == mpkTransform::transl_rot1 ||
	      joints[i].Tq->get_type() == mpkTransform::rot1 ) {
      // for revolute joint, we keep the cylinder radius
      double cyl_radius = joints[i].cyl_r;

      // 1) enclose cylinder in sphere
      double cyl_height = point_dist(joints[i].cyl_c1, joints[i].cyl_c2);
      sphere_radius = sqrt(cyl_height*cyl_height/4 +
			   cyl_radius*cyl_radius);
      sphere_center[0] = 0.5*(joints[i].cyl_c1[0]+joints[i].cyl_c2[0]);
      sphere_center[1] = 0.5*(joints[i].cyl_c1[1]+joints[i].cyl_c2[1]);
      sphere_center[2] = 0.5*(joints[i].cyl_c1[2]+joints[i].cyl_c2[2]);
      
      // 1') alternative: sweep sphere around axis:
      double sphere_center2[3];
      project_onto_line(sphere_center2, joints[i].sphere_c,
			Orig, joints[i].Tq->get_axis());
      double sphere_radius2 =
	joints[i].sphere_r + point_dist(sphere_center, joints[i].sphere_c);
      
      // 1) and 1'): take the smaller sphere
      if ( sphere_radius2 < sphere_radius ) {
	copy_point(sphere_center,sphere_center2);
	sphere_radius = sphere_radius2;
      }
      if ( joints[i].pqp_model[0] )
	joints[i].bound_fact.push_back(cyl_radius);
    }
    else { // cannot handle transform with generic boundfact algorithm
      joints[i].bounds_available=false;
      continue; // for-i loop with next joint
    }
    
    //#define DUMP_RADII
#ifdef DUMP_RADII
    cout << "Link #" << i << "(" << joints[i].iv_fname0 << ")" << endl;
    cout << "Radius around own axis: " << cyl_radius << " (" << sphere_radius << ")" << endl;
#endif

#ifdef DEBUG_SPHERE_VISUALIZATION
    show_sphere(joints,i,i,sphere_center,sphere_radius);
#endif


    // 2) now go back along kinematic chain, recursively construct sphere
    //    approximations of volume swept by current link #i and compute
    //    bound factors for the other parameters encountered
    int prev = i;
    for (int curr = joints[i].parent_idx;
	 curr >= 0;
	 prev=curr, curr = joints[curr].parent_idx) {

      // transform sphere from previous approximation into current frame
      mpkTransform sphereT(*joints[prev].T0);
      if ( joints[prev].Tq->get_type() == mpkTransform::transl_rot1 ) {
	sphereT.T[0] += joints[prev].Tq->T[0];
	sphereT.T[1] += joints[prev].Tq->T[1];
	sphereT.T[2] += joints[prev].Tq->T[2];
      }
      double tmp[3] = {sphere_center[0], sphere_center[1], sphere_center[2]};
      sphereT.apply(sphere_center, tmp);

      // visualization (for debugging only)
#ifdef DEBUG_SPHERE_VISUALIZATION
      show_sphere(joints,i,curr,sphere_center,sphere_radius);
#endif

      // we only handle revolute joints, prismatic and constant joints
      if ( joints[curr].Tq->get_type() == mpkTransform::constant ) {
	double tmp[3] = {sphere_center[0],sphere_center[1],sphere_center[2]};
	joints[curr].Tq->apply(sphere_center, tmp);
	if ( joints[i].pqp_model[0] )
	  joints[i].bound_fact.push_back(0);
      }
      else if ( joints[curr].Tq->get_type() == mpkTransform::transl1 ) {
	// sweep sphere along translation within bounds [0,1]
	double param_val;
	double c0[3], c1[3];
	mpkTransform T(*joints[curr].Tq);
	T.bind(&param_val);
	param_val=0; T.update(); T.apply(c0, sphere_center);
	param_val=1; T.update(); T.apply(c1, sphere_center);
	sphere_center[0] = 0.5 * (c0[0] + c1[0]);
	sphere_center[1] = 0.5 * (c0[1] + c1[1]);
	sphere_center[2] = 0.5 * (c0[2] + c1[2]);
	sphere_radius += 0.5 * point_dist(c0,c1);
	if ( joints[i].pqp_model[0] )
	  joints[i].bound_fact.push_back(point_dist(Orig,joints[curr].Tq->get_axis()));
      }
      else if ( joints[curr].Tq->get_type() == mpkTransform::transl_rot1 ||
		joints[curr].Tq->get_type() == mpkTransform::rot1 ) {
	// compute distance of (previous) sphere center from current axis
	const double* ax = joints[curr].Tq->get_axis();
	double tmp2[3] = {sphere_center[0],sphere_center[1],sphere_center[2]};
	project_onto_line(sphere_center, tmp2, Orig, ax);
	
	// the new sphere is centered at the current axis and its
	// radius is the sum of the previous sphere's radius plus
	// that one's center distance from current axis
	double d = point_dist(sphere_center, tmp2);
	sphere_radius += d;
	if ( joints[i].pqp_model[0] )
	  joints[i].bound_fact.push_back(sphere_radius);
      }
      else {
	// cannot be handled with generic algorithm -> disable
	joints[i].bounds_available=false;
	break; // for-curr loop (hoing back to root of kinematic chain)
	       // --> goto next joint
      }

	// output only
#ifdef DUMP_RADII
	cout << "Radius around axis " << curr << ": " << sphere_radius
	     << ", center of sphere: ("
	     << sphere_center[0] << ", " 
	     << sphere_center[1] << ", " 
	     << sphere_center[2] << ")"
	     << endl;
#endif

    }

    joints[i].base_sphere_c[0]=joints[i].base_sphere_c[1]=joints[i].base_sphere_c[2]=joints[i].base_sphere_r=0;

    if ( joints[i].pqp_model[0] && joints[i].bounds_available ) {
      // now transform biggest (previous) sphere into robot base frame
      mpkTransform sphereT(*joints[prev].T0);
      if ( joints[prev].Tq->get_type() == mpkTransform::transl_rot1 ) {
	sphereT.T[0] += joints[prev].Tq->T[0];
	sphereT.T[1] += joints[prev].Tq->T[1];
	sphereT.T[2] += joints[prev].Tq->T[2];
      }
      sphereT.apply(joints[i].base_sphere_c, sphere_center);
      joints[i].base_sphere_r = sphere_radius;

      // visualization (for debugging only)
#ifdef DEBUG_SPHERE_VISUALIZATION
      show_sphere(joints,i,-1,joints[i].base_sphere_c,joints[i].base_sphere_r);
#endif
      
#ifdef DUMP_RADII
      cout << endl;
#endif
    }

  }

  // debugging output
  /*
  for ( int i=0; i<num_joints; i++ ) {
    cout << joints[i].name << ": ";
    if ( joints[i].can_move ) cout << "(mov) ";  
    if ( joints[i].bounds_available ) cout << "(bounds) ";
    cout << endl;
    for ( int j=0; j<joints[i].bound_fact.size(); j++ )
      cout << joints[i].bound_fact[j] << " ";
    cout << endl;
    for (int k=0; k<joints[i].kchain_idx.size(); k++ )
      cout << joints[i].kchain_idx[k] << " ";
    cout << endl;
    cout << endl;
  }
  */

}


// the function that returns upper bound on motions in workspace.  It
// works for all generic kinematic trees with any combinations of
// revolute and prismatic joints.  If your robot contains other types
// of joints and more general kinematics, you need to derive a
// 'hardcoded' robot from mpkBaseRobot and overload this function.
// See mpkIrb2400 for an example of how to 'hardwire' a robot.
double
mpkBaseRobot::
greedy_pathlen_upbound(const mpkConfig& q0, const mpkConfig& q1,
		       int first_param_idx, int joint_idx, int root_idx)
{
  if ( !joints[joint_idx].bounds_available ) {
    // no bound factors computed: we can't compute generic bound for
    // workspace pathlength
    cerr << "Error: mpkBaseRobot::pathlen_upbound(): no generic pathlength bound factors for\n"
	 << "robot link " << get_name() << ":" << joints[joint_idx].name << "\n"
	 << "Please either exclude this link from collision checking"
	 << "or overload pathlen_upbound for this robot type with "
	 << "specialized code (generic factors are only computed "
	 << "for trees with revolute and prismatic DOFs)." << endl;
    mpk_exit(1);
  }

  const double Orig[] = {0,0,0};
  double upb = 0;

  double sphere1_center[3],sphere2_center[3];
  double sphere1_radius,sphere2_radius;

  Joint& jcurr = joints[joint_idx];

  if ( jcurr.Tq->get_type() == mpkTransform::constant ) {
    copy_point(sphere1_center,jcurr.sphere_c);
    sphere1_radius = jcurr.sphere_r;
    copy_point(sphere2_center,jcurr.sphere_c);
    sphere2_radius = jcurr.sphere_r;
  }
  else if ( jcurr.Tq->get_type() == mpkTransform::transl1 ) {
    // sweep sphere along translation within bounds given by segment
    double c0[3], c1[3];
    mpkTransform T(*jcurr.Tq);
    int idx = first_param_idx + jcurr.param_idx;
    T.bind(&(q0[idx]));
    T.update(); T.apply(c0, jcurr.sphere_c);
    T.bind(&(q1[idx]));
    T.update(); T.apply(c1, jcurr.sphere_c);
    sphere1_center[0] = 0.5 * (c0[0] + c1[0]);
    sphere1_center[1] = 0.5 * (c0[1] + c1[1]);
    sphere1_center[2] = 0.5 * (c0[2] + c1[2]);
    sphere1_radius = jcurr.sphere_r + 0.5 * point_dist(c0,c1);
    copy_point(sphere2_center, sphere1_center);
    sphere2_radius = sphere1_radius;
    
    // for prismatic joint, factor equals axis length:
    upb += point_dist(Orig,jcurr.Tq->get_axis()) * // axis length
      fabs(jcurr.Tq->max_param()-jcurr.Tq->min_param()) * // param scaling
      fabs(q0[idx]-q1[idx]); // abs. param difference

  }
  else if ( jcurr.Tq->get_type() == mpkTransform::transl_rot1 ||
	    jcurr.Tq->get_type() == mpkTransform::rot1 ) {
    // 1) consider full 2\pi sweep
    //    --> sphere1
    project_onto_line(sphere1_center,jcurr.sphere_c,Orig,jcurr.Tq->get_axis());
    sphere1_radius = jcurr.sphere_r+point_dist(sphere1_center,jcurr.sphere_c);
    
    // 2) sweep sphere around axis using interval bounds
    //    --> sphere2
    double c0[3], c1[3];
    mpkTransform T(*jcurr.Tq);
    int idx = first_param_idx + jcurr.param_idx;
    T.bind(&(q0[idx]));
    T.update(); T.apply(c0, jcurr.sphere_c);
    T.bind(&(q1[idx]));
    T.update(); T.apply(c1, jcurr.sphere_c);
    sphere2_center[0] = 0.5 * (c0[0] + c1[0]);
    sphere2_center[1] = 0.5 * (c0[1] + c1[1]);
    sphere2_center[2] = 0.5 * (c0[2] + c1[2]);
    sphere2_radius = jcurr.sphere_r + 0.5 * point_dist(c0,c1);

    // add (using cyl_radius) to pathlen bound
    upb += jcurr.cyl_r * // cylinder radius
      fabs(jcurr.Tq->max_param()-jcurr.Tq->min_param()) * // param scaling
      fabs(q0[idx]-q1[idx]); // abs. param difference
  }
  else { // cannot handle transform with generic boundfact algorithm
    return -1; // should not happen
  }
  
  // 2) now go back along kinematic chain, recursively construct sphere
  //    approximations of volume swept by current link #i and compute
  //    bound factors for the other parameters encountered
  int prev = joint_idx;
  for (int curr = joints[joint_idx].parent_idx;
       curr > root_idx;
       prev=curr, curr = joints[curr].parent_idx) {
  
    Joint& jcurr = joints[curr];
    Joint& jprev = joints[prev];
  
    // transform spheres from previous approximation into current frame
    mpkTransform sphereT(*jprev.T0);
    if ( jprev.Tq->get_type() == mpkTransform::transl_rot1 ) {
      sphereT.T[0] += jprev.Tq->T[0];
      sphereT.T[1] += jprev.Tq->T[1];
      sphereT.T[2] += jprev.Tq->T[2];
    }
    double tmp1[3]={sphere1_center[0],sphere1_center[1],sphere1_center[2]};
    double tmp2[3]={sphere2_center[0],sphere2_center[1],sphere2_center[2]};
    sphereT.apply(sphere1_center, tmp1);
    sphereT.apply(sphere2_center, tmp2);
        
    // we only handle revolute joints, prismatic and constant joints
    if ( joints[curr].Tq->get_type() == mpkTransform::constant ) {
      // simply pass both spheres down
      double tmp1[3]={sphere1_center[0],sphere1_center[1],sphere1_center[2]};
      double tmp2[3]={sphere2_center[0],sphere2_center[1],sphere2_center[2]};
      jcurr.Tq->apply(sphere1_center, tmp1);
      jcurr.Tq->apply(sphere2_center, tmp2);
    }
    else if ( joints[curr].Tq->get_type() == mpkTransform::transl1 ) {
      // sweep spheres along translation within bounds of current segment
      int idx = first_param_idx + jcurr.param_idx;
      double c10[3], c11[3], c20[3], c21[3];
      mpkTransform T(*joints[curr].Tq);
      T.bind(&(q0[idx]));
      T.update(); T.apply(c10, sphere1_center); T.apply(c20, sphere2_center);
      T.bind(&(q1[idx]));
      T.update(); T.apply(c11, sphere1_center); T.apply(c21, sphere2_center);

      sphere1_center[0] = 0.5 * (c10[0] + c11[0]);
      sphere1_center[1] = 0.5 * (c10[1] + c11[1]);
      sphere1_center[2] = 0.5 * (c10[2] + c11[2]);
      sphere1_radius += 0.5 * point_dist(c10,c11);

      sphere2_center[0] = 0.5 * (c20[0] + c21[0]);
      sphere2_center[1] = 0.5 * (c20[1] + c21[1]);
      sphere2_center[2] = 0.5 * (c20[2] + c21[2]);
      sphere2_radius += 0.5 * point_dist(c20,c21);

      // for prismatic joint, factor equals axis length:
      upb += point_dist(Orig,jcurr.Tq->get_axis()) * // axis length
	fabs(jcurr.Tq->max_param()-jcurr.Tq->min_param()) * // param scaling
	fabs(q0[idx]-q1[idx]); // abs. param difference
    }
    else if ( joints[curr].Tq->get_type() == mpkTransform::transl_rot1 ||
	      joints[curr].Tq->get_type() == mpkTransform::rot1 ) {
      int idx = first_param_idx + jcurr.param_idx;
      // compute distance of (previous) spheres' centers from current axis
      // and (greedily) pick the "better" one
      const double* curr_ax = jcurr.Tq->get_axis();
      double proj1[3], proj2[3];
      project_onto_line(proj1, sphere1_center, Orig, curr_ax);
      project_onto_line(proj2, sphere2_center, Orig, curr_ax);

      // greedy pick: the "better" sphere is the one resulting in a
      // smaller sweep radius
      double d1 = point_dist(sphere1_center, proj1);
      double d2 = point_dist(sphere2_center, proj2);
      double sphere_center[3], sphere_radius;
      double proj_dist;
      if ( d1 + sphere1_radius < d2 + sphere2_radius ) {
	copy_point(sphere_center,sphere1_center);
	sphere_radius = sphere1_radius;
	proj_dist = d1;
      }
      else {
	copy_point(sphere_center,sphere2_center);
	sphere_radius = sphere2_radius;
	proj_dist = d2;
      }

      // compute bound
      upb += (sphere_radius+proj_dist) * // sweep radius
	fabs(jcurr.Tq->max_param()-jcurr.Tq->min_param()) * // param scaling
	fabs(q0[idx]-q1[idx]); // abs. param difference

      // now construct two new candidate spheres by sweeping current
      // sphere 1) by 2\pi and 2) according to interval bounds
      
      // 1) consider full 2\pi sweep
      //    --> sphere1
      project_onto_line(sphere1_center,sphere_center,Orig,curr_ax);
      sphere1_radius = sphere_radius+point_dist(sphere1_center,sphere_center);
      
      // 2) sweep sphere around axis using interval bounds
      //    --> sphere2
      double c0[3], c1[3];
      mpkTransform T(*jcurr.Tq);
      T.bind(&(q0[idx]));
      T.update(); T.apply(c0, sphere_center);
      T.bind(&(q1[idx]));
      T.update(); T.apply(c1, sphere_center);
      sphere2_center[0] = 0.5 * (c0[0] + c1[0]);
      sphere2_center[1] = 0.5 * (c0[1] + c1[1]);
      sphere2_center[2] = 0.5 * (c0[2] + c1[2]);
      sphere2_radius = sphere_radius + 0.5 * point_dist(c0,c1);
    }
    else {
      // cannot be handled with generic algorithm -> disable
      jcurr.bounds_available=false;
      break; // for-curr loop (going back to root of kinematic chain)
      // --> goto next joint
    }
    
  }
  
  return upb;
}




// get triangles from inventor models
void mpkBaseRobot::triang_CB(void *data, SoCallbackAction *,
		      const SoPrimitiveVertex *vertex1,
		      const SoPrimitiveVertex *vertex2,
		      const SoPrimitiveVertex *vertex3)
{
  tri_info* info = (tri_info*)data;
  if ( !info->pqp_model ) return;

  SbVec3f points[] = { vertex1->getPoint(), vertex2->getPoint(), vertex3->getPoint() };
  double verts[3][3];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      verts[i][j] = points[i][j] * info->scalef;
      //cout << points[i][j] << " ";
    }
    //    cout << " * ";
  }
  info->pqp_model->AddTri(verts[0], verts[1], verts[2], info->tri_cnt++);
  //  cout << endl;
}


// update all transforms according to current parameter values in
// param.
void
mpkBaseRobot::
compute_forward_kinematics()
{
  for ( int i=0; i<num_joints; i++ ) {
    Joint& curr_joint = joints[i];
    int parent = curr_joint.parent_idx;
    if ( parent < 0 ) {
#ifdef ROBOT_BASE_MOVABLE
      curr_joint.Taccu.set_product(base_T, *curr_joint.T0);
      curr_joint.Tq->update();
      curr_joint.Taccu.append(*curr_joint.Tq);      
#else
      if ( curr_joint.Tq->update() )
	curr_joint.Taccu.set_product(*curr_joint.T0,*curr_joint.Tq);
#endif
    }
    else {
      curr_joint.Taccu.set_product(joints[parent].Taccu, *curr_joint.T0);
      curr_joint.Tq->update();
      curr_joint.Taccu.append(*curr_joint.Tq);
    }
  }
}


// update transforms of joint with index joint_idx according to
// current parameter values in param.
void
mpkBaseRobot::
compute_forward_kinematics(int joint_idx)
{
  const vector<int>& idx = joints[joint_idx].kchain_idx;
  int i = idx.size()-1;

  // first compute Taccu for the root of path to joints[joint_idx]
  Joint& curr_joint = joints[idx[i]];
#ifdef ROBOT_BASE_MOVABLE
  curr_joint.Taccu.set_product(base_T, *curr_joint.T0);
  curr_joint.Tq->update();
  curr_joint.Taccu.append(*curr_joint.Tq);      
#else
  if ( curr_joint.Tq->update() )
    curr_joint.Taccu.set_product(*curr_joint.T0,*curr_joint.Tq);
#endif

  // then compute Taccu down the chain to joints[joints_idx]
  for (--i; i>=0; i-- ) {
    int parent = idx[i+1];
    Joint& curr_joint = joints[idx[i]];
    curr_joint.Taccu.set_product(joints[parent].Taccu, *curr_joint.T0);
    curr_joint.Tq->update();
    curr_joint.Taccu.append(*curr_joint.Tq);
  }
}

// update transforms of joint with index joint_idx according to
// current parameter values in param (using the transform cache).
#ifndef NO_TRANSFORM_CACHE
void
mpkBaseRobot::
compute_forward_kinematics(int joint_idx, unsigned t, mpkTransformCache* cache)
{
  const vector<int>& idx = joints[joint_idx].kchain_idx;

  int i;
  for ( i=1; i<idx.size(); i++ ) {
    if ( cache->load(joints[idx[i]].Taccu, (unsigned)this, (unsigned)idx[i], t) )
      break;
  }
  --i;

  if ( i == idx.size()-1 ) {

    // first compute Taccu for the root of path to joints[joint_idx]
    Joint& curr_joint = joints[idx[i]];
#ifdef ROBOT_BASE_MOVABLE
    curr_joint.Taccu.set_product(base_T, *curr_joint.T0);
    curr_joint.Tq->update();
    curr_joint.Taccu.append(*curr_joint.Tq);      
#else
    if ( curr_joint.Tq->update() )
      curr_joint.Taccu.set_product(*curr_joint.T0,*curr_joint.Tq);
#endif
    cache->insert(curr_joint.Taccu, (unsigned)this, (unsigned)idx[i], t);
    --i;
  }

  // then compute Taccu down the chain to joints[joints_idx]
  for (; i>=0; i-- ) {
    int parent = idx[i+1];
    Joint& curr_joint = joints[idx[i]];
    curr_joint.Taccu.set_product(joints[parent].Taccu, *curr_joint.T0);
    curr_joint.Tq->update();
    curr_joint.Taccu.append(*curr_joint.Tq);
    cache->insert(curr_joint.Taccu, (unsigned)this, (unsigned)idx[i], t);
  }
}
#endif

// simple scanner for .rob files (to skip comments and read tokens)
// -----------------------------------------------------------------

mpkBaseRobot::RobFileScanner::
RobFileScanner(const char* fname, const char* path1, const char* path2)
{
  this->fname = fname;
  f = new ifstream(fname);
  if ( !(*f) ) {
    delete f;
    if ( path1 ) f = new ifstream(string(string(path1) + "/" + fname).c_str());
    if ( !path1 || !(*f) ) {
      delete f;
      if ( path2 ) f = new ifstream(string(string(path2) + "/" + fname).c_str());
      if ( !path2 || !(*f) ) {
	cerr << "Could not find file " << fname << endl;
	mpk_exit(1);
      }
    }
  }
  string s1,s2,s3;
  (*f) >> s1;
  (*f) >> s2;
  (*f) >> s3;
  if ( s1!="#MPK" || s2!="v1.0" || s3!="robot" ) {
    cerr << "Error: not an MPK v1.0 robot file: " << fname << endl;
    mpk_exit(1);
  }
}


bool
mpkBaseRobot::RobFileScanner::
get_token(string& s, const char* expected, bool exit_on_eof)
{
  if ( !read_string(s,expected) ) {
    if ( !exit_on_eof ) return false;
    cerr << "Problem reading file " << fname << endl;
    mpk_exit(1);
  }
  if ( expected && s != expected ) {
    cerr << "Error in file " << fname
	 << ": expected '" << expected << "' but read '" << s << "'" << endl;
    mpk_exit(1);
  }
  return true;
}

void
mpkBaseRobot::RobFileScanner::
get_string_token(string& s)
{
  if ( !read_string(s) ) {
    cerr << "Problem reading file " << fname << endl;
    mpk_exit(1);
  }
  if ( s[0]!='"' || s[s.length()-1]!='"' ) {
    cerr << "Error in file " << fname
	 << ": expected string delimiter \"" << endl;
    mpk_exit(1);
  }
  s.replace(0,1,"");
  s.replace(s.length()-1,1,"");
}

bool
mpkBaseRobot::RobFileScanner::
read_string(string& s, const char* expected)
{
  s = "";
  int c;
  // read "garbage": whitespace and comments
  while ( 1 ) {
    c=f->get();
    if ( c==EOF ) return false;
    if ( c=='#' ) {
      while ( c!=EOF && c!=10 )
	c=f->get();
    }
    if ( !isspace(c) ) break;
  }
  s += c;
  while ( 1 ) {
    if ( expected && s==expected ) break;
    c=f->get();
    if ( c==EOF ) break;
    if ( isspace(c) || c=='{' || c=='}' || c=='#' ) {
      f->putback(c);
      break;
    }
    s += c;
  }
  return true;
}


#ifndef MPK_BASE_ROBOT_H
#define MPK_BASE_ROBOT_H

#include <string>
#include <vector>
#include <fstream>

#include "mpk_defs.h"
#include "mpk_inventor.h"
#include "PQP.h"
#include "mpkConfig.H"
#include "mpkTransform.H"

#ifdef TRANSFORM_CACHE
#include "mpkTransformCache.H"
#endif

/**@memo The base class for all robots.  It can represent arbitrary
   kinematic trees, free-flying objects and objects moving along
   complex constrained trajectories.

   @doc The constructor either reads the kinematics from a .rob file
   or takes the description directly as a pointer to an array
   (jointdefs).  It creates a collision model of the robot and an
   inventor scene graph (for displaying only).  Note that the scene
   graph is only loosely coupled to the collision model: an explicit
   call to {@link update_iv_transf update_iv_transf()} is required to
   synchronize the inventor model with the collision model.  This
   allows for more efficient computations during planning where
   displayed model should not move around.  The array {@link params
   params} of size {@link num_params num_params} contains all
   parameters of the robot.  After changing an entry in this array,
   one of the functions {@link compute_forward_kinematics
   compute_forward_kinematics()} must be called to update the
   collision model.
 */
class mpkBaseRobot {

public:

  /**@memo Structure to define a joint.  Each joint contains
     information about its parent joint, spatial transforms and an
     optional attached triangulated link model.  This information is
     necessary to define a robot.
   */
  struct JointDef {
    const char* name;      // name of joint
    int parent_idx;        // idx of parent joint
    mpkTransform* T0;      // parent transform (not parameterized)
    mpkTransform* Tq;      // parameter transform (need not be parameterized)
    int param_idx;         // index of first parameter in param that Tq reads
    const char* iv_fname0; // file name of link (collision) model
    const char* iv_fname1; // file name of (optional) second model
    bool check_coll;       // check this link model for collisions?
  };

  /**@memo Structure to represent a joint (adds some working data to
     JointDef).  The fields not present in JointDef are derived
     automatically from those in JointDef.
   */
  struct Joint : JointDef {
    mpkTransform Taccu;       // accumulated transforms along kinematic chain
    PQP_Model* pqp_model[2];  // pqp collision models for associated link
    vector<int> kchain_idx;   // joint indices along kinematic chain back to root
    vector<double> bound_fact;// bound factors along kinematic chain back to root
    double pathlen_upbound;   // upper bound on path length (in workspace)
    double base_sphere_c[3];  // enclosing sphere for link in any configuration
    double base_sphere_r;     // of the robot (in base coordinate frame)
    bool bounds_available;    // bound_fact and base_sphere computed?
    bool can_move;            // at least one parameter transform on path to root?
    double cyl_c1[3];         // enclosing cyl (for revolute joints only)
    double cyl_c2[3];
    double cyl_r;
    double sphere_c[3];       // enclosing sphere for link (in local frame)
    double sphere_r;
    SoSeparator* iv_sep;      // separator for link in inventor scene graph
    SoTransform* iv_Tq;       // relative transform of link in iv scene graph
  };

  /**@memo Constructs a robot with the given name from a file in .rob
     format.  The robot can be placed at any place in the world
     coordinate frame using base_transf and it can be rescaled using
     scalef.
   */
  mpkBaseRobot(const char* name, const char* rob_file_name,
	       const mpkTransform& base_transf, double scalef=1.0);

  /**@memo Constructs a robot with the given name from a 'hardcoded'
     description given by jointdefs.  num_joints is the number of
     entries in jointdefs.  The robot can be placed at any place in
     the world coordinate frame using base_transf and it can be
     rescaled using scalef.
   */
  mpkBaseRobot(const char* name, int num_joints, JointDef* jointdefs,
	       const mpkTransform& base_transf, double scalef=1.0);

  ~mpkBaseRobot();

  ///@memo Compute transforms (Taccu) for all joints.
  void compute_forward_kinematics();

  /**@memo Compute transforms (Taccu) for path along kinematic chain
     from joint_idx to root of kinematic tree.*/
  void compute_forward_kinematics(int joint_idx);

  /**@memo Compute transforms (Taccu) for path along kinematic chain
     from joint_idx to root of kinematic tree.  This version takes
     advantage of the kinematics cache.*/
#ifdef TRANSFORM_CACHE
  void compute_forward_kinematics(int joint_idx, unsigned t, mpkTransformCache* cache);
#endif

  ///@memo Updates the transformations of the robot model in the scene graph.
  void update_iv_transf();

  ///@memo Checks links {\bf i} and {\bf j} for collision.
  bool collision(int i, int j, int model_id=0);

  /**@memo Checks link {\bf i} and the given PQP model {\bf obstacle} at
      pose {\bf (R,T)} for collision.
  */
  bool collision(int i, double R[3][3], double T[3], PQP_Model* obstacle,
		 int model_id=0);

  /**@memo Returns the exact distance between the links {\bf i} and
     {\bf j}.*/
  double distance(int i, int j, double rel_err=0, int model_id=0);

  /**@memo Returns the exact distance between link {\bf i} and the
      given PQP model {\bf obstacle} at pose {\bf (R,T)}.
  */
  double distance(int i, double R[3][3], double T[3], PQP_Model* obstacle,
		  double rel_err=0, int model_id=0);

  /**@memo Determines if the exact distance between links {\bf i}
     and {\bf j} is smaller than {\bf thres}.  Note that this is much
     faster than exact distance computation.*/
  bool closer_than_thres(int i, int j, double thres=0, int model_id=0);

  /**@memo Determines if the exact distance between link {\bf i} the
     given PQP model {\bf obstacle} at pose {\bf (R,T)} is smaller
     than {\bf thres}.  Note that this is much faster than exact
     distance computation.*/
  bool closer_than_thres(int i, double R[3][3], double T[3], PQP_Model* obstacle,
			 double thres=0, int model_id=0 );

  /**@memo Translate kinematic tree by {\bf (dx,dy,dz)} (moves entire
     robot).
   */
  void translate_base(double dx, double dy, double dz);

  ///@memo Returns the name of the robot as given in the scene file.
  const char* get_name() {return iv_graph->getName().getString();};

  ///@memo Array of joints (created from jointdefs by constructor).
  Joint* joints;

  ///@memo Number of entries in joints.
  int num_joints;

  ///@memo The vector of normalized parameters [0,1) of the robot (its DOFs).
  double* params;

  ///@memo Number of DOFs (entries in params, param_frozen and param_passive).
  int num_params;

  ///@memo Structure to bundle attributes of a single parameter
  struct param_info {
    param_info() {
      default_val = 0.5; is_cyclic = false; is_passive = false;
      is_frozen = false; weight = 1;
    };
    double default_val; // default parameter value (usually 0.5)
    bool is_cyclic;     // cyclic parameter (currently not used)
    bool is_passive;    // passive parameters should not be sampled by planner
    bool is_frozen;     // to temporarily make non-passive parameters passive
    double weight;      // weighting parameters is useful for animation etc.
  };

  ///@memo Attributes for the parameters
  param_info *param_opts;

  /**@memo The scale factor of the robot (each instantiated robot can
     have a different scale factor).*/
  double scale_factor;

  /**@memo Self-collision pair structure (simply a pair of indices into
     {@link joints joints}).*/
  struct selfcoll_pair {
    selfcoll_pair(int i, int j): joint1(i), joint2(j) {};
    int joint1, joint2;
  };

  /**@memo Pairs of joints whose links should be checked for collisions
     (can be set individually by the derived class).*/
  vector<selfcoll_pair> selfcoll_info;

  /**@memo Inventor scene graph for robot (note: must be updated
     explicitly using {@link update_iv_transf update_iv_transf()}).*/
  SoSeparator* iv_graph;

  /**@memo Index of joint for which the traced path in workspace will
     be visualized by the class TraceVis (typically the end-effector
     tip but can be an arbitrary point).*/
  int tracepoint_joint_idx;

  /**@memo Function that returns an upper bound on the motion of all
     points of the link model in the joint at joint_idx during a
     motion from q0 to q1.  first_param_idx must tell the robot where
     its parameters start in the configuration vectors (which may
     contain multi-robot params).  If root_idx >= 0, then the
     computation is w.r.t. the frame of joint root_idx (used for
     self-collision bounds of two links on the same kinematic
     chain).*/
  virtual double pathlen_upbound(const mpkConfig& q0, const mpkConfig& q1,
				 int first_param_idx,
				 int joint_idx, int root_idx=-1);

  /** @memo Movable base transform for collision model of robot.*/
  mpkTransform base_T;
  
  /**@memo Movable base transform for scene graph of robot.*/
  SoTransform* iv_base_T;
  
private:

  // used internally by the constructors
  void init(const char* name, int num_joints, JointDef* jointdefs,
	    const mpkTransform& base_transf, double scalef=1.0);

  // computes parameter factors and enclosing spheres for all links
  // (called by init only)
  void compute_workspace_bounds();

  // disable copy constructor & assignment operator for the public 
  mpkBaseRobot(const mpkBaseRobot& rob) {};
  mpkBaseRobot& operator=(const mpkBaseRobot& rob) {return *this;};

  // information for communication with triangulation callback (only needed
  // in constructor to get the triangles from the inventor nodes)
  struct tri_info {
    tri_info(PQP_Model* model, double scaleFactor, int cnt=0) {
      pqp_model = model; scalef = scaleFactor; tri_cnt=cnt;
    };
    PQP_Model* pqp_model;
    double scalef;
    int tri_cnt;
  };
  // the triangulation callback: given three vertices, it adds the 
  // triangle to the pqp model that is referenced in the tri_info
  // structure (and to which data points)
  static void triang_CB(void *data, SoCallbackAction *,
			const SoPrimitiveVertex *vertex1,
			const SoPrimitiveVertex *vertex2,
			const SoPrimitiveVertex *vertex3);

  // helper function to read a link model
  SoSeparator* read_link_model(const char link_fname[]);

  // temporary storage for the results inside the collision(),
  // distance() and closer_than_thres() member functions
  PQP_CollideResult pqp_coll_result;
  PQP_DistanceResult pqp_dist_result;
  PQP_ToleranceResult pqp_tol_result;

  // simple scanner for .rob file format (to skip comments and get tokens)
  class RobFileScanner {
  public:
    RobFileScanner(const char* fname, const char* path1=0, const char* path2=0);
    ~RobFileScanner() {delete f;};
    bool get_token(string& s, const char* expected=0, bool exit_on_eof=true);
    void get_string_token(string& s);
  private:
    bool read_string(string& s, const char* expected=0);
    ifstream* f;
    const char* fname;
  };


};


/////////////////////////////////////////////////////////////////////
// Below are only inline implementations of some member functions. //
/////////////////////////////////////////////////////////////////////

// updates the transforms in the robot scene graph
inline
void
mpkBaseRobot::
update_iv_transf()
{
  for ( int i=0; i<num_joints; i++ )
    joints[i].iv_Tq->setMatrix(*joints[i].Tq);
}


// collision of link i and link j ??
inline
bool
mpkBaseRobot::
collision(int i, int j, int model_id)
{
  Joint& joint_i = joints[i];
  Joint& joint_j = joints[j];
  if ( !joint_i.pqp_model[model_id] ||
       !joint_j.pqp_model[model_id] )
    return false;
  PQP_Collide(&pqp_coll_result,
	      joint_i.Taccu.R, joint_i.Taccu.T, joint_i.pqp_model[model_id],
	      joint_j.Taccu.R, joint_j.Taccu.T, joint_j.pqp_model[model_id],
	      PQP_FIRST_CONTACT);
  return pqp_coll_result.Colliding();
}

// collision of link i and obstacle ??
inline
bool
mpkBaseRobot::
collision(int i, double R[3][3], double T[3], PQP_Model* obstacle, int model_id )
{
  Joint& joint_i = joints[i];
  if ( !joint_i.pqp_model[model_id] )
    return false;
  PQP_Collide(&pqp_coll_result,
	      joint_i.Taccu.R, joint_i.Taccu.T, joint_i.pqp_model[model_id],
	      R, T, obstacle,
	      PQP_FIRST_CONTACT);
  return pqp_coll_result.Colliding();
}

// distance between link i and link j 
inline
double
mpkBaseRobot::
distance(int i, int j, double rel_err, int model_id)
{
  Joint& joint_i = joints[i];
  Joint& joint_j = joints[j];
  if ( !joint_i.pqp_model[model_id] ||
       !joint_j.pqp_model[model_id] )
    return DBL_MAX;
  PQP_Distance(&pqp_dist_result,
	       joint_i.Taccu.R, joint_i.Taccu.T,
	       joint_i.pqp_model[model_id],
	       joint_j.Taccu.R, joint_j.Taccu.T,
	       joint_j.pqp_model[model_id],
	       rel_err,  // rel. error bound (see PQP doc)
	       DBL_MAX);    // abs error bound
  return pqp_dist_result.Distance();
}

// distance between link i and obstacle 
inline
double
mpkBaseRobot::
distance(int i, double R[3][3], double T[3], PQP_Model* obstacle, double rel_err,
	 int model_id)
{
  Joint& joint_i = joints[i];
  if ( !joint_i.pqp_model[model_id] )
    return DBL_MAX;
  PQP_Distance(&pqp_dist_result,
	       joint_i.Taccu.R, joint_i.Taccu.T,
	       joint_i.pqp_model[model_id],
	       R, T, obstacle,
	       rel_err,  // rel. error bound (see PQP doc)
	       DBL_MAX);    // abs error bound
  return pqp_dist_result.Distance();
}

// closer than thres?
inline 
bool 
mpkBaseRobot::
closer_than_thres(int i, int j, double thres, int model_id)
{
  Joint& joint_i = joints[i];
  Joint& joint_j = joints[j];
  if ( !joint_i.pqp_model[model_id] ||
       !joint_j.pqp_model[model_id] )
    return false;
  PQP_Tolerance(&pqp_tol_result,
		joint_i.Taccu.R, joint_i.Taccu.T,
		joint_i.pqp_model[model_id],
		joint_j.Taccu.R, joint_j.Taccu.T,
		joint_j.pqp_model[model_id],
		thres);
  return pqp_tol_result.CloserThanTolerance();
}

// closer than thres?
inline
bool
mpkBaseRobot::
closer_than_thres(int i, double R[3][3], double T[3], PQP_Model* obstacle,
		  double thres, int model_id)
{
  Joint& joint_i = joints[i];
  if ( !joint_i.pqp_model[model_id] )
    return false;
  PQP_Tolerance(&pqp_tol_result,
		joint_i.Taccu.R, joint_i.Taccu.T,
		joint_i.pqp_model[model_id],
		R, T, obstacle,
		thres);
  return pqp_tol_result.CloserThanTolerance();
}

// translate the robot base
inline
void
mpkBaseRobot::
translate_base(double dx, double dy, double dz)
{
  base_T.T[0] += dx;
  base_T.T[1] += dy;
  base_T.T[2] += dz;
  iv_base_T->setMatrix(base_T);
}

#endif

#include "mpkCollPair.H"
#include "mpk_rand.h"

mpkCollPair::
mpkCollPair(mpkBaseRobot* rob1, int first_param_idx1, int joint_idx1,
	     PQP_Model* obst_pqp, mpkTransform* obst_transf,
	     const char obst_name[],
	     double delta)
{
  this->delta = delta;
  this->root_idx = -1; // common static reference is world frame
  
  o1.rob = rob1;
  o1.first_param_idx = first_param_idx1;
  o1.joint_idx = joint_idx1;
  o1.pqp[0] = rob1->joints[joint_idx1].pqp_model[0];
  o1.pqp[1] = rob1->joints[joint_idx1].pqp_model[1];
  o1.Tr = &rob1->joints[joint_idx1].Taccu;
  // concat robot name and link name into o1.name
  const char* joint_name = rob1->joints[joint_idx1].name;
  o1.name = new char[strlen(rob1->get_name())+strlen(joint_name)+2];
  strcpy(o1.name,rob1->get_name());
  o1.name[strlen(rob1->get_name())] = ':';
  strcpy(o1.name+strlen(rob1->get_name())+1, joint_name);
  
  o2.rob = 0;
  o2.first_param_idx = -1;
  o2.joint_idx = -1;
  o2.pqp[0] = obst_pqp;
  o2.pqp[1] = obst_pqp;
  o2.Tr = obst_transf;
  o2.name = new char[strlen(obst_name)+1];
  strcpy(o2.name, obst_name);
}

mpkCollPair::
mpkCollPair(mpkBaseRobot* rob1, int first_param_idx1, int joint_idx1,
	     mpkBaseRobot* rob2, int first_param_idx2, int joint_idx2,
	     double delta)
{
  this->delta = delta;
  
  o1.rob = rob1;
  o1.first_param_idx = first_param_idx1;
  o1.joint_idx = joint_idx1;
  o1.pqp[0] = rob1->joints[joint_idx1].pqp_model[0];
  o1.pqp[1] = rob1->joints[joint_idx1].pqp_model[1];
  o1.Tr = &rob1->joints[joint_idx1].Taccu;
  // concat robot name and link name into o1.name
  const char* joint_name = rob1->joints[joint_idx1].name;
  o1.name = new char[strlen(rob1->get_name())+strlen(joint_name)+2];
  strcpy(o1.name,rob1->get_name());
  o1.name[strlen(rob1->get_name())] = ':';
  strcpy(o1.name+strlen(rob1->get_name())+1, joint_name);

  o2.rob = rob2;
  o2.first_param_idx = first_param_idx2;
  o2.joint_idx = joint_idx2;
  o2.pqp[0] = rob2->joints[joint_idx2].pqp_model[0];
  o2.pqp[1] = rob2->joints[joint_idx2].pqp_model[1];
  o2.Tr = &rob2->joints[joint_idx2].Taccu;
  // concat robot name and link name into o1.name
  joint_name = rob2->joints[joint_idx2].name;
  o2.name = new char[strlen(rob2->get_name())+strlen(joint_name)+2];
  strcpy(o2.name,rob2->get_name());
  o2.name[strlen(rob2->get_name())] = ':';
  strcpy(o2.name+strlen(rob2->get_name())+1, joint_name);

  root_idx = -1;
  // if pair is a self-collision pair (i.e., two links of same robot),
  // set root_idx to first common ancestor in kinematic tree (avoids
  // computing bad pathlength bounds in world-coordinates)
  if ( rob1 == rob2 ) {
    mpkBaseRobot::Joint& j1 = rob1->joints[joint_idx1];
    mpkBaseRobot::Joint& j2 = rob2->joints[joint_idx2];
    int i1,i2;
    //cout << j1.name << "-" << j2.name << " backtracking" << endl;
    for ( i1 = j1.kchain_idx.size()-1, i2 = j2.kchain_idx.size()-1;
	  i1 >= 0 && i2 >= 0 && j1.kchain_idx[i1] == j2.kchain_idx[i2];
	  i1--, i2--) {
      //cout << j1.kchain_idx[i1] << " " << j2.kchain_idx[i2] << endl;
    }
    if ( i1 < 0 || i1 < j1.kchain_idx.size()-1 )
      root_idx = j1.kchain_idx[i1+1];
    // else root_idx remains -1
    //if ( root_idx > 0 )
    //cout << "root=" << rob1->joints[root_idx].name << endl;
    //cout << endl;
  }
}


// copy constructor
mpkCollPair::
mpkCollPair(const mpkCollPair& x)
{
  delta = x.delta;
  root_idx = x.root_idx;

  o1 = x.o1;
  o1.name = new char[strlen(x.o1.name)+1];
  strcpy(o1.name, x.o1.name);

  o2 = x.o2;
  o2.name = new char[strlen(x.o2.name)+1];
  strcpy(o2.name, x.o2.name);
}

void
mpkCollPair::
compute_transf(const mpkConfig& q) const
{
  // set parameters of first robot
  for ( int i=0; i < o1.rob->num_params; i++ )
    o1.rob->params[i] = q[o1.first_param_idx + i];
  // compute link transform
  o1.rob->compute_forward_kinematics(o1.joint_idx);

  // if second object is a robot link, too...
  if ( o2.rob ) {
    // ... then do the same as above
    for ( int i=0; i < o2.rob->num_params; i++ )
      o2.rob->params[i] = q[o2.first_param_idx + i];
    o2.rob->compute_forward_kinematics(o2.joint_idx);
  }
}

#ifdef TRANSFORM_CACHE
void
mpkCollPair::
compute_transf(const mpkConfig& q, unsigned t, mpkTransformCache* cache) const
{
  // first, try to lookup the transform in cache
  if (!cache->load(o1.rob->joints[o1.joint_idx].Taccu,
		  (unsigned)o1.rob, (unsigned)o1.joint_idx, t)) {
    // if not found, set parameters of first robot...
    for ( int i=0; i < o1.rob->num_params; i++ )
      o1.rob->params[i] = q[o1.first_param_idx + i];
    // and compute transform
    o1.rob->compute_forward_kinematics(o1.joint_idx, t, cache);
  }

  // if second object is a robot link, too...
  if ( o2.rob ) {
    // ... then do the same as above
    if (!cache->load(o2.rob->joints[o2.joint_idx].Taccu,
		    (unsigned)o2.rob, (unsigned)o2.joint_idx, t)) {
      for ( int i=0; i < o2.rob->num_params; i++ )
	o2.rob->params[i] = q[o2.first_param_idx + i];
      o2.rob->compute_forward_kinematics(o2.joint_idx, t, cache);
    }
  }
}
#endif


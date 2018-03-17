#ifndef MPK_ROBOT_COLLECTION_H
#define MPK_ROBOT_COLLECTION_H

#include <vector>
#include "mpk_inventor.h"
#include "mpkBaseRobot.H"

/**@memo Extracts all robots (nodes of type {@link mpkRobot mpkRobot})
   from an Inventor scene graph and builds PQP collision models for
   them.
   
   @see mpkObstacle
 */
class mpkRobotCollection {

public:

  /**@doc {\bf scenegraph} is the inventor scene graph which will be
     parsed for mpkRobot nodes.
  */
  mpkRobotCollection(SoGroup* scenegraph);
  ~mpkRobotCollection() {};

  ///
  mpkBaseRobot* operator[](int i) {return rob[i];};
  const mpkBaseRobot* operator[](int i) const {return rob[i];};

  ///@doc Returns the number of robots in the collection.
  int num_robots() const {return rob.size();};

  /**@doc Returns the total number of parameters (DOFs) of all robots
     together (the dimension of the cross-product c-space of all
     robots).
   */
  int num_params() const {return num_all_params;};

  /**@doc Returns for robot with index {\bf robidx} where its
     parameters start in the joint configuration of all robots in the
     collection.  For {\bf robidx} this value is always 0 and for
     subsequent robots it equals the sum of the numbers of parameters
     of all robots with smaller indices.
  */
  int first_param_idx(int robidx) const {return first_idx[robidx];};

  /**@doc Sets the joint configuration of the robots to {\bf conf}.
     Note: subsequent explicit calls to
     {@link compute_forward_kinematics compute_forward_kinematics}
     and
     {@link update_iv_transf update_iv_transf}
     necessary to update the collision model and Inventor scene graph
     transforms of the robots.
   */
  void set_config(const vector<double>& conf);

  ///@doc Copies the joint configuration of the robots to {\bf conf}.
  void get_config(vector<double>& conf) const;

  /**@doc Computes the forward kinematics for robot with index {\bf
     rob_idx} or all robots (if {\bf rob_idx==-1}).     
   */
  void compute_forward_kinematics(int rob_idx=-1);

  /**@doc Updates the Inventor scene graph transfors for robot with
     index {\bf rob_idx} or all robots (if {\bf rob_idx==-1}).
     Note: 
     {@link compute_forward_kinematics compute_forward_kinematics}
     must have been called before to update the kinematics of the robot.
   */
  void update_iv_transf(int rob_idx=-1); // -1 == all

  ///@memo Pointers to the robots in the collection.
  vector<mpkBaseRobot*> rob;

  /**@memo weights for parameters (useful for animation discretization
     etc.).*/
  vector<double> param_weights; 

private:

  // first_idx contains for each robot in rob vector its first index
  // in the configuration of all robots
  vector<int> first_idx;

  int num_all_params;
};

inline void mpkRobotCollection::set_config(const vector<double>& conf)
{
  assert(conf.size() == num_all_params );
  int c=0;
  for ( int i=0; i<num_robots(); i++ )
    for ( int j=0; j<rob[i]->num_params; j++ ) {
      if ( !rob[i]->param_opts[j].is_frozen )
	rob[i]->params[j] = conf[c];
      c++;
    }
}

inline void mpkRobotCollection::get_config(vector<double>& conf) const
{
  conf = vector<double>(num_all_params);
  int c=0;
  for ( int i=0; i<num_robots(); i++ )
    for ( int j=0; j<rob[i]->num_params; j++ )
      conf[c++] = rob[i]->params[j];
}

inline void mpkRobotCollection::compute_forward_kinematics(int rob_idx)
{
  if ( rob_idx >=0 )
    rob[rob_idx]->compute_forward_kinematics();
  else {
    for ( int i=0; i<num_robots(); i++ )
      rob[i]->compute_forward_kinematics();
  }
}

inline void mpkRobotCollection::update_iv_transf(int rob_idx)
{
  if ( rob_idx >=0 )
    rob[rob_idx]->update_iv_transf();
  else {
    for ( int i=0; i<num_robots(); i++ )
      rob[i]->update_iv_transf();
  }
}
#endif

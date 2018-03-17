#ifndef MPK_BV_CONTROL_H
#define MPK_BV_CONTROL_H

#include "mpk_inventor.h"
#include "mpkRobotCollection.H"
#include "mpkObstacleCollection.H"

/**@memo Functions for visualization of bounding volumes (BVs) and
   related functions.*/
class mpkBVControl {

public:

  ///
  enum bv_type {none, OBB, RSS};

  ///
  static void init(mpkRobotCollection* robots, mpkObstacleCollection* obstacles,
		   int bv_level=0);

  ///
  static SoSwitch* ivroot() {return root;};
  ///
  static void set_shown_bv(bv_type new_bv_type);
  ///
  static bv_type get_shown_bv() {return curr_type;};
  ///
  static void update_iv_transf();

  ///
  static void add_wire_cube(SoSeparator* root, double min[3], double max[3],
			    double r=1, double g=1, double b=1);

  /**@memo Samples a point uniformly at random from RSS bv and stores
     the result in p.*/
  static void sample_on_RSS(double p[3], const BV& bv);

private:

  ///
  static mpkRobotCollection* robots;
  ///
  static vector<SoTransform*> bv_transf;
  ///
  static bv_type curr_type;
  ///
  static SoSwitch* root;
  ///
  static SoSeparator* obb_sep;
  ///
  static SoSeparator* rss_sep;
  
  ///
  static void add_bv(SoSeparator* sep,
		     bv_type bv_type, const BV& bv, mpkTransform parent_tr);

  ///
  static void add_bv_level(SoSeparator* sep,
			   bv_type bv_type, PQP_Model* pqp, int level2add=0,
			   mpkTransform parent_tr=mpkTransform(),
			   int rec_level=0, int bv_idx=0);
};
#endif


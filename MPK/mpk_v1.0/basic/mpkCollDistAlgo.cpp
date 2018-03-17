#include <iostream>

#include "BVTQ.h"
#include "Build.h"
#include "MatVec.h"
#include "TriDist.h"
#include "GetTime.h"

#include "mpkCollDistAlgo.H"

PQP_CollideResult   mpkCollDistAlgo::coll_res;
PQP_DistanceResult  mpkCollDistAlgo::dist_res;
PQP_ToleranceResult mpkCollDistAlgo::tol_res;


double 
mpkCollDistAlgo::
SphereDist(double c1[3], double c2[3], double r1, double r2) {
  double d[] = {c1[0]-c2[0], c1[1]-c2[1], c1[2]-c2[2]};
  return sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) - r1 - r2;
}

bool
mpkCollDistAlgo::
Collision(double R1[3][3], double T1[3], PQP_Model *o1,
	  double R2[3][3], double T2[3], PQP_Model *o2)
{
  PQP_Collide(&coll_res,R1,T1,o1,R2,T2,o2,PQP_FIRST_CONTACT);
  return coll_res.Colliding();
}

double
mpkCollDistAlgo::
Distance(double R1[3][3], double T1[3], PQP_Model *o1,
	 double R2[3][3], double T2[3], PQP_Model *o2,
	 double rel_err, double abs_err)
{
  PQP_Distance(&dist_res,R1,T1,o1,R2,T2,o2,rel_err,abs_err);
  return dist_res.Distance();
}


inline
double
TriDistance(double R[3][3], double T[3],
	    Tri *t1, Tri *t2,
            double p[3], double q[3])  
{
  double tri1[3][3], tri2[3][3];

  VcV(tri1[0], t1->p1);
  VcV(tri1[1], t1->p2);
  VcV(tri1[2], t1->p3);
  MxVpV(tri2[0], R, t2->p1, T);
  MxVpV(tri2[1], R, t2->p2, T);
  MxVpV(tri2[2], R, t2->p3, T);

  return TriDist(p,q,tri1,tri2);
}


double
mpkCollDistAlgo::
GreedyDistance(double R1[3][3], double T1[3], PQP_Model *o1,
	       double R2[3][3], double T2[3], PQP_Model *o2,
	       double delta)
{
  MTxM(dist_res.R,R1,R2);
  double Ttemp[3];
  VmV(Ttemp, T2, T1);  
  MTxV(dist_res.T, R1, Ttemp);
  
  dist_res.num_bv_tests = 0;
  dist_res.num_tri_tests = 0;

  double Rtemp[3][3], R[3][3], T[3];

  MxM(Rtemp,dist_res.R,o2->child(0)->R);
  MTxM(R,o1->child(0)->R,Rtemp);
  
  MxVpV(Ttemp,dist_res.R,o2->child(0)->Tr,dist_res.T);
  VmV(Ttemp,Ttemp,o1->child(0)->Tr);
  MTxV(T,o1->child(0)->R,Ttemp);

  // first, see if top-level BVs are separated
  dist_res.num_bv_tests += 1;
  double d = BV_Distance(R, T, o1->child(0), o2->child(0));

  // if we already found separation don't bother recursing to refine it
  if ( d > delta )
    return d;
  else // if top-level BVs are not separated, recurse
    return GreedyDistanceRecurse(R,T,o1,0,o2,0,delta);
}

double
mpkCollDistAlgo::
GreedyDistanceRecurse(double R[3][3], double T[3], // b2 relative to b1
		      PQP_Model *o1, int b1,
		      PQP_Model *o2, int b2,
		      double delta)
{
  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2)
  {
    // both leaves.  return their distance
    dist_res.num_tri_tests++;

    double p[3], q[3];

    Tri *t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    Tri *t2 = &o2->tris[-o2->child(b2)->first_child - 1];

    return TriDistance(dist_res.R,dist_res.T,t1,t2,p,q);  
  }

  // First, perform distance tests on the children. Then traverse 
  // them recursively, but test the closer pair first, the further 
  // pair second.

  int a1,a2,c1,c2;  // new bv tests 'a' and 'c'
  double R1[3][3], T1[3], R2[3][3], T2[3], Ttemp[3];

  double sz1 = o1->child(b1)->GetSize();
  double sz2 = o2->child(b2)->GetSize();

  if (l2 || (!l1 && (sz1 > sz2)))
  {
    // visit the children of b1

    a1 = o1->child(b1)->first_child;
    a2 = b2;
    c1 = o1->child(b1)->first_child+1;
    c2 = b2;
    
    MTxM(R1,o1->child(a1)->R,R);
    VmV(Ttemp,T,o1->child(a1)->Tr);
    MTxV(T1,o1->child(a1)->R,Ttemp);

    MTxM(R2,o1->child(c1)->R,R);
    VmV(Ttemp,T,o1->child(c1)->Tr);
    MTxV(T2,o1->child(c1)->R,Ttemp);
  }
  else 
  {
    // visit the children of b2

    a1 = b1;
    a2 = o2->child(b2)->first_child;
    c1 = b1;
    c2 = o2->child(b2)->first_child+1;

    MxM(R1,R,o2->child(a2)->R);
    MxVpV(T1,R,o2->child(a2)->Tr,T);

    MxM(R2,R,o2->child(c2)->R);
    MxVpV(T2,R,o2->child(c2)->Tr,T);
  }

  double d1 = BV_Distance(R1, T1, o1->child(a1), o2->child(a2));
  double d2 = BV_Distance(R2, T2, o1->child(c1), o2->child(c2));
  dist_res.num_bv_tests += 2;

  // if we already found separation, don't further recurse to refine it
  double min_d1d2 = (d1 < d2) ? d1 : d2;
  if ( min_d1d2 > delta )
    return min_d1d2;

  // else recurse
  if (d2 < d1) // and thus at least d2 <= delta (d1 maybe too)
  {
    double alpha = GreedyDistanceRecurse(R2, T2, o1, c1, o2, c2,
					   delta);
    
    if ( alpha > delta ) {
      if ( d1 > delta )
	return (alpha < d1)? alpha: d1;
      double beta = GreedyDistanceRecurse(R1, T1, o1, a1, o2, a2,
					    delta);
      if ( beta > delta )
	return (alpha < beta)? alpha: beta;
    }
    return 0;
  }
  else // at least d1 <= delta (d2 maybe too)
  {
    double alpha = GreedyDistanceRecurse(R1, T1, o1, a1, o2, a2,
					   delta);

    if ( alpha > delta ) {
      if ( d2 > delta )
	return (alpha < d2)? alpha: d2;
      double beta = GreedyDistanceRecurse(R2, T2, o1, c1, o2, c2,
					    delta);
      if ( beta > delta )
	return (alpha < beta)? alpha: beta;
    }
    return 0;
  }
}

double
mpkCollDistAlgo::
GreedyDistance(double center[3], double radius,
	       double R2[3][3], double T2[3], PQP_Model *o2,
	       double delta)
{
  dist_res.num_bv_tests = 0;
  dist_res.num_tri_tests = 0;

  McM(dist_res.R,R2);
  VmV(dist_res.T, T2, center);  
  double R[3][3], T[3];
  MxM(R,dist_res.R,o2->child(0)->R);
  MxVpV(T,dist_res.R,o2->child(0)->Tr,dist_res.T);

  // create BV from sphere
  BV sphere;
  Midentity(sphere.R);
  Videntity(sphere.Tr);
  Videntity(sphere.To);
  Videntity(sphere.d);
  sphere.l[0]=sphere.l[1]=0;
  sphere.r = radius;

  // first, see if top-level BVs are separated
  dist_res.num_bv_tests += 1;
  double d = BV_Distance(R, T, &sphere, o2->child(0));

  // if we already found separation don't bother recursing to refine it
  if ( d > delta )
    return d;
  else // if top-level BVs are not separated, recurse
    return GreedyDistanceRecurse(R,T,&sphere,o2,0,delta);
}

double
mpkCollDistAlgo::
GreedyDistanceRecurse(double R[3][3], double T[3], // b2 relative to b1
		      BV* sphere,
		      PQP_Model *o2, int b2,
		      double delta)
{
  if ( o2->child(b2)->Leaf() )
  {
    // leaf.  return distance from sphere
    dist_res.num_tri_tests++;
    double p[3], q[3];
    Tri t1;
    Videntity(t1.p1);
    Videntity(t1.p2);
    Videntity(t1.p3);
    Tri *t2 = &o2->tris[-o2->child(b2)->first_child - 1];
    return TriDistance(dist_res.R,dist_res.T,&t1,t2,p,q) - sphere->r;
  }

  // First, perform distance tests on the children. Then traverse 
  // them recursively, but test the closer pair first, the further 
  // pair second.

  int a2,c2;  // new bv tests 'a' and 'c'
  double R1[3][3], T1[3], R2[3][3], T2[3], Ttemp[3];

  // visit the children of b2
  a2 = o2->child(b2)->first_child;
  c2 = o2->child(b2)->first_child+1;
  
  MxM(R1,R,o2->child(a2)->R);
  MxVpV(T1,R,o2->child(a2)->Tr,T);
  
  MxM(R2,R,o2->child(c2)->R);
  MxVpV(T2,R,o2->child(c2)->Tr,T);

  double d1 = BV_Distance(R1, T1, sphere, o2->child(a2));
  double d2 = BV_Distance(R2, T2, sphere, o2->child(c2));
  dist_res.num_bv_tests += 2;

  // if we already found separation, don't further recurse to refine it
  double min_d1d2 = (d1 < d2) ? d1 : d2;
  if ( min_d1d2 > delta )
    return min_d1d2;

  // else recurse
  if (d2 < d1) // and thus at least d2 <= delta (d1 maybe too)
  {
    double alpha = GreedyDistanceRecurse(R2, T2, sphere, o2, c2,
					   delta);
    
    if ( alpha > delta ) {
      if ( d1 > delta )
	return (alpha < d1)? alpha: d1;
      double beta = GreedyDistanceRecurse(R1, T1, sphere, o2, a2,
					    delta);
      if ( beta > delta )
	return (alpha < beta)? alpha: beta;
    }
    return 0;
  }
  else // at least d1 <= delta (d2 maybe too)
  {
    double alpha = GreedyDistanceRecurse(R1, T1, sphere, o2, a2,
					   delta);

    if ( alpha > delta ) {
      if ( d2 > delta )
	return (alpha < d2)? alpha: d2;
      double beta = GreedyDistanceRecurse(R2, T2, sphere, o2, c2,
					    delta);
      if ( beta > delta )
	return (alpha < beta)? alpha: beta;
    }
    return 0;
  }
}

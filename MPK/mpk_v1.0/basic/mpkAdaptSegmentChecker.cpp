#include "mpkAdaptSegmentChecker.H"

mpkAdaptSegmentChecker::
mpkAdaptSegmentChecker(const mpkConfigChecker* ccheck,
		       const mpkConfig* q0, const mpkConfig* q1,
		       bool use_transf_cache)
{
  // check configurations for compatibility (debug)
  assert(q0->size() == q1->size());

  this->test_pairs = ccheck->test_pairs;
  this->q0 = q0;
  this->q1 = q1;
  this->model_id = ccheck->model_id;

#ifdef TRANSFORM_CACHE
  if ( use_transf_cache ) transf_cache = new mpkTransformCache;
  else transf_cache = 0;
#endif // NO_TRANSFORM_CACHE

  init(ccheck->robots,0,0);
}

mpkAdaptSegmentChecker::
mpkAdaptSegmentChecker(const mpkConfigChecker* ccheck0,
		       const mpkConfigChecker* ccheck1,
		       bool use_transf_cache)
{
  // check if one of the endpoints is colliding
  if (ccheck0->min_sep <= 0 || ccheck1->min_sep <= 0) {
    done = true;
    coll_status = true;
#ifndef SAVE_MEMORY
    if ( ccheck0->min_sep <= 0 ) coll_t = 0;
    else coll_t = 1; // ccheck1->min_sep <= 0
    num_tri_tests = 0;
    num_bv_tests = 0;
    num_pairs_dist_comp=0;
#endif // SAVE_MEMORY
    return;
  }

  // check if endpoint configuration checkers are compatible (debug)
  assert(ccheck0->robots == ccheck1->robots);
  assert(ccheck0->lbdist.size() == ccheck1->lbdist.size());
  assert(ccheck0->test_pairs == ccheck1->test_pairs);
  assert(ccheck0->model_id == ccheck1->model_id);
  assert(ccheck0->q);
  assert(ccheck1->q);

  // setup
  this->test_pairs = ccheck0->test_pairs;
  this->q0 = ccheck0->q;
  this->q1 = ccheck1->q;
  this->model_id = ccheck0->model_id;

#ifdef TRANSFORM_CACHE
  if ( use_transf_cache ) transf_cache = new mpkTransformCache;
  else transf_cache = 0;
#endif // TRANSFORM_CACHE

  init(ccheck0->robots,&ccheck0->lbdist, &ccheck1->lbdist);
}

void
mpkAdaptSegmentChecker::
init(const mpkRobotCollection* robots,
     const vector<double>* lbdist0,
     const vector<double>* lbdist1)
{
  int i;
  for ( i=0; i<robots->num_robots(); i++ )
    for ( int j=0; j<robots->rob[i]->num_joints; j++ ) {
      //robots->rob[i]->joints[j].pathlen_upbound = -1;
    }

#ifndef SAVE_MEMORY
  num_tri_tests = 0;
  num_bv_tests = 0;
  num_pairs_dist_comp=0;
#endif // SAVE_MEMORY

  done = false;
  coll_status = true;

  testpair_subseg* tps = 0;

  // setup the priority queue
  for ( i=0; i < test_pairs->size(); i++ ) {

    const mpkCollPair& test_pair_i = (*test_pairs)[i];

    // initialize pair-specific subsegment for pair i
    if ( !tps ) tps = new testpair_subseg;
    tps->pair_idx = i;
    tps->t0 = 0;
    tps->t1 = MAX_UNSIGNED;

    // A) setup lower distance bounds at segment endpoints

    // consider first segment endpoint (index 0)
    if ( lbdist0 ) { // use lower distance bound (if provided)
      tps->lbdist0 = (*lbdist0)[i];
    }
    else { // else compute it from scratch
      test_pair_i.compute_transf(*q0);
      tps->lbdist0 =
	mpkCollDistAlgo::GreedyDistance(test_pair_i.o1.Tr->R,
					test_pair_i.o1.Tr->T,
					test_pair_i.o1.pqp[model_id], 
					test_pair_i.o2.Tr->R,
					test_pair_i.o2.Tr->T,
					test_pair_i.o2.pqp[model_id],
					test_pair_i.delta);
#ifndef SAVE_MEMORY
      num_tri_tests += mpkCollDistAlgo::dist_res.num_tri_tests;
      num_bv_tests  += mpkCollDistAlgo::dist_res.num_bv_tests;
      num_pairs_dist_comp++;
#endif // SAVE_MEMORY
    }

    // check if endpoint collides
    if ( tps->lbdist0 <= 0 ) {
      done = true;
      coll_status = true;
      delete tps;
      clear_mem();
#ifndef SAVE_MEMORY
      coll_t = 0;
#endif // SAVE_MEMORY
      return;
    }
    
    // consider second segment endpoint (index 1)
    if ( lbdist1 ) {  // use lower distance bound (if provided)
      tps->lbdist1 = (*lbdist1)[i];
    }
    else { // else compute it from scratch
      test_pair_i.compute_transf(*q1);
      tps->lbdist1 =
	mpkCollDistAlgo::GreedyDistance(test_pair_i.o1.Tr->R,
					test_pair_i.o1.Tr->T,
					test_pair_i.o1.pqp[model_id], 
					test_pair_i.o2.Tr->R,
					test_pair_i.o2.Tr->T,
					test_pair_i.o2.pqp[model_id],
					test_pair_i.delta);
#ifndef SAVE_MEMORY
      num_tri_tests += mpkCollDistAlgo::dist_res.num_tri_tests;
      num_bv_tests  += mpkCollDistAlgo::dist_res.num_bv_tests;
      num_pairs_dist_comp++;
#endif // SAVE_MEMORY
    }

    // check if endpoint collides
    if ( tps->lbdist1 <= 0 ) {
      done = true;
      coll_status = true;
      delete tps;
      clear_mem();
#ifndef SAVE_MEMORY
      coll_t = 1;
#endif // SAVE_MEMORY
      return;
    }

    // B) bound (relative) motions of object pairs in workspace

    tps->pathlen_bound = 0;

    // B2) bound motions of objects in root_idx frame
    double upb1 = test_pair_i.o1.rob->
      pathlen_upbound(*q0, *q1, 
		      test_pair_i.o1.first_param_idx,
		      test_pair_i.o1.joint_idx,
		      test_pair_i.root_idx);
    if ( upb1 < 0 )
      cerr << "Warning: mpkAdaptSegmentChecker(): no pathlen_upbound() for "
	   << test_pair_i.o1.rob->get_name() << endl
	   << "         Segment checker will not work properly." << endl;
    else
      tps->pathlen_bound += upb1;
    
    if ( test_pair_i.o2.rob ) { // bound for second object only if robot link
      double upb2 = test_pair_i.o2.rob->
	pathlen_upbound(*q0, *q1, 
			test_pair_i.o2.first_param_idx,
			test_pair_i.o2.joint_idx,
			test_pair_i.root_idx);
      if ( upb2 < 0 )
	cerr << "Warning: mpkAdaptSegmentChecker(): no pathlen_upbound() for "
	     << test_pair_i.o2.rob->get_name() << endl
	     << "         Segment checker will not work properly." << endl;
      else
	tps->pathlen_bound += upb2;
    }


    // C) add pair-specific subsegment to priority queue
    if ( tps->prio() >= 0 ) {  // only if not covered yet
      seg_queue.push(tps);
      tps = 0;
    }
    // else tps is save (covered by endpoints) and can be skipped

  }
  
  if ( tps ) delete tps;

}


void
mpkAdaptSegmentChecker::
clear_mem()
{
  while (seg_queue.size()) {
    testpair_subseg* curr = seg_queue.top();
    delete curr;
    seg_queue.pop();
  }  
#ifdef TRANSFORM_CACHE
  if ( transf_cache ) delete transf_cache;
  transf_cache = 0;
#endif // TRANSFORM_CACHE
}

bool
mpkAdaptSegmentChecker::
iteration_step()
{
  if ( done ) return false;

  // if priority queue is empty, we are done and the segment is free
  if ( seg_queue.size() == 0 ) {
    done = true;
    coll_status = false;
#ifdef TRANSFORM_CACHE
    if ( transf_cache ) delete transf_cache;
    transf_cache = 0;
#endif // TRANSFORM_CACHE
    return false;
  }

  // otherwise, we process the first element in the queue
  // (note that the queue is initialized and maintained such that it
  // does only contain testpair_subseg with prio > 0 and
  // collision-free endpoints)

  testpair_subseg* curr_tps = seg_queue.top();
  const mpkCollPair& curr_pair = (*test_pairs)[curr_tps->pair_idx];

  // bisect subsegment
  mpkConfig qmid(q0->size());
  unsigned tmid = (curr_tps->t0 >> 1) + (curr_tps->t1 >> 1);

  // interpolate qmid (between q0 and q1)
  // for multi-robot scenarios, work can be saved by only
  // interpolating for the robot(s) involved:
  double tmid_r = double(tmid)/MAX_UNSIGNED;
  qmid.lin_interpol(tmid_r,*q0,*q1,                 // interpolate indices 
		    curr_pair.o1.first_param_idx,   // from here...
		    curr_pair.o1.first_param_idx+   // ... to here...
		    curr_pair.o1.rob->num_params-1);// ..............
  if (curr_pair.o2.rob) {
    qmid.lin_interpol(tmid_r,*q0,*q1,               // and the same for o2
		      curr_pair.o2.first_param_idx,
		      curr_pair.o2.first_param_idx+
		      curr_pair.o2.rob->num_params-1);
  }


  // now compute spatial transforms for the two objects at qmid
#ifdef TRANSFORM_CACHE
  if ( transf_cache )
    curr_pair.compute_transf(qmid,tmid,transf_cache);
  else
#endif // TRANSFORM_CACHE
    curr_pair.compute_transf(qmid);

  // compute lower distance bound at midpoint
  double lbdist_mid =
    mpkCollDistAlgo::GreedyDistance(curr_pair.o1.Tr->R, curr_pair.o1.Tr->T,
				    curr_pair.o1.pqp[model_id], 
				    curr_pair.o2.Tr->R, curr_pair.o2.Tr->T,
				    curr_pair.o2.pqp[model_id],
				    curr_pair.delta);

#ifndef SAVE_MEMORY
  num_pairs_dist_comp++;
  num_tri_tests += mpkCollDistAlgo::dist_res.num_tri_tests;
  num_bv_tests  += mpkCollDistAlgo::dist_res.num_bv_tests;
#endif // SAVE_MEMORY

  if ( lbdist_mid <= 0 ) {
    // current pair of objects is too close (or even collides) at midpoint
    done = true; 
    coll_status = true;
    clear_mem();
#ifndef SAVE_MEMORY
    coll_t = tmid_r;
#endif // SAVE_MEMORY
    return false;  // no more steps necessary
  }

  // otherwise: midpoint is free.  we thus split curr_tps into two
  // subsegments and replace in prio-queue
  
  // adjust estimate of traveled path length
  // (the simple division by 2 works only with parameter factors that
  // are configuration-independent (on the subsegment)
  double pathlen_bound_0mid = curr_tps->pathlen_bound / 2;
  double pathlen_bound_mid1 = pathlen_bound_0mid;

  double prio_0mid = pathlen_bound_0mid - curr_tps->lbdist0 - lbdist_mid;
  double prio_mid1 = pathlen_bound_mid1 - lbdist_mid - curr_tps->lbdist1;
  
  testpair_subseg* tps_0mid = 0;
  if ( prio_0mid >= 0 ) {
    tps_0mid = new testpair_subseg;
    tps_0mid->pair_idx = curr_tps->pair_idx;
    tps_0mid->t0 = curr_tps->t0;
    tps_0mid->t1 = tmid;
    tps_0mid->lbdist0 = curr_tps->lbdist0;
    tps_0mid->lbdist1 = lbdist_mid;
    tps_0mid->pathlen_bound = pathlen_bound_0mid;
  }
  
  testpair_subseg* tps_mid1 = 0;
  if ( prio_mid1 >= 0 ) {
    tps_mid1 = new testpair_subseg;
    tps_mid1->pair_idx = curr_tps->pair_idx;
    tps_mid1->t0 = tmid;
    tps_mid1->t1 = curr_tps->t1;
    tps_mid1->lbdist0 = lbdist_mid;
    tps_mid1->lbdist1 = curr_tps->lbdist1;
    tps_mid1->pathlen_bound = pathlen_bound_mid1;
  }

  // remove current segment (curr_tps)
  delete curr_tps;
  seg_queue.pop();

  // and replace by its two split parts (if they are not covered yet)
  if ( tps_0mid ) seg_queue.push(tps_0mid);
  if ( tps_mid1 ) seg_queue.push(tps_mid1);
 
  return true;
  // true indicates that the segment status has not been determined
  // and iteration_step() needs to be called again (if the queue is
  // empty now, this will be detected in the next call of
  // iteration_step().
}

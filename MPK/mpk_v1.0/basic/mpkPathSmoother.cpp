#include <list>

#include "mpk_rand.h"
#include "mpkPathSmoother.H"

mpkPathSmoother::
mpkPathSmoother(const vector<mpkConfig>& path,
		const vector<mpkCollPair>* test_pairs,
		mpkRobotCollection* robots,
		double min_shortcut_len,
		double epsilon)
{
  len = 0;
  node* curr;
  node* prev = 0;

  // copy the path into internal list representation and compute its
  // length
  for ( int i=0; i < path.size(); i++ ) {
    if ( i>0 ) len += path[i-1].dist(path[i]);
    curr = new node;
    if ( i==0 ) first = curr;
    curr->next = 0;
    curr->q = path[i];
    if ( prev ) prev->next = curr;
    prev = curr;
  }
  num_points = path.size();
  this->min_shortcut_len = min_shortcut_len;
  p1 = mpkConfig(path[0].size());
  p2 = mpkConfig(path[0].size());
  this->test_pairs = test_pairs;
  this->robots = robots;
  this->epsilon = epsilon;  // only used by simple segment checker
}

mpkPathSmoother::
~mpkPathSmoother()
{
  // delete internal list representation of path
  node* curr = first;
  while ( curr ) {
    node* tmp = curr;
    curr = curr->next;
    delete tmp;
  }
}

mpkPathSmoother::
node*
mpkPathSmoother::seg_node(int seg_idx)
{
  assert( seg_idx <= num_points-2 );
  node* curr = first;
  for ( int i=0; i<seg_idx; i++ ) {
    curr = curr->next;
  }
  assert(curr->next);
  return curr;
}

mpkPathSmoother::
node*
mpkPathSmoother::lin_interpol(double t, mpkConfig& q)
{
  assert( t >= 0.0 && t <= 1.0 );

  if ( !first->next || t==0.0 ) {
    q = first->q;
    return first;
  }
  else if ( t==1.0 ) {
    node* curr;
    for ( curr = first; curr->next && curr->next->next; curr = curr->next )
      ;
    if ( curr->next )
      q = curr->next->q;
    else
      q = curr->q;
    return curr;
  }
  
  double tcurr = 0;
  for ( node* curr = first; curr->next; curr = curr->next ) {

    double deltat = curr->q.dist(curr->next->q) / length();
    double tnext = tcurr + deltat;

    if ( tnext > t && deltat > 0 ) {
      q.lin_interpol((t-tcurr) / deltat, curr->q, curr->next->q);
      return curr;
    }

    tcurr = tnext;
  }

  return 0;
}

void
mpkPathSmoother::
replace_section(mpkPathSmoother::node* n1, mpkPathSmoother::node* n2,
		const mpkConfig& q1, const mpkConfig& q2)
{
  if ( n1==n2 ) {
    node* nnew1 = new node;
    node* nnew2 = new node;
    nnew2->next = n1->next;
    nnew1->next = nnew2;
    n1->next = nnew1;
    nnew2->q = q2;
    nnew1->q = q1;
    return;
  }
  assert(n1);
  assert(n2);
  assert(n1->next);
  assert(n2->next);
  // recompute length
  node* curr=n1;
  while ( 1 ) {
    len -= curr->q.dist(curr->next->q);
    if ( curr==n2 ) break;
    curr=curr->next;
  }
  // remove old section
  while ( 1 ) {
    node* del_node = n1->next;
    n1->next = del_node->next;
    delete del_node;
    num_points--;
    if (del_node == n2 ) break;
  }
  // insert three bridging segments between n1 and n1->next
  node* last = n1;
  if ( q1 != n1->q ) {
    node* new1 = new node;
    new1->q = q1;
    new1->next = n1->next;
    n1->next = new1;
    last = new1;
    num_points++;
  }
  if ( q2 != last->next->q ) {
    node* new2 = new node;
    new2->q = q2;
    new2->next = last->next;
    last->next = new2;
    last = new2;
    num_points++;
  }
  len += (n1->q.dist(q1)
    + q1.dist(q2)
    + q2.dist(last->next->q));
}

void
mpkPathSmoother::
get_path(vector<mpkConfig>& path)
{
  path.clear();
  // copy internal list representation into path
  for (node* curr = first; curr; curr=curr->next)
    path.push_back(curr->q);
}

void
mpkPathSmoother::
get_path(list<mpkConfig>& path)
{
  path.clear();
  // copy internal list representation into path
  for (node* curr = first; curr; curr=curr->next)
    path.push_back(curr->q);
}


void
mpkPathSmoother::
smoothe(int num_steps)
{
  mpkConfigChecker check1(test_pairs, robots);
#ifdef ADAPT_COLLCHECKER
  mpkConfigChecker check2(test_pairs, robots);
#endif

  double t1 = 0;
  
  int num_shortcuts = 0;
  int orig_num_segs = num_segs();
  double orig_len = length();

  for ( int i=0; i<num_steps; i++ ) {

    if ( num_segs() <= 1 ) break;

    double t = mpk_drand();
    double t1 = 0.5*t;
    double t2 = 0.5*(t+1.0);
 
    bool coll;
    node *n1;
    node *n2;
    do {

      n1 = lin_interpol(t1, p1);
      n2 = lin_interpol(t2, p2);

#ifdef ADAPT_COLLCHECKER
      if ( check1.clearance(&p1) > 0 && check2.clearance(&p2) > 0 ) {
	mpkAdaptSegmentChecker seg_check(&check1,&check2);
#else
      // actually, p1 and p2 are on path and should be collision-free
      if ( !check1.collision(&p1) && !check1.collision(&p2) ) {
	mpkSimpleSegmentChecker seg_check(&check1, &p1, &p2, epsilon);
#endif	
	while(seg_check.iteration_step())
	  ;
	coll = seg_check.collision();
      }
      else coll = true;

      if ( coll ) {
	t1 = 0.5*(t1+t);
	t2 = 0.5*(t+t2);
      }
	
   } while (coll && t2-t1 > min_shortcut_len);

   if ( !coll && n1!=n2 ) {
     replace_section(n1,n2,p1,p2);
     num_shortcuts++;
     cerr << "shortcut ";
   }
   else
     cerr << "         ";

   cerr << "step: " << i+1 << ", "
	<< "#segs: " << num_segs()
	<< " (";
   fprintf(stderr,"%.2f", double(num_segs())/orig_num_segs*100);
   cerr << "%), "
	<< "length: " << length()
	<< " (";
   fprintf(stderr,"%.2f",length()/orig_len*100);
   cerr << "%), ";
   cerr << "shortcuts: ";
   fprintf(stderr,"%.2f",100*double(num_shortcuts)/(i+1));
   cerr << "%";
   if ( !coll && n1!=n2 ) {
     cerr << ", shortcut interval [" << t1 << "," << t2 << "]";
   }
   cerr << endl;
  }

}


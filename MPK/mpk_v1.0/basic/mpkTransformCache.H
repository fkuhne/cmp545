#ifndef MPK_TRANSFORM_CACHE_H
#define MPK_TRANSFORM_CACHE_H

#include <hash_map.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#include "bjhash.h"
#include "mpkTransform.H"

/**@memo Class for caching spatial transforms.  This class is used by
   {@link mpkAdaptSegmentChecker mpkAdaptSegmentChecker} to avoid
   redundant computations of transforms while checking a segment for
   collisions.  Each segment checker maintains its own cache.  

   
   @doc Each cached transform is indexed by a triple of values: the
   robot reference (i), the joint reference (j) and the position on
   the segment for which the transform is stored (k).  For the latter,
   this means that the segment is discretized.  In practice,
   discretization by a variable of type unsigned should be sufficient.
*/
class mpkTransformCache {
  
public:
  
  ///
  mpkTransformCache(int initsz=1) { cache.resize(initsz); }

  ///
  ~mpkTransformCache() { clear(); }
  
  ///
  void clear() {cache.clear();};

  /**@doc Inserts a new transform into the cache.  See above for a
     description of the index triple (i,j,k).*/
  bool insert(const mpkTransform& T, unsigned i, unsigned j, unsigned k)
  {
    tr t(T);
    get_key(i,j,k);
    return cache.insert(hash_map<key_type,tr,key_hash,key_eq>::
			value_type(key,t)).second;
  }
  
  /**@doc Tries to retrieve a transform from the cache.  See above for
     a description of the index triple (i,j,k).  Returns false if the
     transform is not in the cache, and true otherwise.*/
  bool load(mpkTransform& T, unsigned i, unsigned j, unsigned k)
  {
    get_key(i,j,k);
    it = cache.find(key);
    if ( it==cache.end() ) return false;
    const tr& t = (*it).second;
    memcpy(T.R,t.R,9*sizeof(double));
    memcpy(T.T,t.T,3*sizeof(double));
    return true;
  }

  
private:

  void get_key(unsigned i, unsigned j, unsigned k)
  {
    memcpy(key.buf,&i,sizeof(i));
    memcpy(key.buf+sizeof(i),&j,sizeof(j));
    memcpy(key.buf+sizeof(i)+sizeof(j),&k,sizeof(k));
  }

  static const int key_sz = 12;
  struct key_type {
    ub1 buf[key_sz];
  };
  
  struct key_eq
  {
    bool operator()(const key_type& p1, const key_type& p2) const
    {
      for ( int i=0; i<key_sz; i++ )
	if (p1.buf[i]!=p2.buf[i]) return false;
      return true;
    }
  };

  struct key_hash {
    size_t operator()(const key_type& k) const
    {
      return bjhash(k.buf,key_sz,0);
    }
  };

  struct tr {
    tr(const mpkTransform& t) {
      memcpy(R,t.R,sizeof(R));
      memcpy(T,t.T,sizeof(T));
    };
    double R[3][3]; // rotation
    double T[3]; // translation
  };

  hash_map<key_type,tr,key_hash,key_eq> cache;
  hash_map<key_type,tr,key_hash,key_eq>::const_iterator it;
  key_type key;
};

#endif

#ifndef MPK_TRANSFORM_H
#define MPK_TRANSFORM_H

#include <iostream>

using namespace std;

#include "mpk_inventor.h"
  
/**@memo The base class for representing spatial transforms.

   @doc For efficiency reasons, there is not an abstract base class
   plus several derived classes, one for each transform type.
   Instead, this single mpkTransform class includes a type flag and
   implements all possible transforms.  The fields {\bf (R,T)} store
   the current values of the transform.  Further private fields store
   information required to update certain transform types, such as
   {\bf axis} for a rotation.

   Since multiple transforms can depend on the same parameter(s), each
   parameterized transform must be bound to a set of (normalized)
   parameters (each in [0,1]) using {\bf bind()}.  Once this has been
   done, the function {\bf update()} can read the current values of the
   assigned parameters to recompute {\bf R} and {\bf T}.

   For further details see \URL[mpkTransform.H]{../../basic/mpkTransform.H}.
 */
class mpkTransform {
  
public:
  
  /**@memo
     \begin{itemize}
     \item {\bf constant}: constant transform
     \item {\bf rot1}: rotation around axis
     \item {\bf transl_rot1}: rotation around axis, then constant transl
     \item {\bf transl1}: translation along axis
     \item {\bf func}: general functional dependency
     \end{itemize}
   */
  enum TYPE {
    constant, // constant transform
    rot1,     // rotation around axis
    transl_rot1,  // rotation around axis, then constant translation
    transl1,  // translation along axis
    func      // general functional dependency (in the function that
	      // computes the transform, be sure to take the scale
	      // factor (scalef) into account!)
  };
  
  ///@doc construct from constant transform given by X
  mpkTransform(const SoTransform* X=0);

  //@doc construct from constant transform given by X
  mpkTransform(const SbMatrix& X);

  //@doc construct from constant inventor rotation and translation
  mpkTransform(const SbRotation& rot, const SbVec3f& transl);

  //@doc construct parameterized rotation or translation
  mpkTransform(TYPE typ, double ax0, double ax1, double ax2, double min=0, double max=1);
  mpkTransform(TYPE typ, const double ax[3], double min=0, double max=1);

  //@doc construct general functional transform
  mpkTransform(TYPE typ, int num_params,
	    void (*f)(mpkTransform& tr, const double* param),
	    void (*finit)(mpkTransform& tr, const double* param)=0 );

  //@doc Returns the number of parameters (dofs)
  int num_params();

  double min_param(int i=0) {return min[i];};
  double max_param(int i=0) {return max[i];};

  /**@doc For binding a parameterized transform to one or more
     normalized parameters.  {\bf param_ptr} must point to the first
     parameter.
   */
  void bind(const double* param_ptr);

  ///@doc Sets the scale factor.
  void rescale(double scalef);
  ///@doc Returns the scale factor.
  double get_scale() {return scalef;};

  ///@doc Returns the type.
  TYPE get_type() {return mytype;};

  ///@doc Returns the axis (only meaningful for some transforms).
  const double* get_axis() {return axis;};

  /**@doc Updates the transform (after at least one parameter has been
     changed).  Returns true for parameterized transform, false for
     constant transform.
  */
  bool update();

  ///@doc set transform to X*Y
  void set_product(const mpkTransform& X, const mpkTransform& Y);

  ///@doc concatenate X to the right to the transform
  void append(const mpkTransform& X);
  ///@doc operator * has the same effect as append()
  mpkTransform& operator*(const mpkTransform& X) {append(X); return *this;};

  ///@doc sets (R,T) to identity transform but does not modify the type flag
  void set_identity();

  ///@doc returns true if (R,T) is identity transform
  bool is_identity() const;

  ///@doc apply transform to in and store result in out
  void apply(double out[3], const double in[3]) const;

  ///@doc apply transform (R,T) to in and store result in out
  static void apply(double out[3],
		    const double R[3][3], const double T[3], 
		    const double in[3]);

  ///@doc apply the inverse of the transform to in and store result in out
  void apply_inv(double out[3], const double in[3]) const;

  ///@doc print to f
  void print(ostream& f=cout) const;

  ///@memo Orientation matrix
  double R[3][3];
  ///@memo Translation vector
  double T[3];

  ///@doc convert to SbMatrix
  operator SbMatrix() const;

protected:

  TYPE mytype;

  // scale factor (important for functional transforms; for transl1,
  // the scaling is done by resetting min and max for the
  // translational components)
  double scalef;

private:

  void setup(TYPE typ, double ax0, double ax1, double ax2,
	     double min=0, double max=1);

  double axis[3];
  const double* param_ptr;
  double min[3];
  double max[3];

  void (*transf_func)(mpkTransform& tr, const double* param);
  void (*transf_init_func)(mpkTransform& tr, const double* param);
  int num_params_f;
};

///
class mpkConstTransl : public mpkTransform {
public:
  mpkConstTransl(double tx, double ty, double tz) {
    scalef = 1.0;
    mytype = constant;
    R[0][0] = R[1][1]  = R[2][2]  = 1;
    R[0][1] = R[0][2] = R[1][0] = R[1][2] = R[2][0] =  R[2][1] = 0;
    T[0] = tx, T[1] = ty, T[2] = tz;    
  };
};

///
class mpkConstRot : public mpkTransform {
public:
  mpkConstRot(double ax0, double ax1, double ax2, double ang) {

    mytype = constant;
    scalef = 1.0;
    double co = cos(ang);
    double si = sin(ang);
    double co1 = 1-co;

    double len = sqrt(ax0*ax0 + ax1*ax1 + ax2*ax2);
    assert(len > 0);
    ax0 /= len; ax1 /= len; ax2 /= len;

    R[0][0] = ax0*ax0 * co1 + co;
    R[0][1] = ax0*ax1 * co1 - ax2*si;
    R[0][2] = ax0*ax2 * co1 + ax1*si;

    R[1][0] = ax0*ax1 * co1 + ax2*si;
    R[1][1] = ax1*ax1 * co1 + co;
    R[1][2] = ax2*ax1 * co1 - ax0*si;

    R[2][0] = ax2*ax0 * co1 - ax1*si;
    R[2][1] = ax2*ax1 * co1 + ax0*si;
    R[2][2] = ax2*ax2 * co1 + co;
    
    T[0] = T[1] = T[2] = 0;

  };
};

///
class mpkConstTransl_Rot1 : public mpkTransform {
public:
  mpkConstTransl_Rot1(double tr0, double tr1, double tr2,
		      double ax0, double ax1, double ax2,
		      double min=1, double max=-1) // min and max angle
    : mpkTransform(rot1, ax0, ax1, ax2, min, max) // abuse the rotation constructor
  {
    T[0] = tr0, T[1] = tr1, T[2] = tr2;
    mytype = transl_rot1;
  };
};

///
class mpkTransl1 : public mpkTransform {
public:
  mpkTransl1(double tx, double ty, double tz, double min=0, double max=1)
    : mpkTransform(transl1, tx, ty, tz, min, max) {};
};

///
class mpkRot1 : public mpkTransform {
public:
  mpkRot1(double ax0, double ax1, double ax2, double min=1, double max=-1)
    : mpkTransform(rot1, ax0, ax1, ax2, min, max) {};
};

///
class mpkFuncTransf : public mpkTransform {
public:
  mpkFuncTransf(int num_params,
		void (*f)(mpkTransform& tr, const double* param),
		void (*finit)(mpkTransform& tr, const double* param)=0 )
    : mpkTransform(func, num_params, f, finit ) {};
};


// setup constant transform given by X
inline mpkTransform::mpkTransform(const SoTransform* X)
{
  scalef = 1.0;
  mytype = constant;
  if(X==0) {
    set_identity();
  }
  else {
    SbVec3f scalFact(1,1,1);
    SbMatrix Tr;
    Tr.setTransform(X->translation.getValue(), X->rotation.getValue(),
		    scalFact);
    // note that inventor matrices are transposed 4x4 matrices!!
    R[0][0] = Tr[0][0];  R[0][1] = Tr[1][0];  R[0][2] = Tr[2][0];
    R[1][0] = Tr[0][1];  R[1][1] = Tr[1][1];  R[1][2] = Tr[2][1];
    R[2][0] = Tr[0][2];  R[2][1] = Tr[1][2];  R[2][2] = Tr[2][2];
    T[0] = Tr[3][0]; T[1] = Tr[3][1]; T[2] = Tr[3][2];
  }
}

// setup constant transform given by X
inline mpkTransform::mpkTransform(const SbMatrix& X)
{
  scalef = 1.0;
  // note that inventor matrices are transposed 4x4 matrices!!
  R[0][0] = X[0][0];  R[0][1] = X[1][0];  R[0][2] = X[2][0];
  R[1][0] = X[0][1];  R[1][1] = X[1][1];  R[1][2] = X[2][1];
  R[2][0] = X[0][2];  R[2][1] = X[1][2];  R[2][2] = X[2][2];
  T[0] = X[3][0]; T[1] = X[3][1]; T[2] = X[3][2];
  mytype = constant;
}

inline mpkTransform::mpkTransform(const SbRotation& rot, const SbVec3f& transl)
{
  scalef = 1.0;
  SbVec3f scalFact(1,1,1);
  SbMatrix Tr;
  Tr.setTransform(transl, rot, scalFact);
  // note that inventor matrices are transposed 4x4 matrices!!
  R[0][0] = Tr[0][0];  R[0][1] = Tr[1][0];  R[0][2] = Tr[2][0];
  R[1][0] = Tr[0][1];  R[1][1] = Tr[1][1];  R[1][2] = Tr[2][1];
  R[2][0] = Tr[0][2];  R[2][1] = Tr[1][2];  R[2][2] = Tr[2][2];
  T[0] = transl[0]; T[1] = transl[1]; T[2] = transl[2];
  mytype = constant;
}

inline mpkTransform::mpkTransform(TYPE typ, double ax0, double ax1, double ax2, double min, double max)
{
  setup(typ, ax0, ax1, ax2, min, max);
}


inline mpkTransform::mpkTransform(TYPE typ, const double ax[3], double min, double max)
{
  setup(typ, ax[0], ax[1], ax[2], min, max);
}


// setup parameterized transform
inline void mpkTransform::setup(TYPE typ, double ax0, double ax1, double ax2,
			     double min, double max)
{
  mytype = typ;
  scalef = 1.0;
  if ( mytype == rot1 || mytype == transl_rot1) { // normalize axis
    double len = sqrt(ax0*ax0 + ax1*ax1 + ax2*ax2);
    assert(len);
    ax0 /= len; ax1 /= len; ax2 /= len;
  }
  axis[0] = ax0, axis[1] = ax1, axis[2] = ax2;

  set_identity();

  this->min[0] = min;
  this->max[0] = max;
}


inline mpkTransform::mpkTransform(TYPE typ, int num_params,
			    void (*f)(mpkTransform& tr, const double* param),
			    void (*finit)(mpkTransform& tr, const double* param) )
{
  if ( typ != func ) {
    cerr << "mpkTransform::mpkTransform(): typ must be func." << endl;
#ifdef WIN32
    getchar();
#endif
    exit(1);
  }
  mytype = typ;
  scalef = 1.0;
  this->transf_func = f;
  this->transf_init_func = finit;
  num_params_f = num_params;
  if ( finit ) finit(*this, 0);
  else set_identity();
}


inline void mpkTransform::set_identity()
{
  R[0][0] = R[1][1]  = R[2][2]  = 1;
  R[0][1] = R[0][2] = R[1][0] = R[1][2] = R[2][0] =  R[2][1] = 0;
  T[0] = 0, T[1] = 0, T[2] = 0;
}

inline bool mpkTransform::is_identity() const
{
  return
    R[0][0] == 1.0 && R[1][1] == 1.0 && R[2][2] == 1.0 
    && R[0][1] == 0.0 && R[0][2] == 0.0 && R[1][0] == 0.0
    && R[1][2] == 0.0 && R[2][0] == 0.0 && R[2][1] == 0.0
    && T[0] == 0.0 && T[1] == 0.0 && T[2] == 0.0;
}

inline mpkTransform::operator SbMatrix() const
{
  // note that inventor matrices are transposed 4x4 matrices!!
  return SbMatrix(R[0][0],R[1][0],R[2][0],0,
		  R[0][1],R[1][1],R[2][1],0,
		  R[0][2],R[1][2],R[2][2],0,
		  T[0],T[1],T[2],1);
}



inline void mpkTransform::bind(const double* param_ptr)
{
  if ( mytype == constant ) {
    cerr << "Warning: mpkTransform::bind(): trying to bind to constant transform." << endl;
  }
  this->param_ptr = param_ptr;
  if ( mytype == func && transf_init_func )
    transf_init_func(*this, param_ptr);
  update();
}

inline void mpkTransform::rescale(double scalef)
{
  this->scalef = scalef;
  T[0] *= scalef;
  T[1] *= scalef;
  T[2] *= scalef;
  if ( mytype == transl1 )
    min[0] *= scalef, max[0] *= scalef;
}


inline int mpkTransform::num_params()
{
  switch ( mytype ) {
  case transl1:
  case rot1:
  case transl_rot1: return 1;
  case func: return num_params_f;
  default: return 0;
  }  
}

// update transform according to normalized parameter values
inline bool mpkTransform::update()
{
  switch ( mytype ) {
  case transl1: {// translation along axis
    double scalparam = min[0] + (max[0]-min[0]) * param_ptr[0];
    T[0] = scalparam*axis[0]; T[1] = scalparam*axis[1]; T[2] = scalparam*axis[2];
    return true;
  }
  case transl_rot1:
  case rot1: {// rotation around axis
    double angle = min[0] + (max[0]-min[0]) * param_ptr[0];
    double co = cos(angle);
    double si = sin(angle);
    double co1 = 1-co;

    R[0][0] = axis[0]*axis[0] * co1 + co;
    R[0][1] = axis[0]*axis[1] * co1 - axis[2]*si;
    R[0][2] = axis[0]*axis[2] * co1 + axis[1]*si;
    
    R[1][0] = axis[0]*axis[1] * co1 + axis[2]*si;
    R[1][1] = axis[1]*axis[1] * co1 + co;
    R[1][2] = axis[2]*axis[1] * co1 - axis[0]*si;
    
    R[2][0] = axis[2]*axis[0] * co1 - axis[1]*si;
    R[2][1] = axis[2]*axis[1] * co1 + axis[0]*si;
    R[2][2] = axis[2]*axis[2] * co1 + co;

    return true;
  }
  case func:
    transf_func(*this,param_ptr);
    return true;
  default:
    return false;
  }
}

// set transform to X*Y
inline void mpkTransform::set_product(const mpkTransform& X, const mpkTransform& Y)
{
  mytype = constant;
  scalef = 1.0;

  if ( X.is_identity() ) {
    T[0] = Y.T[0], T[1] = Y.T[1], T[2] = Y.T[2];
    R[0][0] = Y.R[0][0], R[0][1] = Y.R[0][1], R[0][2] = Y.R[0][2];
    R[1][0] = Y.R[1][0], R[1][1] = Y.R[1][1], R[1][2] = Y.R[1][2];
    R[2][0] = Y.R[2][0], R[2][1] = Y.R[2][1], R[2][2] = Y.R[2][2];
  }
  else if ( Y.is_identity() ) {
    T[0] = X.T[0], T[1] = X.T[1], T[2] = X.T[2];
    R[0][0] = X.R[0][0], R[0][1] = X.R[0][1], R[0][2] = X.R[0][2];
    R[1][0] = X.R[1][0], R[1][1] = X.R[1][1], R[1][2] = X.R[1][2];
    R[2][0] = X.R[2][0], R[2][1] = X.R[2][1], R[2][2] = X.R[2][2];
  }

  else {

    T[0] = X.T[0] + X.R[0][0]*Y.T[0] + X.R[0][1]*Y.T[1] + X.R[0][2]*Y.T[2];
    T[1] = X.T[1] + X.R[1][0]*Y.T[0] + X.R[1][1]*Y.T[1] + X.R[1][2]*Y.T[2];
    T[2] = X.T[2] + X.R[2][0]*Y.T[0] + X.R[2][1]*Y.T[1] + X.R[2][2]*Y.T[2];
    
    R[0][0] = X.R[0][0]*Y.R[0][0] + X.R[0][1]*Y.R[1][0] + X.R[0][2]*Y.R[2][0];
    R[0][1] = X.R[0][0]*Y.R[0][1] + X.R[0][1]*Y.R[1][1] + X.R[0][2]*Y.R[2][1];
    R[0][2] = X.R[0][0]*Y.R[0][2] + X.R[0][1]*Y.R[1][2] + X.R[0][2]*Y.R[2][2];
    
    R[1][0] = X.R[1][0]*Y.R[0][0] + X.R[1][1]*Y.R[1][0] + X.R[1][2]*Y.R[2][0];
    R[1][1] = X.R[1][0]*Y.R[0][1] + X.R[1][1]*Y.R[1][1] + X.R[1][2]*Y.R[2][1];
    R[1][2] = X.R[1][0]*Y.R[0][2] + X.R[1][1]*Y.R[1][2] + X.R[1][2]*Y.R[2][2];
    
    R[2][0] = X.R[2][0]*Y.R[0][0] + X.R[2][1]*Y.R[1][0] + X.R[2][2]*Y.R[2][0];
    R[2][1] = X.R[2][0]*Y.R[0][1] + X.R[2][1]*Y.R[1][1] + X.R[2][2]*Y.R[2][1];
    R[2][2] = X.R[2][0]*Y.R[0][2] + X.R[2][1]*Y.R[1][2] + X.R[2][2]*Y.R[2][2];
  }
}
  
// concatenate X to the right to the transform
inline void mpkTransform::append(const mpkTransform& X)
{
  if ( is_identity() ) {
    mytype = constant;
    scalef = 1.0;
    T[0] = X.T[0], T[1] = X.T[1], T[2] = X.T[2];
    R[0][0] = X.R[0][0], R[0][1] = X.R[0][1], R[0][2] = X.R[0][2];
    R[1][0] = X.R[1][0], R[1][1] = X.R[1][1], R[1][2] = X.R[1][2];
    R[2][0] = X.R[2][0], R[2][1] = X.R[2][1], R[2][2] = X.R[2][2];
  }
    
  else if ( !X.is_identity() ) {
  
    mytype = constant;
    scalef = 1.0;
    
    T[0] = T[0] + R[0][0] * X.T[0] +  R[0][1] * X.T[1] + R[0][2] * X.T[2];
    T[1] = T[1] + R[1][0] * X.T[0] +  R[1][1] * X.T[1] + R[1][2] * X.T[2];
    T[2] = T[2] + R[2][0] * X.T[0] +  R[2][1] * X.T[1] + R[2][2] * X.T[2];
    
    double tmpR0[3] = {R[0][0], R[0][1], R[0][2]};
    
    R[0][0] = tmpR0[0]*X.R[0][0] + tmpR0[1]*X.R[1][0] + tmpR0[2]*X.R[2][0];
    R[0][1] = tmpR0[0]*X.R[0][1] + tmpR0[1]*X.R[1][1] + tmpR0[2]*X.R[2][1];
    R[0][2] = tmpR0[0]*X.R[0][2] + tmpR0[1]*X.R[1][2] + tmpR0[2]*X.R[2][2];
    
    double tmpR1[3] = {R[1][0], R[1][1], R[1][2]};
    
    R[1][0] = tmpR1[0]*X.R[0][0] + tmpR1[1]*X.R[1][0] + tmpR1[2]*X.R[2][0];
    R[1][1] = tmpR1[0]*X.R[0][1] + tmpR1[1]*X.R[1][1] + tmpR1[2]*X.R[2][1];
    R[1][2] = tmpR1[0]*X.R[0][2] + tmpR1[1]*X.R[1][2] + tmpR1[2]*X.R[2][2];
    
    double tmpR2[3] = {R[2][0], R[2][1], R[2][2]};
    
    R[2][0] = tmpR2[0]*X.R[0][0] + tmpR2[1]*X.R[1][0] + tmpR2[2]*X.R[2][0];
    R[2][1] = tmpR2[0]*X.R[0][1] + tmpR2[1]*X.R[1][1] + tmpR2[2]*X.R[2][1];
    R[2][2] = tmpR2[0]*X.R[0][2] + tmpR2[1]*X.R[1][2] + tmpR2[2]*X.R[2][2];

  }
}

inline void mpkTransform::apply(double out[3], const double in[3]) const
{
  if ( is_identity() ) {
    out[0] = in[0], out[1] = in[1], out[2] = in[2];
  }
  else {
    out[0] = R[0][0]*in[0] + R[0][1]*in[1] + R[0][2]*in[2] + T[0];
    out[1] = R[1][0]*in[0] + R[1][1]*in[1] + R[1][2]*in[2] + T[1];
    out[2] = R[2][0]*in[0] + R[2][1]*in[1] + R[2][2]*in[2] + T[2];
  }
}

inline void mpkTransform::apply(double out[3],
				const double R[3][3], const double T[3], 
				const double in[3])
{
  out[0] = R[0][0]*in[0] + R[0][1]*in[1] + R[0][2]*in[2] + T[0];
  out[1] = R[1][0]*in[0] + R[1][1]*in[1] + R[1][2]*in[2] + T[1];
  out[2] = R[2][0]*in[0] + R[2][1]*in[1] + R[2][2]*in[2] + T[2];
}


inline void mpkTransform::apply_inv(double out[3], const double in[3]) const
{
  if ( is_identity() ) {
    out[0] = in[0], out[1] = in[1], out[2] = in[2];
  }
  else {
    double tmp[3] = {in[0]-T[0], in[1]-T[1], in[2]-T[2]};
    out[0] = R[0][0]*tmp[0] + R[1][0]*tmp[1] + R[2][0]*tmp[2];
    out[1] = R[0][1]*tmp[0] + R[1][1]*tmp[1] + R[2][1]*tmp[2];
    out[2] = R[0][2]*tmp[0] + R[1][2]*tmp[1] + R[2][2]*tmp[2];
  }
}


inline void mpkTransform::print(ostream& f) const
{
  for ( int r=0; r<3; r++ ) {
    for ( int c=0; c<3; c++ )
      f << R[r][c] << "\t";
    f << "\t\t";
    f << T[r] << endl;
  }
}

#endif

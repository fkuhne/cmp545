#include "mpkTraceVis.H"

#include <Inventor/nodes/SoSphere.h> 
#include <Inventor/nodes/SoCube.h> 

mpkTraceVis::
mpkTraceVis(mpkRobotCollection* robots, const vector<mpkConfig>& path,
	double eps, const float* rgb)
{
  int k;
  this->eps = eps;
  this->robots = robots;
  mpkConfig qbak(robots->num_params());
  robots->get_config(qbak);

  // setup iv nodes
  root = new SoSeparator;
  root->ref();
  ivswitch = new SoSwitch();
  root->addChild(ivswitch);
  ivswitch->whichChild = SO_SWITCH_NONE;
  SoBaseColor* col = new SoBaseColor;
  ivswitch->addChild(col);
  float cval[][3] = {{1,1,0}};
  if ( rgb ) cval[0][0] = rgb[0], cval[0][1] = rgb[1], cval[0][2] = rgb[2];
  col->rgb.setValues(0,3,cval);
  SoDrawStyle *linestyle = new SoDrawStyle;
  ivswitch->addChild(linestyle);
  linestyle->lineWidth.setValue(2);
  SoLineSet *lineset = new SoLineSet;
  ivswitch->addChild(lineset);
  SoVertexProperty* vprop = new SoVertexProperty;
  lineset->vertexProperty = vprop;

  if ( path.size() < 1 ) return;

  // add line sequences
  int line_idx = 0;
  int vertex_idx = 0;
  for ( int i=0; i<robots->num_robots(); i++ ) {
    int joint_idx = robots->rob[i]->tracepoint_joint_idx;
    if ( joint_idx < 0 ) continue;
    
    // first compute vertices and collect them in vts
    vts.clear();
    for ( k=1; k<path.size(); k++ ) {
      approx_seg(i, joint_idx, path[k-1], path[k]);
    }
    add_vertex(i, joint_idx, path[path.size()-1]);
    add_sphere(i, joint_idx, path[path.size()-1]);

    // now add line sequence through them to the iv node
    lineset->numVertices.set1Value(line_idx++, vts.size());
    
    for ( int n=0; n<vts.size(); n++ )
      vprop->vertex.set1Value(vertex_idx++, vts[n].x, vts[n].y, vts[n].z);

    // to show the segment endpoints
    for ( k=1; k<path.size(); k++ ) {
      add_sphere(i, joint_idx, path[k], true);
    }
  }
  
  robots->set_config(qbak);
}

void 
mpkTraceVis::
add_vertex(int rob_idx, int joint_idx, const mpkConfig& q)
{
  robots->set_config(q);
  robots->rob[rob_idx]->compute_forward_kinematics(joint_idx);
  double p0[3] = {0,0,0}, p1[3];
  robots->rob[rob_idx]->joints[joint_idx].Taccu.apply(p1,p0);
  point3 v;
  v.x = p1[0];
  v.y = p1[1];
  v.z = p1[2];
  vts.push_back(v);
}

void 
mpkTraceVis::
add_sphere(int rob_idx, int joint_idx, const mpkConfig& q, bool seg_endp)
{
  robots->set_config(q);
  robots->rob[rob_idx]->compute_forward_kinematics(joint_idx);
  double p0[3] = {0,0,0}, p1[3];
  robots->rob[rob_idx]->joints[joint_idx].Taccu.apply(p1,p0);

  // add a sphere to show vertex
  SoSeparator* sep = new SoSeparator;
  ivswitch->addChild(sep);
  if ( seg_endp ) {
    SoBaseColor* col = new SoBaseColor;
    sep->addChild(col);
    float cval[][3] = {{1,0,0}};
    col->rgb.setValues(0,3,cval);
  }
  SoTransform* tr = new SoTransform;
  sep->addChild(tr);
  tr->translation.setValue(p1[0],p1[1],p1[2]);
  SoCube* cb = new SoCube;
  sep->addChild(cb);
  cb->width  = 0.01;
  cb->height = 0.01;
  cb->depth  = 0.01;
}

void 
mpkTraceVis::
approx_seg(int rob_idx, int joint_idx,
	   const mpkConfig& q0, const mpkConfig& q1)
{
  if ( q0.Linf_dist(q1) < eps ) {
    add_vertex(rob_idx, joint_idx, q0);
    add_sphere(rob_idx, joint_idx, q0);
    return;
  }
  mpkConfig qmid(q0.size());
  qmid.lin_interpol(0.5, q0, q1);
  approx_seg(rob_idx, joint_idx, q0, qmid);
  approx_seg(rob_idx, joint_idx, qmid, q1);
}


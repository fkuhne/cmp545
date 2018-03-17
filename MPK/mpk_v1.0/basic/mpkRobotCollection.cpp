#include <vector>
#include "mpkRobot.H"
#include "mpkRobotCollection.H"

mpkRobotCollection::mpkRobotCollection(SoGroup* scenegraph)
{
  SoSearchAction robotSearchAction;
  robotSearchAction.setType(mpkRobot::getClassTypeId());
  robotSearchAction.setInterest(SoSearchAction::ALL);
  robotSearchAction.setSearchingAll(TRUE);
  robotSearchAction.apply(scenegraph);
  SoPathList robs;
  robs = robotSearchAction.getPaths();

  num_all_params = 0;
  for (int i = 0; i < robs.getLength(); i++) {

    mpkRobot *r = (mpkRobot *)robs[i]->getNodeFromTail(0);

    r->init();
    rob.push_back(r->robot);
    first_idx.push_back(num_all_params);
    num_all_params += r->robot->num_params;

  }

  param_weights.resize(num_all_params);
  int c=0;
  {for ( int i=0; i<rob.size(); i++ ) {
    for ( int j=0; j<rob[i]->num_params; j++ ) {
      param_weights[c] = rob[i]->param_opts[j].weight;
      c++;
    }
  }}
}

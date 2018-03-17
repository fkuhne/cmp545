#include "mpkObstacle.H"

SO_NODE_SOURCE(mpkObstacle);

void
mpkObstacle::initClass()
{
  SO_NODE_INIT_CLASS(mpkObstacle, SoSeparator, "Separator");
}

mpkObstacle::mpkObstacle()
{
  SO_NODE_CONSTRUCTOR(mpkObstacle);
}


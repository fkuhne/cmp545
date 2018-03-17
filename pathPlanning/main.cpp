/*
 * Felipe Kuhne
 * fkuhne at gmail dot com
 * June, 2017
 */

#include "Grid.h"
#include <unistd.h>

int main(int argc, char** argv)
{

//  Grid map(CELL_SIZE * 50, CELL_SIZE * 25);
  Grid map(1000, 500);

  while(map.isRunning())
  {
    if(map.hasEvent())
    {
      map.handleEvent();
    }

    usleep(5000);
  }

  return 0;
}

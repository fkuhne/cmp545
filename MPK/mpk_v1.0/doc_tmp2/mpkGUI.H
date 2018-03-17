#ifndef MPK_GUI_H
#define MPK_GUI_H

#include <vector>
#include "mpk_inventor.h"
#include "mpkRobotCollection.H"
#include "mpkObstacleCollection.H"

/**@memo Functions for reading scene files and for realizing a simple
   but powerful GUI.
 */
class mpkGUI {

public:

  ///
  static void init(char *argv[],
		   void (*myKeyPressCB)(void *, SoEventCallback *)=0,
		   void (*myMouseButtonCB)(void *, SoEventCallback *)=0);

  ///
  static void read_scene(const char* scene_fname);

  ///
  static mpkRobotCollection* get_robots()
  {return new mpkRobotCollection(scene);};
  
  ///
  static mpkObstacleCollection* get_obstacles()
  {return new mpkObstacleCollection(scene);};

  ///
  static void start(const char* win_title = "MPK");
  
  ///
  static void save_snapshot(const char* image_fname);
  
#ifdef WIN32
  ///@memo Windows version only
  static SoWinExaminerViewer* myViewer;
  ///@memo Windows version only
  static HWND myWindow;
#else
  ///@memo Unix version only
  static SoXtExaminerViewer* myViewer;
  ///@memo Unix version only
  static Widget myWindow;
#endif

  ///
  static SoPerspectiveCamera *myCamera;
  ///
  static SoSeparator* root;
  ///
  static SoSeparator* scene;

private:

  ///
  static SoSeparator* scene_sep;
  ///
  static void init_extender_classes();

};


#endif


#include "mpk_rand.h"
#include "mpkGUI.H"
#include "mpkRobot.H"
#include "mpkObstacle.H"
#include "mpkIncludeFile.H"

#ifdef WIN32
SoWinExaminerViewer* mpkGUI::myViewer;
HWND mpkGUI::myWindow;
#else
SoXtExaminerViewer* mpkGUI::myViewer;
Widget mpkGUI::myWindow;
#endif
SoPerspectiveCamera* mpkGUI::myCamera;
SoSeparator* mpkGUI::root;
SoSeparator* mpkGUI::scene;
SoSeparator* mpkGUI::scene_sep;

void
mpkGUI::
init(char *argv[],
     void (*myKeyPressCB)(void *, SoEventCallback *),
     void (*myMouseButtonCB)(void *, SoEventCallback *))
{
#ifdef WIN32
  myWindow = SoWin::init(argv[0]);
#else
  myWindow = SoXt::init(argv[0]);
#endif
  if (myWindow == NULL) {
#ifdef WIN32
    getchar();
#endif
    exit(1);
  }
  init_extender_classes();

  root = new SoSeparator;
  root->ref();

  SoEventCallback *myEventCB = new SoEventCallback;
  root->addChild(myEventCB);
  if ( myKeyPressCB )
    myEventCB->addEventCallback(SoEvent::getClassTypeId(),myKeyPressCB, 0);
  if ( myMouseButtonCB )
    myEventCB->addEventCallback(SoEvent::getClassTypeId(),myMouseButtonCB, 0);

  myCamera = new SoPerspectiveCamera;
  root->addChild(myCamera);

  scene = new SoSeparator;
  root->addChild(scene);
  scene_sep = 0;
  myViewer = 0;
}

void mpkGUI::read_scene(const char *scene_fname)
{
  if ( scene_sep ) { scene->removeChild(scene_sep); }
  
  SoInput sceneInput;
  sceneInput.addDirectoryFirst("ivmodels");
  sceneInput.addEnvDirectoriesFirst("IVMODELS");
  sceneInput.addDirectoryLast("ivmodels.private");
  sceneInput.openFile(scene_fname);
  scene_sep = SoDB::readAll(&sceneInput);
  if ( !scene_sep ) {
    cerr << "mpkGUI::init(): Error reading file " << scene_fname << endl;
#ifdef WIN32
    getchar();
#endif
    exit(1);
  }
  scene->addChild(scene_sep);

  SoSearchAction searchincl;
  searchincl.setType(mpkIncludeFile::getClassTypeId());
  searchincl.setInterest(SoSearchAction::ALL);
  searchincl.setSearchingAll(TRUE);
  searchincl.apply(root);
  SoPathList incllist = searchincl.getPaths();
  for ( int k=0; k < incllist.getLength(); k++ ) {
    mpkIncludeFile* inclf = (mpkIncludeFile*)incllist[k]->getNodeFromTail(0);
    inclf->read_and_init();
  }
  if ( myViewer )
    myCamera->viewAll(root, myViewer->getViewportRegion());
}

void
mpkGUI::
start(const char* win_title)
{
#ifdef WIN32
  myViewer = new SoWinExaminerViewer(myWindow);
#else
  myViewer = new SoXtExaminerViewer(myWindow);
#endif
  myCamera->viewAll(root, myViewer->getViewportRegion());
  myViewer->setSceneGraph(root);
  myViewer->setTitle(win_title);
  myViewer->setBackgroundColor(SbColor(0.6,0.6,1));
  myViewer->setSize(SbVec2s(600,600));
  myViewer->setDecoration(TRUE);
  myViewer->setViewing(TRUE);
  myViewer->setPopupMenuEnabled(TRUE);
#ifndef WIN32
  myViewer->setDrawStyle(SoXtViewer::INTERACTIVE,
			 SoXtViewer::VIEW_SAME_AS_STILL);
#endif

  myViewer->show();

#ifdef WIN32
  SoWin::show(myWindow);  // Display main window
  SoWin::mainLoop();      // Main Inventor event loop
#else
  SoXt::show(myWindow);  // Display main window
  SoXt::mainLoop();      // Main Inventor event loop
#endif
}

void
mpkGUI::
save_snapshot(const char* image_fname)
{
#ifdef WIN32
  cerr << "mpkGUI::save_snapshot(): not supported." << endl;
#else
  char out_fname[100];
  sprintf(out_fname,"tmp%4.4d.ps", mpk_lrand()%10000);
  FILE* outfile = fopen(out_fname, "wb");
  if (outfile) {
    SoOffscreenRenderer* offscreen = new
      SoOffscreenRenderer(myViewer->getGLRenderAction());
    SoNode* rootNode = myViewer->getSceneManager()->getSceneGraph();
    SbColor col = myViewer->getBackgroundColor();
    offscreen->setBackgroundColor(col);
    offscreen->render(rootNode);
    //renderOffscreen(getWholeSceneGraph());
    offscreen->writeToPostScript(outfile);
    fclose(outfile);
    char buf[1000];
    sprintf(buf,"convert %s %s", out_fname, image_fname);
    system(buf);
    sprintf(buf,"rm %s", out_fname);
    system(buf);
    delete offscreen;
  }
  else
    cerr << "mpkGUI::save_snapshot(): problem temporary file " << out_fname << endl;
#endif
}


void
mpkGUI::
init_extender_classes()
{
  mpkRobot::initClass();
  mpkObstacle::initClass();
  mpkIncludeFile::initClass();
}


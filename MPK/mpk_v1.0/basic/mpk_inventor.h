
#ifndef INVENTOR_ALL_h
#define INVENTOR_ALL_h


#include <Inventor/SbLinear.h>
#include <Inventor/SoNodeKitPath.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPrimitiveVertex.h>

#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoBoxHighlightRenderAction.h>

#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/events/SoLocation2Event.h>

#include <Inventor/fields/SoSFName.h>
#include <Inventor/fields/SoSFVec4f.h>

#include <Inventor/nodes/SoBaseColor.h> 
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoLineSet.h> 
#include <Inventor/nodes/SoDrawStyle.h> 
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodes/SoSwitch.h> 
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h> 

#include <Inventor/sensors/SoIdleSensor.h>

#ifdef WIN32
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/Viewers/SoWinExaminerViewer.h>
#include <Inventor/Win/devices/SoWinKeyboard.h> 
#else
#include <Inventor/Xt/SoXt.h>
#include <Inventor/Xt/viewers/SoXtExaminerViewer.h>
#endif

#endif

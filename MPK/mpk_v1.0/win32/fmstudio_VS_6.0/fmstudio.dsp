# Microsoft Developer Studio Project File - Name="fmstudio" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=fmstudio - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "fmstudio.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "fmstudio.mak" CFG="fmstudio - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "fmstudio - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "fmstudio - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "fmstudio - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "C:\Coin3D\include" /I "C:\PQP_v1.3\src" /I "..\..\basic" /I "..\..\gui" /I "..\..\robots" /I "..\..\sbl" /I "..\..\devel" /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /D "ROBOT_BASE_MOVABLE" /D "NO_TRANSFORM_CACHE" /D "SAVE_MEMORY" /D "COIN_NOT_DLL" /D "SOWIN_NOT_DLL" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib coin1.lib sowin0.lib PQP.lib /nologo /subsystem:console /machine:I386 /libpath:"C:\PQP_v1.3\lib" /libpath:"C:\Coin3D\lib"
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=copy Release\fmstudio.exe ..\..\prog\fmstudio.exe
# End Special Build Tool

!ELSEIF  "$(CFG)" == "fmstudio - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /I "C:\Coin3D\include" /I "C:\PQP_v1.3\src" /I "..\..\basic" /I "..\..\gui" /I "..\..\robots" /I "..\..\sbl" /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /D "ROBOT_BASE_MOVABLE" /D "SOWIN_NOT_DLL" /D "COIN_NOT_DLL" /D "NO_TRANSFORM_CACHE" /D "SAVE_MEMORY" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib coin1.lib sowin0.lib PQP.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept /libpath:"C:\PQP_v1.3\lib" /libpath:"C:\Coin3D\lib"

!ENDIF 

# Begin Target

# Name "fmstudio - Win32 Release"
# Name "fmstudio - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Group "basic"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\basic\mpkAdaptSegmentChecker.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkBaseRobot.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkCollDistAlgo.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkCollPair.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkConfigChecker.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkIncludeFile.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkObstacle.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkObstacleCollection.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkPathSmoother.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkRobot.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkRobotCollection.cpp
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkSimpleSegmentChecker.cpp
# End Source File
# End Group
# Begin Group "gui"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\gui\mpk_opt.cpp
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkBVControl.cpp
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkGUI.cpp
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkMouseControl.cpp
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkTraceVis.cpp
# End Source File
# End Group
# Begin Group "robots"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\robots\mpkDemoRobot.cpp
# End Source File
# Begin Source File

SOURCE=..\..\robots\mpkFreeFlyingObject.cpp
# End Source File
# End Group
# Begin Group "sbl"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\sbl\sblPlanner.cpp
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblRandVal.cpp
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblRn.cpp
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblTree.cpp
# End Source File
# End Group
# Begin Group "prog"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\prog\fmstudio.cpp
# End Source File
# End Group
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Group "basic_h"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\basic\bjhash.h
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpk_defs.h
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpk_inventor.h
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpk_rand.h
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpk_robot_type.h
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkAdaptSegmentChecker.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkBaseRobot.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkCollDistAlgo.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkCollPair.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkCollPairSet.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkConfig.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkConfigChecker.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkIncludeFile.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkObstacle.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkObstacleCollection.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkPathSmoother.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkRobot.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkRobotCollection.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkSimpleSegmentChecker.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkTransform.H
# End Source File
# Begin Source File

SOURCE=..\..\basic\mpkTransformCache.H
# End Source File
# End Group
# Begin Group "gui_h"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\gui\mpkBVControl.H
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkGUI.H
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkMouseControl.H
# End Source File
# Begin Source File

SOURCE=..\..\gui\mpkTraceVis.H
# End Source File
# End Group
# Begin Group "robots_h"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\robots\mpkDemoRobot.H
# End Source File
# Begin Source File

SOURCE=..\..\robots\mpkFreeFlyingObject.H
# End Source File
# End Group
# Begin Group "sbl_h"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\sbl\sblBin.H
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblEdge.H
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblMilestone.H
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblPlanner.H
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblRandVal.H
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblRn.H
# End Source File
# Begin Source File

SOURCE=..\..\sbl\sblTree.H
# End Source File
# End Group
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project

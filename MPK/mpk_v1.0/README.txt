==================================================================

MPK_v1.0 - The Motion Planning Kit

Copyright (c) 2003 Stanford University, Stanford, CA 94305
MPK comes with absolutely no warranty and is free for research and
non-commercial use only.

Please visit the MPK web site for downloading future versions and
submitting questions or problem reports:
http://robotics.stanford.edu/~latombe/mpk.html

==================================================================

Authors:
--------
Fabian Schwarzer & Mitul Saha (MPK library, A-SBL)
Gildardo Sanchez (original SBL planner)

--------
Contents
--------

1  Description
2  Requirements
3  Contents of the archive
4  Getting started

--------------
1  Description
--------------

Motion Planning Kit (MPK) is a C++ library and toolkit for developing
single- and multi-robot motion planning applications.  It allows the
user to define arbitrary kinematic tree structures and an arbitrary
number of robots and obstacles at the same time.  New robots with any
combination of prismatic and revolute joints can be defined and added
easily without recompiling the code.

MPK contains a fast and reliable dynamic collision checker
(mpkAdaptSegmentChecker) for testing straight-line segments in
configuration space.  This collision checker automatically adapts the
required resolution by relating bounds for workspace distances between
pairs of objects and lengths of curves traced by all points on these
objects.  The checker can handle triangulated models of high
complexity and is guaranteed not to miss collisions even if the
objects are very thin.  For comparison, MPK also includes a simple
quasi-dynamic collision checker (mpkSimpleSegmentChecker) that tests
segments in c-space by sampling them up to a user-specified c-space
resolution "epsilon".

MPK includes SBL, a fast single query lazy collision checking
probabilistic path planner.  SBL was originally developed by Gildardo
Sanchez and has been incorporated in two forms into MPK by Mitul Saha:
SBL uses the simple epsilon-collision-checker for segments and A-SBL
uses MPK's new adaptive collision checker.

Further details of MPK's adaptive collision checker can be found in:

* F. Schwarzer, M. Saha, and J.-C. Latombe.  Exact collision checking
  of robot paths. In Workshop on Algorithmic Foundations of Robotics
  (WAFR), Nice, Dec 2002.

Further details of SBL can be found in:

* G. Sanchez and J.C. Latombe.  A Single-Query Bi-Directional
  Probabilistic Roadmap Planner with Lazy Collision Checking. Int.
  Symposium on Robotics Research (ISRR'01), Lorne, Victoria,
  Australia, November 2001.

---------------
2  Requirements
---------------

- Standard Template Library (STL)
  STL comes with most modern C++ compilers
 
- Open Inventor (Unix) or Coin3D/SoWin (free Windows port of Open Inventor)
  Free versions can be downloaded from the web.  MPK uses the Open
  Inventor file format for its triangulated geometry models.
  Furthermore, MPK extends Open Inventor's scene file format for
  defining planning scenarios, and uses Open Inventor classes for
  graphics output and a simple keyboard command interface.
  For Windows we recommend that you install Coin3D/SoWin from:
  http://robotics.stanford.edu/~mitul/mpk/coin3d.zip
  as the latest version of the Coin3D from the official Coin3D
  site may not be compatible.

- Proximity Query Package (PQP)
  PQP is a library for static collision checking and distance
  computation that uses oriented bounding boxes (OBBs) and rectangle
  swept spheres (RSSs).  It was developed at the University of North
  Carolina and can be obtained from http://www.cs.unc.edu/~geom/SSV/.
  MPK's collision checking and distance computation algorithms are
  based on PQP's OBB and RSS hierarchies.  MPK requires PQP to be
  installed.  MPK should work with any version of PQP greater than 1.1.

--------------------------
3  Contents of the archive
--------------------------

3.1 Source code subdirectories
------------------------------

basic/          Basic mpk_v1.0 classes
gui/            GUI related code
sbl/		SBL planner code
robots/         Source code for hard-coded robots
prog/           Demo programs

3.2 Runtime subdirectories
--------------------------
 
prog/           Demo programs and Unix executables
win32/...       Windows executables & project files
ivmodels/       Open Inventor models
scenes/         Scene files and planning problem descriptions
robots/		Robot description files (.rob)

3.3 Other subdirectories
------------------------

doc/            documentation, including html class documentation
lib/            directory for (static) libraries (UNIX only)

------------------
4  Getting Started
------------------

4.1 Installing and compiling
----------------------------
 
Unix (with g++):

1) Make sure that Open Inventor and PQP are installed
2) "tar xfz mpk_v1.0.tgz" or "unzip mpk_v1.0.zip"(creates a directory mpk_v1.0/)
3) "cd mpk_v1.0" and edit the file Make.config to set PQPDIR properly
4) "make" - this should create the MPK libraries in the lib/
   subdirectory and the demo programs in the prog/ subdirectory
 
Windows (how to build the program fmstudio with Visual Studio.NET2002 and Visual
Studio 6.0):

1) Unzip the file "mpk_v1.0.zip". Install Coin3D, SoWin and PQP (PQP contains a Windows 
   project file to build PQP.lib). (See the "Requirements" section to find out where 
   to get them from)
2) Copy coin1.dll and sowin0.dll in the $SYSTEM32 directory
   (C:\Windows\System32)
3) Start Visual Studio and open the Visual Studio project file from the "win32"
   sub-folders.
4) Make sure that the include and library paths for Coin3D/SoWin and PQP are
   properly set in the project properties/settings
5) Select "build project" (the executable fmstudio.exe is generated in either
   mpk_v1.0\win32\fmstudio*\Debug or mpk_v1.0\win32\fmstudio*\Release
   or mpk_v1.0\prog, depending on the project properties)
6) When executing fmstudio.exe, set the working directory to mpk_v1.0\
   (make sure it contains the scenes\ and ivmodels\ subdirectories).

To build the programs demo and randwalk under Windows, create project
files similar to fmstudio.vcproj or fmstudio.dsw but replace fmstudio.cpp
by demo.cp or randwalk.cpp.

Note: If the Visual Studio 6.0 project dosn't compile then most probably
your Visual Studio 6.0 is not up to date. Update it using the Visual
Studio service pack available at:
http://msdn.microsoft.com/vstudio/downloads/updates/sp/vs6/sp5/
(If this site has expired then type "Visual Studio 6.0 service pack" on google)

4.2 Running and understanding the demo programs
-----------------------------------------------

Run and examine the source code of the demo programs in the prog/
subdirectory in the following order:

1. fmstudio - Run this to see the PATH-PLANNER in action:
   A testbed that integrates a powerful GUI to setup
   individual configurations and complete tasks with the SBL planner and
   a simple path smoother.  A HTML user manual for fmstudio can be found
   in the file 'doc/fmstudio/index.html'.

2. demo - This program opens scene file and, whenever the SPACE key is
   pressed, samples a random collision-free segment in c-space that
   originates at the current configuration and then moves the
   configuration of the system to the other end-point of the segment.
   Call the program from the mpk_v1.0/ subdirectory as follows:
     "prog/demo scenes/demo1.iv"
   Repeat with other iv-files from the scenes subdirectory.  The file
   demo.cpp is extensively documented and should provide a good
   starting point to understanding the MPK library.

3. randwalk - This program performs a random walk in free c-space once
   the 'S' key is pressed.
   Call the program from the mpk_v1.0/ subdirectory as follows:
     "prog/randwalk scenes/demo1.iv"
   Repeat with other iv-files from the scenes subdirectory or without
   any arguments to show further available options.


4.3 Further documentation
-------------------------

Read the detailed documentation in the following order:

1.  doc/fmstudio/index.html (fmstudio user manual: Read this to know 
    how to run the PATH PLANNER)

2.  doc/mpk_new/index.html (how to create new scenes and robot types)

3.  doc/index.html (MPK class index)


--
ALWAYS FEEL FREE TO WRITE US FOR ANY OTHER INFORMATION OR DIFFICLUTIES
(mpk@robotics.stanford.edu)
--
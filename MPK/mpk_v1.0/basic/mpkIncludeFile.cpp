#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "mpkIncludeFile.H"

using namespace std;

SO_NODE_SOURCE(mpkIncludeFile);

void
mpkIncludeFile::initClass()
{
  SO_NODE_INIT_CLASS(mpkIncludeFile, SoSeparator, "Separator");
}

mpkIncludeFile::mpkIncludeFile()
{
  SO_NODE_CONSTRUCTOR(mpkIncludeFile);
  SO_NODE_ADD_FIELD(name, (""));
}

mpkIncludeFile::~mpkIncludeFile()
{
}

SoSeparator* mpkIncludeFile::read_include_file(const char fname[])
{
  SoInput fileInput;
  fileInput.addEnvDirectoriesFirst("IVMODELS");
  fileInput.addDirectoryLast("ivmodels");
  fileInput.addDirectoryLast("ivmodels.private");
  fileInput.addDirectoryLast("scenes");
  fileInput.addDirectoryLast("scenes.private");
  fileInput.openFile(fname);
  SoSeparator* scene = SoDB::readAll(&fileInput);
  if ( !scene ) {
#ifdef WIN32
    getchar();
#endif
    exit(1);
  }
  return scene;
}

void mpkIncludeFile::read_and_init()
{
  // read include file and add it to this node
  SoSeparator* incl_sep = read_include_file(name.getValue().getString());
  addChild(incl_sep);

  // prepend mpkIncludeFile node name to all node names (except __triangulate__)
  SoSearchAction inclsearch;
  inclsearch.setType(SoNode::getClassTypeId());
  inclsearch.setInterest(SoSearchAction::ALL);
  inclsearch.setSearchingAll(TRUE);
  inclsearch.apply(incl_sep);
  SoPathList pathlist;
  pathlist = inclsearch.getPaths();
  for (int i=0; i < pathlist.getLength(); i++ ) {
    SoNode* ob = pathlist[i]->getNodeFromTail(0);
    if ( strlen(ob->getName().getString()) && 
	 // don't add prefix to __triangulate__ tags:
	 strcmp(ob->getName().getString(), "__triangulate__" ) ) {
      // if not a __triangulate__ node: add prefix to included nodes
      char* buf = new char[strlen(ob->getName().getString())+strlen(getName().getString())+2];
      strcpy(buf,getName().getString());
      buf[strlen(getName().getString())] = ':';
      strcpy(buf+strlen(getName().getString())+1, ob->getName().getString());
      ob->setName(buf);
      delete [] buf;
    }
  }

}

#ifndef MPK_INCLUDE_FILE_H
#define MPK_INCLUDE_FILE_H

#include "mpk_inventor.h"

/**@memo Extension of SoSeparator to include environments in scene
   files.  Required only for reading scene files.


 */
class mpkIncludeFile : public SoSeparator {
  
  SO_NODE_HEADER(mpkIncludeFile);
  
public:

  ///@memo The name of the file to be included (read by this node).
  SoSFName name;
  
  static void initClass();
  
  ///
  mpkIncludeFile();

  /**@memo When the scene file is read initially, mpkIncludeFile nodes
     are not resolved (follwed).  Calling this function then reads the
     actual include file and expands the node with the contents read
     from the included file.  To keep it simple, this does not work
     recursively.
  */
  void read_and_init();

private:
  virtual ~mpkIncludeFile();

  // helper function
  SoSeparator* read_include_file(const char fname[]);
  
};

#endif


#ifndef DUMMYCONTROLLER_H
#define DUMMYCONTROLLER_H

#include <string>
#include "Controller.h"

namespace ift {


/**
  * class DummyController
  * 
  */
class DummyController : public Controller
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  DummyController ();

  /**
   * Empty Destructor
   */
  virtual ~DummyController ();

  
  // methods
  //
  virtual ift::Control getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal) ;


};
}; // end of package namespace

#endif // DUMMYCONTROLLER_H

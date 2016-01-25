
#ifndef PCONTROLLER_H
#define PCONTROLLER_H

#include <string>
#include "Controller.h"

namespace ift {


/**
  * class PController
  * 
  */
class PController : public Controller
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  PController ();

  /**
   * Empty Destructor
   */
  virtual ~PController ();


public:
  
  // methods
  //
  virtual ift::Control getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal) ;


};
}; // end of package namespace

#endif // PCONTROLLER_H

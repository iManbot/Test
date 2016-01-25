#include "Transmission.h"
#include <string>

// Constructors/Destructors
//  
namespace ift {

  Transmission::Transmission () {
    initAttributes();
  }
  
  Transmission::~Transmission () { }
  
  Transmission::Transmission(string trsmName, KinematicModelPtr Kptr, ControllerPtr Cptr) 
  : _trsmName(trsmName), _knmatcMdlPtr(Kptr), _ctrlPtr(Cptr)
  {
      
  }
 
  // Methods
  //  
  
  /**
   */
  void Transmission::updateGoal (Goal goal)
  {
    _goal = goal;
  }
  
  
  /**
   * control(): publish control topic
   */
  ift::Control Transmission::control ()
  {
    return _ctrlPtr->getControl(*_knmatcMdlPtr, _goal);
  }
  
  
} //end of package ift

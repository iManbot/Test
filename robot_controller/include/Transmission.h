
#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include <string>
#include "sosdata.h"
#include "KinematicModel.h"
#include "Controller.h"
#include <boost/shared_ptr.hpp>
using namespace std;

namespace ift {

class Transmission;

typedef boost::shared_ptr<Transmission> TransmissionPtr;

/**
  * class Transmission
  * 
  */
class Transmission
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  Transmission ();
  
  /*
   * Constructor
   */
  Transmission(string trsmName, KinematicModelPtr Kptr, ControllerPtr Cptr);

  /**
   * Empty Destructor
   */
  virtual ~Transmission ();

  //Methods
  //

  /**
   */
  void updateGoal (Goal goal) ;


  /**
   * control(): publish control topic
   */
  ift::Control control ();


private:

  ift::KinematicModelPtr _knmatcMdlPtr; // pointer to kinematic model
  ift::ControllerPtr _ctrlPtr; // pointer to controller 
  ift::TrajectoryPoint _goal; // goal
  string _trsmName; 
  

public:

  // Private attribute accessor methods
  //  


  /**
   * Set the value of _knmatcMdl
   * @param new_var the new value of _knmatcMdl
   */
  void set_knmatcMdlPtr (ift::KinematicModelPtr new_var)   {
      _knmatcMdlPtr = new_var;
  }

  /**
   * Get the value of _knmatcMdl
   * @return the value of _knmatcMdl
   */
  ift::KinematicModelPtr get_knmatcMdlPtr ()   {
    return _knmatcMdlPtr;
  }

  /**
   * Set the value of _ctrl
   * @param new_var the new value of _ctrl
   */
  void set_ctrlPtr (ift::ControllerPtr new_var)   {
      _ctrlPtr = new_var;
  }

  /**
   * Get the value of _ctrl
   * @return the value of _ctrl
   */
  ift::ControllerPtr get_ctrlPtr ()   {
    return _ctrlPtr;
  }

  /**
   * Set the value of _goal
   * @param new_var the new value of _goal
   */
  void set_goal (ift::TrajectoryPoint new_var)   {
      _goal = new_var;
  }

  /**
   * Get the value of _goal
   * @return the value of _goal
   */
  ift::TrajectoryPoint get_goal ()   {
    return _goal;
  }

  /**
   * Set the value of _trsmName
   * @param new_var the new value of _trsmName
   */
  void set_port (string new_var)   {
      _trsmName = new_var;
  }

  /**
   * Get the value of _trsmName
   * @return the value of _trsmName
   */
  string get_port ()   {
    return _trsmName;
  }
private:


  void initAttributes () {}

};
}; // end of package namespace

#endif // TRANSMISSION_H

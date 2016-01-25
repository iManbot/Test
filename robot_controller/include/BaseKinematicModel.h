#ifndef BASEKINEMATICMODEL_H
#define BASEKINEMATICMODEL_H

#include <string>
#include "KinematicModel.h"

namespace ift {


/**
  * class BaseKinematicModel
  * 
  */

class BaseKinematicModel : public KinematicModel
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  BaseKinematicModel ();

  /**
   * Constructor
   */
  BaseKinematicModel (unsigned int dim, double L);

  
  /**
   * Empty Destructor
   */
  virtual ~BaseKinematicModel ();


private:
  // Private attributes
  //  
  
  
  double _L = 0.1; // distance from drive unit to center of mass (CM) 


public:


  // Private attribute accessor methods
  //  

    /**
   * Set the value of _L
   * @param new_var the new value of _L
   */
  void set_L (unsigned int new_var)   {
      _L = new_var;
  }

  /**
   * Get the value of _L
   * @return the value of _L
   */
  unsigned int get_L ()   {
    return _L;
  }
  


  // Public methods
  //
  
  /**
   * @param  ctrl 
   * @param  pos
   * @param  accel
   */
  virtual void kinematicODE (ift::Control ctrl, geometry_msgs::Vector3 pos, ift::AccelStamped accel);


  /**
   * @return ift::Control
   * @param  vel
   * @param  accel
   */
  virtual ift::Control inv_kinematicODE (ift::VelStamped vel) const;


};
}; // end of package namespace

#endif // BASEKINEMATICMODEL_H

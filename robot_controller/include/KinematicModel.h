
#ifndef KINEMATICMODEL_H
#define KINEMATICMODEL_H

#include <string>
#include <boost/shared_ptr.hpp>
#include "sosdata.h"



namespace ift {


/**
  * class KinematicModel
  * 
  */

class KinematicModel;

typedef boost::shared_ptr<KinematicModel> KinematicModelPtr;

class KinematicModel
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  KinematicModel ();

  /**
   * Empty Destructor
   */
  virtual ~KinematicModel ();

 

  /**
   * @param  ctrl control
   * @param  pos
   * @param  accel
   */
  virtual void kinematicODE (ift::Control ctrl, geometry_msgs::Vector3 pos, ift::AccelStamped accel) = 0 ;


  /**
   * @return ift::Control
   * @param  vel
   * @param  accel
   */
  virtual ift::Control inv_kinematicODE (ift::VelStamped vel) const = 0 ;


protected:
  
  // protected attributes
  //  
  unsigned int _dimension;
  ift::PoseStamped _pos;
  ift::VelStamped _spd;
  ift::Control _ctrl;


public:


  // Private attribute accessor methods
  //  


  /**
   * Set the value of _dimension
   * @param new_var the new value of _dimension
   */
  void set_dimension (unsigned int new_var)   {
      _dimension = new_var;
  }

  /**
   * Get the value of _dimension
   * @return the value of _dimension
   */
  unsigned int get_dimension () const  {
    return _dimension;
  }

  /**
   * Set the value of _pos
   * @param new_var the new value of _pos
   */
  void set_pos (ift::PoseStamped new_var)   {
      _pos = new_var;
  }

  /**
   * Get the value of _pos
   * @return the value of _pos
   */
  ift::PoseStamped get_pos () const  {
    return _pos;
  }

  /**
   * Set the value of _spd
   * @param new_var the new value of _spd
   */
  void set_spd (ift::VelStamped new_var)   {
      _spd = new_var;
  }

  /**
   * Get the value of _spd
   * @return the value of _spd
   */
  ift::VelStamped get_spd ()  const {
    return _spd;
  }

  /**
   * Set the value of _ctrl
   * @param new_var the new value of _ctrl
   */
  void set_ctrl (ift::Control new_var)   {
      _ctrl = new_var;
  }

  /**
   * Get the value of _ctrl
   * @return the value of _ctrl
   */
  ift::Control get_ctrl () const  {
    return _ctrl;
  }



private:


  void initAttributes () ;

};
}; // end of package namespace

#endif // KINEMATICMODEL_H

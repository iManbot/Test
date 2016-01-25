
#ifndef ARMKINEMATICMODEL_H
#define ARMKINEMATICMODEL_H

#include <string>
#include "KinematicModel.h"
namespace ift {


/**
  * class ArmKinematicModel
  * 
  */

class ArmKinematicModel : public KinematicModel
{
public:

  // Constructors/Destructors
  //  


  /**
   * Empty Constructor
   */
  ArmKinematicModel ();

  /**
   * Empty Destructor
   */
  virtual ~ArmKinematicModel ();


public:


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

#endif // ARMKINEMATICMODEL_H

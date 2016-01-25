#include "ArmKinematicModel.h"

// Constructors/Destructors
//
namespace ift
{
ArmKinematicModel::ArmKinematicModel()
{
    // initialize position and velocity
    _dimension = 1;
    _pos.header.stamp = ros::Time::now();
    _spd.header.stamp = ros::Time::now();
    _pos.pose.position.x = 0;
    _spd.pose.position.y = 0;
}

ArmKinematicModel::~ArmKinematicModel() { }

//
// Methods
//

/**
 * @return ift::Control
 * @param  vel
 * @param  accel
 */
ift::Control ArmKinematicModel::inv_kinematicODE(ift::VelStamped vel) const
{
    ift::Control result;
    result.x = vel.pose.position.x;
    result.y = vel.pose.position.y;
    return result;
}

/**
 * @param  ctrl
 * @param  pos
 * @param  accel
 */
void ArmKinematicModel::kinematicODE(ift::Control ctrl, geometry_msgs::Vector3 pos, ift::AccelStamped accel)
{
    //not required here
}

};// end of package ift


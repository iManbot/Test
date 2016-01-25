#include "PController.h"
#include "sosdata.h"
#include <cmath>
#include <string>
#include <boost/concept_check.hpp>
#include <tf/transform_datatypes.h>

// Constructors/Destructors
//

namespace ift
{
PController::PController()
{
}

PController::~PController() { }

//
// Methods
//

ift::Control PController::getControl(const ift::KinematicModel &knmatcMdl, const Goal &goal)
{
    // get position and velocity
    ift::PoseStamped pos = knmatcMdl.get_pos();
    ift::VelStamped spd = knmatcMdl.get_spd();
    
    // difference between goal position and current position
    ift::VelStamped vel_pos;
    vel_pos.pose.position.x = goal.positions[0] - pos.pose.position.x;
    vel_pos.pose.position.y = goal.positions[1] - pos.pose.position.y;
    //heading difference
    double pos_z = tf::getYaw(pos.pose.orientation);
    double dangle =  goal.positions[3] - pos_z;
    vel_pos.pose.position.z = atan2(sin(dangle), cos(dangle));

    // velocity difference
    ift::VelStamped vel_vel;
    vel_vel.pose.position.x = goal.velocities[0] - spd.pose.position.x;
    vel_vel.pose.position.y = goal.velocities[1] - spd.pose.position.y;
    vel_vel.pose.position.z = goal.velocities[3] - spd.pose.position.z;

    // direction to move
    ift::VelStamped vel;
    vel.header = pos.header;
    vel.pose.position.x = vel_vel.pose.position.x * 0.2 + vel_pos.pose.position.x * 0.5 + _target_dir.pose.position.x * 0.001;
    vel.pose.position.y = vel_vel.pose.position.y * 0.2 + vel_pos.pose.position.y * 0.5 + _target_dir.pose.position.y * 0.001;
    vel.pose.position.z = vel_vel.pose.position.z * 0.2 + vel_pos.pose.position.z * 0.5 + _target_dir.pose.position.z * 0.001;
    
    // constrain angular speed
    double angular_speed_max = 0.2;
    if (fabs(vel.pose.position.z) > angular_speed_max)
        vel.pose.position.z = vel.pose.position.z > 0 ? angular_speed_max : -angular_speed_max;
    
    // constrain velocity in x and y direction
    double len = pow(vel.pose.position.x, 2) + pow(vel.pose.position.y, 2);
    //std::cout<<"len:"<<len<<std::endl;
    len = sqrt(len);
    if (len > 0.1) {
        vel.pose.position.x = vel.pose.position.x * 0.1 / len;
        vel.pose.position.y = vel.pose.position.y * 0.1 / len;
        vel.pose.position.z = vel.pose.position.z;
    }
    /*
    std::cout<<"POS:" <<std::endl;
    std::cout<<"    x:    "<<pos.pose.position.x<<std::endl;
    std::cout<<"    y:    "<<pos.pose.position.y<<std::endl;
    std::cout<<"    theta:"<<pos.pose.position.z<<std::endl;

    std::cout<<"Goal:" <<std::endl;
    std::cout<<"    x:    "<<goal.positions[0]<<std::endl;
    std::cout<<"    y:    "<<goal.positions[1]<<std::endl;
    std::cout<<"    theta:"<<goal.positions[2]<<std::endl;

    std::cout<<"DIR:" <<std::endl;
    std::cout<<"    x:    "<<vel.pose.position.x<<std::endl;
    std::cout<<"    y:    "<<vel.pose.position.y<<std::endl;
    std::cout<<"    theta:"<<vel.pose.position.z<<std::endl;
    */
    
    // update control
    _target_dir = vel;
    return knmatcMdl.inv_kinematicODE(vel); // get control message
}

}; //end of package ift



#include "DummyController.h"
#include "sosdata.h"
#include <cmath>
#include <string>
#include <boost/concept_check.hpp>
#include <tf/transform_datatypes.h>

// Constructors/Destructors
//  

namespace ift{
DummyController::DummyController () {
}

DummyController::~DummyController () { }

//  
// Methods
//  

 ift::Control DummyController::getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal )
{
  ift::VelStamped vel;
  vel.pose.position.x = goal.positions[2];
  vel.pose.position.y = goal.velocities[2];
  return knmatcMdl.inv_kinematicODE(vel);
}


}; //end of package ift



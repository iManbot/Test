#ifndef SOSDATA_H
#define SOSDATA_H


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Path.h>
namespace ift{
  
  typedef trajectory_msgs::JointTrajectory Trajectory;
  typedef trajectory_msgs::JointTrajectoryPoint TrajectoryPoint, Goal;
  typedef geometry_msgs::PoseStamped AccelStamped, PoseStamped, VelStamped;
  typedef geometry_msgs::Vector3 Control;
  typedef nav_msgs::Path NavTraj; 
  
}; // end of package namespace

#endif // SOSDATA_H
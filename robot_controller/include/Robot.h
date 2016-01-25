
#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <unordered_map>
#include "Transmission.h"
#include <ros/time.h>
#include <ros/ros.h>
#include <queue>      
#include "EKF.h"
using namespace std;

namespace ift
{


/**
  * class Robot
  *
  */

class Robot
{
public:

    // Constructors/Destructors
    //


    /**
     * Empty Constructor
     */
    Robot ();


    /**
     * Constructor
     * @param  robot_name
     * @param L 
     */
    Robot ( string robot_name, double L = 0.1 );

    /**
     * Empty Destructor
     */
    virtual ~Robot ();
    
        
    // Mehtods
    //

    /**
     * @param  trajFile
     */
    void loadTrajectory ( string trajFile );


    /**
     */
    void updateGoalNow ();


    /**
     */
    void control ();
    
   /**
     */
    void pubTraj ();

   /**
     */
    void pubGoal ();


private:
    
    // Mehtods
    //
  
    /**
     * @param  msg
     */
    void mapCallback ( const nav_msgs::OccupancyGrid::ConstPtr& msg );
    
    /**
     * @param  msg
     */
    void poseCallback ( const geometry_msgs::PoseStamped::ConstPtr& msg );
    
    
private:

    // Private attributes
    //

    unordered_map<string, ift::TransmissionPtr> _trsmMap; // transmission map (component map)
    ift::Trajectory _traj;
    ift::TrajectoryPoint _goalNow;
    ift::PoseStamped _pos;
    ift::PoseStamped _vel;
    string _robotName;    
    ift::NavTraj _navtraj;
    std::queue<geometry_msgs::PoseStamped> _msg_queue; // message / measurement queue
    ros::Time _start_time;
    unsigned int _base_ctrl_rate = 0;
         
    //EKF
    EKF _filter;
    
    // ros message relative
    ros::NodeHandle _n;
    ros::Publisher _cmd_base_pub;
    ros::Publisher _cmd_arm_pub;
    ros::Publisher _traj_pub;
    ros::Publisher _goal_pub;
    ros::Publisher _pose_pub;
    ros::Subscriber _slam_map_sub;
    ros::Subscriber _slam_pose_sub;
  
    //param
    double _dt = 0.04; // (s)
    double _eta = 0.5;//max radius (mm)
    double _dt_m = 60; //(s)
    double _z_xy_ratio = 1000; 

public:


    // Private attribute accessor methods
    //


    /**
     * Set the value of _trsmMap
     * @param new_var the new value of _trsmMap
     */
    void set_trsmMap ( std::unordered_map<string, ift::TransmissionPtr> new_var )   {
        _trsmMap = new_var;
    }

    /**
     * Get the value of _trsmMap
     * @return the value of _trsmMap
     */
    std::unordered_map<string, ift::TransmissionPtr> get_trsmMap ()   {
        return _trsmMap;
    }

    /**
     * Set the value of _traj
     * @param new_var the new value of _traj
     */
    void set_traj ( ift::Trajectory new_var )   {
        _traj = new_var;
    }

    /**
     * Get the value of _traj
     * @return the value of _traj
     */
    ift::Trajectory get_traj ()   {
        return _traj;
    }

    /**
     * Set the value of _goalNow
     * @param new_var the new value of _goalNow
     */
    void set_goalNow ( ift::TrajectoryPoint new_var )   {
        _goalNow = new_var;
    }

    /**
     * Get the value of _goalNow
     * @return the value of _goalNow
     */
    ift::TrajectoryPoint get_goalNow ()   {
        return _goalNow;
    }

    /**
     * Set the value of _robotName
     * @param new_var the new value of _robotName
     */
    void set_robotName ( string new_var )   {
        _robotName = new_var;
    }

    /**
     * Get the value of _robotName
     * @return the value of _robotName
     */
    string get_robotName ()   {
        return _robotName;
    }
private:


    void initAttributes () ;

};
}; // end of package namespace

#endif // ROBOT_H

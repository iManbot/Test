#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "sosdata.h"
#include "Robot.h"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>
#include <ros/duration.h>
#include <iostream>
#include <cmath>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <cmath>
#include <boost/asio.hpp>
# define PI 3.14159

namespace ift
{
/**
  * class SimpleSerial
  * 
  */
class SimpleSerial
{
public:
    /**
     * Constructor.
     * \param port device name,
     * \param baud_rate communication speed
     * \throws boost::system::system_error if cannot open the serial device
     */
    SimpleSerial(std::string port, unsigned int baud_rate)
        : io(), serial(io, port) {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s) {
        boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine() {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for (;;) {
            asio::read(serial, asio::buffer(&c, 1));
            switch (c) {
            case '\r':
                break;
            case '\n':
                return result;
            default:
                result += c;
            }
        }
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

/**
  * class ArmDriver
  * 
  */
class ArmDriver
{
public:
    
    // Constructor
    ArmDriver(string rn, SimpleSerial *s): _robotName(rn), _s(s) {
        _arm_ctrl_sub = _n.subscribe(_robotName + "/arm", 1, &ArmDriver::armCtrlCallback, this);
        _arm_ctrl_sub_KB = _n.subscribe(_robotName + "/armKB", 1, &ArmDriver::armCtrlCallbackKB, this);
        _arm_ctrl_sub_ini = _n.subscribe(_robotName + "/arm_initialize", 1, &ArmDriver::armCtrlCallback_ini, this);
	_arm_laser_sub = _n.subscribe(_robotName + "/laser",1,&ArmDriver::armLaserCallback,this);
        _KB_time = ros::Time::now();
    }
    
private:
    // call back functions
    void armCtrlCallback(const ift::Control::ConstPtr &msg) {
        ros::Duration escape_time = ros::Time::now() - _KB_time;
        double secs = escape_time.toSec();
	
	// if controlled by other call back function
        if (secs < 1)
            return;

        std::ostringstream s;
        std::vector<float> v(2);
        v[0] = msg->x; // position
        v[1] = msg->y; // speed

        s.precision(4);
        s << "p" << (v[0] * 1000) << "P";
        s << "s" << (v[1] * 1000) << "S";
        _s->writeString(s.str());
    }

    void armCtrlCallbackKB(const ift::Control::ConstPtr &msg) {

        std::vector<float> v(2);
        v[0] = msg->x; // position
        v[1] = msg->y; // speed

        std::ostringstream s;
        s.precision(4);
        s << "u" << (v[0] * 1000) << "U";
std::cout<< s.str()<<"   Arm"<<std::endl;
        _s->writeString(s.str());
        _KB_time = ros::Time::now(); // update callback time
    }

    void armCtrlCallback_ini(const ift::Control::ConstPtr &msg) {
        std::vector<float> v(1);
        v[0] = msg->x; // position
        std::ostringstream s;
        s.precision(4);
        s << "i" << (v[0] * 1000) << "I";
        std::cout<<s.str()<<"    Arm"<<std::endl;
        _s->writeString(s.str());
	_KB_time = ros::Time::now(); // update callback time
    }
   
    void armLaserCallback(const ift::Control::ConstPtr &msg) {
	std::vector<float> v(1);
	v[0] = msg->x; //laser index
	std::ostringstream s;
	s.precision(4);
	s << " l" << v[0] << "L";
	std::cout << s.str()<< "    Arm laser" << std::endl;
	_s->writeString(s.str());
    } 

private:
    ros::NodeHandle _n;
    ros::Subscriber _arm_ctrl_sub;
    ros::Subscriber _arm_ctrl_sub_KB;
    ros::Subscriber _arm_ctrl_sub_ini;
    ros::Subscriber _arm_laser_sub;
    string _robotName;
    SimpleSerial *_s;
    ros::Time _KB_time; //time keyboard was hit

};

}; //end of package ift

int main(int argc, char **argv)
{
    // initialize ROS
    ros::init(argc, argv, "Satellite_Orbit_System_Arm_Driver");
    ROS_INFO("IFT SOS Arm Driver ");
    ros::NodeHandle pn("~");

    // get paramters from ROS
    std::string robot_name;
    std::string port_name;
    pn.param("robot_name", robot_name, std::string("robot1"));
    pn.param("port_name",port_name,std::string("/dev/ttyUSB1"));
    
    // create SimpleSerial and ArmDriver
    ift::SimpleSerial s(port_name, 115200); // make sure arm is connected to ttyUSB1
    ift:: ArmDriver armdriver(robot_name, &s);
    
    ros::Rate r(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    return 0;
}


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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
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
     * \param port device name
     * \param baud_rate communication speed
     * \throws boost::system::system_error
     * serial device
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
  * class BaseDriverUSB
  *
  */
class BaseDriverUSB
{
public:
    // Constructor
    BaseDriverUSB(string rn, SimpleSerial *s, float L, std::string ini_pos): _robotName(rn), _s(s) , _L(L) {
        _base_ctrl_sub = _n.subscribe(_robotName + "/base", 10, &BaseDriverUSB::baseCtrlCallback, this);
        _base_ctrl_sub_KB = _n.subscribe(_robotName + "/baseKB", 10, &BaseDriverUSB::baseCtrlCallbackKB, this);
        _base_ctrl_sub_test = _n.subscribe(_robotName + "/base_test", 10, &BaseDriverUSB::baseCtrlCallback_test, this);
        //_base_odom_pub = _n.advertise<nav_msgs::Odometry>(_robotName + "/odom", 10);
        _KB_time = ros::Time::now();
        _current_time = _KB_time;
        _last_time = _KB_time;

        // update initial position
        std::string delimiter = " ";
        vector <string> fields;
        boost::split(fields, ini_pos, boost::is_any_of(delimiter), boost::token_compress_on);
        try {
            _x = std::stod(fields[0]);
            _y = std::stod(fields[1]);
            _th = std::stod(fields[2]);
        } catch (...) {
            std::cout << "warning: " + _robotName + " check position initialization ";
        }
    }

    //update and publish odom
    void publishOdom() {
        _current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
        double dt = (_current_time - _last_time).toSec();
        double delta_x = (_vx * cos(_th) - _vy * sin(_th)) * dt;
        double delta_y = (_vx * sin(_th) + _vy * cos(_th)) * dt;
        double delta_th = _vth * dt;

        _x += delta_x;
        _y += delta_y;
        _th += delta_th;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);

        //publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = _current_time;
        odom_trans.header.frame_id = _robotName + "/odom";
        odom_trans.child_frame_id = _robotName + "/base_link";

        odom_trans.transform.translation.x = _x;
        odom_trans.transform.translation.y = _y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
       // _odom_broadcaster.sendTransform(odom_trans);
        _last_time = _current_time;
    }
    
    // publish odom
    void publishOdom(std::vector<float> v) {

        namespace ublas = boost::numeric::ublas;
        // obtain robot velocity in global frame
        double theta = _th;//PI / 6.0;
        std::vector<double> vec {sin(theta), sin(PI / 3.0 - theta), -sin(theta + PI / 3.0),
                                 -cos(theta), cos(PI / 3.0 - theta), cos(theta + PI / 3.0),
                                 _L, _L, _L
                                };

        ublas::unbounded_array<double> uvec(_dimension * _dimension);
        for (unsigned int i = 0; i < _dimension * _dimension; ++i) {
            uvec[i] = vec[i];
        }
        ublas::matrix<double> M(_dimension, _dimension, uvec);

        // calculate u = M^(-1)*v
        ublas::matrix<double> invM(_dimension, _dimension);

        typedef ublas::permutation_matrix<std::size_t> pmatrix;
        ublas::matrix<double> A(ublas::trans(M));
        pmatrix pm(A.size1());
        ublas::lu_factorize(A, pm);
        invM.assign(ublas::identity_matrix<double> (A.size1()));
        ublas::lu_substitute(A, pm, invM);
        ublas::vector<double> vv(3);
        for (unsigned int i = 0; i < 3; ++i)
            vv[i] = v [i];

        ublas::vector<double> u = -ublas::prod(invM, vv);

        _current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (_current_time - _last_time).toSec();
        double delta_x = (_vx * cos(_th) - _vy * sin(_th)) * dt;
        double delta_y = (_vx * sin(_th) + _vy * cos(_th)) * dt;
        double delta_th = _vth * dt;

        _x += delta_x;
        _y += delta_y;
        _th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);

        //publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = _current_time;
        odom_trans.header.frame_id = _robotName + "/odom";
        odom_trans.child_frame_id = _robotName + "/base_link";

        odom_trans.transform.translation.x = _x;
        odom_trans.transform.translation.y = _y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        //_odom_broadcaster.sendTransform(odom_trans);
	_last_time = _current_time;

        //update _vx, _vy, _vth
        _vx = u[0];
        _vy = u[1];
        _vth = u[2];

    }
    // callback functions
    void baseCtrlCallback(const ift::Control::ConstPtr &msg) {
        ros::Duration escape_time = ros::Time::now() - _KB_time;
        double secs = escape_time.toSec();
        // if controlled by other call back function
        if (secs < 1)
            return;


        std::vector<float> v(3);
        v[0] = msg->x;
        v[1] = msg->y;
        v[2] = msg->z;

        // contrain control input
        float len = 0;
        for (unsigned int i = 0; i < 3; ++i) {
            len += pow(v[i], 2);
        }
        len = sqrt(len);

        if (len > 1.0)
            for (unsigned int i = 0; i < 3; ++i) {
                v[i] = v[i] * 1.0 / len;
            }

        std::ostringstream s;
        s.precision(4);
        s << "u" << (v[0] * 1000) << "U ";
        s << "v" << (v[1] * 1000) << "V ";
        s << "w" << (v[2] * 1000) << "W";
        const char *buf = s.str().c_str();
        std::cout << s.str() << "       length : " << strlen(buf) << std::endl;
        //write(_s, buf, strlen(buf));
        _s->writeString(s.str());
        //publishOdom(v);
    }

    void baseCtrlCallbackKB(const ift::Control::ConstPtr &msg) {
        std::ostringstream s;

        std::vector<float> v(3);
        v[0] = msg->x;
        v[1] = msg->y;
        v[2] = msg->z;

        for (unsigned int i = 0; i < 3; ++i) {
            if (fabs(v[i]) > 0.1)
                v[i] = v[i] > 0 ? 0.1 : -0.1;
        }
        s.precision(4);

        s << "u" << (v[0] * 1000) << "U ";
        s << "v" << (v[1] * 1000) << "V ";
        s << "w" << (v[2] * 1000) << "W";
        const char *buf = s.str().c_str();
        std::cout << s.str() << "       length : " << strlen(buf) << std::endl;

        _s->writeString(s.str());
        _KB_time = ros::Time::now();
       // publishOdom(v);
    }

    void baseCtrlCallback_test(const ift::Control::ConstPtr &msg) {

        // need to calculate u = M*v;

        // prepare v
        using namespace boost::numeric;
        ublas::vector<double> vv(3);
        int _dimension = 3;
        vv[0] = msg->x;
        vv[1] = msg->y;
        vv[2] = msg->z;

        // prepare M
        double theta = PI / 6.0;
        std::vector<double> vec {sin(theta), sin(PI / 3.0 - theta), -sin(theta + PI / 3.0),
                                 -cos(theta), cos(PI / 3.0 - theta), cos(theta + PI / 3.0),
                                 0.1, 0.1, 0.1
                                };

        ublas::unbounded_array<double> uvec(_dimension * _dimension);
        for (unsigned int i = 0; i < _dimension * _dimension; ++i) {
            uvec[i] = vec[i];
        }
        ublas::matrix<double> M(_dimension, _dimension, uvec);

        // calculate u = M*v
        ublas::vector<double> u = -ublas::prod(ublas::trans(M), vv);

        // publish control
        ift::Control result;
        result.x = u[0];
        result.y = u[1];
        result.z = u[2];

        std::ostringstream s;

        std::vector<float> v(3);
        v[0] = result.x;
        v[1] = result.y;
        v[2] = result.z;

        // constrain control
        float len = 0;
        for (unsigned int i = 0; i < 3; ++i) {
            len += pow(v[i], 2);
        }
        len = sqrt(len);

        if (len > 0.1)
            for (unsigned int i = 0; i < 3; ++i) {
                v[i] = v[i] * 0.1 / len;
            }

        s.precision(4);

        s << "u" << (v[0] * 1000) << "U ";
        s << "v" << (v[1] * 1000) << "V ";
        s << "w" << (v[2] * 1000) << "W";
        const char *buf = s.str().c_str();
        std::cout << s.str() << "       length : " << strlen(buf) << std::endl;
        _s->writeString(s.str());
        _KB_time = ros::Time::now();
       // publishOdom(v);
    }

private:
    ros::NodeHandle _n;
    ros::Subscriber _base_ctrl_sub;
    ros::Subscriber _base_ctrl_sub_KB;
    ros::Subscriber _base_ctrl_sub_test;
    //ros::Publisher _base_odom_pub;
    //tf::TransformBroadcaster _odom_broadcaster;
    string _robotName;
    float _L;
    int _dimension = 3;
    SimpleSerial *_s;
    ros::Time _KB_time; //time keyboard was hit
    ros::Time _current_time, _last_time;

    // double
    double _x = 0.0;
    double _y = 0.0;
    double _th = 0.0;

    double _vx = 0.0;
    double _vy = 0.0;
    double _vth = 0.0;


};

}; //end of package ift

int main(int argc, char **argv)
{
    //ROS
    ros::init(argc, argv, "Satellite_Orbit_System_Base_Driver");
    ROS_INFO("IFT SOS Base Driver ");

    // get parameters
    ros::NodeHandle pn("~");
    std::string robot_name;
    std::string port_name;
    std::string ini_pos;
    double L;
    pn.param("robot_name", robot_name, std::string("robot1"));
    pn.param("port_name", port_name, std::string("/dev/ttyUSB0"));
    pn.param("L", L, 0.1);
    pn.param("ini_pos", ini_pos, std::string("0 0 0"));
    // create SimpleSerial and BaserDriverUSB
    ift::SimpleSerial s(port_name, 38400);
    ift:: BaseDriverUSB basedriver(robot_name, &s, L, ini_pos);

    ros::Rate r(10.0);
    while (ros::ok()) {
        ros::spinOnce();
	basedriver.publishOdom();
        ros::Duration(0.5).sleep();
    }

    return 0;
}


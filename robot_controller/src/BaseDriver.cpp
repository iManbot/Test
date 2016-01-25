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
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <cmath>

# define PI 3.14159

namespace ift{
class BaseDriver
{
public:
  BaseDriver(string rn, int s): _robotName(rn),_s(s)
  {
    _base_ctrl_sub = _n.subscribe ( _robotName + "/base", 10, &BaseDriver::baseCtrlCallback, this );
    _base_ctrl_sub_KB = _n.subscribe ( _robotName + "/baseKB", 10, &BaseDriver::baseCtrlCallbackKB, this );
  _base_ctrl_sub_test = _n.subscribe ( _robotName + "/base_test", 10, &BaseDriver::baseCtrlCallback_test, this );
    _KB_time = ros::Time::now();
  }
  
    void baseCtrlCallback(const ift::Control::ConstPtr& msg)
  {
    ros::Duration escape_time = ros::Time::now() - _KB_time;
    double secs = escape_time.toSec();
    
    if (secs < 1)
      return;
    
    std::ostringstream s;
    
    std::vector<float> v(3);
    v[0] = msg->x;
    v[1] = msg->y;
    v[2] = msg->z;
    
    float len=0;
    for (unsigned int i=0;i<3;++i)
    {
      len+= pow(v[i],2);
    }
    len = sqrt(len);
    
    if (len>1.0)
    for (unsigned int i=0;i<3;++i)
    {
      v[i] = v[i]*1.0/len; 
    }
    
    s.precision(4);
    
    s << "u" << (v[0]*1000) << "U ";
    s << "v" << (v[1]*1000) << "V ";
    s << "w" << (v[2]*1000) << "W";
    const char *buf = s.str().c_str();
    std::cout << s.str()<<"       length : "<< strlen(buf)<<std::endl; 
    write(_s, buf, strlen(buf));
  }
  
  void baseCtrlCallbackKB(const ift::Control::ConstPtr& msg)
  {
    std::ostringstream s;
    
    std::vector<float> v(3);
    v[0] = msg->x;
    v[1] = msg->y;
    v[2] = msg->z;
    
    for (unsigned int i=0;i<3;++i)
    {
      if (fabs(v[i]) > 0.1)
	v[i] = v[i] >0 ? 0.1:-0.1;
    }
    s.precision(4);
    
    s << "u" << (v[0]*1000) << "U ";
    s << "v" << (v[1]*1000) << "V ";
    s << "w" << (v[2]*1000) << "W";
    const char *buf = s.str().c_str();
    std::cout << s.str()<<"       length : "<< strlen(buf)<<std::endl;
    //std::cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<std::endl;
    //char buf[15]="u20U v20V w20W";  
    write(_s, buf, strlen(buf));
    _KB_time = ros::Time::now();
  }

  void baseCtrlCallback_test(const ift::Control::ConstPtr& msg)
  {
      using namespace boost::numeric;
  ublas::vector<double> vv(3);
  int _dimension = 3;
  vv[0] = msg->x;
  vv[1] = msg->y;
  vv[2] = msg->z;
  
  double theta = PI/6.0;
  std::vector<double> vec {sin(theta), sin(PI/3.0 - theta), -sin(theta+PI/3.0),
			    -cos(theta), cos(PI/3.0 -theta), cos(theta+PI/3.0),
			   0.1, 0.1, 0.1};
  
  ublas::unbounded_array<double> uvec (_dimension*_dimension);
  
  for (unsigned int i = 0; i<_dimension*_dimension;++i)
  {
    uvec[i] = vec[i];
  }
  ublas::matrix<double> M(_dimension,_dimension, uvec);
  
  ublas::vector<double> u = -ublas::prod(ublas::trans(M),vv);

  
  ift::Control result;
  result.x = u[0];
  result.y = u[1];
  result.z = u[2];
  
    std::ostringstream s;
    
    std::vector<float> v(3);
    v[0] = result.x;
    v[1] = result.y;
    v[2] = result.z;
    
    float len=0;
    for (unsigned int i=0;i<3;++i)
    {
      len+= pow(v[i],2);
    }
    len = sqrt(len);
    
    if (len>0.1)
    for (unsigned int i=0;i<3;++i)
    {
      v[i] = v[i]*0.1/len; 
    }
    
    s.precision(4);
    
    s << "u" << (v[0]*1000) << "U ";
    s << "v" << (v[1]*1000) << "V ";
    s << "w" << (v[2]*1000) << "W";
    const char *buf = s.str().c_str();
    std::cout << s.str()<<"       length : "<< strlen(buf)<<std::endl;
    //std::cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<std::endl;
    //char buf[15]="u20U v20V w20W";  
    write(_s, buf, strlen(buf));
    _KB_time = ros::Time::now();
  }

private:
  ros::NodeHandle _n;
  ros::Subscriber _base_ctrl_sub;
  ros::Subscriber _base_ctrl_sub_KB;
  ros::Subscriber _base_ctrl_sub_test;
  string _robotName;
  int _s;
  ros::Time _KB_time; //time keyboard was hit
 

};

}; //end of package ift

int main(int argc, char **argv)
{
    /*
    inquiry_info *ii = NULL;
    int max_rsp, num_rsp;
    int dev_id, sock, len, flags;
    int i;
    char addr[19] = { 0 };
    char name[248] = { 0 };
 
    dev_id = hci_get_route(NULL);
    sock = hci_open_dev( dev_id );
    if (dev_id < 0 || sock < 0) {
        perror("opening socket");
        exit(1);
    }
 
    len = 8;
    max_rsp = 255;
    flags = IREQ_CACHE_FLUSH;
    ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));
 
    num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
    if( num_rsp < 0 ) perror("hci_inquiry");
 
    for (i = 0; i < num_rsp; i++) {
        ba2str(&(ii+i)->bdaddr, addr);
        memset(name, 0, sizeof(name));
        if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name),
            name, 0) < 0)
        strcpy(name, "[unknown]");
        printf("%s %s\n", addr, name);
    }
 
    free( ii );
    close( sock );

    */
    struct sockaddr_rc addrR = { 0 };
    int s, status, lenR=0;
    char dest[18] = "98:D3:31:40:3B:B6";//"98:D3:31:40:3B:BE";
    char buf[256];
    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
 
    // set the connection parameters (who to connect to)
    addrR.rc_family = AF_BLUETOOTH;
    addrR.rc_channel = (uint8_t) 1;
    str2ba( dest, &addrR.rc_bdaddr );
 
    // connect to server
    status = connect(s, (struct sockaddr *)&addrR, sizeof(addrR));
 
 
    if(status){
        printf(" failed to connect the device!\n");
        return -1;
    }
 

    ros::init ( argc, argv, "Satellite_Orbit_System_Base_Driver" );

    ROS_INFO ( "IFT SOS Base Driver " );

    ros::NodeHandle pn ( "~" );
    
    std::string robot_name;

    pn.param ( "robot_name", robot_name, std::string("robot1") );
    
    ift:: BaseDriver basedriver(robot_name,s);
    ros::Rate r ( 10.0 );


    while ( ros::ok() ) {
	
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    return 0;
}


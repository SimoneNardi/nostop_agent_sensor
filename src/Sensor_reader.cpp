#include "ros/ros.h"
#include "Sensor_reader.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "serial/serial.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

Sensor_reader::Sensor_reader()
{
  ROS_INFO("SENSOR READER : ON");
  // Publish Sensor Information:
  m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("test_imu", 5);
  m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("test_odom", 5);
  try{
    m_serial_port.setPort("/dev/ttyUSB0");
    m_serial_port.setBaudrate(115200);
    m_serial_port.open();
  } catch(std::exception& e){
    std::cerr<<"Error open serial port"<< e.what() << std::endl;
  }
  ROS_INFO_STREAM("Serial port opened");
}

/////////////////////////////////////////////
Sensor_reader::~Sensor_reader()
{}

/////////////////////////////////////////////
void Sensor_reader::run()
{
  while(ros::ok()){
     
     ros::spinOnce();
  }
  
}











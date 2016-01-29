#include "ros/ros.h"
#include "Sensor_reader.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reading_from_serial");
  
  std::string l_robot_name,l_port;
  ros::NodeHandle l_node("~");
  l_node.getParam("robot_name", l_robot_name);
  l_node.getParam("port_name",l_port);
  Sensor_reader agent_sensor_reader(l_robot_name,l_port);
  
  agent_sensor_reader.start();
  
  ros::spin();
  
  return 0;
}
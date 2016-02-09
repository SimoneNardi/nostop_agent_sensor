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
//   l_robot_name = "thief";


  
  Sensor_reader agent_sensor_reader(l_robot_name);
  ros::spin(); 
  return 0;
}
#include "ros/ros.h"
#include "Roomba_sensor_reader.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "raspberry_read_directly");
  std::string l_robot_name,l_port;
  ros::NodeHandle l_node("~");
  l_node.getParam("robot_name", l_robot_name);
  Roomba_sensor_reader roomba_reader(l_robot_name);


    while(ros::ok())
    {
      roomba_reader.imu_reading();
      ros::spinOnce();
    }
  return 0;
}
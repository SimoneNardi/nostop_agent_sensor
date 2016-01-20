#include "ros/ros.h"
#include "Sensor_reader.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reading_from_serial");
  
  Sensor_reader agent_sensor_reader;
  
  agent_sensor_reader.start();
  
  ros::spin();
  
  return 0;
}
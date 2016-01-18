#include "ros/ros.h"
#include "Raspberry_sensor_reader.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "raspberry_read_directly");
  
  Raspberry_sensor_reader raspberry_reader;
  
  raspberry_reader.start();
  
  ros::spin();
  
  return 0;
}
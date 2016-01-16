#include "ros/ros.h"
#include "Sensor_reader.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

struct s_color 
{ 
	unsigned char pippo[4]; 
};

// union u_color { 
// 	// first representation (member of union) 
// 	s_color c_color;
//  
// 	// second representation (member of union) 
// 	unsigned int i_color; 
// };

int main(int argc, char **argv)
{
//   u_color clr;
//   //reading from tile to clr.i_color 
//   clr.i_color = pow(2,32)-1;
//  
//   // printing from clr.uc_color to output stream 
//   cout << "R=" << int(clr.c_color.pippo[0]) << " ";
//   cout << "G=" << int(clr.c_color.pippo[1]) << " ";
//   cout << "B=" << int(clr.c_color.pippo[2]) << " ";
//   cout << "A=" << int(clr.c_color.pippo[3]) << endl;
//   
//   cout << "Unsigned Int=" << int(clr.i_color) << endl;
  
  
  ros::init(argc, argv, "reading_from_serial");
  
  Sensor_reader agent_sensor_reader;
  
  agent_sensor_reader.start();
  
  ros::spin();
  
  return 0;
}
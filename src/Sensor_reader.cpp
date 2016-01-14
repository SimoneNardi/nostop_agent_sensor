#include "ros/ros.h"
#include "Sensor_reader.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "serial/serial.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//global variables
int package_elements = 0;
uint8_t data_array[message_size];
std::vector<uint8_t> data_vector;


Sensor_reader::Sensor_reader():
count(0)
{
  ROS_INFO("SENSOR READER : ON");
  // Publish Sensor Information:
  m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("test_imu", 5);
  m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("test_odom", 5);
  
  try{
     m_serial_port.setPort("/dev/ttyACM1");
//     m_serial_port.setPort("/dev/ttyUSB0");
    m_serial_port.setBaudrate(115200);
//     m_serial_port.
    m_serial_port.open();
  } catch(std::exception& e){
    std::cerr<<"Error open serial port"<< e.what() << std::endl;
  }
  ROS_INFO_STREAM("Serial port opened");
}

/////////////////////////////////////////////
Sensor_reader::~Sensor_reader()
{ m_serial_port.close();}

bool msg_start = false;
/////////////////////////////////////////////
void Sensor_reader::run()
{
  while(ros::ok()){
     arduino_data arduino_values;
     while ( package_elements < message_size)
     { 
       int size_int;
       std::vector<uint8_t> read_byte;
       size_t size;
       size = m_serial_port.read(read_byte,1);
       size_int = size;
       if (size == 1)
       {
		uint8_t single_byte = read_byte.at(0);
		if(single_byte == 90 && msg_start == false)
		{
			msg_start =true;
			data_vector.push_back(single_byte);
			package_elements = package_elements + 1;
		}
		if (msg_start)
		{
			data_vector.push_back(single_byte);
			package_elements = package_elements + 1;
		}
       }
     }
     if (package_elements == message_size)
     {		
		for(int i = 0 ;i<message_size;i++)
		{	
			arduino_msg.buffer_char[i] = data_vector[i];
			ROS_INFO("data-->%d",data_vector[i]);
			ROS_INFO("buffer-->%d",arduino_msg.buffer_char[i]);
			ros::Duration(0.25).sleep();
		}
	arduino_values = arduino_msg.sensor_data;
	ROS_INFO("%f",arduino_values.qw);
	arduino_to_imu(arduino_values);
	m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
	arduino_to_odometry(arduino_values);
	m_reader_odom_pub.publish<nav_msgs::Odometry>(m_odometry);
	msg_start = false;
	package_elements = 0;
	data_vector.clear();
    }    
  }
}
void Sensor_reader::arduino_to_imu(arduino_data& from_arduino)
{
  // IMU values publish
      m_imu.header.frame_id = "nome/odom";
      m_imu.header.stamp = ros::Time::now();
      m_imu.orientation.x = from_arduino.qx;
      m_imu.orientation.y = from_arduino.qy;
      m_imu.orientation.z = from_arduino.qz;
      m_imu.orientation.w = from_arduino.qw;
      m_imu.linear_acceleration.x = from_arduino.ax/8192;
      m_imu.linear_acceleration.y = from_arduino.ay/8192;
      m_imu.linear_acceleration.z = from_arduino.az/8192;
      m_imu.angular_velocity.x = from_arduino.wx;
      m_imu.angular_velocity.y = from_arduino.wy;
      m_imu.angular_velocity.z = from_arduino.wz;
}


void Sensor_reader::arduino_to_odometry(arduino_data& from_arduino)
{
      //ODOMETRY values publish
      std::vector<float> xyz;
      m_odometry.child_frame_id = "nome/base_link";
      m_odometry.header.frame_id = "nome/odom";
      m_odometry.header.stamp = ros::Time::now();
      xyz = encoder_to_odometry(from_arduino.lw,from_arduino.rw);
}


std::vector<float> Sensor_reader::encoder_to_odometry(int& left_wheel, int& right_wheel)
{
      //TODO     
//   ROS_INFO("%d",left_wheel);
  std::vector<float> l_xyz;
  return l_xyz;
}








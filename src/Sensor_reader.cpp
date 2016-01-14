#include "ros/ros.h"
#include "Sensor_reader.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "serial/serial.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

Sensor_reader::Sensor_reader():
count(0)
{
  ROS_INFO("SENSOR READER : ON");
  // Publish Sensor Information:
  m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("test_imu", 5);
  m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("test_odom", 5);
  try{
    m_serial_port.setPort("/dev/ttyACM0");
//     m_serial_port.setPort("/dev/ttyUSB0");
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
     arduino_data arduino_values;
     std::vector<uint8_t> data_vector;
     size_t output_size;
     output_size = m_serial_port.read(data_vector,message_size);
     int size_out = output_size;
     if(size_out == message_size){
		count= count+1;
		  for(int i = 0 ;i<message_size;i++)
		  {
			arduino_msg.buffer_char[i] = data_vector[i]; 
		  }
		arduino_values = arduino_msg.sensor_data;
		
      arduino_to_imu(arduino_values);
      m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
      arduino_to_odometry(arduino_values);
      m_reader_odom_pub.publish<nav_msgs::Odometry>(m_odometry);
//       ROS_INFO("NORMA-->%f",sqrt(pow(arduino_values.qx,2)+pow(arduino_values.qy,2)+pow(arduino_values.qz,2)+pow(arduino_values.qw,2)));     
      }
      ROS_INFO("message_ok-->%d",count);
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








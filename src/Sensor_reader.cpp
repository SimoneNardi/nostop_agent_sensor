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
std::vector<uint8_t> data_vector;
bool msg_start = false;



//CRC-8 - algoritmo basato sulle formule di CRC-8 di Dallas/Maxim
//codice pubblicato sotto licenza GNU GPL 3.0
uint8_t CRC8(const uint8_t *data, int8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t extract = *data++;
    for (int tempI = 8; tempI; tempI--) {
      uint8_t sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}


Sensor_reader::Sensor_reader():
count(0)
{
  ROS_INFO("SENSOR READER : ON");
  // Publish Sensor Information:
  m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("test_imu", 5);
  m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("test_odom", 5);
  
  try{
     m_serial_port.setPort("/dev/ttyACM0");// arduino uno
//     m_serial_port.setPort("/dev/ttyUSB0");// arduino duemilanove
    m_serial_port.setBaudrate(115200);
//     m_serial_port.
    m_serial_port.open();
  } catch(std::exception& e){
    std::cerr<<"Error open serial port"<< e.what() << std::endl;
  }data_vector.clear();
  ROS_INFO_STREAM("Serial port opened");
  
}



/////////////////////////////////////////////
Sensor_reader::~Sensor_reader()
{ m_serial_port.close();}


/////////////////////////////////////////////
void Sensor_reader::run()
{
  while(ros::ok()){
     arduino_msg_union arduino_msg;
     arduino_data arduino_values;
     while ( package_elements < message_size)
     { 
       uint8_t *single_read_byte;
       size_t size;
       int size_int;
       size = m_serial_port.read(single_read_byte,1);
       size_int = size;
       if (size == 1)
       {
		if(*single_read_byte == 90 && msg_start == false )
		{
			msg_start = true;
		}
		if (msg_start)
		{
			data_vector.push_back(*single_read_byte);
			package_elements = package_elements + 1;
		}
       }
     }
     if (package_elements == message_size)
     {		
		for(int i = 0 ;i<package_elements-1;i++)
		{	
			arduino_msg.byte_buffer[i] = data_vector[i];
		}
	uint8_t sended_msg_checksum_value, received_msg_checksum_value;
	received_msg_checksum_value = data_vector[package_elements-1];
	sended_msg_checksum_value = CRC8(arduino_msg.byte_buffer,package_elements-2);
	if(sended_msg_checksum_value == received_msg_checksum_value)
	{
		arduino_values = arduino_msg.sensor_data;
		arduino_to_imu(arduino_values);
		m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
		arduino_to_odometry(arduino_values);
		m_reader_odom_pub.publish<nav_msgs::Odometry>(m_odometry);
		msg_start = false;
		package_elements = 0;
		data_vector.clear();
		ROS_INFO("qx --> %f",arduino_values.qx);
		ROS_INFO("qx (arduino)--> %f",arduino_msg.sensor_data.qx);
// 		ROS_INFO("received --> %d",received_msg_checksum_value);
// 		ROS_INFO("check (0 is ok) --> %d",received_msg_checksum_value-sended_msg_checksum_value);
	}else{
		msg_start = false;
		package_elements = 0;
		data_vector.clear();
		ROS_ERROR("Invalid package");
	}
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
//       ROS_INFO("%d",from_arduino.lw);
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




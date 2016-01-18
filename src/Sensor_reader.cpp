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

void test()
{
	float f = 0.6;
	unsigned char *pc;
	pc = (unsigned char*)&f;
	*(unsigned int*)&f = (pc[3] << 24) | (pc[2] << 16) | (pc[1] << 8) | (pc[0] << 0);
	ROS_INFO("%d",pc[0]);
	ROS_INFO("%d",pc[1]);
	ROS_INFO("%d",pc[2]);
	ROS_INFO("%d",pc[3]);
	ROS_INFO("--->%f<---",f);
}

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
, m_right_wheel_count(0)
, m_left_wheel_count(0)
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
//     test();
     arduino_msg_union arduino_msg;
     arduino_data arduino_values;
     while ( package_elements < message_size)
     { 
       unsigned char single_read_byte;
       size_t size;
       int size_int;
       size = m_serial_port.read(&single_read_byte,1);
       size_int = size;
       if (size == 1)
       {
		if(single_read_byte == 90 && msg_start == false )
		{
			msg_start = true;
		}
		if (msg_start)
		{
			data_vector.push_back(single_read_byte);
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
// 		arduino_values = arduino_msg.sensor_data;
		arduino_values = buffer_to_struct(arduino_msg.byte_buffer);
		arduino_to_imu(arduino_values);
		m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
		arduino_to_odometry(arduino_values);
		m_reader_odom_pub.publish<nav_msgs::Odometry>(m_odometry);
		msg_start = false;
		package_elements = 0;
		data_vector.clear();
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
      int l_left_wheel,l_right_wheel; 
      std::vector<float> data;
//       ROS_INFO("%d",from_arduino.lw);
      l_left_wheel = from_arduino.lw - m_left_wheel_count;
      l_right_wheel = from_arduino.rw - m_right_wheel_count;
      m_odometry.child_frame_id = "nome/base_link";
      m_odometry.header.frame_id = "nome/odom";
      m_odometry.header.stamp = ros::Time::now();
      data = encoder_to_odometry(l_left_wheel,l_right_wheel);
      float phi = 0;//ROLL
      float theta = 0;//PITCH
      float psi = data.at(2);
      m_odometry.pose.pose.position.x = data.at(0);
      m_odometry.pose.pose.position.y = data.at(1);
      m_odometry.pose.pose.orientation.x = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2); 
      m_odometry.pose.pose.orientation.y = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2); 
      m_odometry.pose.pose.orientation.z = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
      m_odometry.pose.pose.orientation.w = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
      m_odometry.twist.twist.linear.x = data.at(3);
      m_odometry.twist.twist.angular.z = data.at(4);
      
}

std::vector<float> Sensor_reader::encoder_to_odometry(int& left_wheel, int& right_wheel)
{	//TODO
  float x,y,yaw,x_dot,yaw_dot; // what ekf want
  std::vector<float> data;
  data.push_back(0.1);
  data.push_back(0.5);
  data.push_back(0.3);
  data.push_back(0.4);
  data.push_back(0.2);
  //...
  return data;
}



arduino_data Sensor_reader::buffer_to_struct(uint8_t byte_buffer[message_size])
{
	float qx,qy,qz,qw,ax,ay,az,wx,wy,wz;
	int lw,rw;
	arduino_data arduino_values;
	*(unsigned int*)&qx = (byte_buffer[4] << 24) | (byte_buffer[3] << 16) | (byte_buffer[2] << 8) | (byte_buffer[1] << 0);
	*(unsigned int*)&qy = (byte_buffer[8] << 24) | (byte_buffer[7] << 16) | (byte_buffer[6] << 8) | (byte_buffer[5] << 0);
	*(unsigned int*)&qz = (byte_buffer[12] << 24) | (byte_buffer[11] << 16) | (byte_buffer[10] << 8) | (byte_buffer[9] << 0);
	*(unsigned int*)&qw = (byte_buffer[16] << 24) | (byte_buffer[15] << 16) | (byte_buffer[14] << 8) | (byte_buffer[13] << 0);
	*(unsigned int*)&ax = (byte_buffer[20] << 24) | (byte_buffer[19] << 16) | (byte_buffer[18] << 8) | (byte_buffer[17] << 0);
	*(unsigned int*)&ay = (byte_buffer[24] << 24) | (byte_buffer[23] << 16) | (byte_buffer[22] << 8) | (byte_buffer[21] << 0);
	*(unsigned int*)&az = (byte_buffer[28] << 24) | (byte_buffer[27] << 16) | (byte_buffer[26] << 8) | (byte_buffer[25] << 0);
	*(unsigned int*)&wx = (byte_buffer[32] << 24) | (byte_buffer[31] << 16) | (byte_buffer[30] << 8) | (byte_buffer[29] << 0);
	*(unsigned int*)&wy = (byte_buffer[36] << 24) | (byte_buffer[35] << 16) | (byte_buffer[34] << 8) | (byte_buffer[33] << 0);
	*(unsigned int*)&wz = (byte_buffer[40] << 24) | (byte_buffer[39] << 16) | (byte_buffer[38] << 8) | (byte_buffer[37] << 0);
	*(unsigned int*)&lw = (byte_buffer[44] << 24) | (byte_buffer[43] << 16) | (byte_buffer[42] << 8) | (byte_buffer[41] << 0);
	*(unsigned int*)&rw = (byte_buffer[48] << 24) | (byte_buffer[47] << 16) | (byte_buffer[46] << 8) | (byte_buffer[45] << 0);
	arduino_values.ax = ax;
	arduino_values.ay = ay;
	arduino_values.az = az;
	arduino_values.qx = qx;
	arduino_values.qy = qy;
	arduino_values.qz = qz;
	arduino_values.qw = qw;
	arduino_values.lw = lw;
	arduino_values.rw = rw;
	arduino_values.wx = wx;
	arduino_values.wy = wy;
	arduino_values.wz = wz;
//	ROS_INFO("qx --> %f",arduino_values.qx);
//	ROS_INFO("qy --> %f",arduino_values.qy);
//	ROS_INFO("qz --> %f",arduino_values.qz);
//	ROS_INFO("qw --> %f",arduino_values.qw);
//	ROS_INFO("ax --> %f",arduino_values.ax);
//	ROS_INFO("ay --> %f",arduino_values.ay);
//	ROS_INFO("az --> %f",arduino_values.az);
//	ROS_INFO("lw --> %d",arduino_values.lw);
//	ROS_INFO("rw --> %d",arduino_values.rw);
//	ROS_INFO("wx --> %f",arduino_values.wx);
//	ROS_INFO("wy --> %f",arduino_values.wy);
//	ROS_INFO("wz --> %f",arduino_values.wz);
	return arduino_values;
}

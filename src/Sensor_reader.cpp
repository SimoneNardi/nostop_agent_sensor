#include <ros/ros.h>
#include <Sensor_reader.h>
#include "time.h"



using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//Dallas/Maxim based algorithm
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


Sensor_reader::Sensor_reader(std::string& robot_name):
count(0)
, m_right_wheel_count(0)
, m_left_wheel_count(0)
, m_previous_time(ros::Time::now().toSec()) //toSec() need double)
, m_wheel_diameter(6.5)
, m_encoder_risolution(10)
, m_robot_name(robot_name)
, m_address(IMU_ADDRESS)
{
	ROS_INFO("SENSOR READER : ON");
	m_step_length = m_wheel_diameter*M_PI/m_encoder_risolution;
	// Publish Sensor Information:
	m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("/"+m_robot_name+"/odom", 5);
	m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("/"+m_robot_name+"/imu_data", 5);
	m_reader_encoder_data = reader.subscribe<std_msgs::UInt8MultiArray>("/"+m_robot_name+"/encoder_data",5,&Sensor_reader::encoder_data_in,this);
	wiringPiSetup();
	try{
	    m_reg_address = wiringPiI2CSetup(m_address);
	} catch(std::exception &e){
	  std::cerr<<"Error open IMU"<< e.what() << std::endl;
	}
	//disable sleep mode
	wiringPiI2CWriteReg8(m_reg_address,0x6B,00);
}



/////////////////////////////////////////////
Sensor_reader::~Sensor_reader()
{}




void Sensor_reader::encoder_data_in(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{	
  Lock l_lock(m_mutex);
  arduino_msg_union l_union;
  for(size_t i = 0 ;i<message_size;++i)
  {
    l_union.byte_buffer[i] = msg->data.at(i);
  }
  arduino_data data;
  data = l_union.sensor_data;
  uint8_t checksum = CRC8(l_union.byte_buffer,message_size-2);
  if(checksum == l_union.byte_buffer[message_size-1])
  {
    std::vector<float> odom_data;
    odom_data = encoder_to_odometry(data.lw,data.rw);
    odometry_publish(odom_data);
  }else{
    ROS_ERROR("Invalid package");
  }
}



std::vector<float> Sensor_reader::encoder_to_odometry(int& left_wheel,int& right_wheel) //TODO
{
  float x,y,yaw,x_dot, yaw_dot; // what ekf want
  float x_lw,x_rw,x_medio;
  std::vector<float> data;
  x_lw = m_step_length * left_wheel;
  x_rw = m_step_length * right_wheel;
  yaw = (360*(x_rw-x_lw))/(m_wheel_diameter*M_PI);
  yaw = atan2((x_rw-x_lw),10);
  double l_actual_time = ros::Time::now().toSec(); 
  double l_time_diff = l_actual_time-m_previous_time;
  yaw_dot = yaw/l_time_diff;
  x_medio = yaw*M_PI*(m_wheel_diameter/2)/360;
  x = x_medio*sin(yaw);
  y = -x_medio*cos(yaw);
  x_dot = x/l_time_diff;
  m_previous_time = l_actual_time;
  data.push_back(x);
  data.push_back(y);
  data.push_back(yaw);
  data.push_back(x_dot);
  data.push_back(yaw_dot);
  return data;
}




void Sensor_reader::imu_reading_publish()
{       
	Lock l_lock(m_mutex);
	short int ax,ay,az,wx,wy,wz;
	ax=wiringPiI2CReadReg8(m_reg_address,H_BYTE_X_ACC_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_X_ACC_ADDRESS);
        ay=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Y_ACC_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Y_ACC_ADDRESS);
        az=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Z_ACC_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Z_ACC_ADDRESS);
        wx=wiringPiI2CReadReg8(m_reg_address,H_BYTE_X_GYRO_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_X_GYRO_ADDRESS);
        wy=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Y_GYRO_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Y_GYRO_ADDRESS);
        wz=wiringPiI2CReadReg8(m_reg_address,H_BYTE_Z_GYRO_ADDRESS)<<8|wiringPiI2CReadReg8(m_reg_address,L_BYTE_Z_GYRO_ADDRESS);

	
	// COMPOSE IMU MSG
	m_imu.linear_acceleration.x = ax;
	m_imu.linear_acceleration.y = ay;
	m_imu.linear_acceleration.z = az;
	m_imu.angular_velocity.x = wx;
	m_imu.angular_velocity.y = wy;
	m_imu.angular_velocity.z = wz;
	m_imu.header.frame_id = m_robot_name+"/odom";
	// TODO orientation
	m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
}


void Sensor_reader::odometry_publish(std::vector<float>& data)
{
     geometry_msgs::Quaternion l_odom_quaternion = tf::createQuaternionMsgFromYaw(data.at(2));
      
      // publish over /tf
      m_odom_tf.header.stamp = ros::Time::now();
      m_odom_tf.header.frame_id = m_robot_name+"/odom";
      m_odom_tf.child_frame_id = m_robot_name+"/base_link";
      m_odom_tf.transform.translation.x = (float)data.at(0);
      m_odom_tf.transform.translation.y = (float)data.at(1);
      m_odom_tf.transform.translation.z = 0.0;
      m_odom_tf.transform.rotation = l_odom_quaternion;
      m_odom_broadcaster.sendTransform(m_odom_tf);
            
      // ODOMETRY MSG
      m_odometry.header.stamp = ros::Time::now();
      m_odometry.child_frame_id = m_robot_name+"/base_link";
      m_odometry.header.frame_id = m_robot_name+"/odom";
      m_odometry.pose.pose.position.x = (float) data.at(0);
      m_odometry.pose.pose.position.y = (float)data.at(1);
      m_odometry.pose.pose.orientation.x = l_odom_quaternion.x;
      m_odometry.pose.pose.orientation.y = l_odom_quaternion.y;
      m_odometry.pose.pose.orientation.z = l_odom_quaternion.z;
      m_odometry.pose.pose.orientation.w = l_odom_quaternion.w;
      m_odometry.twist.twist.linear.x = (float)data.at(3);
      m_odometry.twist.twist.angular.z = (float)data.at(4);
      m_reader_odom_pub.publish<nav_msgs::Odometry>(m_odometry);
}

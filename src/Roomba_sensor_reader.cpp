#include <ros/ros.h>
#include <Roomba_sensor_reader.h>
#include "time.h"



using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//global variables
int package_elements = 0;
std::vector<uint8_t> data_vector;
bool msg_start = false;

Roomba_sensor_reader::Roomba_sensor_reader(std::string& robot_name):
count(0)
, m_right_wheel_count(0)
, m_left_wheel_count(0)
, m_previous_time(ros::Time::now().toSec()) //toSec() need double)
, m_wheel_diameter(6.5)
, m_encoder_risolution(10)
, m_robot_name(robot_name)
, m_address(0x68)// imu adress --> sudo i2cdetect -y 1
{
  ROS_INFO("SENSOR READER : ON");
	m_step_length = m_wheel_diameter*M_PI/m_encoder_risolution;
	// Publish Sensor Information:
	m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("/"+m_robot_name+"/odom", 5);
	m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("/"+m_robot_name+"/imu_data", 5);
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
Roomba_sensor_reader::~Roomba_sensor_reader()
{}


void Roomba_sensor_reader::odometry_publish() //TODO
{
}




void Roomba_sensor_reader::imu_reading()
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




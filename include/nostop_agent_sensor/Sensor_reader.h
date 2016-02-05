////////////////////////////////////////////////////////////
//	Sensor_reader.h
//	Created on:	18-jan-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef SENSOR_READER_H
#define SENSOR_READER_H
#pragma once

#include "ros/ros.h"
#include "serial/serial.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <Threads.h>

#define message_size 10
//// By http://www.botched.co.uk/pic-tutorials/mpu6050-setup-data-aquisition/
#define IMU_ADDRESS 0x68 // imu address --> sudo i2cdetect -y 1
#include <wiringPi.h>
#include <wiringPiI2C.h>
#define H_BYTE_X_ACC_ADDRESS 0x3B
#define L_BYTE_X_ACC_ADDRESS 0x3C
#define H_BYTE_Y_ACC_ADDRESS 0x3D
#define L_BYTE_Y_ACC_ADDRESS 0x3E
#define H_BYTE_Z_ACC_ADDRESS 0x3F
#define L_BYTE_Z_ACC_ADDRESS 0x40

#define H_BYTE_X_GYRO_ADDRESS 0x43
#define L_BYTE_X_GYRO_ADDRESS 0x44
#define H_BYTE_Y_GYRO_ADDRESS 0x45
#define L_BYTE_Y_GYRO_ADDRESS 0x46
#define H_BYTE_Z_GYRO_ADDRESS 0x47
#define L_BYTE_Z_GYRO_ADDRESS 0x48

namespace Robotics 
{	
	namespace GameTheory
	{
	  
	   struct arduino_data{ 
	      uint8_t start;
	      int lw;
	      int rw;
	      uint8_t checksum;
	      };
	      
	  union arduino_msg_union{
	    uint8_t byte_buffer[message_size];
	    arduino_data sensor_data;
	  };
	 
	    
	  
		class Sensor_reader
		{
		public:
		        Mutex m_mutex;
			// odometry
			tf::TransformBroadcaster m_odom_broadcaster;
			geometry_msgs::TransformStamped m_odom_tf;
			ros::NodeHandle reader;
			ros::Publisher m_reader_odom_pub;
			serial::Serial m_serial_port;
			std::string m_robot_name;
			nav_msgs::Odometry m_odometry;
			double m_previous_time;
			int m_encoder_risolution;
			float m_wheel_diameter,m_step_length;
			int count,m_right_wheel_count,m_left_wheel_count;
			// IMU
			char m_address;
			char m_buf[1];
			int m_reg_address;
			int m_value;
			int m_ret;
			ros::Publisher m_reader_imu_pub;
			sensor_msgs::Imu m_imu;

		public:
			Sensor_reader(std::string& robot_name,std::string& port_name); 
			void arduino_to_odometry(arduino_data& from_arduino);
			arduino_data buffer_to_struct(uint8_t buffer[]);
			std::vector<float> encoder_to_odometry(int& left_wheel,int& right_wheel);
			void imu_reading();
			void reading();
			~Sensor_reader();
		};

	}
}


#endif // SENSOR_READER_H
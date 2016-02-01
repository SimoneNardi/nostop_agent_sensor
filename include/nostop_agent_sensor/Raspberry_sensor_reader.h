////////////////////////////////////////////////////////////
//	Raspberry_sensor_reader.h
//	Created on:	18-jan-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef RASPBERRY_SENSOR_READER_H
#define RASPBERRY_SENSOR_READER_H
#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>


#define H_BYTE_X_ACC_ADDRESS 59
#define L_BYTE_X_ACC_ADDRESS 60
#define H_BYTE_Y_ACC_ADDRESS 61
#define L_BYTE_Y_ACC_ADDRESS 62
#define H_BYTE_Z_ACC_ADDRESS 63
#define L_BYTE_Z_ACC_ADDRESS 64

#define H_BYTE_X_GYRO_ADDRESS 67
#define L_BYTE_X_GYRO_ADDRESS 68
#define H_BYTE_Y_GYRO_ADDRESS 69
#define L_BYTE_Y_GYRO_ADDRESS 70
#define H_BYTE_Z_GYRO_ADDRESS 71
#define L_BYTE_Z_GYRO_ADDRESS 72

namespace Robotics 
{
	namespace GameTheory
	{
	  
		class Raspberry_sensor_reader 
		{
		public:
			ros::NodeHandle reader;
			std::string m_robot_name;
			char m_address;
			char m_buf[1];
			char m_regaddr[2];
			int m_value;
			int m_ret;
			ros::Publisher m_raspberry_reader_imu_pub;
			sensor_msgs::Imu m_imu;
			
		public:
			Raspberry_sensor_reader(std::string& robot_name);
			~Raspberry_sensor_reader();
			void imu_reading();
			double x_acceleration();
			double y_acceleration();
			double z_acceleration();
			double x_gyro_axis();
			double y_gyro_axis();
			double z_gyro_axis();
		};

	}
}


#endif // RASPBERRY_SENSOR_READER_H
////////////////////////////////////////////////////////////
//	Sensor_reader.h
//	Created on:	18-jan-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef SENSOR_READER_H
#define SENSOR_READER_H
#pragma once

#include "ros/ros.h"
#include "ThreadBase.h"
#include "serial/serial.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#define message_size 50

namespace Robotics 
{
	namespace GameTheory
	{
	   struct arduino_data{ 
	      uint8_t start;
	      float qx;
	      float qy;
	      float qz;
	      float qw;
	      float ax;
	      float ay;
	      float az;
	      float wx;
	      float wy;
	      float wz;
	      int lw;
	      int rw;
	      uint8_t checksum;
	      };
	      
	  union arduino_msg_union{
	    uint8_t byte_buffer[message_size];
	    arduino_data sensor_data;
	  };
	  
		class Sensor_reader : public ThreadBase
		{
		public:
			Mutex m_mutex;
			ros::NodeHandle reader;
			ros::Publisher m_reader_imu_pub;
			ros::Publisher m_reader_odom_pub;
			serial::Serial m_serial_port;
			sensor_msgs::Imu m_imu;
			nav_msgs::Odometry m_odometry;
			double m_previous_time;
			int m_encoder_risolution;
			float m_wheel_diameter,m_step_length;
			int count,m_right_wheel_count,m_left_wheel_count;
			
		protected:
			virtual void run();
		public:
			Sensor_reader(); 
			void arduino_to_imu(arduino_data& from_arduino);
			void arduino_to_odometry(arduino_data& from_arduino);
			std::vector<float> encoder_to_odometry(int& left_wheel,int& right_wheel, float& diameter);
			arduino_data buffer_to_struct(uint8_t buffer[]);
			~Sensor_reader();
		};

	}
}


#endif // SENSOR_READER_H
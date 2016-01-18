////////////////////////////////////////////////////////////
//	Raspberry_sensor_reader.h
//	Created on:	18-jan-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef RASPBERRY_SENSOR_READER_H
#define RASPBERRY_SENSOR_READER_H
#pragma once

#include "ros/ros.h"
#include "ThreadBase.h"

namespace Robotics 
{
	namespace GameTheory
	{
	  
		class Raspberry_sensor_reader : public ThreadBase
		{
		public:
			Mutex m_mutex;
			ros::NodeHandle reader;
			char m_address = 0x75; // MPU6050.h ( RA_WHO_AM_I )
			char m_buf[1];
			char m_addr[2];
			int m_value;
			int m_ret;
			
		protected:
			virtual void run();
		public:
			Raspberry_sensor_reader();
			~Raspberry_sensor_reader();
		};

	}
}


#endif // RASPBERRY_SENSOR_READER_H
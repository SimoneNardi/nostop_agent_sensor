////////////////////////////////////////////////////////////
//	Sensor_reader.h
//	Created on:	12-jan-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef SENSOR_READER_H
#define SENSOR_READER_H
#pragma once

#include "ros/ros.h"
#include "ThreadBase.h"
#include "serial/serial.h"

namespace Robotics 
{
	namespace GameTheory
	{
	  union msg{
	    struct { 
	      float qx,qy,qz,qw;
	      float ax,ay,az;
	      float rw,lw;
	      }sensor_data;
	    char dim[36];
	  };
	  
		class Sensor_reader : public ThreadBase
		{
		public:
			ros::NodeHandle reader;
			ros::Publisher m_reader_imu_pub;
			ros::Publisher m_reader_odom_pub;
			serial::Serial m_serial_port;
			
		protected:
			virtual void run();
		public:
			Sensor_reader();  
			~Sensor_reader();
		};

	}
}


#endif // SENSOR_READER_H
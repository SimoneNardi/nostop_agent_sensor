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
	  
		class Raspberry_sensor_reader : public ThreadBase
		{
		public:
			Mutex m_mutex;
			ros::NodeHandle reader;
			char m_address;
			char m_buf[1];
			char m_regaddr[2];
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
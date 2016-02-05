////////////////////////////////////////////////////////////
//	My_serial_manager.h
//	Created on:	5-feb-16
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef MY_SERIAL_MANAGER_H
#define MY_SERIAL_MANAGER_H
#pragma once

#include "ros/ros.h"
#include "serial/serial.h"
#include <Threads.h>

#define message_size 10 

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
	 
	    
	  
		class My_serial_manager
		{
		public:
		        Mutex m_mutex;
			std::string m_port_name;
			serial::Serial m_serial;
	

		public:
		  My_serial_manager();
		  ~My_serial_manager();
		  void initialize();
		  std::vector<int> Read();
		  std::vector< int > buffer_to_struct(uint8_t byte_buffer[message_size]);
		
		};

	}
}


#endif // MY_SERIAL_MANAGER_H
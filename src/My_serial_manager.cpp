#include <ros/ros.h>
#include "My_serial_manager.h"


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


int package_elements = 0;
bool msg_start = false;
std::vector<uint8_t> data_vector;


My_serial_manager::My_serial_manager()
{}

My_serial_manager::~My_serial_manager()
{
  m_serial.close();
}


void My_serial_manager::initialize()
{
  try{
		m_serial.setPort(m_port_name);// arduino uno
		m_serial.setBaudrate(115200);
		m_serial.open();
	} catch(std::exception& e){
		std::cerr<<"Error open serial port"<< e.what() << std::endl;
				  }
	data_vector.clear();
}




vector< int > My_serial_manager::Read()
{
	Lock l_lock(m_mutex);
	arduino_msg_union arduino_msg;
	std::vector< int > encoder_data;
	while ( package_elements < message_size)
	{ 
		unsigned char single_read_byte;
		size_t size;
		int size_int;
		size = m_serial.read(&single_read_byte,1);
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
		encoder_data = buffer_to_struct(arduino_msg.byte_buffer);
		ROS_INFO("SI CI SONO");
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
	return encoder_data;
}



std::vector< int > My_serial_manager::buffer_to_struct(uint8_t byte_buffer[message_size])
{
	int lw,rw;
	std::vector<int> encoder_data;
	*(unsigned int*)&lw = (byte_buffer[4] << 24) | (byte_buffer[3] << 16) | (byte_buffer[2] << 8) | (byte_buffer[1] <<0);
 	*(unsigned int*)&rw = (byte_buffer[8] << 24) | (byte_buffer[7] << 16) | (byte_buffer[6] << 8) | (byte_buffer[5] <<0);

 	encoder_data.push_back(lw);
 	encoder_data.push_back(rw);

	ROS_INFO("lw --> %d",encoder_data.at(0));
	ROS_INFO("rw --> %d",encoder_data.at(1));
	ROS_INFO("BYTE BUFFER 0 --> %d",byte_buffer[0]);
 	ROS_INFO("BYTE BUFFER 1 --> %d",byte_buffer[1]);
 	ROS_INFO("BYTE BUFFER 2 --> %d",byte_buffer[2]);
 	ROS_INFO("BYTE BUFFER 3 --> %d",byte_buffer[3]);
	ROS_INFO("BYTE BUFFER 4 --> %d",byte_buffer[4]);
	ROS_INFO("BYTE BUFFER 5 --> %d",byte_buffer[5]);
	ROS_INFO("BYTE BUFFER 6 --> %d",byte_buffer[6]);
	ROS_INFO("BYTE BUFFER 7 --> %d",byte_buffer[7]);
 	ROS_INFO("BYTE BUFFER 8 --> %d",byte_buffer[8]);
	
	return encoder_data;
}

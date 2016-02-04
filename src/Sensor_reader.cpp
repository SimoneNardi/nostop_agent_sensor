#include <ros/ros.h>
#include <Sensor_reader.h>
#include "time.h"
#include <bcm2835/src/bcm2835.h>

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

//global variables
int package_elements = 0;
std::vector<uint8_t> data_vector;
bool msg_start = false;

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

Sensor_reader::Sensor_reader(std::string& robot_name,std::string& port_name):
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
	bcm2835_init();
	bcm2835_i2c_setSlaveAddress(m_address);
	m_step_length = m_wheel_diameter*M_PI/m_encoder_risolution;
	// Publish Sensor Information:
	m_reader_odom_pub = reader.advertise<nav_msgs::Odometry>("/"+m_robot_name+"/odom", 5);
	m_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("/"+m_robot_name+"/imu_data", 5);
  
  try{
     m_serial_port.setPort(port_name);// arduino uno
//     m_serial_port.setPort("/dev/ttyUSB0");// arduino duemilanove
    m_serial_port.setBaudrate(115200);
    m_serial_port.open();
  } catch(std::exception& e){
    std::cerr<<"Error open serial port"<< e.what() << std::endl;
  }data_vector.clear();
  ROS_INFO_STREAM("Serial port opened");
  
}



/////////////////////////////////////////////
Sensor_reader::~Sensor_reader()
{
  m_serial_port.close();
  bcm2835_i2c_end();
}


void Sensor_reader::arduino_to_odometry(arduino_data& from_arduino)
{
      //ODOMETRY values publish
      // SR in ENU representation
      int l_left_wheel,l_right_wheel; 
      std::vector<float> data;
      l_left_wheel = from_arduino.lw - m_left_wheel_count;
      l_right_wheel = from_arduino.rw - m_right_wheel_count;
      m_left_wheel_count = l_left_wheel;
      m_right_wheel_count = l_right_wheel;
      ROS_INFO("%d,%d",l_left_wheel,l_right_wheel);
      data = encoder_to_odometry(l_left_wheel,l_right_wheel);
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
    
}


arduino_data Sensor_reader::buffer_to_struct(uint8_t byte_buffer[message_size])
{
	int lw,rw,lw2,rw2;
	arduino_data arduino_values;
//	arduino_msg_union tiop;
	*(unsigned int*)&lw = (byte_buffer[4] << 24) | (byte_buffer[3] << 16) | (byte_buffer[2] << 8) | (byte_buffer[1] <<0);
 	*(unsigned int*)&rw = (byte_buffer[8] << 24) | (byte_buffer[7] << 16) | (byte_buffer[6] << 8) | (byte_buffer[5] <<0);

 	arduino_values.lw = lw;
 	arduino_values.rw = rw;
// 	for(int i = 0 ;i<message_size;i++){	  
// 	tiop.byte_buffer[i]= byte_buffer[i];
// 	}
//	arduino_values= tiop.sensor_data;
	ROS_INFO("lw --> %d",arduino_values.lw);
	ROS_INFO("rw --> %d",arduino_values.rw);
// 	ROS_INFO("byte buffer 0 --> %d",byte_buffer[0]);
// 	ROS_INFO("byte buffer 1 --> %d",byte_buffer[1]);
// 	ROS_INFO("byte buffer 2 --> %d",byte_buffer[2]);
// 	ROS_INFO("byte buffer 3 --> %d",byte_buffer[3]);
// 	ROS_INFO("byte buffer 4 --> %d",byte_buffer[4]);
// 	ROS_INFO("byte buffer 5 --> %d",byte_buffer[5]);
// 	ROS_INFO("byte buffer 6 --> %d",byte_buffer[6]);
// 	ROS_INFO("byte buffer 7 --> %d",byte_buffer[7]);
// 	ROS_INFO("byte buffer 8 --> %d",byte_buffer[8]);
	
	return arduino_values;
}


std::vector<float> Sensor_reader::encoder_to_odometry(int& left_wheel, int& right_wheel) //TODO
{
  float x,y,yaw,x_dot, yaw_dot; // what ekf want
  float x_lw,x_rw,x_medio;
  std::vector<float> data;
  x_lw = m_step_length * left_wheel;
  x_rw = m_step_length * right_wheel;
  yaw = (360*(x_rw-x_lw))/(m_wheel_diameter*M_PI);
//   yaw = atan2((x_rw-x_lw),10);
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




void Sensor_reader::imu_reading()
{
	double x_acc,y_acc,z_acc,wx_acc,wy_acc,wz_acc;
	bcm2835_i2c_begin();
	m_regaddr[0] = 107; // register address
	m_regaddr[1] = 0; // value
	//disable sleep mode
	bcm2835_i2c_write(m_regaddr, 2);//where and how many bytes
	// READ ACCELERATION VALUES
	x_acc = x_acceleration();
	y_acc = y_acceleration();
	z_acc = z_acceleration();
	// ACC TO m/s^2
	x_acc = x_acc / 16384;
	y_acc = y_acc / 16384;
	z_acc = z_acc / 16384;
	
	
	// READ GYROSCOPE VALUES
	wx_acc = x_gyro_axis();
	wy_acc = y_gyro_axis();
	wz_acc = z_gyro_axis();
	
	// COMPOSE IMU MSG
	m_imu.linear_acceleration.x = x_acc;
	m_imu.linear_acceleration.y = y_acc;
	m_imu.linear_acceleration.z = z_acc;
	m_imu.angular_velocity.x = wx_acc;
	m_imu.angular_velocity.y = wy_acc;
	m_imu.angular_velocity.z = wz_acc;
	m_imu.header.frame_id = m_robot_name+"/odom";
	// TODO orientation
	m_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
	bcm2835_i2c_end();
}




////////////////////////////////////////////
void Sensor_reader::reading()
{
	arduino_msg_union arduino_msg;
	arduino_data arduino_values;
	while ( package_elements < message_size)
	{ 
		unsigned char single_read_byte;
		size_t size;
		int size_int;
		size = m_serial_port.read(&single_read_byte,1);
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
		arduino_values = buffer_to_struct(arduino_msg.byte_buffer);
		ROS_INFO("SI CI SONO");
		arduino_to_odometry(arduino_values);
		m_reader_odom_pub.publish<nav_msgs::Odometry>(m_odometry);
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
	imu_reading();
}



double Sensor_reader::x_acceleration()
{
	int acc_x;
	m_regaddr[0] = H_BYTE_X_ACC_ADDRESS;//x-axis acc value (first byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_ret != BCM2835_I2C_REASON_OK)
	{
		//This is the basic operation to read an register
		//m_regaddr[0] is the register address
		//m_buf[0] is the value
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);//where and how many bytes
	}
	acc_x = m_buf[0]<<8;
	m_regaddr[0] = L_BYTE_X_ACC_ADDRESS;//x-axis acc value (second-byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_buf[0] == 99)//WHY??
	{
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);
	}
	acc_x += m_buf[0];
	//because of the sign, we have here 32-bit integers,
	//the value is 16-bit signed.
	if (acc_x & 1<<15)
	{
	    acc_x -= 1<<16;
	}
	return acc_x;
}



double Sensor_reader::y_acceleration()
{
	int acc_y;
	m_regaddr[0] = H_BYTE_Y_ACC_ADDRESS;//y-axis acc value (first byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_ret != BCM2835_I2C_REASON_OK)
	{
		//This is the basic operation to read an register
		//m_regaddr[0] is the register address
		//m_buf[0] is the value
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);//where and how many bytes
	}
	acc_y = m_buf[0]<<8;
	m_regaddr[0] = L_BYTE_Y_ACC_ADDRESS;//y-axis acc value (second-byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_buf[0] == 99)//WHY??
	{
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);
	}
	acc_y += m_buf[0];
	//because of the sign, we have here 32-bit integers,
	//the value is 16-bit signed.
	if (acc_y & 1<<15)
	{
	    acc_y -= 1<<16;
	}
	return acc_y;
}



double Sensor_reader::z_acceleration()
{
	int acc_z;
	m_regaddr[0] = H_BYTE_Z_ACC_ADDRESS;//z-axis acc value (first byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_ret != BCM2835_I2C_REASON_OK)
	{
		//This is the basic operation to read an register
		//m_regaddr[0] is the register address
		//m_buf[0] is the value
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);//where and how many bytes
	}
	acc_z = m_buf[0]<<8;
	m_regaddr[0] = L_BYTE_Z_ACC_ADDRESS;//z-axis acc value (second-byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_buf[0] == 99)//WHY??
	{
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);
	}
	acc_z += m_buf[0];
	//because of the sign, we have here 32-bit integers,
	//the value is 16-bit signed.
	if (acc_z & 1<<15)
	{
	    acc_z -= 1<<16;
	}
	return acc_z;
}



double Sensor_reader::x_gyro_axis()
{
	int w_x;
	m_regaddr[0] = H_BYTE_X_GYRO_ADDRESS;//x_axis gyroscope values (first byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_ret != BCM2835_I2C_REASON_OK)
	{
		//This is the basic operation to read an register
		//m_regaddr[0] is the register address
		//m_buf[0] is the value
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);//where and how many bytes
	}
	w_x = m_buf[0]<<8;
	m_regaddr[0] = L_BYTE_X_GYRO_ADDRESS;//x_axis gyroscope values (second-byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_buf[0] == 99)//WHY??
	{
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);
	}
	w_x += m_buf[0];
	//because of the sign, we have here 32-bit integers,
	//the value is 16-bit signed.
	if (w_x & 1<<15)
	{
	    w_x -= 1<<16;
	}
	return w_x;
}



double Sensor_reader::y_gyro_axis()
{
	int w_y;
	m_regaddr[0] = H_BYTE_Y_GYRO_ADDRESS;//y_axis gyroscope values (first byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_ret != BCM2835_I2C_REASON_OK)
	{
		//This is the basic operation to read an register
		//m_regaddr[0] is the register address
		//m_buf[0] is the value
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);//where and how many bytes
	}
	w_y = m_buf[0]<<8;
	m_regaddr[0] = L_BYTE_Y_GYRO_ADDRESS;//y_axis gyroscope values (second-byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_buf[0] == 99)//WHY??
	{
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);
	}
	w_y += m_buf[0];
	//because of the sign, we have here 32-bit integers,
	//the value is 16-bit signed.
	if (w_y & 1<<15)
	{
	    w_y -= 1<<16;
	}
	return w_y;
}



double Sensor_reader::z_gyro_axis()
{
	int w_z;
	m_regaddr[0] = H_BYTE_Z_GYRO_ADDRESS;//z_axis gyroscope values (first byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_ret != BCM2835_I2C_REASON_OK)
	{
		//This is the basic operation to read an register
		//m_regaddr[0] is the register address
		//m_buf[0] is the value
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);//where and how many bytes
	}
	w_z = m_buf[0]<<8;
	m_regaddr[0] = L_BYTE_Z_GYRO_ADDRESS;//z_axis gyroscope values (second-byte)
	m_ret = BCM2835_I2C_REASON_ERROR_DATA;
	while(m_buf[0] == 99)//WHY??
	{
		bcm2835_i2c_write(m_regaddr, 1);
		m_ret = bcm2835_i2c_read(m_buf, 1);
	}
	w_z += m_buf[0];
	//because of the sign, we have here 32-bit integers,
	//the value is 16-bit signed.
	if (w_z & 1<<15)
	{
	    w_z -= 1<<16;
	}
	return w_z;
}




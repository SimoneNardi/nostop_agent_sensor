#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Raspberry_sensor_reader.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835/src/bcm2835.h>


using namespace std;
using namespace Robotics;
using namespace GameTheory;


Raspberry_sensor_reader::Raspberry_sensor_reader(std::string& robot_name):
m_value(0)
, m_robot_name(robot_name)
, m_address(0x68)// imu adress --> sudo i2cdetect -y 1
{
	bcm2835_init();
	bcm2835_i2c_setSlaveAddress(m_address);
	m_raspberry_reader_imu_pub = reader.advertise<sensor_msgs::Imu>("/"+m_robot_name+"/imu_data", 5);
}


Raspberry_sensor_reader::~Raspberry_sensor_reader()
{    bcm2835_i2c_end();}



void Raspberry_sensor_reader::imu_reading()
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
	m_raspberry_reader_imu_pub.publish<sensor_msgs::Imu>(m_imu);
	printf("accel: %g\n", x_acc);
	bcm2835_i2c_end();
}




double Raspberry_sensor_reader::x_acceleration()
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



double Raspberry_sensor_reader::y_acceleration()
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



double Raspberry_sensor_reader::z_acceleration()
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



double Raspberry_sensor_reader::x_gyro_axis()
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



double Raspberry_sensor_reader::y_gyro_axis()
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



double Raspberry_sensor_reader::z_gyro_axis()
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




#include <ros/ros.h>
#include <Raspberry_sensor_reader.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835/src/bcm2835.h>


using namespace std;
using namespace Robotics;
using namespace GameTheory;


Raspberry_sensor_reader::Raspberry_sensor_reader():
m_value(0)
, m_address(0x68)// imu adress --> sudo i2cdetect -y 1
{
	bcm2835_init();
	bcm2835_i2c_setSlaveAddress(m_address);
}


Raspberry_sensor_reader::~Raspberry_sensor_reader()
{    bcm2835_i2c_end();}


void Raspberry_sensor_reader::run()//TODO
{
	

	while(ros::ok())
	{
		int acc_x;
		bcm2835_i2c_begin();// OK here or setSlave... must be after this one?
		//This is the basic operation to write to an register
		//m_regaddr[0] is the register address
		//m_regaddr[1] is the value
		m_regaddr[0] = 107;
		m_regaddr[1] = 0;
		//disable sleep mode
		bcm2835_i2c_write(m_regaddr, 2);//where and how many bytes
		
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

		double x_val = acc_x;
		x_val = x_val / 16384;
		//This is only valid if the accel-mode is +- 2g
		//The range can be controlled via the 
		//GYRO_CONFIG and ACCEL_CONFIG registers

		printf("accel: %g\n", x_val);
		bcm2835_i2c_end();
		ros::spinOnce();
	}

}
